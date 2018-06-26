
namespace imajuscule {
  namespace audio {

    template<
    int nOuts,
    XfadePolicy XF,
    MaxQueueSize MQS,
    AudioOutPolicy P
    >
    struct Channels {
        static constexpr auto policy = P;
        static constexpr auto atomicity = getAtomicity<policy>();
        static constexpr auto nAudioOut = nOuts;
        using Channel = Channel<atomicity, nAudioOut, XF, MQS>;
        using Request = typename Channel::Request;
        using Volumes = typename Channel::Volumes;
        using LockPolicy = AudioLockPolicyImpl<policy>;
        static constexpr auto XFPolicy = XF;
        static constexpr auto canRealloc =
          (atomicity==Atomicity::Yes) ? CanRealloc::No : CanRealloc::Yes;

        using LockFromRT = LockIf<LockPolicy::useLock, ThreadType::RealTime>;
        using LockFromNRT = LockIf<LockPolicy::useLock, ThreadType::NonRealTime>;
        using LockCtrlFromNRT = LockCtrlIf<LockPolicy::useLock, ThreadType::NonRealTime>;

      /*
       * The function is executed once, and then removed from the queue.
       */
      using OneShotFunc = std::function<void(Channels &)>;

      /*
       * returns false when the lambda can be removed
       */
      using OrchestratorFunc = std::function<bool(Channels &
                                                  , int // the max number of frames computed in one chunk
                                                  )>;
      /*
       * returns false when the lambda can be removed
       */
      using ComputeFunc = std::function<bool(bool // the clock
                                            ,int  // the number of frames to skip
                                             )>;

        Channels() : _lock(GlobalAudioLock<policy>::get()) {
            Assert(0 && "The other constructor should be used");
        }

        Channels(AudioLockPolicyImpl<policy> & l
                , int nChannelsMax
                , int nOrchestratorsMaxPerChannel=0):
        _lock(l)
      , orchestrators(nOrchestratorsMaxPerChannel * nChannelsMax)
      // (almost) worst case scenario : each channel is playing an audiolement crossfading with another audio element
      // "almost" only because the assumption is that requests vector holds at most one audioelement at any time
      // if there are multiple audioelements in request vector, we need to be more precise about when audioelements start to be computed...
      // or we need to constrain the implementation to add requests in realtime, using orchestrators.
      , computes(2*nChannelsMax)
      , oneShots(2*nChannelsMax)
        {
            Assert(nChannelsMax >= 0);
            Assert(nChannelsMax <= std::numeric_limits<uint8_t>::max()); // else need to update AUDIO_CHANNEL_NONE

            channels.reserve(nChannelsMax);
            autoclosing_ids.reserve(nChannelsMax);
            available_ids.reserve(nChannelsMax);
        }

      // this method should not be called from the real-time thread
      // because it yields() and retries.
      template<typename F>
      void enqueueOneShot(F f) {
        if constexpr(shouldNRTThreadUseOneshotsQueue<policy>()) {
          ++nOrchestratorsAndComputes;
          while(!oneShots.tryEnqueue(f)) {
            std::this_thread::yield();
          }
        }
        else {
          f(*this);
        }
      }

        bool add_orchestrator(OrchestratorFunc f) {
          if(!orchestrators.tryInsert(std::move(f))) {
            return false;
          }
          ++nOrchestratorsAndComputes;
          return true;
        }

        template<typename F>
        bool registerCompute(F f) {
          if(!computes.tryInsert(std::move(f))) {
            return false;
          }
          ++nOrchestratorsAndComputes;
          return true;
        }

        Channel & editChannel(uint8_t id) {
          Assert(id != AUDIO_CHANNEL_NONE);
          return channels[id];
        }

        Channel const & getChannel(uint8_t id) const {
          Assert(id != AUDIO_CHANNEL_NONE);
          return channels[id];
        }

        bool empty() const { return channels.empty(); }

        bool hasOrchestratorsOrComputes() const {
          return nOrchestratorsAndComputes > 0;
        }

        void toVolume(uint8_t channel_id, float volume, int nSteps) {
          LockFromNRT l(get_lock());
          enqueueOneShot([channel_id, volume, nSteps](auto&chans){
            chans.editChannel(channel_id).toVolume(volume, nSteps);
          });
        }

      template<typename Algo>
      [[nodiscard]] bool playComputable(PackedRequestParams<nAudioOut> params,
                                        audioelement::FinalAudioElement<Algo> & e) {

          // it's important to register and enqueue in the same lock cycle
          // else we miss some audio frames,
          // or the callback gets unscheduled

          LockCtrlFromNRT l(get_lock());


          bool res = false;
          // TODO when lock-free, instead of disallowing reallocation,
          // allocate the new buffer in the non-realtime thread,
          // and swap buffers in the oneShot.
          auto & c = editChannel(params.channel_id);
          if(reserveAndLock<canRealloc>(1,c.edit_requests(),l)) {
            static_assert(sizeof(PackedRequestParams<nAudioOut>) <= 8, "so that the lambda captures are <= 16 bytes");
            enqueueOneShot([&e,params](auto&chans){
              // error is ignored
              auto & c = chans.editChannel(params.channel_id);
              chans.playComputableNoLock(c, e.fCompute(), {&e.buffer->buffer[0], {params.volumes}, params.length });
            });
            res = true;
          }

          l.unlock();

          return res;
        }

      /* We expect that the caller:
       * - either took the out lock, or is in the audio realtime thread
       * - has grown the capacity of the channel request queue, if needed
       */
      template<typename F>
      [[nodiscard]] bool playComputableNoLock( Channel & channel, F compute, Request && req) {

        // we enqueue first, so that the buffer has the "queued" state
        // because when registering compute lambdas, they can be executed right away
        // so the buffer needs to be in the right state

        Assert(req.valid());
        if(!channel.addRequest( std::move(req) )) {
          return false;
        }

        if(!this->registerCompute(compute)) {
          channel.cancelLastRequest();
          return false;
        }
        return true;
      }

        [[nodiscard]] bool play( uint8_t channel_id, StackVector<Request> && v) {
          LockCtrlFromNRT l(get_lock());

          auto & c = editChannel(channel_id);

          bool res = false;
          if(reserveAndLock<canRealloc>(v.size(),c.edit_requests(),l)) {
            res = playNolock(channel_id, std::move(v));
          }

          l.unlock();

          return res;
        }

        void closeAllChannels(int xfade) {
          LockFromNRT l(get_lock());
          enqueueOneShot([xfade](auto&chans){
            if(!xfade) {
              chans.channels.clear();
            }
            else {
              for(auto & c:chans.channels) {
                if(c.isPlaying()) {
                  c.stopPlayingByXFadeToZero(xfade);
                }
              }
            }
          });
        }

        template<WithLock lock>
        [[nodiscard]] uint8_t openChannel(float volume, ChannelClosingPolicy l, int xfade_length = 0) {
            uint8_t id = AUDIO_CHANNEL_NONE;
            if(channels.size() == channels.capacity() && available_ids.size() == 0) {
                // Channels are at their maximum number and all are used...
                // Let's find one that is autoclosing and not playing :
                for( auto it = autoclosing_ids.begin(), end = autoclosing_ids.end(); it != end; ++it )
                {
                    id = *it;
                    if(lock==WithLock::No) {
                        if(channels[id].isPlaying()) {
                            id = AUDIO_CHANNEL_NONE;
                            continue;
                        }
                    }
                    else {
                        // take the lock in the loop so that at the end of each iteration
                        // the audio thread has a chance to run
                        LockFromNRT l(get_lock());  // TODO lockfree
                        if(channels[id].isPlaying()) {
                            id = AUDIO_CHANNEL_NONE;
                            continue;
                        }
                    }
                    // channel 'id' is auto closing and not playing, so we will assign it to the caller.
                    if(l != ChannelClosingPolicy::AutoClose) {
                        autoclosing_ids.erase(it);
                    }
                    break;
                }
                if(id == AUDIO_CHANNEL_NONE) {
                    LG(WARN, "no more channels available");
                    return AUDIO_CHANNEL_NONE;
                }
                Assert(!editChannel(id).isPlaying());
            }
            else {
                id = available_ids.Take(channels);
                if(id == AUDIO_CHANNEL_NONE) {
                    LG(WARN, "no more channels available");
                    return AUDIO_CHANNEL_NONE;
                }
                Assert(!editChannel(id).isPlaying());
                if(l == ChannelClosingPolicy::AutoClose) {
                    convert_to_autoclosing(id);
                }
            }

            // no need to lock here : the channel is not playing
            if(!editChannel(id).isActive()) {
                editChannel(id).reset();
            }
            if(XF==XfadePolicy::UseXfade) {
                editChannel(id).set_xfade(xfade_length);
            }
            else {
                Assert(xfade_length == 0); // make sure user is aware xfade will not be used
            }
            editChannel(id).setVolume(volume);
            return id;
        }

        template<WithLock l>
        void closeChannel(uint8_t channel_id, CloseMode mode, int nStepsForXfadeToZeroMode = -1)
        {
            if(l==WithLock::No) {
                closeChannelNoLock(channel_id, mode, nStepsForXfadeToZeroMode);
            }
            else {
                closeChannel(channel_id, mode, nStepsForXfadeToZeroMode);
            }
        }

        void closeChannel(uint8_t channel_id, CloseMode mode, int nStepsForXfadeToZeroMode = -1)
        {
            LockFromNRT l(get_lock());  // TODO lockfree
            closeChannelNoLock(channel_id, mode, nStepsForXfadeToZeroMode);
        }

        void closeChannelNoLock(uint8_t channel_id, CloseMode mode, int nStepsForXfadeToZeroMode = -1)
        {
            auto & c = editChannel(channel_id);
            if(mode != CloseMode::NOW && c.isPlaying()) {
                if(mode == CloseMode::XFADE_ZERO) {
                    auto it = std::find(autoclosing_ids.begin(), autoclosing_ids.end(), channel_id);
                    if(it == autoclosing_ids.end()) {
                        convert_to_autoclosing(channel_id);
                    }
                    c.stopPlayingByXFadeToZero(nStepsForXfadeToZeroMode);
                }
                else if(mode == CloseMode::WHEN_DONE_PLAYING) {
#ifndef NDEBUG
                    auto it = std::find(autoclosing_ids.begin(), autoclosing_ids.end(), channel_id);
                    Assert(it == autoclosing_ids.end()); // if channel is already autoclosing, this call is redundant
#endif
                    convert_to_autoclosing(channel_id);
                }
                return;
            }
#ifndef NDEBUG
            auto it = std::find(autoclosing_ids.begin(), autoclosing_ids.end(), channel_id);
            Assert(it == autoclosing_ids.end()); // if channel is autoclosing, we should remove it there?
#endif
            c.reset();
            available_ids.Return(channel_id);
        }

        void run_computes(bool tictac, int nFrames) {
          nOrchestratorsAndComputes -= oneShots.dequeueAll([this](auto const & f) { f(*this); });

          nOrchestratorsAndComputes -= orchestrators.forEach([this](auto const & orchestrate) {
            return orchestrate(*this, audioelement::n_frames_per_buffer);
          });

          nOrchestratorsAndComputes -= computes.forEach([tictac, nFrames](auto const & compute) {
            return compute(tictac, nFrames);
          });
        }

        template <typename F>
        void forEach(F f) {
            for(auto & c : channels) {
                f(c);
            }
        }

        decltype(std::declval<AudioLockPolicyImpl<policy>>().lock()) get_lock() { return _lock.lock(); }


    private:
        AudioLockPolicyImpl<policy> & _lock;

        AvailableIndexes<uint8_t> available_ids;
        std::vector<Channel> channels;
        std::vector<uint8_t> autoclosing_ids;

        lockfree::scmp::fifo<OneShotFunc> oneShots;

        // orchestrators and computes could be owned by the channels but it is maybe cache-wise more efficient
        // to group them here (if the lambda owned is small enough to not require dynamic allocation)
        //
        // We use the 'Synchronization::SingleThread' synchronization because
        // these vectors are accessed from a single thread only (the audio real-time thread).
        static_vector<Synchronization::SingleThread, OrchestratorFunc> orchestrators;
        static_vector<Synchronization::SingleThread, ComputeFunc> computes;

        // This counter is used to be able to know, without locking, if there are
        // any computes / orchestrators / oneshot functions ATM.
        int32_t nOrchestratorsAndComputes = 0;

        [[nodiscard]] bool playNolock( uint8_t channel_id, StackVector<Request> && v) {
            bool res = true;
            auto & c = editChannel(channel_id);
            for( auto & sound : v ) {
                Assert(sound.valid());
                if(!c.addRequest( std::move(sound) )) {
                    res = false;
                }
            }
            return res;
        }

        void convert_to_autoclosing(uint8_t channel_id) {
            Assert(autoclosing_ids.size() < autoclosing_ids.capacity());
            // else logic error : some users closed manually some autoclosing channels
            autoclosing_ids.push_back(channel_id);
        }

    };
  }
}
