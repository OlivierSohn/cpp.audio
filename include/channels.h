
namespace imajuscule::audio {


#ifndef CUSTOM_SAMPLE_RATE

// TODO This trick was done when we wanted to minimize captured data in realtime functors,
// but now tha we use folly::Function, we can keep full-precision midi timestamps,
// and store the extra information in extra variables. This will also allow
// to support generalized sample rates.

  // The MIDI timestamps don't need sub-frame precision:
  // we drop bits strictly after the highest 1-bit of 'nanosPerFrame',
  // and use those dropped bits to store information about the source.
  //
  // 0000000000000000000000011100101 = nanosPerFrame
  //                        |
  //                        highest 1-bit of 'nanosPerFrame'
  //                        |
  // 1100010101011100100010010101011 = MIDI timestamp A
  //                         vvvvvvv : these bits are dropped, but taken into account for rounding.
  // 1100010101011100100010010000000 = MIDI timestamp A, rounded (down)
  //
  // 1100010101011100100010011101011 = MIDI timestamp B
  //                         vvvvvvv : these bits are dropped, but taken into account for rounding.
  // 1100010101011100100010100000000 = MIDI timestamp B, rounded (up)
  //
  struct MIDITimestampAndSource {

    static constexpr uint64_t nanosPerFrame = static_cast<uint64_t>(0.5f + nanos_per_frame<float>(SAMPLE_RATE));
    static constexpr uint64_t highestBitNanosPerFrame = relevantBits(nanosPerFrame);
    static constexpr uint64_t nSourceKeyBits = highestBitNanosPerFrame - 1;
    static constexpr uint64_t nTimingBits = 64 - nSourceKeyBits;
    using SplitT = Split<uint64_t, nTimingBits, nSourceKeyBits>;
    static constexpr uint64_t nSources = 1 + SplitT::maxLow;
    static_assert(nSources == pow2(nSourceKeyBits));

    static constexpr uint64_t getNanosTimeGranularity() {
      return 1 << nSourceKeyBits;
    }

    MIDITimestampAndSource():split(0,0) {}

    MIDITimestampAndSource(uint64_t t, uint64_t sourceKey) : split(approximate(t),sourceKey) {
    }

    uint64_t getNanosTime() const {
      return split.getHighWithZeros();
    }
    uint64_t getSourceKey() const {
      return split.getLow();
    }

private:
    SplitT split;

    static constexpr uint64_t approximate(uint64_t t) {
      // round up or down, depending on the first ignored bit value.
      uint64_t firstIgnoredBit = t & (static_cast<uint64_t>(1) << (nSourceKeyBits - 1));
      uint64_t floored = t >> nSourceKeyBits;
      return (firstIgnoredBit == 0) ? floored : floored + 1;
    }
  };
#endif

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
      using OneShotFunc = folly::Function<void(Channels &
                                            , const uint64_t // the time of the start of the buffer that will be computed.
                                            )>;

#ifndef CUSTOM_SAMPLE_RATE
      /*
       * The function is executed once, and then removed from the queue.
       */
       // I added this because I needed to capture 'MIDITimestampAndSource' and
       // didn't have any space left in the std::function (and didn't want to dynamically allocate).
      // TODO now that we use folly::Function, this might not be needed anymore (we can capture more without allocating)
      using OneShotMidiFunc = folly::Function<void(Channels &
                                                , MIDITimestampAndSource
                                                , const uint64_t // the time of the start of the buffer that will be computed.
                                                )>;
#endif

      /*
       * returns false when the lambda can be removed
       */
      using ComputeFunc = folly::Function<bool(const int  // the number of frames to compute
                                             , const uint64_t // the time of the first frame
                                             )>;

        Channels() : _lock(GlobalAudioLock<policy>::get()) {
            Assert(0 && "The other constructor should be used");
        }

        Channels(AudioLockPolicyImpl<policy> & l
                , int nChannelsMax):
        _lock(l)
      // (almost) worst case scenario : each channel is playing an audiolement crossfading with another audio element
      // "almost" only because the assumption is that requests vector holds at most one audioelement at any time
      // if there are multiple audioelements in request vector, we need to be more precise about when audioelements start to be computed...
      // Also, if the output buffer size is big, we may have multiple events in the queue for the same note,
      // so we use a multiplicative factor of 4:
      , computes(4*nChannelsMax)
      , oneShots(4*nChannelsMax)
#ifndef CUSTOM_SAMPLE_RATE
      , oneShotsMIDI(4*nChannelsMax)
#endif
      {
        Assert(nChannelsMax >= 0);
        Assert(nChannelsMax <= std::numeric_limits<uint8_t>::max()); // else need to update AUDIO_CHANNEL_NONE

        channels.reserve(nChannelsMax);
        autoclosing_ids.reserve(nChannelsMax);
        available_ids.reserve(nChannelsMax);

        nRealtimeFuncs.store(0,std::memory_order_relaxed);
      }

      // this method should not be called from the real-time thread
      // because it yields() and retries.
      template<typename F>
      void enqueueOneShot(F f) {
        if constexpr(shouldNRTThreadUseOneshotsQueue<policy>()) {
          nRealtimeFuncs.fetch_add(1, std::memory_order_relaxed);
          bool failed = false;
          while(!oneShots.tryEnqueue(f)) {
            if (!failed) {
              failed = true;
              ++retried_oneshot_insertion;
            }
            std::this_thread::yield();
          }
        }
        else {
          f(*this, 0);
        }
      }

#ifndef CUSTOM_SAMPLE_RATE
      // this method should not be called from the real-time thread
      // because it yields() and retries.
      template<typename F>
      void enqueueMIDIOneShot(MIDITimestampAndSource m, F f) {
        if constexpr(shouldNRTThreadUseOneshotsQueue<policy>()) {
          nRealtimeFuncs.fetch_add(1, std::memory_order_relaxed);
          bool failed = false;
          while(!oneShotsMIDI.tryEnqueue({m,f})) {
            if (!failed) {
              failed = true;
              ++retried_midioneshot_insertion;
            }
          }
        }
        else {
          f(*this, m, 0);
        }
      }
#endif

        template<typename F>
        bool registerCompute(F f) {
          if(!computes.tryInsert(std::move(f))) {
            ++failed_compute_insertion;
            return false;
          }
          nRealtimeFuncs.fetch_add(1, std::memory_order_relaxed);
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
      
      uint8_t getChannelId(Channel const & c) const {
        std::ptrdiff_t diff = &c - channels.data();
        Assert(diff >= 0);
        Assert(diff <= std::numeric_limits<uint8_t>::max());
        return static_cast<uint8_t>(diff);
      }

        bool empty() const { return channels.empty(); }

        bool hasRealtimeFunctions() const {
          return nRealtimeFuncs.load(std::memory_order_relaxed) > 0;
        }

        void toVolume(uint8_t channel_id, float volume, int nSteps) {
          LockFromNRT l(get_lock());
          enqueueOneShot([channel_id, volume, nSteps](auto&chans, uint64_t){
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
            enqueueOneShot([&e,params](auto&chans, uint64_t){
              // error is ignored
              auto & c = chans.editChannel(params.channel_id);
              chans.playComputableNoLock(c, e.fCompute(), Request{&e.buffer->buffer[0], {params.volumes}, params.count_frames });
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
          enqueueOneShot([xfade](auto&chans, uint64_t){
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

        /*
        * Called from the audio realtime thread.
        */
        void run_computes(int const nFrames, uint64_t const tNanos) {
          int nRemoved(0);

#ifndef CUSTOM_SAMPLE_RATE
          nRemoved += oneShotsMIDI.dequeueAll([this, tNanos](auto & p) {
            if (p.second.heapAllocatedMemory()) {
              throw std::runtime_error("no allocation is allowed in functors");
            }
            p.second(*this, p.first, tNanos);
          });
#endif

          nRemoved += oneShots.dequeueAll([this, tNanos](auto & f) {
            if (f.heapAllocatedMemory()) {
              throw std::runtime_error("no allocation is allowed in functors");
            }
            f(*this, tNanos);
          });

          nRemoved += computes.forEach([tNanos, nFrames](auto & compute) {
            if (compute.heapAllocatedMemory()) {
              throw std::runtime_error("no allocation is allowed in functors");
            }
            return compute(nFrames, tNanos);
          });

          nRealtimeFuncs.fetch_sub(nRemoved, std::memory_order_relaxed);
        }

        template <typename F>
        void forEach(F f) {
            for(auto & c : channels) {
                f(c);
            }
        }

        decltype(std::declval<AudioLockPolicyImpl<policy>>().lock()) get_lock() { return _lock.lock(); }

      int countFailedComputeInsertions() const {
        return failed_compute_insertion;
      }
      int countRetriedOneshotInsertions() const {
        return retried_oneshot_insertion;
      }

    private:
        AudioLockPolicyImpl<policy> & _lock;

        AvailableIndexes<uint8_t> available_ids;
        std::vector<Channel> channels;
        std::vector<uint8_t> autoclosing_ids;

#ifndef CUSTOM_SAMPLE_RATE
        lockfree::scmp::fifo<std::pair<MIDITimestampAndSource,OneShotMidiFunc>> oneShotsMIDI;
#endif
        lockfree::scmp::fifo<OneShotFunc> oneShots;

        // computes could be owned by the channels but it is maybe cache-wise more efficient
        // to group them here (if the lambda owned is small enough to not require dynamic allocation)
        //
        // We use the 'Synchronization::SingleThread' synchronization because
        // these vectors are accessed from a single thread only (the audio real-time thread).
        static_vector<Synchronization::SingleThread, ComputeFunc> computes;

        // This counter is used to be able to know, without locking, if there are
        // any computes / oneshot functions ATM.
        std::atomic_int nRealtimeFuncs; // all operations are 'memory_order_relaxed'
        static_assert(std::atomic_int::is_always_lock_free);
      
      std::atomic_int failed_compute_insertion = 0;
      std::atomic_int retried_oneshot_insertion = 0;
#ifndef CUSTOM_SAMPLE_RATE
      std::atomic_int retried_midioneshot_insertion = 0;
#endif

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
} // NS imajuscule::audio
