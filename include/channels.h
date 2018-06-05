
namespace imajuscule {
  namespace audio {

    template<
    int nOuts,
    XfadePolicy XF,
    MaxQueueSize MQS,
    AudioOutPolicy policy
    >
    struct Channels {
        static constexpr auto Policy = policy;
        static constexpr auto nAudioOut = nOuts;
        using Channel = Channel<nAudioOut, XF, MQS>;
        using Request = typename Channel::Request;
        using Volumes = typename Channel::Volumes;
        using LockPolicy = AudioLockPolicyImpl<policy>;
        static constexpr auto XFPolicy = XF;

        using LockFromRT = LockIf<LockPolicy::useLock, ThreadType::RealTime>;
        using LockFromNRT = LockIf<LockPolicy::useLock, ThreadType::NonRealTime>;
        using LockCtrlFromNRT = LockCtrlIf<LockPolicy::useLock, ThreadType::NonRealTime>;

        using OrchestratorFunc = std::function<bool(Channels &, int)>;
        Channels() : _lock(GlobalAudioLock<Policy>::get()) {
            Assert(0 && "The other constructor should be used");
        }

        Channels(AudioLockPolicyImpl<policy> & l
                , int nChannelsMax
                , int nOrchestratorsMaxPerChannel=0):
        _lock(l)
        {
            Assert(nChannelsMax >= 0);
            Assert(nChannelsMax <= std::numeric_limits<uint8_t>::max()); // else need to update AUDIO_CHANNEL_NONE

            // (almost) worst case scenario : each channel is playing an audiolement crossfading with another audio element
            // "almost" only because the assumption is that requests vector holds at most one audioelement at any time
            // if there are multiple audioelements in request vector, we need to be more precise about when audioelements start to be computed...
            // or we need to constrain the implementation to add requests in realtime, using orchestrators.
            computes.reserve(2*nChannelsMax);

            orchestrators.reserve(nOrchestratorsMaxPerChannel * nChannelsMax);

            channels.reserve(nChannelsMax);
            autoclosing_ids.reserve(nChannelsMax);
            available_ids.reserve(nChannelsMax);
        }

        void add_orchestrator(OrchestratorFunc f) {
            Assert(orchestrators.capacity() > orchestrators.size()); // we are in the audio thread, we shouldn't allocate dynamically
            orchestrators.push_back(std::move(f));
        }

        template<typename F, typename Out>
        void registerCompute(Out const & out, F f) {
            Assert(computes.capacity() > computes.size()); // we are in the audio thread, we shouldn't allocate dynamically
            computes.push_back(std::move(f));
            if(out.isInbetweenTwoComputes()) {
                computes.back()(out.getTicTac());
            }
        }

        Channel & editChannel(uint8_t id) { return channels[id]; }

        Channel const & getChannel(uint8_t id) const { return channels[id]; }

        bool empty() const { return channels.empty(); }

        bool hasOrchestrators() const {
            return !orchestrators.empty();
        }

        void toVolume(uint8_t channel_id, float volume, int nSteps) {
            LockFromNRT l(get_lock());
            editChannel(channel_id).toVolume(volume, nSteps);
        }
        void setVolume(uint8_t channel_id, float volume) {
            editChannel(channel_id).setVolume(volume);
        }
        void setXFade(uint8_t channel_id, int xf) {
            editChannel(channel_id).set_xfade(xf);
        }
      
        template<typename Out, typename T>
        bool playGeneric( Out & out, uint8_t channel_id, T & buf, Request && req) {

          // it's important to register and enqueue in the same lock cycle
          // else we miss some audio frames,
          // or the callback gets unscheduled
          
          LockCtrlFromNRT l(get_lock());
          
          auto & c = editChannel(channel_id);
          reserveAndLock(1,c.edit_requests(),l);

          auto res = playGenericNoLock(out, channel_id, buf, std::move(req));

          l.unlock();

          return res;
        }


        /* We expect that the caller has (if needed):
         * - taken the out lock
         * - grown the capacity of the channel request queue
         */
        template<typename Out, typename T>
        bool playGenericNoLock( Out & out, uint8_t channel_id, T & buf, Request && req) {

            // we enqueue first, so that the buffer has the "queued" state
            // because when registering compute lambdas, they can be executed right away
            // so the buffer needs to be in the right state

            bool res = playNolock(channel_id, {std::move(req)});

            if(auto f = audioelement::fCompute(buf)) {
                this->registerCompute(out, std::move(f));
            }

            return res;
        }

        void play( uint8_t channel_id, StackVector<Request> && v) {
          LockCtrlFromNRT l(get_lock());
          
          auto & c = editChannel(channel_id);
          reserveAndLock(v.size(),c.edit_requests(),l);

          playNolock(channel_id, std::move(v));

          l.unlock();
        }

        void closeAllChannels(int xfade) {
            LockFromNRT l(get_lock());
            if(!xfade) {
                channels.clear();
            }
            else {
                for(auto & c:channels) {
                    if(c.isPlaying()) {
                        c.stopPlayingByXFadeToZero(xfade);
                    }
                }
            }
        }

        template<WithLock lock>
        uint8_t openChannel(float volume, ChannelClosingPolicy l, int xfade_length = 0) {
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
                        LockFromNRT l(get_lock());
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
            LockFromNRT l(get_lock());
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

        void run_computes(bool tictac) {
            for(auto it = orchestrators.begin(), end = orchestrators.end(); it!=end;) {
                if(!((*it)(*this, audioelement::n_frames_per_buffer))) { // can grow 'computes' vector
                    it = orchestrators.erase(it);
                    end = orchestrators.end();
                }
                else {
                    ++it;
                }
            }

            for(auto it = computes.begin(), end = computes.end(); it!=end;) {
                if(!((*it)(tictac))) {
                    it = computes.erase(it);
                    end = computes.end();
                }
                else {
                    ++it;
                }
            }
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


        // orchestrators and computes could be owned by the channels but it is maybe cache-wise more efficient
        // to group them here (if the lambda owned is small enough to not require dynamic allocation)
        std::vector<OrchestratorFunc> orchestrators;
        std::vector<audioelement::ComputeFunc> computes;

        bool playNolock( uint8_t channel_id, StackVector<Request> && v) {
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
