#define WITH_DELAY 0

namespace imajuscule {
   
    namespace Sensor {
        
        class RAIILock {
        public:
            RAIILock( std::atomic_bool & l ) : l(l) {
                while (l.exchange(true)) { }
            }
            ~RAIILock() {
                l = false;
            }
        private:
            std::atomic_bool & l;
            
            RAIILock(const RAIILock &) = delete;
            RAIILock & operator = (const RAIILock &) = delete;
        };
    }
    
    template<int nAudioOut>
    struct DelayLine {
        DelayLine(int size, float attenuation): delay(size,{{}}), it(0), end(size), attenuation(attenuation) {}
        
        void step(SAMPLE *outputBuffer, int nFrames) {
            for( int i=0; i < nFrames; i++ ) {
                auto & d = delay[it];
                for(auto j=0; j<nAudioOut; ++j) {
                    auto delayed = d[j];
                    d[j] = *outputBuffer;
                    *outputBuffer += attenuation * delayed;
                    ++outputBuffer;
                }
                ++it;
                if( unlikely(it == end) ) {
                    it = 0;
                }
            }
        }

        std::vector<std::array<float, nAudioOut>> delay;
        int32_t it, end;
        float attenuation;
    };
    
    
    template<typename T, typename Init, size_t... Inds>
    std::array<T, sizeof...(Inds)> makeArrayImpl(Init val, std::integer_sequence<size_t, Inds...>)
    {
        return { (val + (Inds - Inds))... };
    }
    
    template<typename T, int N, typename Init>
    std::array<T, N> makeArray(Init val)
    {
        return makeArrayImpl<T, Init>(val, std::make_index_sequence<N>{});
    }

    /*
    struct Compressor {
        // some parts inspired from https://github.com/audacity/audacity/blob/master/src/effects/Compressor.cpp
        
        static constexpr auto length_sliding_avg = 40;
        
        Compressor(Compressor&&) = default;
        Compressor& operator=(Compressor&&) = default;
        
        Compressor() : avgs(makeArray<slidingAverage<float, KEEP_INITIAL_VALUES>, nAudioOut>(length_sliding_avg)) {
        }
        std::array<slidingAverage<float, KEEP_INITIAL_VALUES>, nAudioOut> avgs;
        
        float threshold = 0.5f;
        static constexpr auto ratio = 3.f;
        float compression = 1.f-1.f/ratio;
        float compute(float value, float env)
        {
            if(env <= 0.f) {
                return 0.f;
            }
            return value * powf(threshold/env, compression);
        }
    };*/
    
    // reserved number to indicate "no channel"
    static constexpr auto AUDIO_CHANNEL_NONE = std::numeric_limits<uint8_t>::max();
    
    enum class CloseMode {
        NOW, // channel is closed now even if it is playing something
        XFADE_ZERO, // channel will be converted to autoclosing and forced to crossfade to zero right now.
        WHEN_DONE_PLAYING, // channel will be converted to autoclosing
    };
    
    enum class PostProcess {
        HARD_LIMIT,
        NONE
    };
    
    struct NoOpLock : public NonCopyable {
        // no copy
        NoOpLock() =default;
        NoOpLock(const NoOpLock &) = delete;
        NoOpLock & operator=(const NoOpLock&) = delete;
        
        NoOpLock(std::atomic_bool const &) {}
    };
    
    enum class ChannelClosingPolicy {
        AutoClose,  // once the request queue is empty (or more precisely
        // once the channel method isPlaying() returns false),
        // the channel can be automatically reassigned without
        // the need to close it explicitely.
        // Explicitely closing an AutoClose channel will result in undefined behaviour.
        ExplicitClose, // the channel cannot be reassigned unless explicitely closed
    };
    
    

    template<
    int nAudioOut,
    XfadePolicy XF = XfadePolicy::UseXfade,
    typename Locking = Sensor::RAIILock,
    PostProcess Post = PostProcess::HARD_LIMIT
    >
    struct outputDataBase {
        using Channel = Channel<nAudioOut, XF>;
        using Request = typename Channel::Request;
        using Volumes = typename Channel::Volumes;
        using LOCK = Locking;
        
        static constexpr auto nOuts = nAudioOut;
        
        static constexpr auto initial_n_audioelements_reserve = 4;
        
        int count_consummed_frames() const { return consummed_frames; }
    private:
        std::atomic_bool used { false };
        
        //////////////////////////
        /// state of last write:
        
        bool clock_ : 1; /// "tic tac" flag, inverted at each new AudioElements buffer writes
        
        /// the number of buffer frames that were used from the previous AudioElements buffer write
        /// "0" means the entire buffers where used
        unsigned int consummed_frames : relevantBits( audioelement::n_frames_per_buffer - 1 );
        ///
        //////////////////////////
        
        AvailableIndexes<uint8_t> available_ids;
        std::vector<Channel> channels;
        std::vector<uint8_t> autoclosing_ids;

        // this could be replaced by traversing the pool of AudioElements, and
        // computing the ones that are active.
        using postProcessFunc = std::function<void(float*)>;

        std::vector<audioelement::ComputeFunc> audioElements_computes;
        
        std::vector<postProcessFunc> post_process;
        
#if WITH_DELAY
        std::vector< DelayLine > delays;
#endif
        
#if TARGET_OS_IOS
    public:
        std::vector<float> outputBuffer;
    private:
#endif
        
    private:
        template<typename F>
        void registerAudioElementCompute(F f) {
            // because we are holding the audio lock, we should not do any system calls.
            // in case cannot know in advance how many will be needed , do 2 step locking:
            // first lock to know how much is needed
            // allocate outside the lock
            // lock : copy existing to new, then swap
            A(audioElements_computes.capacity() > audioElements_computes.size());
            audioElements_computes.push_back(std::move(f));
            if(isInbetweenTwoAudioElementComputes()) {
                audioElements_computes.back()(clock_);
            }
        }
        
    public:
        
        std::atomic_bool & lock() { return used; }
        
        bool isInbetweenTwoAudioElementComputes() const {
            A(consummed_frames >= 0);
            A(consummed_frames < audioelement::n_frames_per_buffer);
            return 0 != consummed_frames;
        }
        
        outputDataBase(int nChannelsMax = std::numeric_limits<uint8_t>::max())
        :
#if WITH_DELAY
        delays{{1000, 0.6f},{4000, 0.2f}, {4300, 0.3f}, {5000, 0.1f}},
#endif
        clock_(false),
        consummed_frames(0)
        {
            A(nChannelsMax >= 0);
            A(nChannelsMax <= std::numeric_limits<uint8_t>::max()); // else need to update AUDIO_CHANNEL_NONE
            
            audioElements_computes.reserve(initial_n_audioelements_reserve);
            
            // to avoid reallocations when we hold the lock
            // we allocate all we need for channel management now:
            channels.reserve(nChannelsMax);
            autoclosing_ids.reserve(nChannelsMax);
            available_ids.reserve(nChannelsMax);

            if(Post == PostProcess::HARD_LIMIT) {
                post_process.emplace_back([](float v[nAudioOut]) {
                    for(int i=0; i<nAudioOut; ++i) {
                        if(likely(-1.f <= v[i] && v[i] <= 1.f)) {
                            continue;
                        }
                        
                        if(v[i] > 1.f) {
                            A(0);
                            v[i] = 1.f;
                        }
                        else if(v[i] < -1.f) {
                            A(0);
                            v[i] = -1.f;
                        }
                        else {
                            A(0);
                            v[i] = 0.f; // v[i] is NaN
                        }
                    }
                });
            }
        }

        // called from audio callback
        void step(SAMPLE *outputBuffer, int nFrames) {
            Locking l(used);
            
            if(consummed_frames != 0) {
                // finish consuming previous buffers
                if(!consume_buffers(outputBuffer, nFrames)) {
                    return;
                }
            }
            
            while(true) {
                // the previous buffers are consumed, we need to compute them again
                computeNextAudioElementsBuffers();
                
                if(!consume_buffers(outputBuffer, nFrames)) {
                    return;
                }
            }
        }

        Channel & editChannel(uint8_t id) { return channels[id]; }
        
        Channel const & getChannel(uint8_t id) const { return channels[id]; }
        
        bool empty() const { return channels.empty(); }
        
        void setVolume(uint8_t channel_id, float volume, int nSteps = Channel::default_volume_transition_length) {
            Locking l(used);
            editChannel(channel_id).toVolume(volume, nSteps);
        }
        
        template<class... Args>
        bool playGeneric( uint8_t channel_id, Args&&... requests) {
            
            // it's important to register and enqueue in the same lock cycle
            // else we miss some audio frames,
            // or the callback gets unscheduled
            
            Locking l(used);

            return playGenericNoLock(channel_id, std::forward<Args>(requests)...);
        }
        
        
        template<class... Args>
        bool playGenericNoLock( uint8_t channel_id, Args&&... requests) {
            
            // we enqueue first, so that the buffer has the "queued" state
            // because when registering compute lambdas, they can be executed right away
            // so the buffer needs to be in the right state

            bool res = playNolock(channel_id, {std::move(requests.second)...});
            
            auto buffers = std::make_tuple(std::ref(requests.first)...);
            for_each(buffers, [this](auto &buf) {
                if(auto f = audioelement::fCompute(buf)) {
                    this->registerAudioElementCompute(std::move(f));
                }
            });
            
            return res;
        }
        
        void play( uint8_t channel_id, StackVector<Request> && v) {
            Locking l(used);
            playNolock(channel_id, std::move(v));
        }
        
        void closeAllChannels(int xfade) {
            Locking l(used);
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

        uint8_t openChannel(float volume, ChannelClosingPolicy l, int xfade_length = 0) {
            uint8_t id = AUDIO_CHANNEL_NONE;
            if(channels.size() == channels.capacity() && available_ids.size() == 0) {
                // Channels are at their maximum number and all are used...
                // Let's find one that is autoclosing and not playing :
                for( auto it = autoclosing_ids.begin(), end = autoclosing_ids.end(); it != end; ++it )
                {
                    id = *it;
                    {
                        // take the lock in the loop so that at the end of each iteration
                        // the audio thread has a chance to run
                        Locking l(used);
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
                A(!editChannel(id).isPlaying());
            }
            else {
                id = available_ids.Take(channels);
                if(id == AUDIO_CHANNEL_NONE) {
                    LG(WARN, "no more channels available");
                    return AUDIO_CHANNEL_NONE;
                }
                A(!editChannel(id).isPlaying());
                if(l == ChannelClosingPolicy::AutoClose) {
                    convert_to_autoclosing(id);
                }
            }

            // no need to lock here : the channel is not playing
            if(!editChannel(id).isActive()) {
                editChannel(id).reset();
            }
            editChannel(id).setVolume(volume);
            if(XF==XfadePolicy::UseXfade) {
                editChannel(id).set_xfade(xfade_length);
            }
            else {
                A(xfade_length == 0); // make sure user is aware xfade will not be used
            }
            return id;
        }
        
        void closeChannel(uint8_t channel_id, CloseMode mode, int nStepsForXfadeToZeroMode = -1)
        {
            {
                Locking l(used);
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
                        A(it == autoclosing_ids.end()); // if channel is already autoclosing, this call is redundant
#endif
                        convert_to_autoclosing(channel_id);
                    }
                    return;
                }
#ifndef NDEBUG
                auto it = std::find(autoclosing_ids.begin(), autoclosing_ids.end(), channel_id);
                A(it == autoclosing_ids.end()); // if channel is autoclosing, we should remove it there?
#endif
                c.reset();
            }
            available_ids.Return(channel_id);
        }
        
    private:
        void convert_to_autoclosing(uint8_t channel_id) {
            A(autoclosing_ids.size() < autoclosing_ids.capacity());
            // else logic error : some users closed manually some autoclosing channels
            autoclosing_ids.push_back(channel_id);
        }
        
        bool playNolock( uint8_t channel_id, StackVector<Request> && v) {
            bool res = true;
            auto & c = editChannel(channel_id);
            for( auto & sound : v ) {
                A(sound.valid());
                if(!c.addRequest( std::move(sound) )) {
                    res = false;
                }
            }
            return res;
        }

        void computeNextAudioElementsBuffers() {
            A(consummed_frames == 0); // else we skip some unconsummed frames
            clock_ = !clock_; // keep that BEFORE passing clock_ to compute functions (dependency on registerAudioElementCompute)
            for(auto it = audioElements_computes.begin(),
                end = audioElements_computes.end(); it!=end;) {
                if(!((*it)(clock_))) {
                    it = audioElements_computes.erase(it);
                    end = audioElements_computes.end();
                }
                else {
                    ++it;
                }
            }
            A(consummed_frames == 0);
        }
        
        // returns true if everything was consummed AND there is more frames remaining
        bool consume_buffers(SAMPLE *& buf, int & nFrames) {
            A(consummed_frames < audioelement::n_frames_per_buffer);
            auto remaining_frames = audioelement::n_frames_per_buffer - consummed_frames;
            A(remaining_frames <= audioelement::n_frames_per_buffer);
            A(remaining_frames > 0);
            if(remaining_frames > nFrames) {
                // partial consume
                do_consume_buffers(buf, nFrames);
                consummed_frames += nFrames;
                A(consummed_frames < audioelement::n_frames_per_buffer);
                return false;
            }
            // total consume
            do_consume_buffers(buf, remaining_frames);
            consummed_frames = 0;
            nFrames -= remaining_frames;
            if(nFrames == 0) {
                return false;
            }
            buf += nAudioOut * remaining_frames;
            return true;
        }
        
        void do_consume_buffers(SAMPLE * outputBuffer, int nFrames) {
            A(nFrames <= audioelement::n_frames_per_buffer); // by design
            A(consummed_frames < audioelement::n_frames_per_buffer); // by design
            
            memset(outputBuffer, 0, nFrames * nAudioOut * sizeof(SAMPLE));
            
            for( auto & c: channels ) {
                c.step(outputBuffer,
                       nFrames,
                       consummed_frames ); // with that, the channel knows when
                                           // the next computation of AudioElements will occur
                if(c.shouldReset()) {
                    c.reset();
                }
            }

            for(int i=0; i<nFrames; ++i) {
                for(auto const & f: post_process) {
                    f(&outputBuffer[i*nAudioOut]); // or call the lambda for the whole buffer at once?
                }
            }

#if WITH_DELAY
            for( auto & delay : delays ) {
                // todo low pass filter for more realism
                delay.step(outputBuffer, nFrames);
            }
#endif
        }
    };
    
    using outputData = outputDataBase<2>;
}
