#define WITH_DELAY 0

namespace imajuscule {
    
    namespace Sensor {
        
        class RAIILock {
        public:
            RAIILock( std::atomic_bool & l ) : l(l) {
                while (l.exchange(true)) {
                    std::this_thread::yield();
                }
            }
            ~RAIILock() {
                auto prev = l.exchange(false);
                A(prev); // make sure l was true
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
    
    enum class WithLock {
        Yes,No
    };
    
    template<WithLock>
    struct LockIf_;
    
    struct NoOpLock {
        NoOpLock(bool) {}
    };
    
    template <>
    struct LockIf_<WithLock::Yes> { using type = Sensor::RAIILock; };
    template <>
    struct LockIf_<WithLock::No> { using type = NoOpLock; };
    
    template<WithLock l>
    using LockIf = typename LockIf_<l>::type;
    
    enum class ChannelClosingPolicy {
        AutoClose,  // once the request queue is empty (or more precisely
        // once the channel method isPlaying() returns false),
        // the channel can be automatically reassigned without
        // the need to close it explicitely.
        // Explicitely closing an AutoClose channel will result in undefined behaviour.
        ExplicitClose, // the channel cannot be reassigned unless explicitely closed
    };
    
    enum class AudioOutPolicy {
        Slave,
        Master
    };
    
    // cooley-tukey leads to error growths of O(log n) (worst case) and O(sqrt(log n)) (mean for random input)
    // so float is good enough
    using FFT_T = float;
    
    template<typename T, int nAudioOut, AudioOutPolicy> struct AudioPolicyImpl;
    
    template<typename T, int nAudioOut>
    struct AudioPolicyImpl<T, nAudioOut, AudioOutPolicy::Slave> {
        /////////////////////////////// lock
        static constexpr WithLock useLock = WithLock::No;
        
        bool lock() { return false; }
        
        /////////////////////////////// postprocess
        void postprocess(T*buffer, int nFrames) const {}
        bool isReady() const { return true; }
        
        /////////////////////////////// convolution reverb
        void dontUseConvolutionReverbs() {
        }
        
        void setConvolutionReverbIR(std::vector<FFT_T>, int, int) {
            A(0);
        }
    };
    
    constexpr auto seconds_to_nanos = 1e9f;
    
    template<typename ConvolutionReverb>
    struct ImpulseResponseOptimizer {
        using PartitionAlgo = PartitionAlgo<ConvolutionReverb>;
        
        static constexpr bool use_spread = true;
        
        ImpulseResponseOptimizer(std::vector<FFT_T> ir, int stride, int n_audiocb_frames, int nAudioOut) :
        stride(stride),
        n_audiocb_frames(n_audiocb_frames),
        nAudioOut(nAudioOut),
        deinterlaced(stride),
        initial_size_impulse_response(0)
        {
            deinterlace(std::move(ir));
            
            truncate_irrelevant_head();
        }
        
    private:
        std::vector<cacheline_aligned_allocated::vector<FFT_T>> deinterlaced;
        int stride, n_audiocb_frames, nAudioOut, initial_size_impulse_response, final_size_impulse_response;
        
        auto size() const { return deinterlaced.empty()? 0 : deinterlaced[0].size(); }
        
        void deinterlace(std::vector<FFT_T> ir) {
            auto sz = ir.size() / stride;
            A(sz * stride == ir.size());
            for(auto & v : deinterlaced) {
                v.reserve(sz);
            }
            for(auto it = ir.begin(), end = ir.end(); it != end;) {
                for(int i=0; i<stride; ++i) {
                    deinterlaced[i].push_back(*it);
                    ++it;
                }
            }
        }
        
        void truncate_irrelevant_head() {
            using namespace std;
            
            auto start = size();
            for(auto const & v : deinterlaced) {
                constexpr auto sliding_average_size = 15;
                constexpr auto relevant_level = .1f;
                
                auto it = find_relevant_start_relaxed(v.begin(),
                                                      v.end(),
                                                      relevant_level,
                                                      sliding_average_size);
                auto dist = distance(v.begin(), it);
                LG(INFO, "dist : %d", dist);
                start = min(start, static_cast<size_t>(dist));
            }
            
            if(start < size()) {
                for(auto & v : deinterlaced) {
                    v.erase(v.begin(), v.begin() + start);
                    v.shrink_to_fit();
                }
            }
            initial_size_impulse_response = size();
        }
    public:
        void optimize_length(int max_avg_time_per_sample, PartitionningSpec & partitionning) {
            // truncate the tail if it is too long
            
            auto sz = size();
            
            optimize_reverb_parameters(use_spread, n_audiocb_frames, max_avg_time_per_sample,
                                       sz, partitionning); // modified by call
            
            if(sz < size()) {
                for(auto & v : deinterlaced) {
                    v.resize(sz);
                    v.shrink_to_fit();
                }
            }
            
            final_size_impulse_response = size();
        }
        
        auto &editDeinterlaced() {
            return deinterlaced;
        }
        
    private:
        void optimize_reverb_parameters(bool use_spread, int n_audiocb_frames, int max_avg_time_per_sample,
                                        size_t & size_impulse_response, PartitionningSpec & partitionning) const {
            using namespace std;
            
            while(1) {
                auto partit = PartitionAlgo::run(nAudioOut,
                                                 n_audiocb_frames,
                                                 size_impulse_response);
                auto & part = partit.getWithSpread(use_spread);
                
                if(part.avg_time_per_sample < max_avg_time_per_sample) {
                    partitionning = move(part);
                    break;
                }
                constexpr auto reduce_amount = .5f;
                auto amount = static_cast<int>(size_impulse_response * reduce_amount);
                if(!amount) {
                    throw logic_error("I expect the audio system to be able to process small"
                                      " impulse responses easily");
                }
                size_impulse_response -= amount;
            }
        }
    public:
        void symetrically_scale() {
            using namespace std;
            FFT_T scale = 1;
            for(auto & v : deinterlaced) {
                auto lobe = max_abs_integrated_lobe(v.begin(), v.end());
                LG(INFO, "lobe : %f", lobe);
                //auto sum = abs_integrated(v.begin(), v.end());
                //LG(INFO, "sum : %f", sum);
                constexpr auto security_lobe_factor = 3;
                scale = max(scale, security_lobe_factor*lobe);
            }
            
            scale = 1. / scale;
            
            for(auto & v : deinterlaced) {
                for(auto &s : v) {
                    s *= scale;
                }
            }
        }

        void logReport(PartitionningSpec const  & partitionning) const {
            using namespace std;
            cout << endl;
            cout << "finished optimizing partition size for " << n_audiocb_frames << " cb frames" << endl;
            if(final_size_impulse_response != initial_size_impulse_response) {
                cout << "reduced impulse response length from " << initial_size_impulse_response << " to " << final_size_impulse_response << endl;
            }
            else {
                cout << "full impulse response is used (size " << final_size_impulse_response << " )" << endl;
            }
            
            cout << "using partition size " << partitionning.size << " with" << (use_spread ? "" : "out") << " spread on " << nAudioOut << " channel(s)." << endl;
        }
        
    };
    
    
    template<typename T, int nAudioOut>
    struct AudioPolicyImpl<T, nAudioOut, AudioOutPolicy::Master> {
        
        //using ConvolutionReverb = FIRFilter<T>;
        //using ConvolutionReverb = FFTConvolution<FFT_T>;
        
        using ConvolutionReverb = PartitionnedFFTConvolution<T>;
        
        AudioPolicyImpl()
#if WITH_DELAY
        : delays{{1000, 0.6f},{4000, 0.2f}, {4300, 0.3f}, {5000, 0.1f}},
#endif
        {}
        
        /////////////////////////////// lock
        static constexpr auto useLock = WithLock::Yes;
        using Locking = LockIf<useLock>;
        
        std::atomic_bool & lock() { return used; }
        
        /////////////////////////////// postprocess
        using postProcessFunc = std::function<void(float*)>;
        
        void postprocess(T*buffer, int nFrames) {
            
            // apply convolution reverbs
            if(nAudioOut && !conv_reverbs[0].empty()) {
                for(int i=0; i<nFrames; ++i) {
                    for(int j=0; j<nAudioOut; ++j) {
                        auto & conv_reverb = conv_reverbs[j];
                        auto & sample = buffer[i*nAudioOut + j];
                        conv_reverb.step(sample);
                        sample = conv_reverb.get();
                    }
                }
            }
            
            // run delays before hardlimiting...
#if WITH_DELAY
            for( auto & delay : delays ) {
                // todo low pass filter for more realism
                delay.step(outputBuffer, nFrames);
            }
#endif
            
            for(int i=0; i<nFrames; ++i) {
                for(auto const & f: post_process) {
                    f(&buffer[i*nAudioOut]); // or call the lambda for the whole buffer at once?
                }
            }
        }
        
        ///////////////////////////////// convolution reverb
        
        void dontUseConvolutionReverbs() {
            ready = true;
        }
        
        static constexpr auto ratio_hard_limit = 1.0f;
        //    because of overhead due to os managing audio, because of "other things running on the device", etc...
        // at 0.38f on ios release we have glitches when putting the app in the background
        static constexpr auto ratio_soft_limit = 0.3f * ratio_hard_limit;
        
        static constexpr auto theoretical_max_avg_time_per_sample =
        seconds_to_nanos /
        (nAudioOut * static_cast<float>(SAMPLE_RATE));
        
        void setConvolutionReverbIR(std::vector<FFT_T> ir, int stride, int n_audiocb_frames) {
            using namespace std;
            
            ImpulseResponseOptimizer<ConvolutionReverb> algo(std::move(ir),
                                                             stride,
                                                             n_audiocb_frames,
                                                             nAudioOut);
            
            assert(nAudioOut == conv_reverbs.size());
            {
                // having the audio thread compute reverbs at the same time would make our calibration not very reliable
                // (due to cache effects for roots and possibly other) so we disable them now
                // (when ready is false, the audio thread doesn't use reverbs)
                // TODO rename ready !!
                Locking l(lock());
                ready = false;
            }
            
            PartitionningSpec partitionning;
            
            
            algo.optimize_length(theoretical_max_avg_time_per_sample * ratio_soft_limit,
                                 partitionning);
            algo.symetrically_scale();
            
            // locking here would possibly incur dropped audio frames due to the time spent setting the coefficients.
            // we ensured reverbs are not used so we don't need to lock.
            
            setCoefficients(partitionning.size,
                            std::move(algo.editDeinterlaced()), decltype(algo)::use_spread);
            
            ready = true;
            
            algo.logReport(partitionning);
            logReport(partitionning);
        }
        
        bool isReady() const {
            // no need to synchronize : aligned memory access of 4 bytes,
            // and only one thread writes it
            return ready;
        }
        
    private:
        
        /////////////////////////////// lock
        std::atomic_bool used{false};
        
        /////////////////////////////// postprocess
        bool ready = false;
        std::vector<postProcessFunc> post_process = {{ [](float v[nAudioOut]) {
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
        }}};
        
#if WITH_DELAY
        std::vector< DelayLine > delays;
#endif
        
        //////////////////////////////// convolution reverb
        using Reverbs = std::array<ConvolutionReverb, nAudioOut>;
        Reverbs conv_reverbs;
        
        void setCoefficients(int partition_size,
                             std::vector<cacheline_aligned_allocated::vector<FFT_T>> deinterlaced_coeffs,
                             bool use_spread) {
            
            // debugging
            
            /*
             for(auto const & v : deinterlaced_coeffs) {
             StringPlot plot(40, 100);
             plot.draw(v);
             plot.log();
             }
             
             {
             using namespace audio;
             write_wav("/Users/Olivier/Dev/Audiofiles", "deinterlaced.wav", deinterlaced_coeffs);
             }
             */
            
            
            int i=0;
            auto n = 0;
            auto stride = deinterlaced_coeffs.size();
            for(auto & rev : conv_reverbs)
            {
                rev.set_partition_size(partition_size);
                {
                    auto & coeffs = deinterlaced_coeffs[i];
                    if(n < static_cast<int>(conv_reverbs.size()) - static_cast<int>(deinterlaced_coeffs.size())) {
                        rev.setCoefficients(coeffs);
                    }
                    else {
                        rev.setCoefficients(move(coeffs));
                    }
                }
                if(use_spread) {
                    // to "dispatch" or "spread" the computations of each channel's convolution reverbs
                    // on different audio callback calls, we separate them as much as possible using a phase:
                    auto phase = ( partition_size * n ) / nAudioOut;
                    for(int j=0; j<phase; ++j) {
                        rev.step(0);
                    }
                }
                ++i;
                ++n;
                if(i == stride) {
                    i = 0;
                }
            }
        }
        
        void logReport(PartitionningSpec & partitionning) {
            using namespace std;
            cout << "[per sample, with 0-overhead hypothesis] allowed computation time : "
            << theoretical_max_avg_time_per_sample
            << " ns" << endl;
            
            cout << "[avg per sample] reverb time computation : "
            << partitionning.avg_time_per_sample
            << " ns" << endl;
            
            auto ratio = partitionning.avg_time_per_sample / static_cast<float>(theoretical_max_avg_time_per_sample);
            
            cout << "ratio : " << ratio;
            static_assert(ratio_soft_limit < ratio_hard_limit, "");
            cout << " which will";
            if(ratio >= ratio_soft_limit) {
                if(ratio < ratio_hard_limit) {
                    cout << " probably";
                }
            }
            else {
                cout << " likely not";
            }
            cout << " generate audio dropouts." << endl;
            cout << "gradient descent report :" << endl;
            partitionning.gd.debug(true);
        }
    };
    
    template<
    int nAudioOut,
    XfadePolicy XF = XfadePolicy::UseXfade,
    AudioOutPolicy policy = AudioOutPolicy::Master
    >
    struct outputDataBase {
        using T = float;
        
        using Channel = Channel<nAudioOut, XF>;
        using Request = typename Channel::Request;
        using Volumes = typename Channel::Volumes;
        
        static constexpr auto Policy = policy;
        static constexpr auto DefLock = (policy==AudioOutPolicy::Slave) ? WithLock::No : WithLock::Yes;
        using Impl = AudioPolicyImpl<T, nAudioOut, policy>;
        using Locking = LockIf<Impl::useLock>;
        
        static constexpr auto nOuts = nAudioOut;
        
        int count_consummed_frames() const { return consummed_frames; }
        
        using OrchestratorFunc = std::function<bool(int)>;
        
        void add_orchestrator(OrchestratorFunc f) {
            A(orchestrators.capacity() > orchestrators.size()); // we are in the audio thread, we shouldn't allocate dynamically
            orchestrators.push_back(std::move(f));
        }
        
    private:
        Impl impl;
        
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
        
        // orchestrators and computes could be owned by the channels but it is maybe cache-wise more efficient
        // to group them here (if the lambda owned is small enough to not require dynamic allocation)
        std::vector<OrchestratorFunc> orchestrators;
        std::vector<audioelement::ComputeFunc> computes;
        
    public:
        template<typename F>
        void registerCompute(F f) {
            A(computes.capacity() > computes.size()); // we are in the audio thread, we shouldn't allocate dynamically
            computes.push_back(std::move(f));
            if(isInbetweenTwoComputes()) {
                computes.back()(clock_);
            }
        }
        
        decltype(std::declval<Impl>().lock()) get_lock() { return impl.lock(); }
        
        bool isInbetweenTwoComputes() const {
            A(consummed_frames >= 0);
            A(consummed_frames < audioelement::n_frames_per_buffer);
            return 0 != consummed_frames;
        }
        
        outputDataBase(int nChannelsMax = std::numeric_limits<uint8_t>::max(), int nOrchestratorsMaxPerChannel=0)
        :
        clock_(false),
        consummed_frames(0)
        {
            A(nChannelsMax >= 0);
            A(nChannelsMax <= std::numeric_limits<uint8_t>::max()); // else need to update AUDIO_CHANNEL_NONE
            
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
        
        void dontUseConvolutionReverbs() {
            impl.dontUseConvolutionReverbs();
        }
        
        void setConvolutionReverbIR(std::vector<FFT_T> impulse_response, int stride, int n_audio_cb_frames) {
            impl.setConvolutionReverbIR(std::move(impulse_response), stride, n_audio_cb_frames);
        }
        
        // called from audio callback
        void step(SAMPLE *outputBuffer, int nFrames) {
            Locking l(get_lock());
            
            if(unlikely(!impl.isReady())) {
                // impl is being initialized in another thread
                memset(outputBuffer, 0, nFrames * nAudioOut * sizeof(SAMPLE));
                return;
            }
            
            if(consummed_frames != 0) {
                // finish consuming previous buffers
                if(!consume_buffers(outputBuffer, nFrames)) {
                    return;
                }
            }
            
            while(true) {
                // the previous buffers are consumed, we need to compute them again
                run_computes();
                
                if(!consume_buffers(outputBuffer, nFrames)) {
                    return;
                }
            }
        }
        
        Channel & editChannel(uint8_t id) { return channels[id]; }
        
        Channel const & getChannel(uint8_t id) const { return channels[id]; }
        
        bool empty() const { return channels.empty(); }
        
        void toVolume(uint8_t channel_id, float volume, int nSteps) {
            Locking l(get_lock());
            editChannel(channel_id).toVolume(volume, nSteps);
        }
        
        template<class... Args>
        bool playGeneric( uint8_t channel_id, Args&&... requests) {
            
            // it's important to register and enqueue in the same lock cycle
            // else we miss some audio frames,
            // or the callback gets unscheduled
            
            Locking l(get_lock());
            
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
                    this->registerCompute(std::move(f));
                }
            });
            
            return res;
        }
        
        void play( uint8_t channel_id, StackVector<Request> && v) {
            Locking l(get_lock());
            playNolock(channel_id, std::move(v));
        }
        
        void closeAllChannels(int xfade) {
            Locking l(get_lock());
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
                        Locking l(get_lock());
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
            Locking l(get_lock());
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
        
        void run_computes() {
            A(consummed_frames == 0); // else we skip some unconsummed frames
            clock_ = !clock_; // keep that BEFORE passing clock_ to compute functions (dependency on registerCompute)
            
            
            for(auto it = orchestrators.begin(), end = orchestrators.end(); it!=end;) {
                if(!((*it)(audioelement::n_frames_per_buffer))) { // can grow 'computes' vector
                    it = orchestrators.erase(it);
                    end = orchestrators.end();
                }
                else {
                    ++it;
                }
            }
            
            for(auto it = computes.begin(), end = computes.end(); it!=end;) {
                if(!((*it)(clock_))) {
                    it = computes.erase(it);
                    end = computes.end();
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
            
            impl.postprocess(outputBuffer, nFrames);
        }
    };
    
    using outputData = outputDataBase<2>;
}
