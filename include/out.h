#define WITH_DELAY 0

namespace imajuscule {

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

    enum class ThreadType {
        RealTime    // we are in a real-time thread, we don't need to raise the priority before locking
      , NonRealTime // we are in a non-realtime thread, to avoid priority inversion,
                    // we will raise the priority to realtime before locking and restore it after.
    };

    struct NoOpLock {
        NoOpLock(bool) noexcept {}
    };

  template<ThreadType>
  struct AudioLockCtrl;

  template<>
  struct AudioLockCtrl<ThreadType::NonRealTime> {
    AudioLockCtrl(std::atomic_flag & l) noexcept : spin(l) {}

    void lock() {
      priority.lock();
      spin.lock();
    }
    void unlock() {
      spin.unlock();
      priority.unlock();
    }

  private:
    thread::CtrlRTPriority priority;
    LockCtrl spin;
  };

  template<>
  struct AudioLockCtrl<ThreadType::RealTime> {
    AudioLockCtrl(std::atomic_flag & l) noexcept : spin(l) {}

    void lock() {
      spin.lock();
    }
    void unlock() {
      spin.unlock();
    }

  private:
    LockCtrl spin;
  };

  struct NoOpLockCtrl {
    NoOpLockCtrl(bool) {}

    void lock() {
    }
    void unlock() {
    }
  };

  template<ThreadType T>
  struct AudioLock {
    AudioLock( std::atomic_flag & l) noexcept : ctrl(l) {
      ctrl.lock();
    }
    ~AudioLock() {
      ctrl.unlock();
    }

  private:
    AudioLockCtrl<T> ctrl;
  };

  template<WithLock, ThreadType>
  struct LockIf_;

  template <ThreadType T>
  struct LockIf_<WithLock::Yes, T> {
    using type = AudioLock<T>;
    using ctrlType = AudioLockCtrl<T>;
  };
  template <ThreadType T>
  struct LockIf_<WithLock::No, T> {
    using type = NoOpLock;
    using ctrlType = NoOpLockCtrl;
  };

  template<WithLock l, ThreadType T>
  using LockIf = typename LockIf_<l,T>::type;

  template<WithLock l, ThreadType T>
  using LockCtrlIf = typename LockIf_<l,T>::ctrlType;

    enum class ChannelClosingPolicy {
        AutoClose,  // once the request queue is empty (or more precisely
        // once the channel method isPlaying() returns false),
        // the channel can be automatically reassigned without
        // the need to close it explicitely.
        // Explicitely closing an AutoClose channel will result in undefined behaviour.
        ExplicitClose, // the channel cannot be reassigned unless explicitely closed
    };

    enum class AudioOutPolicy {
        // no synchronization is needed.
        Slave,
        // synchronization is done using a global lock
        MasterGlobalLock,
        // synchronization is done using lock-free datastructures
        MasterLockFree
    };

    template<AudioOutPolicy> struct AudioLockPolicyImpl;

    template <>
    struct AudioLockPolicyImpl<AudioOutPolicy::Slave> {
      static constexpr auto sync = Synchronization::SingleThread;
      static constexpr auto useLock = WithLock::No;

      bool lock() { return false; }
    };

    template <>
    struct AudioLockPolicyImpl<AudioOutPolicy::MasterGlobalLock> {
      static constexpr auto sync = Synchronization::SingleThread;
      static constexpr auto useLock = WithLock::Yes;

      std::atomic_flag & lock() { return used; }
    private:
      std::atomic_flag used = ATOMIC_FLAG_INIT;
    };

    template <>
    struct AudioLockPolicyImpl<AudioOutPolicy::MasterLockFree> {
      static constexpr auto sync = Synchronization::Lockfree_SingleConsumerMultipleProducer;
      static constexpr auto useLock = WithLock::No;

      bool lock() { return false; }
    };

  template<AudioOutPolicy p>
  constexpr auto getAtomicity() {
    using OutTraits = AudioLockPolicyImpl<p>;
    return (OutTraits::sync == Synchronization::Lockfree_SingleConsumerMultipleProducer) ?
    Atomicity::Yes :
    Atomicity::No;
  }

  /*
   Returns true if a non-realtime thread should use the one shot queue,
   false if it should run the lambda in its own thread.
   */
  template<AudioOutPolicy p>
  constexpr bool shouldNRTThreadUseOneshotsQueue() {
    using OutTraits = AudioLockPolicyImpl<p>;
    return OutTraits::sync == Synchronization::Lockfree_SingleConsumerMultipleProducer;
  }

    // cooley-tukey leads to error growths of O(log n) (worst case) and O(sqrt(log n)) (mean for random input)
    // so float is good enough
    using FFT_T = float;

    constexpr auto seconds_to_nanos = 1e9f;

    template<typename ConvolutionReverb>
    struct ImpulseResponseOptimizer {
        using PartitionAlgo = PartitionAlgo<ConvolutionReverb>;
        using PartitionningSpec = typename PartitionAlgo::PartitionningSpec;

        static constexpr bool use_spread = true;

        ImpulseResponseOptimizer(std::vector<FFT_T> ir, int n_channels, int n_audiocb_frames, int nAudioOut) :
        n_channels(n_channels),
        n_audiocb_frames(n_audiocb_frames),
        nAudioOut(nAudioOut),
        deinterlaced(n_channels),
        initial_size_impulse_response(0)
        {
            deinterlace(std::move(ir));

            truncate_irrelevant_head();
        }

    private:
        std::vector<a64::vector<FFT_T>> deinterlaced;
        int n_channels, n_audiocb_frames, nAudioOut, initial_size_impulse_response, final_size_impulse_response;

        auto size() const { return deinterlaced.empty()? 0 : deinterlaced[0].size(); }

        void deinterlace(std::vector<FFT_T> ir) {
            auto sz = ir.size() / n_channels;
            Assert(sz * n_channels == ir.size());
            for(auto & v : deinterlaced) {
                v.reserve(sz);
            }
            for(auto it = ir.begin(), end = ir.end(); it != end;) {
                for(int i=0; i<n_channels; ++i) {
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
        void optimize_length(int n_channels, int max_avg_time_per_sample, PartitionningSpec & partitionning) {
            // truncate the tail if it is too long

            auto sz = size();

            optimize_reverb_parameters(n_channels, use_spread, n_audiocb_frames, max_avg_time_per_sample,
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
        void optimize_reverb_parameters(int n_channels, bool use_spread, int n_audiocb_frames, int max_avg_time_per_sample,
                                        size_t & size_impulse_response, PartitionningSpec & partitionning) const {
            using namespace std;

            while(1) {
                auto partit = PartitionAlgo::run(n_channels,
                                                 n_audiocb_frames,
                                                 size_impulse_response);
                auto & part = partit.getWithSpread(use_spread);

                if(part.getCost() < max_avg_time_per_sample) {
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
                //auto res = avg_windowed_abs_integrated(v.begin(), v.end(), 20, [](auto r){ return 1.f; });
                //auto res = max_auto_corr(v);

                //scale = max(scale, res);

                /*auto res = max_freq_amplitude(v.begin(), v.end());

                LG(INFO,
                   "max bin: freq %f amplitude %f",
                   res.relative_freq * SAMPLE_RATE,
                   res.amplitude);
                constexpr auto security_factor = 3;
                scale = max(scale, security_factor * res.amplitude);*/
                auto lobe = max_abs_integrated_lobe(v.begin(), v.end());
                LG(INFO, "lobe : %f", lobe);
                //auto sum = abs_integrated(v.begin(), v.end());
                //LG(INFO, "sum : %f", sum);
                constexpr auto security_lobe_factor = 10; // 3,5 not enough for long reverbs
                scale = max(scale, security_lobe_factor*lobe);
            }

            scale = 1. / scale;
            LG(INFO, "impulse response volume scaled by : %f", scale);
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
                cout << "full impulse response is used (size " << final_size_impulse_response << ")" << endl;
            }

            cout << "using partition size " << partitionning.size << " with" << (use_spread ? "" : "out") << " spread on " << n_channels << " channel(s)." << endl;
        }

    };


    template<typename T, int nAudioOut, AudioOutPolicy policy>
    struct AudioPostPolicyImpl {
        static constexpr auto nOut = nAudioOut;
        using LockPolicy = AudioLockPolicyImpl<policy>;

    // We disable postprocessing for audio plugins (i.e 'AudioOutPolicy::Slave')
    static constexpr bool disable = policy == AudioOutPolicy::Slave;

      using LockFromRT = LockIf<LockPolicy::useLock, ThreadType::RealTime>;
      using LockFromNRT = LockIf<LockPolicy::useLock, ThreadType::NonRealTime>;

        //using ConvolutionReverb = FIRFilter<T>;
        //using ConvolutionReverb = FFTConvolution<FFT_T>;
        //using ConvolutionReverb = PartitionnedFFTConvolution<T>;
        using ConvolutionReverb = FinegrainedPartitionnedFFTConvolution<T>;
        using Spatializer = audio::Spatializer<nAudioOut, ConvolutionReverb>;

        using SetupParam = typename ConvolutionReverb::SetupParam;
        using PartitionningSpec = PartitionningSpec<SetupParam>;

        AudioPostPolicyImpl(LockPolicy &l) :
        _lock(l)
#if WITH_DELAY
        , delays{{1000, 0.6f},{4000, 0.2f}, {4300, 0.3f}, {5000, 0.1f}},
#endif
        {}

        LockPolicy & _lock;

        /////////////////////////////// postprocess
        using postProcessFunc = std::function<void(float*)>;

        void postprocess(T*buffer, int nFrames) {
          if(disable) {
            return;
          }

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

            if(!spatializer.empty()) {
                for(int i=0; i<nFrames; ++i) {
                    auto & samples = buffer[i*nAudioOut];
                    spatializer.step(&samples);
                    spatializer.get(&samples);
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

        void dontUseConvolutionReverbs()
        {
          if constexpr (disable) {
            return;
          }

          muteAudio();

          for(auto & r : conv_reverbs) {
              r.clear();
          }
          spatializer.clear();

          unmuteAudio();
        }

        static constexpr auto ratio_hard_limit = 1.0f;
        //    because of overhead due to os managing audio, because of "other things running on the device", etc...
        // at 0.38f on ios release we have glitches when putting the app in the background
        static constexpr auto ratio_soft_limit = 0.3f * ratio_hard_limit;

        static constexpr auto theoretical_max_avg_time_per_frame = seconds_to_nanos / static_cast<float>(SAMPLE_RATE);

        void setConvolutionReverbIR(std::vector<FFT_T> ir, int n_channels, int n_audiocb_frames)
        {
          if constexpr (disable) {
            Assert(0);
            return;
          }
            using namespace std;

            ImpulseResponseOptimizer<ConvolutionReverb> algo(move(ir),
                                                             n_channels,
                                                             n_audiocb_frames,
                                                             nAudioOut);

            assert(nAudioOut == conv_reverbs.size());

            // having the audio thread compute reverbs at the same time would make our calibration not very reliable
            // (due to cache effects for roots and possibly other) so we disable them now
            // (when ready is false, the audio thread doesn't use reverbs)
            muteAudio();

            PartitionningSpec partitionning;

            algo.optimize_length(n_channels, theoretical_max_avg_time_per_frame * ratio_soft_limit / static_cast<float>(n_channels),
                                 partitionning);
            algo.symetrically_scale();

            // locking here would possibly incur dropped audio frames due to the time spent setting the coefficients.
            // we ensured reverbs are not used so we don't need to lock.

            setCoefficients(partitionning,
                            move(algo.editDeinterlaced()), decltype(algo)::use_spread);

            unmuteAudio();

            algo.logReport(partitionning);
            logReport(n_channels, partitionning);
        }

        bool isReady() const
        {
          if constexpr (disable) {
            return true;
          }
          return readyTraits::read(ready, std::memory_order_relaxed);
        }


        bool hasSpatializer() const { return !spatializer.empty(); }

    private:

        /////////////////////////////// postprocess
        using readyTraits = maybeAtomic<getAtomicity<policy>(),unsigned int>;
        using readyType = typename readyTraits::type;

        readyType ready = false;
        std::vector<postProcessFunc> post_process = {{ [](float v[nAudioOut]) {
            for(int i=0; i<nAudioOut; ++i) {
                if(likely(-1.f <= v[i] && v[i] <= 1.f)) {
                    continue;
                }

                if(v[i] > 1.f) {
                    Assert(0);
                    v[i] = 1.f;
                }
                else if(v[i] < -1.f) {
                    Assert(0);
                    v[i] = -1.f;
                }
                else {
                    Assert(0);
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

        Spatializer spatializer;

        void setCoefficients(PartitionningSpec const & spec,
                             std::vector<a64::vector<FFT_T>> deinterlaced_coeffs,
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

            spatializer.clear();
            for(auto & r : conv_reverbs) {
                r.clear();
            }

            auto const n_channels = deinterlaced_coeffs.size();
            auto nSources = n_channels / nAudioOut;
            if(nSources <= 1) {
                int i=0;
                auto n = 0;
                for(auto & rev : conv_reverbs)
                {
                    rev.set_partition_size(spec.size);
                    {
                        auto & coeffs = deinterlaced_coeffs[i];
                        if(n < static_cast<int>(conv_reverbs.size()) - static_cast<int>(deinterlaced_coeffs.size())) {
                            rev.setCoefficients(coeffs);
                        }
                        else {
                            rev.setCoefficients(move(coeffs));
                        }
                    }
                    rev.applySetup(spec.cost);
                    if(use_spread) {
                        // to "dispatch" or "spread" the computations of each channel's convolution reverbs
                        // on different audio callback calls, we separate them as much as possible using a phase:
                        auto phase = n * spec.cost.phase;
                        for(int j=0; j<phase; ++j) {
                            rev.step(0);
                        }
                    }
                    ++i;
                    ++n;
                    if(i == n_channels) {
                        i = 0;
                    }
                }
            }
            else {
                if(nSources * nAudioOut != n_channels) {
                    throw std::logic_error("wrong number of channels");
                }

                assert(spatializer.empty());
                spatializer.set_partition_size(spec.size);
                assert(spatializer.empty());
                for(int i=0; i<nSources; ++i) {
                    std::array<a64::vector<T>, nAudioOut> a;
                    for(int j=0; j<nAudioOut; ++j) {
                        // for wir files of wave, it seems the order is by "ears" then by "source":

                        // ear Left source 1
                        // ...
                        // ear Left source N
                        // ear Right source 1
                        // ...
                        // ear Right source N

                        a[j] = std::move(deinterlaced_coeffs[i+nAudioOut*j]);
                    }
                    spatializer.addSourceLocation(std::move(a));
                    assert(!spatializer.empty());
                }
                spatializer.applySetup(spec.cost);
                assert(!spatializer.empty());
                if(use_spread) {
                    spatializer.dephaseComputations(spec.cost.phase);
                }
            }
        }

        void logReport(int n_channels, PartitionningSpec & partitionning) {
            using namespace std;
            cout << "[per sample, with 0-overhead hypothesis] allowed computation time : "
            << theoretical_max_avg_time_per_frame / static_cast<float>(n_channels)
            << " ns" << endl;

            cout << "[avg per sample] reverb time computation : "
            << partitionning.getCost()
            << " ns" << endl;

            auto ratio = partitionning.getCost() * static_cast<float>(n_channels) / static_cast<float>(theoretical_max_avg_time_per_frame);

            cout << "ratio : " << ratio;
            static_assert(ratio_soft_limit < ratio_hard_limit);
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
            auto index = 1;
            for(auto const & r : conv_reverbs)
            {
                if(r.empty()) {
                    continue;
                }
                cout << " reverb " << index << " :" << endl;
                {
                    auto lat = r.getLatency();
                    cout
                    << "  latency : " << lat << " frames ("
                    << lat * 1000.f / SAMPLE_RATE <<  " ms)" << endl;
                }

                {
                    auto per = r.getGranularMinPeriod();
                    cout
                    << "  grain compute period : " << per << " frames ("
                    << per * 1000.f / SAMPLE_RATE <<  " ms)" << endl;
                }
                ++index;
            }

            if(!spatializer.empty()) {
                cout <<  "spatializer with '" << spatializer.countSources() << "' sources" << endl;
            }

            cout << partitionning.cost << endl;
            constexpr auto debug_gradient_descent = false;
            if(debug_gradient_descent) {
                cout << "gradient descent report :" << endl;
                partitionning.gd.debug(true);
            }
        }

        void muteAudio() {
          LockFromNRT l(_lock.lock());
          readyTraits::write(ready, false, std::memory_order_relaxed);
          if constexpr (policy == AudioOutPolicy::MasterLockFree) {
            // To avoid modifying postprocessing data while postprocessing is used,
            // we sleep a duration corresponding to twice the size of a callback buffer,
            // to be sure that any audio cb call in progress when we set 'ready' to false
            // will be finished when we wake up (and subsequent cb calls will see the updated
            // 'ready' value)
            int n = audio::wait_for_first_n_audio_cb_frames();
            float millisPerBuffer = ceil(1000 * n/static_cast<float>(SAMPLE_RATE));
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5f + 2.f * millisPerBuffer)));
          }
        }

        void unmuteAudio() {
          readyTraits::write(ready, true, std::memory_order_relaxed);
        }

    };

    template< typename ChannelsType >
    struct outputDataBase {
        using T = SAMPLE;

        static constexpr auto policy = ChannelsType::policy;
        static constexpr auto nOuts = ChannelsType::nAudioOut;
        using ChannelsT = ChannelsType;
        using Request = typename ChannelsType::Request;
        using PostImpl = AudioPostPolicyImpl<T, nOuts, policy>;
        using LockFromRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::RealTime>;
      using LockFromNRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;
      using LockCtrlFromNRT = LockCtrlIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;

    private:
        //////////////////////////
        /// state of last write:
        bool clock_ : 1; /// "tic tac" flag, inverted at each new AudioElements buffer writes
        ///
        //////////////////////////

        AudioLockPolicyImpl<policy> & _lock;
        ChannelsT channelsT;
        PostImpl post;

    public:

        template<typename ...Args>
        outputDataBase(AudioLockPolicyImpl<policy>&l, Args ... args):
        channelsT(l, args ...)
        , post(l)
        , _lock(l)
        , clock_(false)
        {}

        outputDataBase(AudioLockPolicyImpl<policy>&l):
          post(l)
        , _lock(l)
        , clock_(false)
        {}

        ChannelsT & getChannels() { return channelsT; }
        ChannelsT const & getConstChannels() const { return channelsT; }

        PostImpl & getPost() { return post; }

        AudioLockPolicyImpl<policy> & get_lock_policy() { return _lock; }
        decltype(std::declval<AudioLockPolicyImpl<policy>>().lock()) get_lock() { return _lock.lock(); }

        // called from audio callback
        void step(SAMPLE *outputBuffer, int nFrames) {
            /*
            static bool first(true);
            if(first) {
                first = false;
                std::cout << "audio thread: " << std::endl;
                thread::logSchedParams();
            }*/

            // To avoid priority inversion, we make sure that other threads
            // taking this lock have their priority raised to realtime before taking the lock.
            LockFromRT l(_lock.lock());
            // TODO locking in the realtime thread should be avoided.

            if(unlikely(!post.isReady())) {
                // post is being initialized in another thread
                memset(outputBuffer, 0, nFrames * nOuts * sizeof(SAMPLE));
                return;
            }

            int start = 0;

            while(nFrames > 0) {
                clock_ = !clock_; // keep that BEFORE passing clock_ to compute functions (dependency on registerCompute)

                // how many frames are we going to compute?
                auto nLocalFrames = std::min(nFrames, audioelement::n_frames_per_buffer);

                channelsT.run_computes(clock_, nLocalFrames);

                consume_buffers(&outputBuffer[start], nLocalFrames);
                start += nLocalFrames * nOuts;
                nFrames -= nLocalFrames;
            }
        }

        bool getTicTac() const { return clock_;}

    private:

        void consume_buffers(SAMPLE * outputBuffer, int nFrames) {
            Assert(nFrames <= audioelement::n_frames_per_buffer); // by design

            memset(outputBuffer, 0, nFrames * nOuts * sizeof(SAMPLE));

            channelsT.forEach([outputBuffer, this, nFrames](auto & c) {
                c.step(outputBuffer, nFrames);
                if(c.shouldReset()) {
                    c.reset();
                }
            });

            post.postprocess(outputBuffer, nFrames);
        }
    };

    template<typename OutputData>
    void dontUseConvolutionReverbs(OutputData & data) {
        data.getPost().dontUseConvolutionReverbs();
    }

}
