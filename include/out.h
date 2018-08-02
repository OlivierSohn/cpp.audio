#define WITH_DELAY 0

namespace imajuscule {

    template<int nAudioOut>
    struct DelayLine {
        DelayLine(int size, double attenuation): delay(size,{{}}), it(0), end(size), attenuation(attenuation) {}

        void step(double *outputBuffer, int nFrames) {
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

        std::vector<std::array<double, nAudioOut>> delay;
        int32_t it, end;
        double attenuation;
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

    constexpr auto seconds_to_nanos = 1e9;

    template<typename ConvolutionReverb>
    struct ImpulseResponseOptimizer {
        using PartitionAlgo = PartitionAlgo<ConvolutionReverb>;
        using PS = typename PartitionAlgo::PS;

        static constexpr bool use_spread = true;

        ImpulseResponseOptimizer(std::vector<double> ir, int n_channels, int n_audiocb_frames, int nAudioOut) :
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
        std::vector<a64::vector<double>> deinterlaced;
        int n_channels, n_audiocb_frames, nAudioOut, initial_size_impulse_response, final_size_impulse_response;

        auto size() const { return deinterlaced.empty()? 0 : deinterlaced[0].size(); }

        void deinterlace(std::vector<double> ir) {
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
                start = min(start, static_cast<size_t>(dist));
            }

            if(start < size()) {
                LG(INFO, "truncating %d first sample(s)", start);
                for(auto & v : deinterlaced) {
                    v.erase(v.begin(), v.begin() + start);
                    v.shrink_to_fit();
                }
            }
            else {
                LG(INFO, "no truncation");
            }
            initial_size_impulse_response = size();
        }
    public:
        void optimize_length(int n_channels, int max_avg_time_per_sample, PS & partitionning) {
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
                                        size_t & size_impulse_response, PS & partitionning) const {
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
                LG(WARN,
                  "cost %f >= %d. impulse response too big : %d (reducing by %d)",
                  part.getCost(), max_avg_time_per_sample, size_impulse_response, amount);

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
            double scale = 1.;
            for(auto & v : deinterlaced) {
                auto lobe = max_abs_integrated_lobe(v.begin(), v.end());
                LG(INFO, "lobe : %f", lobe);
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

        void logReport(PS const  & partitionning) const {
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

    /*
    Not thread safe.

    Allows to control the variation of a value, to avoid brusk changes.
    */
    template<typename T>
    struct smoothVar {
      smoothVar(T v) {
        smoothly(v,0);
      }

      /*
      * Sets the target value and the count of steps of the interpolation.
      */
      void smoothly(T v, int nSteps) {
        if(nSteps == 0) {
          current = target = v;
          increment = 0;
          return;
        }
        if(unlikely(nSteps < 0)) {
          nSteps = -nSteps;
        }
        target = v;
        increment = std::abs(target - current) / static_cast<T>(nSteps);
        Assert(increment >= 0);
      }

      /*
      * Gets the current value, using linear interpolation to perform the transition.
      */
      T step() {
        Assert(increment >= 0);
        if(current == target) {
        }
        else if(current < target-increment) {
          current += increment;
        }
        else if(current > target+increment) {
          current -= increment;
        }
        else {
          current = target;
        }
        return current;
      }
    private:
      T current, target, increment;
    };

    template<int nAudioOut, AudioOutPolicy policy>
    struct AudioPostPolicyImpl {
        static constexpr auto nOut = nAudioOut;
        using LockPolicy = AudioLockPolicyImpl<policy>;

    // We disable postprocessing for audio plugins (i.e 'AudioOutPolicy::Slave')
    static constexpr bool disable = policy == AudioOutPolicy::Slave;

      using LockFromRT = LockIf<LockPolicy::useLock, ThreadType::RealTime>;
      using LockFromNRT = LockIf<LockPolicy::useLock, ThreadType::NonRealTime>;

        //using ConvolutionReverb = FIRFilter<double>;
        //using ConvolutionReverb = FFTConvolution<double>;
        //using ConvolutionReverb = PartitionnedFFTConvolution<double>;
        //using ConvolutionReverb = FinegrainedPartitionnedFFTConvolution<double>;
        using ConvolutionReverb = ZeroLatencyScaledFineGrainedPartitionnedConvolution<double>;
        using Spatializer = audio::Spatializer<nAudioOut, ConvolutionReverb>;

        using SetupParam = typename ConvolutionReverb::SetupParam;
      using PS = PartitionningSpec<typename SetupParam::BParam>;

        AudioPostPolicyImpl(LockPolicy &l) :
        _lock(l)
#if WITH_DELAY
        , delays{{1000, 0.6f},{4000, 0.2f}, {4300, 0.3f}, {5000, 0.1f}},
#endif
        {
          readyTraits::write(ready, false, std::memory_order_relaxed);
        }

        LockPolicy & _lock;

        /////////////////////////////// postprocess
        using postProcessFunc = std::function<void(double*)>;

        void postprocess(double*buffer, int nFrames) {
          if(disable) {
            return;
          }

            // apply convolution reverbs: raw convolutions and spatializer are mutually exclusive.
            if(nAudioOut && !conv_reverbs[0].empty()) {
              Assert(conv_reverbs.size() == nAudioOut);
              Assert(spatializer.empty());
                for(int i=0; i<nFrames; ++i) {
                    double const wet = wetRatio.step();
                    double const dry = 1.-wet;
#ifndef NDEBUG
                    if(wet < 0. || wet > 1.) {
                      LG(ERR, "wet %f", wet);
                      Assert(0);
                    }
                    if(dry < 0. || dry > 1.) {
                      LG(ERR, "dry %f", wet);
                      Assert(0);
                    }
#endif
                    for(int j=0; j<nAudioOut; ++j) {
                        auto & conv_reverb = conv_reverbs[j];
                        auto & sample = buffer[i*nAudioOut + j];
                        // TODO merge these 2 calls in 1
                        conv_reverb.step(sample);
                        sample = dry * sample + wet * conv_reverb.get();
                    }
                }
            }
            else if(!spatializer.empty()) {
                for(int i=0; i<nFrames; ++i) {
                    double const wet = wetRatio.step();
                    double const dry = 1.-wet;
#ifndef NDEBUG
                    if(wet < 0. || wet > 1.) {
                      LG(ERR, "wet %f", wet);
                      Assert(0);
                    }
                    if(dry < 0. || dry > 1.) {
                      LG(ERR, "dry %f", wet);
                      Assert(0);
                    }
#endif
                    auto & samples = buffer[i*nAudioOut];
                    spatializer.step(&samples);
                    spatializer.get(&samples, dry, wet);
                }
            }

            // run delays
#if WITH_DELAY
            for( auto & delay : delays ) {
                // todo low pass filter for more realism
                delay.step(outputBuffer, nFrames);
            }
#endif

            // compress / hardlimit
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

      static constexpr auto theoretical_max_avg_time_per_frame = nanos_per_frame<double>();

        void setConvolutionReverbIR(std::vector<double> ir, int n_channels, int n_audiocb_frames)
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
            muteAudio();

          PS partitionning;

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

        // Must be called from the audio realtime thread.
        void transitionConvolutionReverbWetRatio(double wet) {
          wetRatio.smoothly(clamp_ret(wet,0.,1.), ms_to_frames(200));
        }

        bool isReady() const
        {
          if constexpr (disable) {
            return true;
          }
          std::atomic_thread_fence(std::memory_order_acquire);
          return readyTraits::read(ready, std::memory_order_relaxed);
        }

        bool hasSpatializer() const { return !spatializer.empty(); }

    private:

        /////////////////////////////// postprocess
        using readyTraits = maybeAtomic<getAtomicity<policy>(),unsigned int>;
        using readyType = typename readyTraits::type;

        readyType ready;
      std::vector<postProcessFunc> post_process = {
        { [this](double v[nAudioOut]) {
          CArray<nAudioOut, double> a{v};
          compressor.feed(a);
        }},
        { [](double v[nAudioOut]) {
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
        smoothVar<double> wetRatio = {1};
        using Reverbs = std::array<ConvolutionReverb, nAudioOut>;
        Reverbs conv_reverbs;

        Spatializer spatializer;

        void setCoefficients(PS const & spec,
                             std::vector<a64::vector<double>> deinterlaced_coeffs,
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
            // if we have enough sources, we can spatialize them, i.e each ear will receive
            // the sum of multiple convolutions.
            if(nSources <= 1) {
                int i=0;
                auto n = 0;
                for(auto & rev : conv_reverbs)
                {
                    setPartitionSize(rev,spec.size);
                    applySetup(rev,SetupParam{{}, spec.cost});
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
                for(int i=0; i<nSources; ++i) {
                    std::array<a64::vector<double>, nAudioOut> a;
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
                    spatializer.addSourceLocation(std::move(a), std::pair<int,SetupParam>{spec.size, {{}, spec.cost}});
                    assert(!spatializer.empty());
                }
                assert(!spatializer.empty());
                if(use_spread) {
                    spatializer.dephaseComputations(spec.cost.phase);
                }
            }
        }

        void logReport(int n_channels, PS & partitionning) {
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
                    << frames_to_ms(lat) <<  " ms)" << endl;
                }

                {
                    auto per = r.getGranularMinPeriod();
                    cout
                    << "  grain compute period : " << per << " frames ("
                    << frames_to_ms(per) <<  " ms)" << endl;
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
          std::atomic_thread_fence(std::memory_order_release);
          if constexpr (policy == AudioOutPolicy::MasterLockFree) {
            // To avoid modifying postprocessing data while postprocessing is used,
            // we sleep a duration corresponding to twice the size of a callback buffer,
            // to be sure that any audio cb call in progress when we set 'ready' to false
            // will be finished when we wake up (and subsequent cb calls will see the updated
            // 'ready' value)
            int n = audio::wait_for_first_n_audio_cb_frames();
            float millisPerBuffer = frames_to_ms(n);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5f + 20.f * ceil(millisPerBuffer))));
          }
        }

        void unmuteAudio() {
          // we use a fence to ensure that previous non atomic writes
          // will not appear after the write of 'ready'.
          std::atomic_thread_fence(std::memory_order_release);
          readyTraits::write(ready, true, std::memory_order_relaxed);
        }

      audio::Compressor compressor;
    };

    namespace detail {
      struct Compute {
        template<typename T>
        void operator () (T & c) const {
          c.step(buf, n, t);
          if(c.shouldReset()) {
              c.reset();
          }
        }
        double * buf;
        int const n;
        uint64_t const t;
      };
    }

    template< typename ChannelsType >
    struct outputDataBase {
        using T = SAMPLE;

        static constexpr auto policy = ChannelsType::policy;
        static constexpr auto nOuts = ChannelsType::nAudioOut;
        using ChannelsT = ChannelsType;
        using Request = typename ChannelsType::Request;
        using PostImpl = AudioPostPolicyImpl<nOuts, policy>;
        using LockFromRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::RealTime>;
      using LockFromNRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;
      using LockCtrlFromNRT = LockCtrlIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;

    private:
      /*
       * The function is executed once, and then removed from the queue.
       */
      using OneShotFunc = std::function<void(outputDataBase &)>;
      // Today, oneshots are only used to change the reverb dry/wet ratio,
      // hence we need just a few.
      // We use a power of 2 minus 1 so that the size of the underlying fifo vector is a power of 2.
      // Keeping the queue small has the advantage of making the work per callback minimal:
      //   if suddenly many elements were added to the queue, maybe the audio callback would miss
      //   its deadline due to the work it has to do. Instead here, only some elements are immediately
      //   added, and remaining elements will retry.
      static constexpr auto nMaxOneshotsPerCb = 16 - 1;

        AudioLockPolicyImpl<policy> & _lock;
        ChannelsT channelsT;
        PostImpl post;
        lockfree::scmp::fifo<OneShotFunc> oneShots{nMaxOneshotsPerCb};

    public:

        template<typename ...Args>
        outputDataBase(AudioLockPolicyImpl<policy>&l, Args ... args):
        channelsT(l, args ...)
        , post(l)
        , _lock(l)
        {}

        outputDataBase(AudioLockPolicyImpl<policy>&l):
          post(l)
        , _lock(l)
        {}

        ChannelsT & getChannels() { return channelsT; }
        ChannelsT const & getConstChannels() const { return channelsT; }

        PostImpl & getPost() { return post; }

        AudioLockPolicyImpl<policy> & get_lock_policy() { return _lock; }
        decltype(std::declval<AudioLockPolicyImpl<policy>>().lock()) get_lock() { return _lock.lock(); }

    // this method should not be called from the real-time thread
    // because it yields() and retries.
    template<typename F>
    void enqueueOneShot(F f) {
      if constexpr(shouldNRTThreadUseOneshotsQueue<policy>()) {
        while(!oneShots.tryEnqueue(f)) {
          std::this_thread::yield();
        }
      }
      else {
        f(*this);
      }
    }

        // called from audio callback
        void step(SAMPLE *outputBuffer, int nFrames, uint64_t const tNanos) {
            /*
            static bool first(true);
            if(first) {
                first = false;
                std::cout << "audio thread: " << std::endl;
                thread::logSchedParams();
            }*/

            LockFromRT l(_lock.lock());

            oneShots.dequeueAll([this](auto const & f) {
              f(*this);
            });

            if(unlikely(!post.isReady())) {
                // post is being initialized in another thread
                memset(outputBuffer, 0, nFrames * nOuts * sizeof(SAMPLE));
                return;
            }

          auto t = tNanos;
          constexpr uint64_t nanos_per_iteration =
            static_cast<uint64_t>(
            0.5f + nanos_per_frame<float>() * static_cast<float>(audioelement::n_frames_per_buffer)
                                 );

          double precisionBuffer[audioelement::n_frames_per_buffer * nOuts];
            while(nFrames > 0) {
              auto const nLocalFrames = std::min(nFrames, audioelement::n_frames_per_buffer);

              channelsT.run_computes(nLocalFrames, t);

              const int nSamples = nLocalFrames * nOuts;

              memset(precisionBuffer, 0, nSamples * sizeof(double));
              consume_buffers(precisionBuffer, nLocalFrames, t);
              for(int i=0;
                  i != nSamples;
                  ++i, ++outputBuffer) {
                *outputBuffer = static_cast<SAMPLE>(precisionBuffer[i]);
              }
              nFrames -= audioelement::n_frames_per_buffer;
              t += nanos_per_iteration;
            }
        }

    private:

        void consume_buffers(double * outputBuffer, int const nFrames, uint64_t const tNanos) {
            Assert(nFrames <= audioelement::n_frames_per_buffer); // by design

            channelsT.forEach(detail::Compute{outputBuffer, nFrames, tNanos});
            post.postprocess(outputBuffer, nFrames);
        }
    };

    template<typename OutputData>
    void dontUseConvolutionReverbs(OutputData & data) {
        data.getPost().dontUseConvolutionReverbs();
    }

}
