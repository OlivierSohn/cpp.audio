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

  /**
   * Performs data transpositions
   */
  template<typename T, int nAudioIn, int nAudioOut, int nFramesMax>
  struct Conversion {
      Conversion()
      : ins(nAudioIn * nFramesMax)
      , outs(nAudioOut * nFramesMax)
      , inputBuffers(nAudioIn)
      , outputBuffers(nAudioOut) {
          for (int in = 0; in < nAudioIn; ++in) {
              inputBuffers[in] = &ins[in * nFramesMax];
          }
          for (int out = 0; out < nAudioOut; ++out) {
              outputBuffers[out] = &outs[out * nFramesMax];
          }
      }

      // data   = frame1 frame2 ...
      // frameN = in1 in2 ...
      T ** transposeInput(T const * data, int const nFrames) {
          for (int i=0; i<nFrames; ++i) {
              for (int in = 0; in < nAudioIn; ++in) {
                  ins[in * nFramesMax + i] = data[i*nAudioIn + in];
              }
          }
          return inputBuffers.data();
      }

      T ** editOutput() { return outputBuffers.data(); }

      // data   = frame1 frame2 ...
      // frameN = out1 out2 ...
      void transposeOutput(T * data, int const nFrames) {
          for (int i=0; i<nFrames; ++i) {
              for (int out = 0; out < nAudioOut; ++out) {
                  data[i*nAudioOut + out] = outs[out * nFramesMax + i];
              }
          }
      }

  private:
      // ins = "frames of input1" "frames of input2" ...
      // outs = "frames of output1" "frames of output2" ...
      std::vector<T> ins, outs;
      std::vector<T*> inputBuffers;
      std::vector<T*> outputBuffers;
  };

  template<int nAudioOut, audio::ReverbType ReverbT, AudioOutPolicy policy>
  struct AudioPostPolicyImpl {
    static constexpr auto nOut = nAudioOut;
    static constexpr auto nAudioIn = nAudioOut;
    using LockPolicy = AudioLockPolicyImpl<policy>;

    // We disable postprocessing for audio plugins (i.e 'AudioOutPolicy::Slave')
    static constexpr bool disable = policy == AudioOutPolicy::Slave;

    using LockFromRT = LockIf<LockPolicy::useLock, ThreadType::RealTime>;
    using LockFromNRT = LockIf<LockPolicy::useLock, ThreadType::NonRealTime>;

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

    void declareBlockSize(int sz) {
        reverbs.declareBlockSize(sz);
    }

    void postprocess(double*buffer, int nFrames) {
      if(disable) {
        return;
      }

#if WITH_DELAY
      for( auto & delay : delays ) {
        // todo low pass filter for more realism
        delay.step(outputBuffer, nFrames);
      }
#endif

      if (reverbs.isActive()) {
        Assert(nFrames <= audio::audioelement::n_frames_per_buffer);
        auto ** ins = conversion.transposeInput(buffer, nFrames);
        reverbs.apply(ins, nAudioIn, conversion.editOutput(), nAudioOut, nFrames);
        conversion.transposeOutput(buffer, nFrames);
      }

      for(int i=0; i<nFrames; ++i) {
        // compress / hardlimit
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

      reverbs.flushToSilence();
      reverbs.clear();

      unmuteAudio();
    }

    template<typename T>
    [[nodiscard]] bool setConvolutionReverbIR(audio::DeinterlacedBuffers<T> const & db, int n_audiocb_frames)
    {
      if constexpr (disable) {
        Assert(0);
        return false;
      }
        bool res = false;
      // having the audio thread compute reverbs at the same time would make our calibration not very reliable
      // (due to cache effects for roots and possibly other) so we disable them now
      muteAudio();

      // locking here would possibly incur dropped audio frames due to the time spent setting the coefficients.
      // we ensured reverbs are not used so we don't need to lock.
        try {
            std::map<int, audio::ConvReverbOptimizationReport> results;
            reverbs.setConvolutionReverbIR(nAudioIn, db, n_audiocb_frames, n_audiocb_frames, audio::sample_rate<double>(), results);
            res = true;
        }
        catch(std::exception const & e) {
            LG(ERR, "setConvolutionReverbIR error : %s", e.what());
        }

      unmuteAudio();

      return res;
    }

    // Must be called from the audio realtime thread.
    void transitionConvolutionReverbWetRatio(double wet) {
        reverbs.transitionConvolutionReverbWetRatio(wet, audio::ms_to_frames(200));
    }

    bool isReady() const
    {
      if constexpr (disable) {
        return true;
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      return readyTraits::read(ready, std::memory_order_relaxed);
    }

    bool hasSpatializer() const { return reverbs.hasSpatializer(); }

  private:

    /////////////////////////////// postprocess
    using readyTraits = maybeAtomic<getAtomicity<policy>(),unsigned int>;
    using readyType = typename readyTraits::type;

    readyType ready;
    std::vector<postProcessFunc> post_process = {
      { [this](double v[nAudioOut]) {
        CArray<nAudioOut, double> a{v};
        compressor.feed(a);
        // by now, the signal is compressed and limited...
      }},
      { [](double v[nAudioOut]) {
        // but just in case, we add this extra post-processing:
        for(int i=0; i<nAudioOut; ++i) {
          if(likely(-1.f <= v[i] && v[i] <= 1.f)) {
            continue;
          }

          if(v[i] > 1.f) {
            std::cout << "clamp " << v[i] << std::endl;
            v[i] = 1.f;
            Assert(0);
          }
          else if(v[i] < -1.f) {
            std::cout << "clamp " << v[i] << std::endl;
            v[i] = -1.f;
            Assert(0);
          }
          else {
            std::cout << "NaN" << std::endl;
            v[i] = 0.f; // v[i] is NaN
            Assert(0);
          }
        }
      }}};

#if WITH_DELAY
    std::vector< DelayLine > delays;
#endif
    Conversion<double, nAudioIn, nAudioOut, audio::audioelement::n_frames_per_buffer> conversion;
    audio::ConvReverbsByBlockSize<audio::Reverbs<nAudioOut, ReverbT, audio::PolicyOnWorkerTooSlow::PermanentlySwitchToDry>> reverbs;
    audio::Compressor<double> compressor;

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
          float millisPerBuffer = audio::frames_to_ms(n);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5f + 20.f * ceil(millisPerBuffer))));
      }
    }

    void unmuteAudio() {
      // we use a fence to ensure that previous non atomic writes
      // will not appear after the write of 'ready'.
      std::atomic_thread_fence(std::memory_order_release);
      readyTraits::write(ready, true, std::memory_order_relaxed);
    }
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

  template< typename ChannelsType, audio::ReverbType ReverbT>
  struct outputDataBase {
    using T = SAMPLE;

    static constexpr auto policy = ChannelsType::policy;
    static constexpr auto nOuts = ChannelsType::nAudioOut;
    using ChannelsT = ChannelsType;
    using Request = typename ChannelsType::Request;
    using PostImpl = AudioPostPolicyImpl<nOuts, ReverbT, policy>;
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
      post.declareBlockSize(nFrames);

      auto t = tNanos;
      MAYBE_CONSTEXPR_SAMPLE_RATE uint64_t nanos_per_iteration =
      static_cast<uint64_t>(
                            0.5f + audio::nanos_per_frame<float>() * static_cast<float>(audio::audioelement::n_frames_per_buffer)
                            );

      double precisionBuffer[audio::audioelement::n_frames_per_buffer * nOuts];
      while(nFrames > 0) {
        auto const nLocalFrames = std::min(nFrames, audio::audioelement::n_frames_per_buffer);

        channelsT.run_computes(nLocalFrames, t);

        const int nSamples = nLocalFrames * nOuts;

        memset(precisionBuffer, 0, nSamples * sizeof(double));
        consume_buffers(precisionBuffer, nLocalFrames, t);
        for(int i=0;
            i != nSamples;
            ++i, ++outputBuffer) {
          *outputBuffer = static_cast<SAMPLE>(precisionBuffer[i]);
        }
        nFrames -= audio::audioelement::n_frames_per_buffer;
        t += nanos_per_iteration;
      }
    }

  private:

    void consume_buffers(double * outputBuffer, int const nFrames, uint64_t const tNanos) {
      Assert(nFrames <= audio::audioelement::n_frames_per_buffer); // by design

      channelsT.forEach(detail::Compute{outputBuffer, nFrames, tNanos});
      post.postprocess(outputBuffer, nFrames);
    }
  };

  template<typename OutputData>
  void dontUseConvolutionReverbs(OutputData & data) {
    data.getPost().dontUseConvolutionReverbs();
  }

}
