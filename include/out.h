
namespace imajuscule {

// reserved number to indicate "no channel"
static constexpr auto AUDIO_CHANNEL_NONE = std::numeric_limits<uint8_t>::max();

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

struct AudioPost {
  
  using postProcessFunc = std::function<void(double*, // buffer
                                             int, // number of frames in the buffer
                                             int // block size (i.e the total number of frames per callback call
                                             )>;

  void set_post_processors(std::vector<postProcessFunc> && v) {
    post_process = std::move(v);
  }
  
  void declareBlockSize(int sz) {
    block_size = sz;
  }
  
  void postprocess(double*buffer,
                   int nFrames) const {
    Assert(block_size);
    
    for(auto const & f: post_process) {
      f(buffer,
        nFrames,
        *block_size);
    }
  }
  
private:
  std::optional<int> block_size;
  std::vector<postProcessFunc> post_process;
};

template<audio::ReverbType ReverbT, int nSources, int nOuts>
struct ReverbPost {

  // Must _not_ be called from the realtime thread
  void dont_use() {
    // clear deletes objects so this should not be called from the realtime thread
    reverbs.clear();
  }
    
  // Must _not_ be called from the realtime thread
  template<typename T>
  bool use(audio::DeinterlacedBuffers<T> const &db,
           int n_audiocb_frames,
           int sample_rate) {
    try {
      std::map<int, audio::ConvReverbOptimizationReport> results;
      reverbs.setConvolutionReverbIR(nSources,
                                     db,
                                     n_audiocb_frames,
                                     n_audiocb_frames,
                                     sample_rate,
                                     results);
      return true;
    }
    catch(std::exception const & e) {
      LG(ERR, "setConvolutionReverbIR error : %s", e.what());
      return false;
    }
  }

  static constexpr int transition_duration_milliseconds = 200;
  
  // Must be called from the realtime thread
  void post_process(double * buf,
                    int const nFrames,
                    int const blockSize) {
    if (!can_process) {
      return;
    }
    reverbs.declareBlockSize(blockSize);
    if (!reverbs.isActive()) {
      return;
    }
    Assert(nFrames <= audio::audioelement::n_frames_per_buffer);
    auto ** ins = conversion.transposeInput(buf, nFrames);
    reverbs.apply(ins, nSources, conversion.editOutput(), nOuts, nFrames);
    conversion.transposeOutput(buf, nFrames);
  }

  // Must be called from the realtime thread
  void transitionConvolutionReverbWetRatio(double wet,
                                           int sample_rate) {
    reverbs.transitionConvolutionReverbWetRatio(wet,
                                                audio::ms_to_frames(transition_duration_milliseconds,
                                                                    sample_rate));
  }

  // Must be called from the realtime thread
  //
  // returns true if the processing has been (or is already) stopped
  bool try_stop_processing() {
    if (!can_process) {
      return true;
    }
    if (reverbs.getWetRatioWithoutStepping != 0) {
      return false;
    }
    can_process = false;
    return true;
  }
  
  // Thread safe
  void start_processing() {
    can_process = true;
  }
  
  // Thread safe
  bool is_processing() const {
    return can_process;
  }

private:
  std::atomic_bool can_process = true;
  Conversion<double, nSources, nOuts, audio::audioelement::n_frames_per_buffer> conversion;
  audio::ConvReverbsByBlockSize<audio::Reverbs<nOuts, ReverbT, audio::PolicyOnWorkerTooSlow::PermanentlySwitchToDry>> reverbs;
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

// !! This is deprecated in favor of SimpleAudioContext, because we try to avoid channels, and replace them by audioelement
template< typename ChannelsType, audio::ReverbType ReverbT>
struct outputDataBase {
  
  using T = SAMPLE;
  
  static constexpr auto policy = ChannelsType::policy;
  static constexpr auto nOuts = ChannelsType::nAudioOut;
  static constexpr auto nAudioIn = nOuts;

  // We disable postprocessing for audio plugins (i.e 'AudioOutPolicy::Slave')
  static constexpr bool disable_post = policy == AudioOutPolicy::Slave;
  
  using ChannelsT = ChannelsType;
  using Request = typename ChannelsType::Request;
  using LockFromRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::RealTime>;
  using LockFromNRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;
  using LockCtrlFromNRT = LockCtrlIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;
  
private:
  using SimpleComputeFunc = folly::Function<bool(double *, // buffer
                                                 const int  // the number of frames to compute
                                                 )>;
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

  using readyTraits = maybeAtomic<getAtomicity<policy>(),unsigned int>;
  using readyType = typename readyTraits::type;

  AudioLockPolicyImpl<policy> & _lock;
  ChannelsT channelsT;
  AudioPost post;
  lockfree::scmp::fifo<OneShotFunc> oneShots{nMaxOneshotsPerCb};
  // We use the 'Synchronization::SingleThread' synchronization because
  // these vectors are accessed from a single thread only (the audio real-time thread).
  static_vector<Synchronization::SingleThread, SimpleComputeFunc> simple_computes;

  readyType post_ready;

  ReverbPost<ReverbT, nAudioIn, nOuts> reverb_post;
  bool has_spatializer = false;

  audio::Limiter<double> limiter;

public:

  template<typename ...Args>
  outputDataBase(AudioLockPolicyImpl<policy>&l, Args ... args)
  : channelsT(l, args ...)
  , _lock(l)
  , simple_computes(500)
  {
    // we start in the "ready" state
    readyTraits::write(post_ready, true, std::memory_order_relaxed);
    if constexpr (!disable_post) {
      initialize_postprocessor();
    }
  }
  
  outputDataBase(AudioLockPolicyImpl<policy>&l)
  : _lock(l)
  , simple_computes(500)
  {
    // we start in the "ready" state
    readyTraits::write(post_ready, true, std::memory_order_relaxed);
    if constexpr (!disable_post) {
      initialize_postprocessor();
    }
  }
  

  ChannelsT & getChannels() { return channelsT; }
  ChannelsT const & getConstChannels() const { return channelsT; }
  
  auto & getPost() { return post; }
  auto const & getPost() const { return post; }
  
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
      f(*this, 0); // TODO pass an accurate time
    }
  }
  
  bool registerSimpleCompute(SimpleComputeFunc && f) {
    if(!simple_computes.tryInsert(std::move(f))) {
      Assert(0);
      return false;
    }
    return true;
  }
  
  // called from audio callback
  
  void step(SAMPLE *outputBuffer,
            int nFrames,
            uint64_t const tNanos,
            uint64_t const nanos_per_audioelement_buffer) {
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
    
    if(unlikely(!postIsReady())) {
      // post-processing is being modified in another thread
      memset(outputBuffer, 0, nFrames * nOuts * sizeof(SAMPLE));
      return;
    }
    post.declareBlockSize(nFrames);
    
    auto t = tNanos;
    
    double precisionBuffer[audio::audioelement::n_frames_per_buffer * nOuts];
    while(nFrames > 0) {
      auto const nLocalFrames = std::min(nFrames, audio::audioelement::n_frames_per_buffer);
      
      channelsT.run_computes(nLocalFrames, t); // this is when the note on, notechange, noteoff callbacks are called, and when the registered computes are called
      
      const int nSamples = nLocalFrames * nOuts;
      
      memset(precisionBuffer, 0, nSamples * sizeof(double));
      
      consume_buffers(precisionBuffer, nLocalFrames, t);
      for(int i=0;
          i != nSamples;
          ++i, ++outputBuffer) {
        *outputBuffer = static_cast<SAMPLE>(precisionBuffer[i]);
      }
      nFrames -= audio::audioelement::n_frames_per_buffer;
      t += nanos_per_audioelement_buffer;
    }
  }

  void dontUseConvolutionReverbs(int sample_rate)
  {
    if constexpr (disable_post) {
      return;
    }
    
    muteAudio(sample_rate);
    
    reverb_post.dont_use();
    
    unmuteAudio();
  }
  
  template<typename T>
  [[nodiscard]] bool setConvolutionReverbIR(int sample_rate,
                                            audio::DeinterlacedBuffers<T> const & db,
                                            int n_audiocb_frames)
  {
    if constexpr (disable_post) {
      Assert(0);
      return false;
    }
    // having the audio thread compute reverbs at the same time would make our calibration not very reliable
    // (due to cache effects for roots and possibly other) so we disable them now
    muteAudio(sample_rate);
    
    // locking here would possibly incur dropped audio frames due to the time spent setting the coefficients.
    // we ensured reverbs are not used so we don't need to lock.
    bool const res = reverb_post.use(db,
                                     n_audiocb_frames,
                                     sample_rate);
    if (res) {
      has_spatializer = db.countChannels() > nAudioIn;
    }
    
    unmuteAudio();
    
    return res;
  }
  
  bool hasSpatializer() const { return has_spatializer; }
  
private:
  
  void consume_buffers(double * outputBuffer, int const nFrames, uint64_t const tNanos) {
    
    struct ComputeFunctor {
      double * buffer;
      int n_frames;
      bool operator()(SimpleComputeFunc & compute) {
        if (compute.heapAllocatedMemory()) {
          throw std::runtime_error("no allocation is allowed in functors");
        }
        return compute(buffer, n_frames);
      }
    };
    
    simple_computes.forEach(ComputeFunctor{outputBuffer, nFrames});
    
    Assert(nFrames <= audio::audioelement::n_frames_per_buffer); // by design
    
    channelsT.forEach(detail::Compute{outputBuffer, nFrames, tNanos});
    post.postprocess(outputBuffer, nFrames);
  }
  
  void initialize_postprocessor() {
    post.set_post_processors({
      {
        [this](double * buf, int const nFrames, int const blockSize) {
          reverb_post.post_process(buf, nFrames, blockSize);
        }
      },
      {
        [this](double * buf, int const nFrames, int const blockSize) {
          for (int i=0; i<nFrames; ++i) {
            CArray<nOuts, double> a{buf + i*nOuts};
            limiter.feedOneFrame(a);
          }
          // by now, the signal is compressed and limited...
        }
      },
      {
        [](double * v, int const nFrames, int const blockSize) {
          // but just in case, we add this extra post-processing:
          for(int i=0, sz=nOuts*nFrames; i<sz; ++i) {
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
        }
      }
    }
                             );
  }
  
  
  bool postIsReady() const
  {
    if constexpr (disable_post) {
      return true;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    return readyTraits::read(post_ready, std::memory_order_relaxed);
  }
  
  void muteAudio(int sample_rate) {
    LockFromNRT l(_lock.lock());
    readyTraits::write(post_ready, false, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    if constexpr (policy == AudioOutPolicy::MasterLockFree) {
      // To avoid modifying postprocessing data while postprocessing is used,
      // we sleep a duration corresponding to twice the size of a callback buffer,
      // to be sure that any audio cb call in progress when we set 'post_ready' to false
      // will be finished when we wake up (and subsequent cb calls will see the updated
      // 'post_ready' value)
      int n = audio::wait_for_first_n_audio_cb_frames();
      float millisPerBuffer = audio::frames_to_ms(n, sample_rate);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5f + 20.f * ceil(millisPerBuffer))));
    }
  }
  
  void unmuteAudio() {
    // we use a fence to ensure that previous non atomic writes
    // will not appear after the write of 'post_ready'.
    std::atomic_thread_fence(std::memory_order_release);
    readyTraits::write(post_ready, true, std::memory_order_relaxed);
  }

};

}
