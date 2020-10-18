
namespace imajuscule::audio {

struct PostNoOp {
  template<typename T>
  PostNoOp(T &){}

  bool isReady() const {
    return true;
  }

  void declareBlockSize(int) const {}

  void postprocess(double*, int) const {}
};

// Limiting is not required as a default: for example in a DAW, it is ok to have a signal with magnitude > 1,
// as long as there is a gain adjustment after the plugin.
//
// Hence, by default, we use 'PostNoOp'
template<int NAudioOuts, AudioOutPolicy Policy, typename PostImpl = PostNoOp>
struct SimpleAudioOutContext   {
  using MeT = SimpleAudioOutContext<NAudioOuts, Policy, PostImpl>;

  static constexpr auto nOuts = NAudioOuts;
  static constexpr auto nAudioOut = nOuts;
  static constexpr auto policy = Policy;

  static constexpr int max_n_frames_per_loop = audio::audioelement::n_frames_per_buffer; // we cannot use anything else for now because we use nanos_per_audioelement_buffer in the loop

  using LockFromRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::RealTime>;
  using LockFromNRT = LockIf<AudioLockPolicyImpl<policy>::useLock, ThreadType::NonRealTime>;

  /*
   * The function is executed once, and then removed from the queue.
   */
  using OneShotFunc = folly::Function<void(MeT &
                                           , const uint64_t // the time of the start of the buffer that will be computed.
                                           )>;

  using ComputeFunc = folly::Function<bool(double *, // buffer
                                           const int  // the number of frames to compute
                                           )>;

public:
  SimpleAudioOutContext(AudioLockPolicyImpl<Policy> & lock,
                        int const sz_one_shots,
                        int const sz_computes)
  : _lock(lock)
  , oneShots(sz_one_shots)
  , computes(sz_computes)
  , post(lock)
  {
  }

  PostImpl & getPost() { return post; }
  PostImpl const & getPost() const { return post; }

  // this method should not be called from the real-time thread
  // because it yields() and retries.
  template<typename F>
  void enqueueOneShot(F && f) {
    if constexpr(shouldNRTThreadUseOneshotsQueue<policy>()) {
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
      f(*this, 0); // TODO pass the right time instead of 0
    }
  }

  // Must be called only from the realtime thread
  bool registerSimpleCompute(ComputeFunc && f) {
    if(!computes.tryInsert(std::move(f))) {
      ++failed_compute_insertion;
      return false;
    }
    return true;
  }

  void step(SAMPLE *outputBuffer,
            int nFrames,
            uint64_t const tNanos,
            uint64_t const nanos_per_audioelement_buffer) {
    LockFromRT l(_lock.lock());

    oneShots.dequeueAll([this, tNanos](auto & f) {
      f(*this, tNanos);
    });

    // TODO make post optional, and use the 'oneShots' to set it:
    // - one oneshot starts the fade-out
    // - X ms later, another one shot swaps and fades-in the new current post.
    // Then, we won't have the need to shutdown the audio output while post is being initialized, like below:
    if(unlikely(!post.isReady())) {
      // post is being initialized in another thread
      memset(outputBuffer, 0, nFrames * nOuts * sizeof(SAMPLE));
      return;
    }
    post.declareBlockSize(nFrames);

    auto t = tNanos;

    double precisionBuffer[max_n_frames_per_loop * nOuts];
    while(nFrames > 0) {
      auto const nLocalFrames = std::min(nFrames,
                                         max_n_frames_per_loop);

      const int nSamples = nLocalFrames * nOuts;

      memset(precisionBuffer,
             0,
             nSamples * sizeof(double));

      struct ComputeFunctor {
        double * buffer;
        int n_frames;
        bool operator()(ComputeFunc & compute) {
          if (compute.heapAllocatedMemory()) {
            throw std::runtime_error("no allocation is allowed in functors");
          }
          return compute(buffer, n_frames);
        }
      };

      computes.forEach(ComputeFunctor{precisionBuffer, nLocalFrames});
      post.postprocess(precisionBuffer, nLocalFrames);

      for(int i=0;
          i != nSamples;
          ++i, ++outputBuffer) {
        *outputBuffer = static_cast<SAMPLE>(precisionBuffer[i]);
      }
      nFrames -= max_n_frames_per_loop;
      t += nanos_per_audioelement_buffer;
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
  lockfree::scmp::fifo<OneShotFunc> oneShots;
  // We use the 'Synchronization::SingleThread' synchronization because
  // these vectors are accessed from a single thread only (the audio real-time thread).
  static_vector<Synchronization::SingleThread, ComputeFunc> computes;
  PostImpl post;

  std::atomic_int failed_compute_insertion = 0;
  std::atomic_int retried_oneshot_insertion = 0;
};

}
