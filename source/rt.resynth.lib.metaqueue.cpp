namespace imajuscule::audio {

struct CountDroppedFrames {
  int count = 0;
};
struct InputSample {
  float value;
};

template<typename Queue>
struct MetaQueue {
  
  template <typename... Args>
  MetaQueue(Args&&... args)
  : queue(std::forward<Args...>(args...)){
  }
  
  bool try_push_buffer(const float * buf,
                       int const nFrames) {
    if (pending_dropped_frames.count) {
      if (unlikely(!queue.try_push(pending_dropped_frames))) {
        int const nLocalDroppedFrames = nFrames;
        pending_dropped_frames.count += nLocalDroppedFrames;
        count_dropped_input_frames += nLocalDroppedFrames;
        return false;
      }
      pending_dropped_frames.count = 0;
    }
    for (int i=0; i<nFrames; ++i) {
      if (unlikely(!queue.try_push(InputSample{buf[i]}))) {
        int const nLocalDroppedFrames = nFrames-i;
        pending_dropped_frames.count += nLocalDroppedFrames;
        count_dropped_input_frames += nLocalDroppedFrames;
        return false;
      }
    }
    return true;
  }
  
  int countDroppedInputFrames() const {
    return count_dropped_input_frames;
  }
  
  Queue queue;
  
private:
  CountDroppedFrames pending_dropped_frames;
  std::atomic_int count_dropped_input_frames = 0;
  static_assert(decltype(count_dropped_input_frames)::is_always_lock_free);
};

// Utility functions

template<typename Q>
void drain_queue_until_size_smaller(Q & q,
                                    unsigned const target_size) {
  while(true) {
    int const drop = static_cast<int>(q.was_size()) - static_cast<int>(target_size);
    if (drop <= 0) {
      return;
    }
    for (int i=0; i<drop; ++i) {
      typename Q::value_type var;
      q.try_pop(var);
    }
  }
}

enum class SampleContinuity {
  Yes, // The sample is the follower of the previous sample
  No   // Some samples were dropped in-between
};

/*
 Starts reading when the queue is half full, to avoid starvation
 */
template<typename Queue, typename Ctxt>
struct ReadQueuedSampleSource {
  void set(Queue & queue
#ifndef NDEBUG
           , Ctxt & context
#endif
  ) {
    initialization = true;
    q = &queue;
#ifndef NDEBUG
    ctxt = &context;
#endif
  }
    
  std::optional<std::pair<double, SampleContinuity>>
  operator()() {
    // we synchronize here so as to:
    // - minimize audio latency, and
    // - avoid queue starvation (which would lead to audio cracks)
    //
    // To minimize audio latency, we should start consuming elements as soon as possible.
    // But to avoid starvation, we should start consuming elements only once the queue has a "big enough" size.
    // Taking into account jitter in both input and output, waiting for the queue to be of size
    //   '2 * max(delayIn, delayOut) * sample_rate' seems reasonnable.
    //
    // Note that queue capacity is
    //   '4 * max(delayIn, delayOut) * sample_rate'
    if (initialization) {
      if (q->was_size() < min_size_queue()) {
        return {};
      }
      drain_queue_until_size_smaller(*q,
                                     min_size_queue());
      initialization = false;
    }
    //std::cout << "vocoder queue producer : size=" << q.queue.was_size() << std::endl;
    
    SampleContinuity continuity = SampleContinuity::Yes;
    typename Queue::value_type var_modulator;
    while(true) {
      if (unlikely(!q->try_pop(var_modulator))) {
        // should never happen
        throw std::runtime_error("vocoder : the queue is empty");
      }
      if (unlikely(std::holds_alternative<CountDroppedFrames>(var_modulator))) {
        // happens when the cpu load of the callback is big, so some frames are dropped
#ifndef NDEBUG
        ctxt->asyncLogger().sync_feed(AudioQueueDroppedFrames{
          "vocoder",
          std::get<CountDroppedFrames>(var_modulator).count
        });
#endif
        // some frames have been dropped, so the queue is probably quite full now, we will
        // re-establish balance by dropping some items:
        drain_queue_until_size_smaller(*q, min_size_queue());
        continuity = SampleContinuity::No;
        continue;
      }
      break;
    }
    Assert(std::holds_alternative<InputSample>(var_modulator));
    return std::make_pair(
      std::get<InputSample>(var_modulator).value,
      continuity
    );
  }
  
  
private:
  bool initialization = true;
  Queue * q = nullptr;
#ifndef NDEBUG
  Ctxt * ctxt = nullptr;
#endif
  
  unsigned min_size_queue() const {
    return q->capacity() / 2;
  }
};


} // NS
