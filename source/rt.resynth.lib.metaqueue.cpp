namespace imajuscule::audio {

struct CountDroppedFrames {
  int count = 0;
};
struct InputSample {
  float value;
};

template<typename Queue>
struct MetaQueue {
  using QueueItem = typename Queue::value_type;
  
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

} // NS
