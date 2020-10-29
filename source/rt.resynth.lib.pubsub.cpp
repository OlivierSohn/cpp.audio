namespace imajuscule::audio {

/*
 Dispatches audio samples to listeners (queues).
 
 Thread safety:
 
 - 'add_listener' and 'remove_listener' must be called from the same thread.
 - 'try_publish_*', 'update_rt_listeners' must be called from a single thread.
 */
template<typename Queue>
struct AudioBufferPubSub {
  AudioBufferPubSub(int num_one_shots_max,
                    int num_listeners_max)
  : oneshots(num_one_shots_max)
  {
    rt_listeners.reserve(num_listeners_max);
  }
    
  MetaQueue<Queue>* add_listener(int num_elts) {
    auto q = std::make_unique<MetaQueue<Queue>>(num_elts);
    auto q_ptr = q.get();
    listeners.push_back(std::move(q));
    
    if (!oneshots.tryEnqueue([this, q_ptr](){
      if (rt_listeners.capacity() == rt_listeners.size()) {
        throw std::runtime_error("rt_listeners too small");
      }
      rt_listeners.push_back(q_ptr);
    })) {
      throw std::runtime_error("oneshots too small");
    }
    
    return q_ptr;
  }
  
  void remove_listener(MetaQueue<Queue> * q) {
    enum class QueueRemoval {
      Scheduled,
      QueueNotFound,
      Done
    };
    std::atomic<QueueRemoval> executed = QueueRemoval::Scheduled;
    
    if (!oneshots.tryEnqueue([this,
                              &executed,
                              q](){
      auto it = std::find(rt_listeners.begin(),
                          rt_listeners.end(),
                          q);
      if (it != rt_listeners.end()) {
        rt_listeners.erase(it);
        executed = QueueRemoval::Done;
      } else {
        executed = QueueRemoval::QueueNotFound;
      }
    })) {
      throw std::runtime_error("oneshots too small");
    }
    
    while(executed == QueueRemoval::Scheduled) {
      std::this_thread::yield();
    }
    
    if (executed == QueueRemoval::QueueNotFound) {
      throw std::runtime_error("queue not found");
    }
    
    {
      auto it = std::find_if(listeners.begin(),
                             listeners.end(),
                             [q](auto & p) { return p.get() == q; });
      if (it == listeners.end()) {
        throw std::logic_error("queue not found in listeners");
      }
      listeners.erase(it);
    }
  }

  bool try_publish_buffer(const float * buf,
                          int nFrames) {
    update_rt_listeners();
    bool res = true;
    for (auto * q : rt_listeners) {
      if(!q->try_push_buffer(buf, nFrames)) {
        res = false;
      }
    }
    return res;
  }
  
  bool try_publish_sample(float s) {
    bool res = true;
    for (auto * q : rt_listeners) {
      if(!q->try_push_buffer(&s, 1)) {
        res = false;
      }
    }
    return res;
  }

  void update_rt_listeners() {
    oneshots.dequeueAll([](auto & f) {
      if (f.heapAllocatedMemory()) {
        throw std::runtime_error("no allocation is allowed in functors");
      }
      f(); // this will modify 'rt_listeners'
    });
  }
private:
  std::vector<std::unique_ptr<MetaQueue<Queue>>> listeners; // managed by "producer" thread
  
  std::vector<MetaQueue<Queue>*> rt_listeners; // managed by "consumer" thread
  
  using OneShotFunc = folly::Function<void(void)>;
  lockfree::scmp::fifo<OneShotFunc> oneshots; // to synchronize modifications of 'rt_listeners'
};

template<typename Queue>
struct AudioBufferAggregator {
  using QueueItem = typename Queue::value_type;

  enum class State {
    Uninitialized,
    Synchronized
  };

  enum class ReceivedItems {
    None,
    Some,
    SomeAfterSynchronization
  };

  AudioBufferAggregator() {
    reset();
    queues.reserve(4);
  }
  
  void reset() {
    state = State::Uninitialized;
    queues.clear();
    reset_iterators();
  }

  void add_stream(Queue & q) {
    queues.push_back(&q);
    reset_iterators();
  }
  
  // if this returns true, call get_data() to read the data
  bool try_pop() {
    if (unlikely(state == State::Uninitialized)) {
      for (auto & q : queues) {
        if (q.state == ReceivedItems::None) {
          QueueItem val;
          if (!q.queue->try_pop(val)) {
            return false;
          }
          q.state = ReceivedItems::Some;
        }
      }
      // By now, each queue has received exactly one item, meanining that all producers are up and running.
      
      // flush the queues
      bool go_on = true;
      do {
        go_on = false;
        for (auto & q : queues) {
          QueueItem val;
          while (q.queue->try_pop(val)) {
            go_on = true;
          }
        }
      } while(go_on);
      
      // All queues are empty and are now said to be "synchronized"
      
      state = State::Synchronized;
      
      // Eventhough the queues are empty now, it is possible that some samples were dropped at the producer side,
      // so we shall ignore the first 'CountDroppedFrames' item of each queue (see comment below).
    }
    
    for (; i<sz; ++i) {
      if (!queues[i].queue->try_pop(data[i])) {
        return false;
      }
      if (unlikely(queues[i].state == ReceivedItems::Some)) {
        if (std::holds_alternative<CountDroppedFrames>(data[i])) {
          // disregard this 'CountDroppedFrames' (see comment above)
          if (!queues[i].queue->try_pop(data[i])) {
            return false;
          }
        }
        queues[i].state = ReceivedItems::SomeAfterSynchronization;
      }
    }
    i = 0;
    return true;
  }
  
  // Contains valid data if the last call to this object was 'try_pop' and it returned true
  std::vector<QueueItem> const & get_data() const {
    return data;
  }
private:
  State state;
  
  struct Q {
    Q(Queue*q)
    : queue(q){}
    
    ReceivedItems state = ReceivedItems::None;
    Queue* queue;
  };
  
  std::vector<Q> queues;
  int i, sz;
  
  std::vector<QueueItem> data;
  
  void reset_iterators() {
    i = 0;
    sz = static_cast<int>(queues.size());
    data.resize(queues.size());
  }
};
}
