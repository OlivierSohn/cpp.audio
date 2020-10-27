namespace imajuscule::audio {

template<typename Queue>
struct Input {
  Input(int num_one_shots_max,
        int num_queues_max)
  : oneshots(num_one_shots_max)
  {
    input_queues.reserve(num_queues_max);
  }
    
  // must be called from a single producer thread
  MetaQueue<Queue>* add_queue(int num_elts) {
    auto q = std::make_unique<MetaQueue<Queue>>(num_elts);
    auto q_ptr = q.get();
    queues.push_back(std::move(q));
    
    if (!oneshots.tryEnqueue([this, q_ptr](){
      if (input_queues.capacity() == input_queues.size()) {
        throw std::runtime_error("input_queues too small");
      }
      input_queues.push_back(q_ptr);
    })) {
      throw std::runtime_error("input_oneshots too small");
    }
    
    return q_ptr;
  }
  
  // must be called from a single producer thread
  void remove_queue(MetaQueue<Queue> * q) {
    enum class QueueRemoval {
      Scheduled,
      QueueNotFound,
      Done
    };
    std::atomic<QueueRemoval> executed = QueueRemoval::Scheduled;
    
    if (!oneshots.tryEnqueue([this,
                              &executed,
                              q](){
      auto it = std::find(input_queues.begin(), input_queues.end(), q);
      if (it != input_queues.end()) {
        input_queues.erase(it);
        executed = QueueRemoval::Done;
      } else {
        executed = QueueRemoval::QueueNotFound;
      }
    })) {
      throw std::runtime_error("input_oneshots too small");
    }
    
    while(executed == QueueRemoval::Scheduled) {
      std::this_thread::yield();
    }
    
    if (executed == QueueRemoval::QueueNotFound) {
      throw std::runtime_error("queue not found");
    }
    
    {
      auto it = std::find_if(queues.begin(),
                             queues.end(),
                             [q](auto & p) { return p.get() == q; });
      if (it == queues.end()) {
        throw std::logic_error("queue not found in queues");
      }
      queues.erase(it);
    }
  }

  // must be called from a single consumer thread
  bool try_push_buffer(const float * buf,
                       int nFrames) {
    oneshots.dequeueAll([](auto & f) {
      if (f.heapAllocatedMemory()) {
        throw std::runtime_error("no allocation is allowed in functors");
      }
      f(); // this will add / remove queues
    });
    bool res = true;
    for (auto * q : input_queues) {
      res = res && q->try_push_buffer(buf,
                                      nFrames);
    }
    return res;
  }
  
private:
  // we might need a mutex to protect this vector in the future in case we can, from the UI thread,
  // activate / deactivate the vocoder, i.e add / remove an additional queue
  std::vector<std::unique_ptr<MetaQueue<Queue>>> queues;
  
  std::vector<MetaQueue<Queue>*> input_queues; // managed by rt input thread
  
  using OneShotFunc = folly::Function<void(void)>;
  lockfree::scmp::fifo<OneShotFunc> oneshots; // these are used to synchronize modifications of 'input_queues'
};

}
