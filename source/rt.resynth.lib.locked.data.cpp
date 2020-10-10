

namespace imajuscule::audio::rtresynth {

/*
 The UI needs access to the "playing pitches" info.
 
 But we don't want to wait on a mutex in the analysis thread.
 
 Alternative 1:
 -------------
 We have a mutex which we try_lock in the analysis thread:
 if we succeed, we copy the infos to another place, protected by the mutex.
 If we fail, we just increment an atomic to say that an analysis frame was dropped.
 
 The ui takes the lock to copy the infos to yet another place, and then processes it
 
 Alternative 2:
 -------------
 The analysis thread fills a queue with items,
 a separate thread aggregates the data, using a mutex to synchronize with the ui thread.
 
 Choice:
 ------
 With Alternative 2, the ui is guaranteed to read the most up-to-date frame,
 while with alternative 1 this is not the case (maybe the most recent frame has been dropped)
 so we chose alternative 2.
 */
struct NonRealtimeAnalysisFrame {
  struct EndOfFrame {
    int64_t frame_id;
    std::chrono::microseconds periodicity;
  };
  
  NonRealtimeAnalysisFrame();
  ~NonRealtimeAnalysisFrame();
  
  // Must be called by a single 'producer' thread.
  void try_push_note_on(PlayedNote const & n);
  // Must be called by a single 'producer' thread.
  void try_push_note_on_dropped(PlayedNote const & n);
  // Must be called by a single 'producer' thread.
  void try_push_note_change(PlayedNote const & n);
  // Must be called by a single 'producer' thread.
  void try_push_note_off(PlayedNote const & n);
  // Must be called by a single 'producer' thread.
  // played_pitches is used only if some data has been dropped
  void try_push(EndOfFrame const &end,
                std::vector<PlayedNote> const & played_pitches);
  
  std::mutex & getMutex() const { return mutex; }
  
  // you must lock the mutex for this call
  std::optional<EndOfFrame> const & getFrameStatus() const { return frame_status; }
  // you must lock the mutex while you read the vector
  std::vector<PlayedNote> const & getPlayingNotes() const { return vec; }
  // you must lock the mutex while you read the vector
  std::vector<PlayedNote> const & getDroppedNotes() const { return vec_dropped; }
  
private:
  struct NoteOn { PlayedNote note; };
  struct NoteOnDropped { PlayedNote note; };
  struct NoteChange { PlayedNote note; };
  struct NoteOff { NoteId note_id; };
  struct CurrentNoteState { PlayedNote note; };
  struct DroppedItems {
    int count = 0;
  };
  
  using Data = std::variant<
  NoteOn,
  NoteOnDropped,
  NoteChange,
  NoteOff,
  CurrentNoteState,
  DroppedItems,
  EndOfFrame
  >;
  using DataQueue = atomic_queue::AtomicQueueB2<
  /* T = */ Data,
  /* A = */ std::allocator<Data>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  static constexpr std::size_t data_queue_capacity = 1024; // if we receive too many 'DroppedItems' that's a sign that this is not enough
  
  DataQueue data_queue{data_queue_capacity};
  DroppedItems pending_dropped_data_items;
  std::atomic_bool thread_active{false};
  std::unique_ptr<std::thread> thread;
  mutable std::mutex mutex;
  bool owns_mutex = false;
  std::optional<std::chrono::microseconds> periodicity;
  std::vector<PlayedNote> vec;
  std::vector<PlayedNote> vec_dropped;
  std::optional<EndOfFrame> frame_status;
  
  void try_push(Data const & item);
  
  void initialize_frame_if_needed();
  
  void onNoteOff(NoteOff const & n);
  void onNoteChange(NoteChange const & n);
};


NonRealtimeAnalysisFrame::NonRealtimeAnalysisFrame() {
  vec.reserve(300);
  vec_dropped.reserve(300);
  
  thread_active = true;
  thread = std::make_unique<std::thread>([this](){
    while(thread_active) {
      NonRealtimeAnalysisFrame::Data i;
      while(data_queue.try_pop(i)) {
        if (!owns_mutex) {
          mutex.lock();
          owns_mutex = true;
          frame_status.reset();
          vec_dropped.clear();
        }
        
        // we need to be fast on reading 'NoteOn', 'NoteOff', 'NoteChange', then 'CurrentNoteState', then 'EndOfFrame', then 'DroppedItems'
        if (std::holds_alternative<NoteOn>(i)) {
          vec.push_back(std::get<NoteOn>(i).note);
        } else if (std::holds_alternative<NoteOff>(i)) {
          onNoteOff(std::get<NoteOff>(i));
        } else if (std::holds_alternative<NoteChange>(i)) {
          onNoteChange(std::get<NoteChange>(i));
        } else if (std::holds_alternative<CurrentNoteState>(i)) {
          vec.push_back(std::get<CurrentNoteState>(i).note);
        } else if (std::holds_alternative<EndOfFrame>(i)) {
          frame_status = std::get<EndOfFrame>(i);
          mutex.unlock();
          owns_mutex = false;
        } else if (std::holds_alternative<NoteOnDropped>(i)) {
          vec_dropped.push_back(std::get<NoteOnDropped>(i).note);
        } else if (std::holds_alternative<DroppedItems>(i)) {
          vec.clear();
        } else {
          throw std::logic_error("unhandled alternative");
        }
      }
      if (owns_mutex) {
        // the frame is not full yet, so the producer will very soon push new items to the queue,
        // hence we don't want to sleep
        continue;
      }
      auto const & status = getFrameStatus();
      if (!status) {
        // no frame has been received yet so we don't know what the periodicity of the producer is
        continue;
      }
      std::chrono::microseconds const period = status->periodicity;
      if (period > std::chrono::microseconds(1000)) {
        std::this_thread::sleep_for(period/30);
      }
    }
    if (owns_mutex) {
      mutex.unlock();
      owns_mutex = false;
    }
  });
}

NonRealtimeAnalysisFrame::~NonRealtimeAnalysisFrame() {
  thread_active = false;
  thread->join();
}

void NonRealtimeAnalysisFrame::try_push_note_on(PlayedNote const & n) {
  if (unlikely(pending_dropped_data_items.count)) {
    ++pending_dropped_data_items.count;
    return;
  }
  try_push(NoteOn{n});
}
void NonRealtimeAnalysisFrame::try_push_note_on_dropped(PlayedNote const & n) {
  if (unlikely(pending_dropped_data_items.count)) {
    ++pending_dropped_data_items.count;
    return;
  }
  try_push(NoteOnDropped{n});
}
void NonRealtimeAnalysisFrame::try_push_note_change(PlayedNote const & n) {
  if (unlikely(pending_dropped_data_items.count)) {
    ++pending_dropped_data_items.count;
    return;
  }
  try_push(NoteChange{n});
}
void NonRealtimeAnalysisFrame::try_push_note_off(PlayedNote const & n) {
  if (unlikely(pending_dropped_data_items.count)) {
    ++pending_dropped_data_items.count;
    return;
  }
  try_push(NoteOff{n.noteid});
}
void NonRealtimeAnalysisFrame::try_push(EndOfFrame const &end,
                                        std::vector<PlayedNote> const & played_pitches) {
  if (unlikely(pending_dropped_data_items.count)) {
    if (!data_queue.try_push(pending_dropped_data_items)) {
      return;
    }
    pending_dropped_data_items.count = 0;
    // because some data has been dropped, we send the full state.
    for (PlayedNote const & note : played_pitches) {
      try_push(CurrentNoteState{note});
      if (pending_dropped_data_items.count) {
        // we failed to send the full state.
        // we will try to re-send the full state at the next frame end
        return;
      }
    }
  }
  Assert(!pending_dropped_data_items.count);
  try_push(end);
}


void NonRealtimeAnalysisFrame::try_push(Data const & item) {
  if (!data_queue.try_push(item)) {
    ++pending_dropped_data_items.count;
  }
}

void NonRealtimeAnalysisFrame::onNoteOff(NoteOff const & n) {
  auto it = std::find_if(vec.begin(),
                         vec.end(),
                         [noteid = n.note_id](PlayedNote const & n) { return n.noteid == noteid; });
  if (it == vec.end()) {
    throw std::logic_error("orphan note off");
  }
  vec.erase(it);
}
void NonRealtimeAnalysisFrame::onNoteChange(NoteChange const & n) {
  auto it = std::find_if(vec.begin(),
                         vec.end(),
                         [noteid = n.note.noteid](PlayedNote const & n) { return n.noteid == noteid; });
  if (it == vec.end()) {
    throw std::logic_error("orphan note change");
  }
  *it = n.note;
}

} // NS
