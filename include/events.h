
namespace imajuscule::audio {

enum class onEventResult {
  OK = 0,
  DROPPED_NOTE
};

std::ostream & operator << (std::ostream &, const onEventResult &);

enum class EventType : uint8_t
{
  NoteOn,
  NoteOff,
  NoteChange
};

struct NoteOnEvent
{
  float frequency;
  float velocity;      ///< range [0.0, 1.0]
};

struct NoteChangeEvent
{
  float changed_frequency;      ///< in Herz
  float changed_velocity;      ///< range [0.0, 1.0]
};

//------------------------------------------------------------------------
/** Note-off event specific data. Used in \ref Event (union)*/
struct NoteOffEvent
{
  float velocity;      ///< range [0.0, 1.0]
};

struct NoteId {
  int64_t noteid;

  bool operator == (NoteId const & o) const {
    return noteid == o.noteid;
  }
  bool operator != (NoteId const & o) const {
    return noteid != o.noteid;
  }
};

inline std::ostream& operator << (std::ostream& os, const NoteId & n) {
  os << "Noteid(" << n.noteid << ")";
  return os;
}

struct Event
{
  Event(EventType t, NoteId const & n)
  : type(t)
  , noteid(n) {}
  
  EventType type;
  
  // identifies the note, doesn't change during the lifetime of the note
  NoteId noteid;
  
  union
  {
    NoteOnEvent noteOn;                ///< type == kNoteOnEvent
    NoteChangeEvent noteChange;     ///< type == kNoteChangeEvent
    NoteOffEvent noteOff;              ///< type == kNoteOffEvent
  };
};

inline Event mkNoteOn(NoteId const & noteid,
                      float frequency,
                      float velocity) {
  Event e(EventType::NoteOn,
          noteid);
  e.noteOn.frequency = frequency;
  e.noteOn.velocity = velocity;
  return e;
}

inline Event mkNoteChange(NoteId const & noteid,
                          float changed_velocity,
                          float new_frequency) {
  Event e(EventType::NoteChange,
          noteid);
  e.noteChange.changed_frequency = new_frequency;
  e.noteChange.changed_velocity = changed_velocity;
  return e;
}

inline Event mkNoteOff(NoteId const & noteid) {
  Event e(EventType::NoteOff,
          noteid);
  e.noteOff.velocity = 0.f;
  return e;
}


constexpr auto event_position_infinite = std::numeric_limits<int>::max();

template<typename EventIterator>
int getNextEventPosition(EventIterator it, EventIterator end) {
  if(it == end) {
    return event_position_infinite;
  }
  return it.getSampleOffset();
}

struct IEventList
{
  int getEventCount () const { return events.size(); }
  
  Event const & getEvent (int32_t index) const {
    Assert(index < events.size());
    return events[index];
  }
  
  std::vector<Event> events;
};


enum class Iterator {
  End,
  Begin
};

struct EventIterator {
  using iterator = EventIterator;
  
  EventIterator(IEventList * l, Iterator i)
  : list(l)
  , cur(0)
  {
    if(list) {
      switch (i) {
        case Iterator::Begin:
          break;
        case Iterator::End:
          cur = list->getEventCount();
          break;
      }
    }
  }
  
  iterator& operator++() { // prefix increment
    ++cur;
    return *this;
  }
  
  bool operator ==(iterator const & o) const {
    return list == o.list && cur == o.cur;
  }
  Event dereference() const {
    assert(list);
    return list->getEvent(cur);
  }

private:
  int cur;
  IEventList * list;
};


template<typename T>
struct EventIteratorFor;

template<typename T>
using EventIteratorOf = typename EventIteratorFor<T>::type;

template<>
struct EventIteratorFor<IEventList> {
  using type = EventIterator;
};

struct MIDITimestampAndSource {
  MIDITimestampAndSource(uint64_t t,
                         uint64_t sourceKey)
  : time(t)
  , src_key(sourceKey)
  {}

  uint64_t getNanosTime() const {
    return time;
  }
  uint64_t getSourceKey() const {
    return src_key;
  }
  
private:
  uint64_t time, src_key;
};

/*
 Can be used to generate note ids for a synth where each physical key can only be pressed
 at most once at any given time, and is identified by 'Key'.
 
 The idea is the following:

 For every note-on event, a new unique note id is created, and associated to that
 key.
 Until a "note off" event for that key is sent, subsequent "note change" events
 for that same key will use the same noteid.
 The subsequent "note off" event will also use the same noteid, and remove the association
 between the key and the note id, so that on a subsequent note-on event for that key,
 a new note id is generated.
 */
struct NoteIdsGenerator {
  using Key = int;
  
  NoteIdsGenerator(int approx_max_simultaneous_chans)
  : noteids(approx_max_simultaneous_chans)
  {}
  
  NoteIdsGenerator(NoteIdsGenerator const &) = delete;
  NoteIdsGenerator & operator = (NoteIdsGenerator const &) = delete;
  NoteIdsGenerator(NoteIdsGenerator &&) = delete;
  NoteIdsGenerator & operator = (NoteIdsGenerator &&) = delete;

  NoteId NoteOnId(Key const key) {
    next.noteid++;
    auto it = noteids.find(key);
    if (it == noteids.end()) {
      noteids.emplace(key, next);
    } else {
      it->second = next;
    }
    return next;
  }
  
  NoteId NoteChangeId(Key const key) {
    auto it = noteids.find(key);
    if (it == noteids.end()) {
      Assert(0); // a notechange must be preceeded by noteon, and must be before noteoff
      return {-key};
    } else {
      return it->second;
    }
  }
  
  NoteId NoteOffId(Key const key) {
    auto it = noteids.find(key);
    if (it == noteids.end()) {
      Assert(0); // a noteoff must be preceded by noteon
      return {-key};
    } else {
      auto res = it->second;
      noteids.erase(it);
      return res;
    }
  }
  
  auto begin() const { return noteids.begin(); }
  auto end() const { return noteids.end(); }

  void clear() {
    noteids.clear();
  }
private:
  NoteId next{};
  std::unordered_map<Key, NoteId> noteids;
};

// @param e [in/out] :
//   In the input, e.noteid.noteid is the key that uniquely identifies the played note among all currently played notes.
//     It must fit in the values of an int.
//     Typically, for a synth played via a midi keyboard it could be the midipitch, because in a midi keyboard,
//       there is a physical constraint that prevents the same key to be pressed twice in a row without being released first.
//   In the output, e.noteid.noteid is the noteid that uniquely identifies the played note among all notes played currently or in the past or in the future:
inline void convertKeyToNoteId(NoteIdsGenerator & gen,
                               Event & e) {
  int const key = static_cast<int>(e.noteid.noteid);
  switch(e.type) {
    case EventType::NoteOn:
      e.noteid = gen.NoteOnId(key);
      break;
    case EventType::NoteChange:
      e.noteid = gen.NoteChangeId(key);
      break;
    case EventType::NoteOff:
      e.noteid = gen.NoteOffId(key);
      break;
  }
}


} // namespace
