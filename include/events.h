
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
} // namespace
