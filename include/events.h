
namespace imajuscule::audio {

enum class onEventResult {
  OK = 0,
  DROPPED_NOTE
};

std::ostream & operator << (std::ostream &, const onEventResult &);


struct ReferenceFrequencyHerz {
  explicit ReferenceFrequencyHerz()
  : freq(0)
  {}
  
  explicit ReferenceFrequencyHerz(float v)
  : freq(v) {}
  
  float getFrequency() const {
    return freq;
  }
  
  bool operator < (ReferenceFrequencyHerz const & o) const {
    return freq < o.freq;
  }
  bool operator == (ReferenceFrequencyHerz const & o) const {
    return freq == o.freq;
  }
  bool operator != (ReferenceFrequencyHerz const & o) const {
    return !this->operator == (o);
  }
  
private:
  float freq;
};

enum class EventType : uint8_t
{
  NoteOn,
  NoteOff,
  NoteChange
};



struct NoteOnEvent
{
  float velocity;      ///< range [0.0, 1.0]
  int32_t length;           ///< in sample frames (optional, Note Off has to follow in any case!)
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

struct Event
{
  Event(EventType t, ReferenceFrequencyHerz const & f)
  : type(t)
  , ref_frequency(f) {}
  
  EventType type;
  
  // identifies the note, doesn't change during the lifetime of the note
  ReferenceFrequencyHerz ref_frequency;
  
  union
  {
    NoteOnEvent noteOn;                ///< type == kNoteOnEvent
    NoteChangeEvent noteChange;     ///< type == kNoteChangeEvent
    NoteOffEvent noteOff;              ///< type == kNoteOffEvent
  };
};

inline Event mkNoteOn(ReferenceFrequencyHerz const & ref,
                      float velocity) {
  Event e(EventType::NoteOn,
          ref);
  e.noteOn.velocity = velocity;
  e.noteOn.length = std::numeric_limits<decltype(e.noteOn.length)>::max();
  return e;
}

inline Event mkNoteOn(Midi const & m,
                      double pitch,
                      float velocity) {
  return mkNoteOn(ReferenceFrequencyHerz(m.midi_pitch_to_freq(pitch)),
                  velocity);
}

inline Event mkNoteChange(ReferenceFrequencyHerz const & ref,
                          float changed_velocity,
                          float new_frequency) {
  Event e(EventType::NoteChange,
          ref);
  e.noteChange.changed_frequency = new_frequency;
  e.noteChange.changed_velocity = changed_velocity;
  return e;
}

inline Event mkNoteChange(Midi const & m,
                          int pitch,
                          float relative_velocity,
                          float new_frequency) {
  return mkNoteChange(ReferenceFrequencyHerz(m.midi_pitch_to_freq(pitch)),
                      relative_velocity,
                      new_frequency);
}

inline Event mkNoteOff(ReferenceFrequencyHerz const & ref) {
  Event e(EventType::NoteOff,
          ref);
  e.noteOff.velocity = 0.f;
  return e;
}

inline Event mkNoteOff(Midi const & m,
                       double pitch) {
  return mkNoteOff(ReferenceFrequencyHerz(m.midi_pitch_to_freq(pitch)));
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

} // namespace
