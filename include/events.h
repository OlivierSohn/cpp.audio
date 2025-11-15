#include <deque>

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

// identifies the note, doesn't change during the lifetime of the note
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

struct TimestampAndSource {
  TimestampAndSource(uint64_t t,
                         uint64_t sourceKey)
  : time(t)
  , src_key(sourceKey)
  {}

  void offsetNanosTime(uint64_t offset) {
    time += offset;
  }

  uint64_t getNanosTime() const {
    return time;
  }
  uint64_t getSourceKey() const {
    return src_key;
  }
  
  auto operator<(const TimestampAndSource& other) const
  {
    return std::tie(time, src_key) < std::tie(other.time, other.src_key);
  }
  auto operator==(const TimestampAndSource& other) const
  {
    return std::tie(time, src_key) == std::tie(other.time, other.src_key);
  }

private:
  // nanoseconds (either MIDI time or a monotonic time, depending on TimeSource of Context).
  uint64_t time;

  uint64_t src_key;
};

/*
 For every note-on event, a new unique NoteId is created, and associated to that
 key.
 We can have multiple note on events for the same key. In that case:
 - note off events will correspond to the _earliest_ note-on
 - note change events will correspond to the _latest_ note-on
 */
struct NoteIdsGenerator {
  using Key = uint64_t;

  NoteIdsGenerator(int approx_max_simultaneous_chans)
  : noteids(approx_max_simultaneous_chans)
  {}

  NoteIdsGenerator(NoteIdsGenerator const &) = delete;
  NoteIdsGenerator & operator = (NoteIdsGenerator const &) = delete;
  NoteIdsGenerator(NoteIdsGenerator &&) = delete;
  NoteIdsGenerator & operator = (NoteIdsGenerator &&) = delete;

  NoteId NoteOnId(Key const key) {
    next.noteid++;
    const auto res = noteids.try_emplace(key, std::deque<NoteId>{});
    res.first->second.push_back(next);
    return next;
  }

  NoteId NoteChangeId(Key const key) {
    auto it = noteids.find(key);
    if (it == noteids.end()) {
      Assert(0); // a notechange must be preceeded by noteon, and must be before noteoff
      return {-static_cast<int64_t>(key)};
    }
    Assert(!it->second.empty());
    return it->second.back();
  }

  NoteId NoteOffId(Key const key) {
    auto it = noteids.find(key);
    if (it == noteids.end()) {
      Assert(0); // a noteoff must be preceded by noteon
      return {-static_cast<int64_t>(key)};
    }
    Assert(!it->second.empty());
    const auto res = it->second.front();
    if (it->second.size() == 1)
      noteids.erase(it);
    else
      it->second.pop_front();
    return res;
  }

  auto begin() const { return noteids.begin(); }
  auto end() const { return noteids.end(); }

  void clear() {
    noteids.clear();
  }
private:
  NoteId next{};
  std::unordered_map<Key, std::deque<NoteId>> noteids;
};

// @param e [in/out] :
//   In the input, e.noteid.noteid is the key that uniquely identifies the played note among all currently played notes.
//     It must fit in the values of an int.
//     Typically, for a synth played via a midi keyboard it could be the midipitch, because in a midi keyboard,
//       there is a physical constraint that prevents the same key from being pressed twice in a row without being released first.
//   In the output, e.noteid.noteid is the noteid that uniquely identifies the played note among all notes played currently or in the past or in the future:
// @param voice : used to group events. a noteoff or notechange can only correspond to a noteon of the same voice.
inline void convertKeyToNoteId(NoteIdsGenerator & gen,
                               int voice, Event & e) {
  uint64_t const key =
    static_cast<uint64_t>(static_cast<uint32_t>(voice)) |
    (static_cast<uint64_t>(static_cast<uint32_t>(e.noteid.noteid)) << 32ull);
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
