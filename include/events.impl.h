
namespace imajuscule::audio {

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
  float velocity;			///< range [0.0, 1.0]
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
    NoteOffEvent noteOff;							///< type == kNoteOffEvent
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
                      int pitch,
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
                       int pitch) {
  return mkNoteOff(ReferenceFrequencyHerz(m.midi_pitch_to_freq(pitch)));
}


struct IEventList
{
  int getEventCount () const { return events.size(); }
  
  void getEvent (int32_t index, Event& e /*out*/) {
    Assert(index < events.size());
    e = events[index];
  }
  
  std::vector<Event> events;
};

template<>
struct EventOf<IEventList> {
  using type = Event;
};

using EventIteratorImpl = EventIterator<IEventList>;

enum SymbolicSampleSizes
{
  kSample32,		///< 32-bit precision
  kSample64		///< 64-bit precision
};

struct ProcessData
{
  ProcessData ()
  : processMode (0), symbolicSampleSize (kSample32), numSamples (0), numInputs (0)
  , numOutputs (0)
  , inputEvents (0), outputEvents (0) {}
  
  //------------------------------------------------------------------------
  int32_t processMode;			///< processing mode - value of \ref ProcessModes
  int32_t symbolicSampleSize;   ///< sample size - value of \ref SymbolicSampleSizes
  int32_t numSamples;			///< number of samples to process
  int32_t numInputs;			///< number of audio input buses
  int32_t numOutputs;			///< number of audio output buses
  IEventList* inputEvents;				///< incoming events for this block (optional)
  IEventList* outputEvents;				///< outgoing events for this block (optional)
};

template<AudioOutPolicy outPolicy, int nAudioOut, audio::SoundEngineMode MODE, bool withNoteOff>
using Voice = imajuscule::audio::voice::Impl_<outPolicy, nAudioOut, MODE, withNoteOff, std::vector<float>, EventIteratorImpl, NoteOnEvent, NoteOffEvent, ProcessData>;


////////////////////

struct Voicing {
  Voicing(int prog, int16_t midiPitch, float gain, float pan, bool random, int seed) : program(prog), pan(pan), gain(gain), random(random), seed(seed), midiPitch(midiPitch) {}
  int program;
  int16_t midiPitch;
  bool random;
  int seed;
  float gain;
  float pan;
};

template<typename Voice, typename OutputData, typename Chans>
onEventResult playOneThing(Midi const & midi,
                           Voice & v,
                           OutputData & out,
                           Chans & chans,
                           Voicing const & b) {
  
  v.initializeSlow(); // does something only the 1st time
  v.useProgram(b.program); // keep it first as it reinitializes params
  v.set_random_pan(false);
  v.set_random(b.random);
  if(!b.random) {
    v.set_seed(b.seed);
  }
  v.set_gain(b.gain);
  v.set_pan(b.pan);
  v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!
  
  return v.onEvent(mkNoteOn(midi,
                            b.midiPitch,
                            1.0),
                   out,
                   chans);
}

template<typename Voice, typename OutputData, typename Chans>
onEventResult stopPlaying(Midi const & midi,
                          Voice & v,
                          OutputData & out,
                          Chans & chans,
                          int16_t midiPitch) {
  return v.onEvent(mkNoteOff(midi,
                             midiPitch),
                   out,
                   chans);
}
} // NS
