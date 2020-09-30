
namespace imajuscule::audio {

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

template<AudioOutPolicy outPolicy, int nAudioOut, audioelement::SoundEngineMode MODE, bool withNoteOff>
using Voice = voice::Impl_<outPolicy, nAudioOut, MODE, withNoteOff, std::vector<float>, EventIterator, ProcessData>;


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
  
  return v.onEvent(mkNoteOn(NoteId{b.midiPitch},
                            midi.midi_pitch_to_freq(b.midiPitch),
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
  return v.onEvent(mkNoteOff(NoteId{midiPitch}),
                   out,
                   chans);
}
} // NS
