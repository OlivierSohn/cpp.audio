// The name is historical : initially we were using only sine oscillators with
// this synth, but it is suited to all kinds of oscillators
namespace imajuscule::audio::sine {

  struct SynthImpl {
    static constexpr int32_t get_xfade_length() { return 401; }
    static constexpr float get_gain() { return 1.f; };

    template<typename Element>
    bool setupAudioElement(float freq, Element & e)
    {
      e.algo.setLoudnessParams(5, // corresponds to 63 Hz
        // using 0.8f here to try to even the volume difference with non compensated oscillators.
                               0.8f, // 1.f = full compensation, 0.f = no compensation
                               30.f);
      e.algo.setAngleIncrements(freq_to_angle_increment(freq));
      return true;
    }

    template<
      SynchronizePhase Sync,
      DefaultStartPhase Phase,
      typename Chans,
      typename MonoNoteChannel,
      typename CS>
    std::function<bool(Chans&,int)> onStartNote(MonoNoteChannel & c, CS & cs)
    {
      setPhase<Sync, Phase>(c,cs);
      return {};
    }
  };

  template<
    AudioOutPolicy outPolicy,
    int nOuts,
    XfadePolicy xfade_policy_,
    typename AE,
    SynchronizePhase Sync,
    DefaultStartPhase Phase,
    bool close_channel_on_note_off,
    typename EventIterator,
    typename NoteOnEvent,
    typename NoteOffEvent,
    int nVoices = 32 // using 32 voices to support long releases
  >
  using Synth = ImplCRTP <
    outPolicy,
    nOuts,
    xfade_policy_,
    AE,
    Sync,
    Phase,
    close_channel_on_note_off,
    EventIterator,
    NoteOnEvent,
    NoteOffEvent,
    SynthImpl,
    nVoices
  >;

} // NS imajuscule::audio::sine
