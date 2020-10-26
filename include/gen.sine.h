// The name is historical : initially we were using only sine oscillators with
// this synth, but it is suited to all kinds of oscillators
namespace imajuscule::audio::sine {

  struct SynthImpl {
    static constexpr float get_xfade_length() { return 0.009f; }
    static constexpr float get_gain() { return 1.f; };

    template<typename Element>
    bool setupAudioElement(float freq,
                           Element & e,
                           int const sample_rate)
    {
      e.setLoudnessParams(sample_rate,
                               5, // corresponds to 63 Hz
        // using 0.8f here to try to even the volume difference with non compensated oscillators.
                               0.8f, // 1.f = full compensation, 0.f = no compensation
                               30.f);
      e.setAngleIncrements(freq_to_angle_increment(freq, sample_rate));
      return true;
    }
  };

  template<
    int nOuts,
    typename AE,
    SynchronizePhase Sync,
    DefaultStartPhase Phase,
    HandleNoteOff handle_note_off,
    int nVoices = 32, // using 32 voices to support long releases
    typename Initializer = NoopElementInitializer
  >
  using Synth = ImplCRTP <
    nOuts,
    AE,
    Sync,
    Phase,
    handle_note_off,
    SynthImpl,
    nVoices,
    Initializer
  >;

} // NS imajuscule::audio::sine
