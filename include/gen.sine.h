// The name is historical : initially we were using only sine oscillators with
// this synth, but it is suited to all kinds of oscillators
namespace imajuscule::audio::sine {

  struct SynthImpl {
    static constexpr float get_xfade_length() { return 0.009f; }
    static constexpr float get_gain() { return 1.f; };

    template<typename Element, int nAudioOut>
    bool setupAudioElement(float freq,
                           Element & e,
                           int const sample_rate,
                           Volumes<nAudioOut> & vol)
    {
      e.algo.setLoudnessParams(sample_rate,
                               5, // corresponds to 63 Hz
        // using 0.8f here to try to even the volume difference with non compensated oscillators.
                               0.8f, // 1.f = full compensation, 0.f = no compensation
                               30.f);
      e.algo.setAngleIncrements(freq_to_angle_increment(freq, sample_rate));
      vol = Volumes<nAudioOut>(Element::baseVolume);
      return true;
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
    int nVoices = 32, // using 32 voices to support long releases
    typename Initializer = NoopElementInitializer
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
    SynthImpl,
    nVoices,
    Initializer
  >;

} // NS imajuscule::audio::sine
