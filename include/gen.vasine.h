// 'va' in the name 'vasine' stand for "Volume-adjusted" sine

namespace imajuscule::audio::vasine {

  struct SynthImpl {

    int32_t get_xfade_length() const { return xfade_length; }

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

    template<typename Chans, typename MonoNoteChannel, typename CS>
    std::function<bool(Chans&,int)> onStartNote(MonoNoteChannel & c, CS & cs)
    {
      setPhase(c,cs);
      return {};
    }

  private:
    int32_t xfade_length = 401;
    float half_tone = compute_half_tone(1.f);
  };

  template<
    AudioOutPolicy outPolicy,
    int nOuts,
    XfadePolicy xfade_policy_,
    typename AE,
    bool close_channel_on_note_off,
    typename EventIterator,
    typename NoteOnEvent,
    typename NoteOffEvent
  >
  using Synth = ImplCRTP <
    outPolicy,
    nOuts,
    xfade_policy_,
    AE,
    close_channel_on_note_off,
    EventIterator,
    NoteOnEvent,
    NoteOffEvent,
    SynthImpl,
    32 // using 32 voices to support long releases
  >;

} // NS imajuscule::audio::vasine
