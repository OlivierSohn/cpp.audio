namespace imajuscule::audio::sine {

  struct SynthImpl {

    static constexpr int32_t get_xfade_length() { return 401; }
    static constexpr float get_gain() { return 1.f; };

    template<typename Element>
    bool setupAudioElement(float freq, Element & e)
    {
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
  private:
    float half_tone = compute_half_tone(1.f);
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
    typename NoteOffEvent
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
    32
  >;

} // NS imajuscule::audio::sine
