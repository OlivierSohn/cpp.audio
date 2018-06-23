namespace imajuscule::audio::sine {

  struct SynthImpl {

    static constexpr int32_t get_xfade_length() { return 401; }
    static constexpr float get_gain() { return 1.f; };

    // the caller is responsible for taking the out lock if needed
    template<typename MonoNoteChannel, typename CS, typename Chans>
    std::pair<std::function<void(void)>,std::function<bool(Chans&,int)>>
    onStartNote(float freq, MonoNoteChannel & c, CS & cs, Chans & chans) {
      auto & osc = c.elem;
      osc.algo.setAngleIncrements(freq_to_angle_increment(freq));
      return {
        [&c, &cs]() {
          setPhase(c,cs);
          c.elem.onKeyPressed();
        }
        ,{}
      };
    }
  private:
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
    32
  >;

} // NS imajuscule::audio::sine
