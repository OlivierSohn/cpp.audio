// 'va' in the name 'vasine' stand for "Volume-adjusted" sine

namespace imajuscule::audio::vasine {

  struct SynthImpl {

    int32_t get_xfade_length() const { return xfade_length; }
    void set_xfade_length(int32_t l) { xfade_length = l; }

    static constexpr float get_gain() { return 1.f; };

    // the caller is responsible for taking the out lock if needed
    template<typename MonoNoteChannel, typename CS, typename Chans>
    std::pair<std::function<void(void)>,std::function<bool(Chans&,int)>>
    onStartNote(float freq, MonoNoteChannel & c, CS & cs, Chans & chans)
    {
      using Request = typename Chans::Request;

      auto & osc = c.elem;
      osc.algo.setLoudnessParams(5,
                                 1.f,
                                 30.f);
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
