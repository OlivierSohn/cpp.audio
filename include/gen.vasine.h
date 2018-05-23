// 'va' in the name 'vasine' stand for "Volume-adjusted" sine

namespace imajuscule {
    namespace audio {
        namespace vasine {

            struct SynthImpl {

                int get_xfade_length() const { return xfade_length; }
                void set_xfade_length(int l) { xfade_length = l; }

                static constexpr float get_gain() { return 1.f; };

                // the caller is responsible for taking the out lock if needed
                template<typename MonoNoteChannel, typename OutputData>
                void onStartNote(float velocity, Phase phase, MonoNoteChannel & c, OutputData & out) {
                    using Request = typename OutputData::Request;

                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto freq = to_freq(tunedNote-Do_midi, half_tone);

                    auto & osc = c.elem;
                    osc.algo.setLoudnessParams(5,
                                               1.f,
                                               30.f);
                    osc.algo.setAngleIncrements(freq_to_angle_increment(freq));
                    setPhase(phase, osc.algo);

                    // the caller is responsible for taking the out lock if needed
                    auto res = out.playGenericNoLock(c.channel,
                                    std::make_pair(std::ref(osc),
                                                   Request{
                                                       &osc.buffer[0],
                                                       velocity,
                                                       // e.noteOn.length is always 0, we must rely on noteOff
                                                       std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
                                                   }));
                    if(!res) {
                      MIDI_LG(ERR,"failed to play");
                    }
                }
              private:
                int xfade_length = 401;
                float half_tone = compute_half_tone(1.f);
            };

            template<
            int nOuts,
            XfadePolicy xfade_policy_,
            typename MNC,
            bool close_channel_on_note_off,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent
            >
            using Synth = ImplCRTP <nOuts, xfade_policy_, MNC, close_channel_on_note_off, EventIterator, NoteOnEvent, NoteOffEvent, SynthImpl>;
        }
    }
} // namespaces
