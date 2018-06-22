// 'va' in the name 'vasine' stand for "Volume-adjusted" sine

namespace imajuscule {
    namespace audio {
        namespace vasine {

          
          struct SynthImpl {

                int32_t get_xfade_length() const { return xfade_length; }
                void set_xfade_length(int32_t l) { xfade_length = l; }

                static constexpr float get_gain() { return 1.f; };

                // the caller is responsible for taking the out lock if needed
                template<typename MonoNoteChannel, typename F, typename CS, typename OutputData, typename Chans>
                bool onStartNote(float velocity, MonoNoteChannel & c, F & shouldKeyRelease, CS & cs, OutputData & out, Chans & chans) {
                    using Request = typename OutputData::Request;

                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto freq = to_freq(tunedNote-Do_midi, half_tone);

                    auto & osc = c.elem;
                    osc.algo.setLoudnessParams(5,
                                               1.f,
                                               30.f);
                    osc.algo.setAngleIncrements(freq_to_angle_increment(freq));
                  
                    osc.oneShot = [&c, &cs] () { setPhase(c,cs); };

                    // The caller is responsible for:
                    // - taking the out lock if needed
                    // - growing the channel request queue if needed
                    return chans.playComputableNoLock(
                                    out, *c.channel, osc.fCompute(shouldKeyRelease),
                                                   Request{
                                                       &osc.buffer->buffer[0],
                                                       velocity,
                                                       // e.noteOn.length is always 0, we must rely on noteOff
                                                       std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
                                                   });
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
              32>; // using 32 voices to support long releases
        }
    }
} // namespaces
