namespace imajuscule {
    namespace audio {
        namespace sine {

            struct SynthImpl {

                static constexpr int get_xfade_length() { return 401; }
                static constexpr float get_gain() { return 1.f; };

                // the caller is responsible for taking the out lock if needed
                template<typename MonoNoteChannel, typename OutputData>
                void onStartNote(float velocity, MonoNoteChannel & c, OutputData & out) {
                    using Request = typename OutputData::Request;

                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto freq = to_freq(tunedNote-Do_midi, half_tone);
                    auto channel = c.channels[0];

                    auto & osc = c.elem;
                    // we could loudness-adjust using a VolumeAdjusted:
                    /*
                    osc.algo.getOsc().setLoudnessParams(value<LOUDNESS_REF_FREQ_INDEX>(),
                                                        value<LOUDNESS_COMPENSATION_AMOUNT>(),
                                                        denorm<LOUDNESS_LEVEL>());
                    */
                    osc.algo.setAngleIncrements(freq_to_angle_increment(freq));


                    // the caller is responsible for taking the out lock if needed
                    out.playGenericNoLock(channel,
                                    std::make_pair(std::ref(osc),
                                                   Request{
                                                       &osc.buffer[0],
                                                       velocity,
                                                       // e.noteOn.length is always 0, we must rely on noteOff
                                                       std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
                                                   }));
                }
              private:
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