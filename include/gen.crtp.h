
#ifndef NDEBUG
#define DO_LOG_MIDI 1
#else
#define DO_LOG_MIDI 0
#endif

#if DO_LOG_MIDI
#define MIDI_LG( x , y, ...) LG( x , y ,##__VA_ARGS__)
#else
#define MIDI_LG(...)
#endif

namespace imajuscule {
    namespace audio {
        using interleaved_buf_t = a64::vector<float>;

        template<typename T>
        struct ForEach;

        template<>
        struct ForEach<interleaved_buf_t> {
            template<typename F>
            static void run(interleaved_buf_t & b, F f) {
                f(b);
            };
        };

        template<int N>
        struct ForEach<std::array<interleaved_buf_t, N>> {
            template<typename F>
            static void run(std::array<interleaved_buf_t, N> & a, F f) {
                for(auto & b : a) {
                    f(b);
                }
            };
        };

        template<typename InterleavedBuffer, typename F>
        void forEach(InterleavedBuffer & b, F f) {
            ForEach<InterleavedBuffer>::run(b, f);
        }

        template<

        int NParams,
        typename Parameters,
        typename InterleavedBuffer,
        int SizeInterleaved,
        typename ProcessData_

        >
        struct Impl {
            using ProcessData = ProcessData_;

            static constexpr auto tune_stretch = 1.f;

            virtual Program const & getProgram(int i) const = 0;
            virtual int countPrograms() const = 0;

            virtual ~Impl() = default;

            static constexpr auto NPARAMS = NParams;
            static constexpr auto size_interleaved = SizeInterleaved;

            void initializeSlow() {
                forEach(interleaved, [](interleaved_buf_t & v) {
                    v.resize(size_interleaved, 0.f);
                });
                params.resize(NPARAMS);
            }

            void initializeSteal(Impl && o) {
                interleaved = std::move(o.interleaved);
                forEach(interleaved, [](interleaved_buf_t & v) {
                    std::fill(v.begin(), v.end(), 0.f);
                    Assert(v.size() == size_interleaved);
                });
                params = std::move(o.params);
                Assert(params.size() == NPARAMS);
            }

            void setParameter(int index, float value, int sampleOffset = 0 /* not supported yet */) {
                Assert(index < params.size());
                params[index] = value;
            }

            void useProgram(int index) {
                auto const & p = getProgram(index);
                MIDI_LG(INFO, "with program %d of %d", index, countPrograms());
                Assert(p.params.size() == NPARAMS);
                for (auto i = 0; i < NPARAMS; i++) {
                    params[i] = p.params[i];
                }
            }

        protected:
            Parameters params;
            InterleavedBuffer interleaved;
            float half_tone = compute_half_tone(tune_stretch);

        };

        // When another oscillator with the same frequency exists,
        // we synchronize the phase to avoid cancellations.
        //
        // Else, we randomize the phase to keep some nice randomness.
        struct Phase {
          Phase(bool deterministic, float value) : deterministic(deterministic), value(value) {}
          bool isDeterministic() const { return deterministic; }
          float getDeterministicValue() const { Assert(deterministic); return value; }
        private:
          bool deterministic;
          float value;
        };
        static inline Phase mkDeterministicPhase(float v) { return Phase{true,v}; }
        static inline Phase mkNonDeterministicPhase() { return Phase{false,{}}; }

        template<typename T>
        void setPhase (Phase const & phase, T&algo) {
          auto angle =
            phase.isDeterministic() ?
            phase.getDeterministicValue() :
            std::uniform_real_distribution<float>{-1.f, 1.f}(mersenne<SEEDED::Yes>());
          algo.setAngle(angle);
        }

        template<

        int nOuts,
        XfadePolicy xfade_policy_,
        typename MNC,
        bool close_channel_on_note_off,
        typename EventIterator,
        typename NoteOnEvent,
        typename NoteOffEvent,
        typename Base

        >
        struct ImplCRTP : public Base {

            static constexpr auto nAudioOut = nOuts;
            static constexpr auto xfade_policy = xfade_policy_;

            using MonoNoteChannel = MNC;

            using Base::get_xfade_length;
            using Base::get_gain;
            using Base::onStartNote;

            using Event = typename EventIterator::object;

            static constexpr auto n_max_voices = 8;
            static constexpr auto n_channels_per_note = MonoNoteChannel::n_channels_per_note;

            // notes played in rapid succession can have a common audio interval during xfades
            // even if their noteOn / noteOff intervals are disjoint.
            // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well':
            static constexpr auto n_max_simultaneous_notes_per_voice = 2;
            static constexpr auto n_channels = n_channels_per_note * n_max_voices * n_max_simultaneous_notes_per_voice;

            // Note: Logic Audio Express 9 calls this when two projects are opened and
            // the second project starts playing, so we are not in an "initialized" state.
            template<WithLock l, typename OutputData>
            void allNotesOff(OutputData & out) {
                MIDI_LG(INFO, "all notes off :");

                constexpr auto allNotesOff_xfade_len = 1401;
                for(auto & c : channels) {
                    if(c.template close<l>(out, CloseMode::XFADE_ZERO, allNotesOff_xfade_len)) {
                        LG(INFO, " x %d", c.pitch);
                    }
                }
            }

            template<WithLock l, typename OutputData>
            void allSoundsOff(OutputData & out) {
                MIDI_LG(INFO, "all sounds off :");
                for(auto & c : channels) {
                    if(c.template close<l>(out, CloseMode::NOW)) {
                        LG(INFO, " x %d", c.pitch);
                    }
                }
            }

            // TODO can we remove filter argument? we own the channels that are filtered so maybe we could
            // check if they are ready or not.
            template<typename OutputData>
            onEventResult onEvent2(Event const & e, OutputData & out) {
                if(e.type == Event::kNoteOnEvent) {
                    // this case is handled by the wrapper... else we need to do a noteOff
                    Assert(e.noteOn.velocity > 0.f );
                    {
                        typename OutputData::Locking L(out.get_lock());

                        if(auto c = editAudioElementContainer_if(channels
                                                                 , [](auto & c) { return c.elem.isInactive() && c.closed(); }))
                        {
                            return this->template startNote(*c, e.noteOn, out);
                        }
                    }
                    return onDroppedNote(e.noteOn.pitch);
                }
                else if(e.type == Event::kNoteOffEvent) {
                    return noteOff(e.noteOff.pitch, out);
                }
                return onEventResult::UNHANDLED;
            }

            bool opened() const {
                for(auto const & c : channels) {
                    if(c.opened()) {
                        return true;
                    }
                }
                return false;
            }
        private:
            // precondition : if needed the lock of out was taken by a caller
            template<typename OutputData>
            onEventResult startNote(MonoNoteChannel & c, NoteOnEvent const & e, OutputData & out) {

                MIDI_LG(INFO, "on  %d", e.pitch);

                int xf_len;
                float initial_volume;
                bool gain_is_set = true;
                if(xfade_policy==XfadePolicy::SkipXfade) {
                    initial_volume = 0.f;
                    xf_len= 0 ;
                    gain_is_set = false;
                }
                else {
                    initial_volume = get_gain();
                    xf_len = get_xfade_length();
                }

                // TODO check if any other active channel has the same freqency, and if so,
                // pass the phase information to 'startNote'.
                //findPhaseForFrequency
                auto phase = mkNonDeterministicPhase();
                for(auto const & o:channels) {
                  if(o.pitch == e.pitch && o.tuning==e.tuning && !o.closed() && !o.elem.isInactive()) {
                    phase = mkDeterministicPhase(o.elem.angle());
                    break;
                  }
                }

                // using WithLock::No : if needed, the caller is responsible to take the out lock.
                if(!c.template open<WithLock::No>(out, initial_volume, xf_len)) {
                    return onDroppedNote(e.pitch);
                }
                c.pitch = e.pitch;
                c.tuning = e.tuning;

                c.elem.forgetPastSignals();

                if(!gain_is_set) {
                    c.toVolume(out, get_gain(), get_xfade_length());
                }

                onStartNote(e.velocity, phase, c, out);
                return onEventResult::OK;
            }

            template<typename OutputData>
            onEventResult noteOff(uint8_t pitch, OutputData & out) {
                if(!close_channel_on_note_off) {
                    MIDI_LG(INFO, "off (ignored) %d", pitch);
                    // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
                    // but close method sets the channel to -1 so it's impossible to fade it to zero
                    // afterwards using close(), so instead we don't do anything here
                    return onEventResult::OK;
                }
                MIDI_LG(INFO, "off %d", pitch);
                auto len = get_xfade_length();
                for(auto & c : channels) {
                    if(c.pitch != pitch) {
                        continue;
                    }
                    if(!c.template close<OutputData::DefLock>(out, CloseMode::XFADE_ZERO, len)) {
                        continue;
                    }
                    // the oscillator is still used for the crossfade,
                    // it will stop being used in the call to openChannel when that channel is finished closing
                    return onEventResult::OK;
                }
                // it could be that the corresponding noteOn was skipped
                // because too many notes where played at that time
                return onDroppedNote(pitch);
            }

        protected:
            onEventResult onDroppedNote(uint8_t pitch) {
                MIDI_LG(WARN, "dropped note '%d'", pitch);
                return onEventResult::DROPPED_NOTE;
            }

            // members
            std::array<MonoNoteChannel, n_channels> channels;
        };

        template<typename T>
        struct Wrapper {
            static constexpr auto n_channels = T::n_channels;
            static constexpr auto with_lock = imajuscule::WithLock::No;

            using ProcessData = typename T::ProcessData;

            using OutputData = outputDataBase<T::nAudioOut, T::xfade_policy, AudioOutPolicy::Slave>;

            Wrapper(int nOrchestratorsMax) :
            out{n_channels, nOrchestratorsMax}
            {
                out.dontUseConvolutionReverbs();
            }

            void doProcessing (ProcessData& data) {
                return plugin.doProcessing(data, out);
            }

            void allNotesOff() {
                return plugin.template allNotesOff<with_lock>(out);
            }

            void allSoundsOff() {
                return plugin.template allSoundsOff<with_lock>(out);
            }

            OutputData out;
            T plugin;
        };
    }
} // namespaces
