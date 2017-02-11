
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
        using interleaved_buf_t = cacheline_aligned_allocated::vector<float>;

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
                    A(v.size() == size_interleaved);
                });
                params = std::move(o.params);
                A(params.size() == NPARAMS);
            }
            
            void setParameter(int index, float value, int sampleOffset /* not supported yet */) {
                A(index < params.size());
                params[index] = value;
            }
            
            void useProgram(int index) {
                auto const & p = getProgram(index);
                A(p.params.size() == NPARAMS);
                for (auto i = 0; i < NPARAMS; i++) {
                    params[i] = p.params[i];
                }
            }

        protected:
            Parameters params;
            InterleavedBuffer interleaved;
            float half_tone = compute_half_tone(tune_stretch);

        };

        template<
        
        int nAudioOut,
        XfadePolicy xfade_policy,
        typename MNC,
        bool close_channel_on_note_off,
        typename EventIterator,
        typename NoteOnEvent,
        typename NoteOffEvent,
        typename Base
        
        >
        struct ImplCRTP : public Base {
            
            using MonoNoteChannel = MNC;
            
            using Base::get_xfade_length;
            using Base::onStartNote;
            
            using Event = typename EventIterator::object;
            
            static constexpr auto n_max_voices = 8;
            static constexpr auto n_channels_per_note = MonoNoteChannel::n_channels_per_note;
            
            // notes played in rapid succession can have a common audio interval during xfades
            // even if their noteOn / noteOff intervals are disjoint.
            // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well':
            static constexpr auto n_max_simultaneous_notes_per_voice = 2;
            static constexpr auto n_channels = n_channels_per_note * n_max_voices * n_max_simultaneous_notes_per_voice;
            
            using OutputData = outputDataBase<nAudioOut, xfade_policy, AudioOutPolicy::Slave>;
            
            // Note: Logic Audio Express 9 calls this when two projects are opened and
            // the second project starts playing, so we are not in an "initialized" state.
            void allNotesOff(OutputData & out) {
                MIDI_LG(INFO, "all notes off :");
                
                static constexpr auto allNotesOff_xfade_len = 1401;
                for(auto & c : channels) {
                    if(c.close(out, CloseMode::XFADE_ZERO, allNotesOff_xfade_len)) {
                        LG(INFO, " x %d", c.pitch);
                    }
                }
            }
            
            void allSoundsOff(OutputData & out) {
                MIDI_LG(INFO, "all sounds off :");
                for(auto & c : channels) {
                    if(c.close(out, CloseMode::NOW)) {
                        LG(INFO, " x %d", c.pitch);
                    }
                }
            }
            
            template<typename FILTER_MONO_NOTE_CHANNELS, typename OutputData>
            onEventResult onEvent(EventIterator it, FILTER_MONO_NOTE_CHANNELS filter, OutputData & out) {
                Event e;
                it.dereference(e);

                if(e.type == Event::kNoteOnEvent) {
                    // this case is handled by the wrapper...
                    A(e.noteOn.velocity > 0.f );
                    // ... in case it were not handled by the wrapper we would need to do:
                    // return noteOff(e.noteOn.pitch);
                    
                    if(auto c = editAudioElementContainer_if(channels, filter))
                    {
                        return startNote(*c, e.noteOn, out);
                    }
                    return onDroppedNote(e.noteOn.pitch);
                }
                else if(e.type == Event::kNoteOffEvent) {
                    return noteOff(e.noteOff.pitch, out);
                }
                return onEventResult::UNHANDLED;
            }
            
            
        private:
            onEventResult startNote(MonoNoteChannel & c, NoteOnEvent const & e, OutputData & out) {
                
                MIDI_LG(INFO, "on  %d", e.pitch);
                
                int xf_len;
                float initial_volume;
                if(xfade_policy==XfadePolicy::SkipXfade) {
                    initial_volume = 0.f;
                    xf_len= 0 ;
                }
                else {
                    initial_volume = 1.f;
                    xf_len = get_xfade_length();
                }
                
                if(!c.open(out, initial_volume, xf_len)) {
                    return onDroppedNote(e.pitch);
                }
                c.pitch = e.pitch;
                c.tuning = e.tuning;
                
                onStartNote(e.velocity, c, out);
                return onEventResult::OK;
            }
            
            onEventResult noteOff(uint8_t pitch, OutputData & out) {
                MIDI_LG(INFO, "off %d", pitch);
                if(!close_channel_on_note_off) {
                    // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
                    // but close method sets the channel to -1 so it's impossible to fade it to zero
                    // afterwards using close(), so instead we don't do anything here
                    return onEventResult::OK;
                }
                auto len = get_xfade_length();
                for(auto & c : channels) {
                    if(c.pitch != pitch) {
                        continue;
                    }
                    if(!c.close(out, CloseMode::XFADE_ZERO, len)) {
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
            using OutputData = typename T::OutputData;
            using ProcessData = typename T::ProcessData;
            
            Wrapper(int nOrchestratorsMax) :
            out{n_channels, nOrchestratorsMax}
            {}
            
            void doProcessing (ProcessData& data) {
                return plugin.doProcessing(data, out);
            }
            
            void allNotesOff() {
                return plugin.allNotesOff(out);
            }
            
            void allSoundsOff() {
                return plugin.allSoundsOff(out);
            }
            
            OutputData out;
            T plugin;
        };
    }
} // namespaces
