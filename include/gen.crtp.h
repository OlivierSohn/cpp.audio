
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
        typename AE,
        bool handle_note_off,
        typename EventIterator,
        typename NoteOnEvent,
        typename NoteOffEvent,
        typename Base,
        int n_max_voices = 8

        >
        struct ImplCRTP : public Base {

            static constexpr auto nAudioOut = nOuts;
            static constexpr auto xfade_policy = xfade_policy_;

            using MonoNoteChannel = MonoNoteChannel<AE, Channel<nOuts, xfade_policy_, MaxQueueSize::One>>;

            using Base::get_xfade_length;
            using Base::get_gain;
            using Base::onStartNote;

            using Event = typename EventIterator::object;

            static constexpr auto n_channels_per_note = 1;

            // notes played in rapid succession can have a common audio interval during xfades
            // even if their noteOn / noteOff intervals are disjoint.
            // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well'.
            // TODO if the release time is long, and notes are consecutive and short, this number
            // should be increased.
            static constexpr auto n_max_simultaneous_notes_per_voice = 2;
            static constexpr auto n_channels = n_channels_per_note * n_max_voices * n_max_simultaneous_notes_per_voice;

            template <class... Args>
          ImplCRTP(Args&&... args) : channels{std::forward<Args>(args)...} {}

            template<typename ChannelsT>
            bool initialize(ChannelsT & chans) {
                for(auto & c : channels) {
                    // using WithLock::No : since we own all these channels and they are not playing, we don't need
                    // to take the audio lock.
                    if(!c.template open<WithLock::No>(chans, 0.f)) {
                        return false;
                    }
                }
                return true;
            }

          ~ImplCRTP() {
            finalize();
          }
          
          void finalize() {
                for(auto & c : channels) {
                    if(c.channel == nullptr) {
                        continue;
                    }
                    c.reset();
                    c.channel = nullptr;
                }
            }

            // returns true if at least one channel has an active enveloppe.
            //
            // currently called only in a single thread context.
            bool areEnvelopesFinished() const {
                for(auto & c : channels) {
                  if(!c.elem.isEnvelopeFinished()) {
                      return false;
                  }
                }
                return true;
            }

            // Note: Logic Audio Express 9 calls this when two projects are opened and
            // the second project starts playing, so we are not in an "initialized" state.
            //
            // currently called only in a single thread context.
            void allNotesOff() {
                MIDI_LG(INFO, "all notes off");

                for(auto & c : channels) {
                    c.elem. template onKeyReleased();
                }
            }

            //
            // currently called only in a single thread context.
            void allSoundsOff() {
                MIDI_LG(INFO, "all sounds off");
                for(auto & c : channels) {
                    c.elem. template onKeyReleased();
                }
            }

            template<typename Out, typename Chans>
            onEventResult onEvent2(Event const & e, Out & out, Chans & chans) {
              if(e.type == Event::kNoteOnEvent)
              {
                Assert(e.noteOn.velocity > 0.f ); // this case is handled by the wrapper... else we need to do a noteOff
                MIDI_LG(INFO, "on  %d", e.noteOn.pitch);
                {
                  // We will reset the channel used, so the request queue will be guaranteed
                  // to have room for a new element : no dynamic allocation will happen
                  // so we can take the lock.
                  typename Out::LockFromNRT L(out.get_lock());

                  MonoNoteChannel * channel = nullptr;
                  auto phase = mkNonDeterministicPhase();

                  for(auto & o:channels) {
                    // Find a channel where the element has finished
                    if(o.elem.isEnvelopeFinished()) {
                      if(channel) {
                        continue;
                      }
                      // Setup the element to start a new note
                      o.elem.setEnvelopeCharacTime(get_xfade_length());
                      o.elem.forgetPastSignals();
                      o.elem.onKeyPressed();

                      // unqueue the (potential) previous request, else an assert fails
                      // when we enqueue the next request, because it's already queued.
                      o.reset();
                      if constexpr (xfade_policy == XfadePolicy::UseXfade) {
                        o.channel->set_xfade(get_xfade_length());
                      }
                      o.channel->setVolume(get_gain());
                      o.pitch = e.noteOn.pitch;
                      o.tuning = e.noteOn.tuning;

                      channel = &o;
                    }
                    // To prevent phase cancellation, the phase of the new note will be
                    // coherent with the phase of any active channel that plays a note at the same frequency.
                    else if(!phase.isDeterministic() && o.pitch == e.noteOn.pitch && o.tuning==e.noteOn.tuning) {
                      phase = mkDeterministicPhase(o.elem.angle());
                    }
                  }

                  if(channel) {
                    onStartNote(e.noteOn.velocity, phase, *channel, out, chans);
                    return onEventResult::OK;
                  }
                }
                // it's important to call onDroppedNote only once the lock has been released.
                return onDroppedNote(e.noteOn.pitch);
              }
              else if(e.type == Event::kNoteOffEvent)
              {
                if(!handle_note_off) {
                    MIDI_LG(INFO, "off (ignored) %d", e.noteOff.pitch);
                    // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
                    // but close method sets the channel to -1 so it's impossible to fade it to zero
                    // afterwards using close(), so instead we don't do anything here
                    return onEventResult::OK;
                }
                MIDI_LG(INFO, "off %d", e.noteOff.pitch);
                {
                  typename Out::LockFromNRT L(out.get_lock());

                  // We can have multiple notes with the same pitch, and different durations.
                  // Hence, here we just close the first opened channel with matching pitch.
                  for(auto & c : channels) {
                      if(c.pitch != e.noteOff.pitch) {
                          continue;
                      }
                      if(!c.elem.onKeyReleased()) {
                          // this one has already been sent a key released
                          continue;
                      }
                      return onEventResult::OK;
                  }
                }

                // The corresponding noteOn was skipped,
                // because too many notes were being played at the same time
                return onDroppedNote(e.noteOff.pitch);
              }
              return onEventResult::UNHANDLED;
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

            using OutputData = outputDataBase<
              AudioOutPolicy::Slave,
              Channels<T::nAudioOut, T::xfade_policy, T::max_queue_size, AudioOutPolicy::Slave>
              >;

            template <class... Args>
            Wrapper(int nOrchestratorsMax, Args&&... args) :
            plugin(std::forward<Args>(args)...),
            out{
                GlobalAudioLock<AudioOutPolicy::Slave>::get(),
                n_channels,
                nOrchestratorsMax
            }
            {
                dontUseConvolutionReverbs(out);
                plugin.template initialize(out.getChannels());
            }

            void doProcessing (ProcessData& data) {
                return plugin.doProcessing(data, out, out.getChannels());
            }

            void allNotesOff() {
                return plugin.allNotesOff();
            }

            void allSoundsOff() {
                return plugin.allSoundsOff();
            }

            OutputData out;
            T plugin;
        };
    }
} // namespaces
