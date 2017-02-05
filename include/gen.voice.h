
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
        namespace voice {            
            
            enum ImplParams {
                MODE,
                XFADE_LENGTH,
                FREQ_SCATTER,
                D1,
                D2,
                PHASE_RATIO1,
                PHASE_RATIO2,
                HARMONIC_ATTENUATION,
                LENGTH,
                INTERPOLATION,
                
                NPARAMS
            };
            
#include "pernamespace.implparams.h"
            
            template<> struct Limits<XFADE_LENGTH> {
                static constexpr auto m = 3;
                static constexpr auto M = 2001; };
            
            template<> struct Limits<FREQ_SCATTER> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO1> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO2> : public NormalizedParamLimits {};
            
            template<> struct Limits<D1> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };
            template<> struct Limits<D2> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };
            
            template<> struct Limits<HARMONIC_ATTENUATION> {
                static const float m;
                static const float M; };
            
            template<> struct Limits<LENGTH> {
                static const float m;
                static const float M; };
            
            
            template<typename Parameters, typename ProcessData>
            struct ImplBase {
                using Params = ImplParams;
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                static constexpr auto tune_stretch = 1.f;
                
            protected:
                using interleaved_buf_t = cacheline_aligned_allocated::vector<float>;
                using SoundEngine = SoundEngine<imajuscule::Logger, UpdateMode::FORCE_SOUND>;
                using Mode = SoundEngine::Mode;
                
                static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
                
                static constexpr auto size_interleaved = size_interleaved_one_cache_line;

            public:
                static std::vector<ParamSpec> const & getParamSpecs() {
                    static std::vector<ParamSpec> params_spec = {
                        {"Mode", SoundEngine::ModeTraversal},
                        {"Crossfade length", static_cast<float>(Limits<XFADE_LENGTH>::m), static_cast<float>(Limits<XFADE_LENGTH>::M) }, // couldn't make it work with nsteps = max-min / 2
                        {"Frequency scatter", Limits<FREQ_SCATTER>::m, Limits<FREQ_SCATTER>::M },
                        {"D1", Limits<D1>::m, Limits<D1>::M },
                        {"D2", Limits<D2>::m, Limits<D2>::M},
                        {"Phase ratio 1", Limits<PHASE_RATIO1>::m, Limits<PHASE_RATIO1>::M},
                        {"Phase ratio 2", Limits<PHASE_RATIO2>::m, Limits<PHASE_RATIO2>::M},
                        {"Harmonic attenuation", Limits<HARMONIC_ATTENUATION>::m, Limits<HARMONIC_ATTENUATION>::M},
                        {"Length", Limits<LENGTH>::m, Limits<LENGTH>::M},
                        {"Interpolation", itp::interpolation_traversal()}
                    };
                    return params_spec;
                }
                
                static Program::ARRAY make(Mode m,
                                           int xfade,
                                           float freq_scat,
                                           float d1, float d2,
                                           float phase_ratio1, float phase_ratio2,
                                           float harmonic_attenuation,
                                           float length,
                                           itp::interpolation i) {
                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    A(b);
                    int mode_index = 0;
                    b = SoundEngine::ModeTraversal.valToRealValueIndex(m, mode_index);
                    A(b);

                    return {{
                        static_cast<float>(mode_index),
                        normalize<XFADE_LENGTH>(xfade),
                        /*normalize<FREQ_SCATTER>*/(freq_scat),
                        /*normalize<D1>*/(d1),
                        /*normalize<D2>*/(d2),
                        normalize<PHASE_RATIO1>(phase_ratio1),
                        normalize<PHASE_RATIO2>(phase_ratio2),
                        normalize<HARMONIC_ATTENUATION>(harmonic_attenuation),
                        normalize<LENGTH>(length),
                        static_cast<float>(itp_index)
                    }};
                }
                static Programs const & getPrograms() {
                    static ProgramsI ps {{
                        {"First",
                            make(Mode::MARKOV, 401, .0f, 12.f, 24.f, 0.f, 0.f, .95f, 100.f, itp::EASE_INOUT_QUART)
                        },
                    }};
                    return ps.v;
                }
                
                // members
                Parameters params;
                
            protected:
                float half_tone = compute_half_tone(tune_stretch);
                
                interleaved_buf_t interleaved;
                
                // methods
            public:
                ImplBase()
                {}
                
                void initializeSlow() {
                    interleaved.resize(size_interleaved, 0.f);
                    params.resize(NPARAMS);
                }
                
                void initializeSteal(ImplBase && o) {
                    interleaved = std::move(o.interleaved);
                    std::fill(interleaved.begin(), interleaved.end(), 0.f);
                    A(interleaved.size() == size_interleaved);
                    params = std::move(o.params);
                    A(params.size() == NPARAMS);
                }
                
                void setParameter(int index, float value, int sampleOffset /* not supported yet */) {
                    A(index < params.size());
                    params[index] = value;
                }
                
                void useProgram(int index) {
                    A(index < getPrograms().size());
                    auto & p = getPrograms()[index];
                    for (auto i = 0; i < NPARAMS; i++) {
                        params[i] = p.params[i];
                    }
                }
                
                virtual void doProcessing (ProcessData& data) = 0;
                virtual void allNotesOff() = 0;
                virtual void allSoundsOff() = 0;
                virtual ~ImplBase() = default;
                
            };
            
            
            template<
            
            int nAudioOut,
            
            // to use it in context of Grid3d, we need to have an implementation of these:
            typename Parameters,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename ProcessData,
            
            typename Base = ImplBase<Parameters, ProcessData>
            
            >
            struct Impl_ : public Base
            {
                using Event = typename EventIterator::object;
                using Programs = imajuscule::audio::Programs;

                using Params = ImplParams;
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                static constexpr auto size_interleaved = Base::size_interleaved;
                static constexpr auto n_frames_interleaved = size_interleaved / nAudioOut;
                static_assert(n_frames_interleaved * nAudioOut == size_interleaved, ""); // make sure we don't waste space
                static constexpr auto size_interleaved_one_cache_line = Base::size_interleaved_one_cache_line;
                
                using Base::half_tone;
                using Base::params;
                using Base::interleaved;

                using OutputData = outputDataBase<nAudioOut, XfadePolicy::UseXfade, NoOpLock, PostProcess::NONE>;
                using SoundEngine = typename Base::SoundEngine;
                using Mode = typename SoundEngine::Mode;
                
                static constexpr auto n_max_voices = 8;
                // notes played in rapid succession can have a common audio interval during xfades
                // even if their noteOn / noteOff intervals are disjoint.
                // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well':
                static constexpr auto n_max_simultaneous_notes_per_voice = 2;
                static constexpr auto n_channels_per_note = 2;
                static constexpr auto n_channels = n_channels_per_note * n_max_voices * n_max_simultaneous_notes_per_voice;
                
                struct EngineAndRamps {
                    using audioElt = audioelement::FreqRamp<float>;
                    
                    EngineAndRamps() : engine{[this]()-> audioElt* {
                        for(auto & r : ramps) {
                            if(r.isInactive()) {
                                return &r;
                            }
                        }
                        return nullptr;
                    }} {}
                    
                    SoundEngine engine;
                    std::array<audioElt, 3> ramps;
                };
                
                using MonoNoteChannel = MonoNoteChannel<n_channels_per_note, EngineAndRamps>;

                //
                //  types
                //
                
            private:
                OutputData out = {n_channels};
                std::array<MonoNoteChannel, n_channels> channels;

                //
                //  methods
                //
            private:
                onEventResult onEvent(Event const & e) {
                    if(e.type == Event::kNoteOnEvent) {
                        // this case is handled by the wrapper...
                        A(e.noteOn.velocity > 0.f );
                        // ... in case it were not handled by the wrapper we would need to do:
                        // return noteOff(e.noteOn.pitch);
                        
                        if(auto c = editAudioElementContainer_if(channels,
                                                                 [](auto & c) -> bool {
                                                                     for(auto & e : c.elem.ramps) {
                                                                         if(!e.isInactive()) {
                                                                             return false;
                                                                         }
                                                                     }
                                                                     // here we know that all elements are inactive
                                                                     // but if the channel has not been closed yet
                                                                     // we cannot use it (if we want to enable that,
                                                                     // we should review the way note on/off are detected,
                                                                     // because it will probably cause bugs)
                                                                     return c.closed();
                                                                 }))
                        {
                            return startNote(*c, e.noteOn);
                        }
                        return onDroppedNote(e.noteOn.pitch);
                    }
                    
                    if(e.type == Event::kNoteOffEvent) {
                        return noteOff(e.noteOff.pitch);
                    }
                    return onEventResult::UNHANDLED;
                }
                
                onEventResult onDroppedNote(uint8_t pitch) {
                    MIDI_LG(WARN, "dropped note '%d'", pitch);
                    return onEventResult::DROPPED_NOTE;
                }
                
                onEventResult startNote(MonoNoteChannel & c, NoteOnEvent const & e) {
                    MIDI_LG(INFO, "on  %d", e.pitch);
                    if(!c.open(out, 1.f, xfade_len())) {
                        return onDroppedNote(e.pitch);
                    }
                    c.pitch = e.pitch;
                    c.tuning = e.tuning;

                    c.elem.engine.set_channels(c.channels[0], c.channels[1]);
                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto f = to_freq(tunedNote-Do_midi, half_tone);
                    c.elem.engine.set_base_freq(f);

                    auto mode = static_cast<Mode>(SoundEngine::ModeTraversal.realValues()[static_cast<int>(.5f + params[MODE])]);
                    c.elem.engine.set_mode(mode);
                                        
                    c.elem.engine.set_xfade(xfade_len());
                    c.elem.engine.set_freq_scatter(denorm<FREQ_SCATTER>());
                    c.elem.engine.set_d1(/*denorm<D1>()*/params[D1]);
                    c.elem.engine.set_d2(/*denorm<D2>()*/params[D2]);
                    c.elem.engine.set_phase_ratio1(denorm<PHASE_RATIO1>());
                    c.elem.engine.set_phase_ratio2(denorm<PHASE_RATIO2>());
                    c.elem.engine.set_har_att(denorm<HARMONIC_ATTENUATION>());
                    c.elem.engine.set_length(denorm<LENGTH>());
                    
                    auto interp = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + params[INTERPOLATION])]);
                    
                    c.elem.engine.set_itp(interp);
                    c.elem.engine.set_active(true);
                    
                    c.elem.engine.update(out);
                    
                    return onEventResult::OK;
                }
                
            public:
                void allNotesOff() override {
                    MIDI_LG(INFO, "all notes off :");
                    for(auto & c : channels) {
                        if(c.close(out, CloseMode::XFADE_ZERO, xfade_len())) {
                            LG(INFO, " x %d", c.pitch);
                        }
                    }
                }
                
                void allSoundsOff() override {
                    MIDI_LG(INFO, "all sounds off :");
                    for(auto & c : channels) {
                        if(c.close(out, CloseMode::NOW)) {
                            LG(INFO, " x %d", c.pitch);
                        }
                    }
                }
                
            private:
                // notes are identified by pitch
                onEventResult noteOff(uint8_t pitch) {
                    MIDI_LG(INFO, "off %d", pitch);
                    for(auto & c : channels) {
                        if(c.pitch != pitch) {
                            continue;
                        }
                        if(!c.close(out, CloseMode::XFADE_ZERO, xfade_len())) {
                            continue;
                        }
                        // the oscillator is still used for the crossfade,
                        // it will stop being used in the call to openChannel when that channel is finished closing
                        return onEventResult::OK;
                    }
                    // it could be that the corresponding noteOn was skipped
                    // because too many notes where played at that time
                    // or ... a bug? todo investigate...
                    return onDroppedNote(pitch);
                }
                
                template<ImplParams N>
                float denorm() const {
                    return denormalize<N>(params[N]);
                }
                
                template<ImplParams N>
                float norm() const {
                    return normalize<N>(params[N]);
                }
                
                int xfade_len() const {
                    auto d = denorm<XFADE_LENGTH>();
                    // make it odd
                    return 1 + (static_cast<int>(.5f+d)/2)*2;
                }
                
            public:
                
                void doProcessing (ProcessData& data) override
                {
                    A(data.numSamples);
                    
                    std::array<float *, nAudioOut> outs;
                    A(1 == data.numOutputs);
                    A(nAudioOut == data.outputs[0].numChannels);
                    for(auto i=0; i<data.outputs[0].numChannels; ++i) {
                        outs[i] = data.outputs[0].channelBuffers32[i];
                    }
                    
                    auto nRemainingFrames = data.numSamples;
                    
                    static_assert((size_interleaved / nAudioOut) * nAudioOut == size_interleaved, "");
                    
                    auto currentFrame = 0;
                    
                    auto * events = data.inputEvents;
                    A( events );
                    
                    EventIterator it(begin(events)), end(end_(events));
                    
                    int nextEventPosition = getNextEventPosition(it, end);
                    A(nextEventPosition >= currentFrame);
                    
                    while(nRemainingFrames) {
                        A(nRemainingFrames > 0);
                        
                        while(nextEventPosition == currentFrame) {
                            // TODO use metaprogrammation to generalize
                            Event e;
                            it.dereference(e);
                            onEvent(e);
                            ++it;
                            nextEventPosition = getNextEventPosition(it, end);
                        }
                        A(nextEventPosition > currentFrame);
                        
                        // compute not more than until next event...
                        auto nFramesToProcess = nextEventPosition - currentFrame;
                        // ... not more than the number of frames remaining
                        nFramesToProcess = std::min(nFramesToProcess, nRemainingFrames);
                        // ... not more than the buffer
                        nFramesToProcess = std::min<int>(nFramesToProcess, n_frames_interleaved);
                        
                        out.step(&interleaved[0], nFramesToProcess);
                        
                        for(auto c = 0; c < nFramesToProcess; ++c) {
                            for(unsigned int i=0; i<nAudioOut; ++i) {
                                *outs[i] = interleaved[nAudioOut*c + i];
                                ++outs[i];
                            }
                        }
                        nRemainingFrames -= nFramesToProcess;
                        currentFrame += nFramesToProcess;
                    }
                    
                    A(nextEventPosition == event_position_infinite); // the events should all have already been processed
                }
            };
            
        }
    }
} // namespaces
