namespace imajuscule {
    namespace audio {
        namespace voice {            
            
            // the order here should be the same as is params_markov and params_robots
            enum ImplParams {
                INTERPOLATION,
                FREQ_SCATTER,
                XFADE_LENGTH,
                LENGTH,
                PHASE_RATIO1,
                PHASE_RATIO2,
                D1,
                D2,
                HARMONIC_ATTENUATION,
                
                NPARAMS
            };
            
            static std::array<ImplParams, 6> params_markov
            {{
                INTERPOLATION,
                FREQ_SCATTER,
                XFADE_LENGTH,
                LENGTH,
                PHASE_RATIO1,
                PHASE_RATIO2
            }};
            
            static std::array<ImplParams, 9> params_robots
            {{
                INTERPOLATION,
                FREQ_SCATTER,
                XFADE_LENGTH,
                LENGTH,
                PHASE_RATIO1,
                PHASE_RATIO2,
                D1,
                D2,
                HARMONIC_ATTENUATION
            }};
            
            static auto params_all = make_tuple(params_markov, params_robots, params_robots);
            
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
            
            
            static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
            static constexpr auto size_interleaved = size_interleaved_one_cache_line;

            using SoundEngine = SoundEngine<imajuscule::Logger, UpdateMode::FORCE_SOUND>;
            using Mode = SoundEngine::Mode;

            template<
            Mode MODE,
            typename Parameters,
            typename ProcessData,
            typename Parent = Impl<
            ImplParams, Parameters, interleaved_buf_t, size_interleaved, ProcessData, std::get<MODE>(params_all).size()
            >
            >
            struct ImplBase : public Parent {
                using Parent::params;
                using Parent::half_tone;
                
            protected:
            
            public:
                static std::vector<ParamSpec> const & getParamSpecs() {
                    static std::vector<ParamSpec> params_spec = {
                        {"Interpolation", itp::interpolation_traversal()},
                        {"Frequency scatter", Limits<FREQ_SCATTER>::m, Limits<FREQ_SCATTER>::M },
                        {"Crossfade length", static_cast<float>(Limits<XFADE_LENGTH>::m), static_cast<float>(Limits<XFADE_LENGTH>::M) }, // couldn't make it work with nsteps = max-min / 2
                        {"Length", Limits<LENGTH>::m, Limits<LENGTH>::M},
                        {"Phase ratio 1", Limits<PHASE_RATIO1>::m, Limits<PHASE_RATIO1>::M},
                        {"Phase ratio 2", Limits<PHASE_RATIO2>::m, Limits<PHASE_RATIO2>::M},
                        {"D1", Limits<D1>::m, Limits<D1>::M },
                        {"D2", Limits<D2>::m, Limits<D2>::M},
                        {"Harmonic attenuation", Limits<HARMONIC_ATTENUATION>::m, Limits<HARMONIC_ATTENUATION>::M}
                    };
                    
                    static std::vector<ParamSpec> filtered;
                    if(filtered.empty()) {
                        filtered.reserve(std::get<MODE>(params_all).size());
                        for(auto i : std::get<MODE>(params_all)) {
                            filtered.push_back(params_spec[i]);
                        }
                    }
                    return filtered;
                }
                
                static Program::ARRAY make(itp::interpolation i,
                                           float freq_scat,
                                           int xfade,
                                           float length,
                                           float phase_ratio1, float phase_ratio2,
                                           float d1, float d2,
                                           float harmonic_attenuation) {
                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    A(b);

                    return {{
                        static_cast<float>(itp_index),
                        /*normalize<FREQ_SCATTER>*/(freq_scat),
                        normalize<XFADE_LENGTH>(xfade),
                        normalize<LENGTH>(length),
                        normalize<PHASE_RATIO1>(phase_ratio1),
                        normalize<PHASE_RATIO2>(phase_ratio2),
                        /*normalize<D1>*/(d1),
                        /*normalize<D2>*/(d2),
                        normalize<HARMONIC_ATTENUATION>(harmonic_attenuation),
                    }};
                }
                
                static Program::ARRAY make_markov(itp::interpolation i,
                                                  float freq_scat,
                                                  int xfade,
                                                  float length,
                                                  float phase_ratio1 = 0.f, float phase_ratio2 = 0.f) {
                    auto a = make(i, freq_scat, xfade, length, phase_ratio1, phase_ratio2, 0,0,0);
                    a.resize(std::get<Mode::MARKOV>(params_all).size());
                    return a;
                }
                
                static Programs const & getPrograms() {
                    if(MODE==Mode::MARKOV) {
                        static ProgramsI ps {{
                            {"Cute bird",
                                make_markov(itp::EASE_INOUT_CIRC, 0.f, 1301, 93.f)
                            },{"Slow bird",
                                make_markov(itp::EASE_IN_EXPO, 0.f, 1301, 73.7f)
                            }
                        }};
                        return ps.v;
                    }
                    else if(MODE==Mode::ROBOTS) {
                        static ProgramsI ps {{
                            {"Robot 1",
                                make(itp::EASE_INOUT_CIRC, 0.f, 1301, 93.f, 0.f, 0.f, 0.f, 0.f, 0.f)
                            },
                        }};
                        return ps.v;
                    }
                    else {
                        A(MODE==Mode::BIRDS);
                        static ProgramsI ps {{
                            {"Bird 1",
                                make(itp::EASE_INOUT_CIRC, 0.f, 1301, 93.f, 0.f, 0.f, 0.f, 0.f, 0.f)
                            },
                        }};
                        return ps.v;
                    }
                }
                
            public:
                
                Program const & getProgram(int i) const override {
                    auto & progs = getPrograms();
                    A(i < progs.size());
                    return progs[i];
                }
                
                template<ImplParams N>
                float denorm() const {
                    return denormalize<N>(params[N]);
                }
                
                template<ImplParams N>
                float norm() const {
                    return normalize<N>(params[N]);
                }
                
            protected:
                template<typename MonoNoteChannel, typename OutputData>
                void onStartNote(float velocity, MonoNoteChannel & c, OutputData & out) {
                    
                    c.elem.engine.set_channels(c.channels[0], c.channels[1]);
                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto f = to_freq(tunedNote-Do_midi, half_tone);
                    c.elem.engine.set_base_freq(f);
                    
                    c.elem.engine.set_mode(MODE);
                    
                    c.elem.engine.set_xfade(get_xfade_length());
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
                }
                
                int get_xfade_length() const {
                    auto d = denorm<XFADE_LENGTH>();
                    // make it odd
                    return 1 + (static_cast<int>(.5f+d)/2)*2;
                }

            };
            
            template<typename SoundEngine>
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
            
            template<
            
            int nAudioOut,
            Mode MODE,
            
            typename Parameters,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename ProcessData,
            
            typename Base = ImplBase<MODE, Parameters, ProcessData>,
            
            typename Parent = ImplCRTP < nAudioOut, XfadePolicy::UseXfade,
            MonoNoteChannel< 2, EngineAndRamps<SoundEngine> >,
            EventIterator, NoteOnEvent, NoteOffEvent, Base >
            
            >
            struct Impl_ : public Parent
            {
                static constexpr auto n_frames_interleaved = size_interleaved / nAudioOut;
                static_assert(n_frames_interleaved * nAudioOut == size_interleaved, ""); // make sure we don't waste space
                
                using Base::interleaved;
                
                using Parent::out;
                using Parent::onEvent;

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
                            onEvent(it, [](auto & c) -> bool {
                                for(auto & e : c.elem.ramps) {
                                    if(!e.isInactive()) {
                                        return false;
                                    }
                                }
                                // here we know that all elements are inactive
                                // but if the channel has not been closed yet
                                // we cannot use it (if we want to enable that,
                                // we should review the way note on/off are detected,
                                // because it would probably cause bugs)
                                return c.closed();
                            });
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
