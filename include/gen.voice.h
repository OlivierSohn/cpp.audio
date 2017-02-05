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
            
            
            static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
            static constexpr auto size_interleaved = size_interleaved_one_cache_line;
            
            template<
            typename Parameters,
            typename ProcessData,
            typename Parent = Impl<
            ImplParams, Parameters, interleaved_buf_t, size_interleaved, ProcessData
            >
            >
            struct ImplBase : public Parent {
                using Parent::params;
                using Parent::half_tone;

                using Params = ImplParams;
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                
                using SoundEngine = SoundEngine<imajuscule::Logger, UpdateMode::FORCE_SOUND>;
            protected:
                using Mode = SoundEngine::Mode;

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
                        {"Cute bird",
                            make(Mode::MARKOV, 1301, 0.f, 12.f, 24.f, 0.f, 0.f, .95f, 93.f, itp::EASE_INOUT_CIRC)
                        },
                    }};
                    return ps.v;
                }
                
                // methods
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
                    
                    auto mode = static_cast<Mode>(SoundEngine::ModeTraversal.realValues()[static_cast<int>(.5f + params[MODE])]);
                    c.elem.engine.set_mode(mode);
                    
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
            
            // to use it in context of Grid3d, we need to have an implementation of these:
            typename Parameters,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename ProcessData,
            
            typename Base = ImplBase<Parameters, ProcessData>,
            
            typename Parent = ImplCRTP < nAudioOut, XfadePolicy::UseXfade,
            MonoNoteChannel< 2, EngineAndRamps<typename Base::SoundEngine> >,
            EventIterator, NoteOnEvent, NoteOffEvent, Base >
            
            >
            struct Impl_ : public Parent
            {
                using Event = typename EventIterator::object;
                using Programs = imajuscule::audio::Programs;

                using Params = ImplParams;
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                static constexpr auto n_frames_interleaved = size_interleaved / nAudioOut;
                static_assert(n_frames_interleaved * nAudioOut == size_interleaved, ""); // make sure we don't waste space
                
                using Base::get_xfade_length;
                using Base::half_tone;
                using Base::interleaved;
                using Base::params;
                
                using Parent::out;
                using Parent::onEvent;

                using OutputData = outputDataBase<nAudioOut, XfadePolicy::UseXfade, NoOpLock, PostProcess::NONE>;
                using SoundEngine = typename Base::SoundEngine;
                using Mode = typename SoundEngine::Mode;

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
