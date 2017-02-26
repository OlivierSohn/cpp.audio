namespace imajuscule {
    namespace audio {

            namespace voice {

            enum ImplParams {

                // Common
                SEED,
                GAIN,
                LOUDNESS_LEVEL,
                LOUDNESS_COMPENSATION_AMOUNT,
                LOUDNESS_REF_FREQ_INDEX,
                
                MARKOV_START_NODE,
                MARKOV_PRE_TRIES,
                MARKOV_MIN_PATH_LENGTH,
                MARKOV_ADDITIONAL_TRIES,
                MARKOV_ARTICULATIVE_PAUSE_LENGTH,

                INTERPOLATION,
                FREQ_SCATTER,
                LENGTH,
                LENGTH_EXPONENT,
                LENGTH_EXPONENT_SCATTER,
                XFADE_LENGTH,
                PHASE_RATIO1,
                PHASE_RATIO2,
                
                // Robot
                D1,
                D2,
                HARMONIC_ATTENUATION,
                
                // Markov
                MARKOV_XFADE_FREQ,
                
                FREQ_TRANSITION_LENGTH,
                FREQ_TRANSITION_INTERPOLATION,
                
                // Sweep
                LOW_FREQ,
                HIGH_FREQ

            };
            
            constexpr std::array<ImplParams, 21> params_markov
            {{
                SEED,
                GAIN,
                LOUDNESS_LEVEL,
                LOUDNESS_COMPENSATION_AMOUNT,
                LOUDNESS_REF_FREQ_INDEX,
                
                MARKOV_START_NODE,
                MARKOV_PRE_TRIES,
                MARKOV_MIN_PATH_LENGTH,
                MARKOV_ADDITIONAL_TRIES,
                MARKOV_ARTICULATIVE_PAUSE_LENGTH,

                INTERPOLATION,
                FREQ_SCATTER,
                LENGTH,
                LENGTH_EXPONENT,
                LENGTH_EXPONENT_SCATTER,
                XFADE_LENGTH,
                
                MARKOV_XFADE_FREQ,
                FREQ_TRANSITION_LENGTH,
                FREQ_TRANSITION_INTERPOLATION,
                
                PHASE_RATIO1,
                PHASE_RATIO2
                
            }};
                
                constexpr std::array<ImplParams, 21> params_robots
                {{
                    SEED,
                    GAIN,
                    LOUDNESS_LEVEL,
                    LOUDNESS_COMPENSATION_AMOUNT,
                    LOUDNESS_REF_FREQ_INDEX,
                    
                    MARKOV_START_NODE,
                    MARKOV_PRE_TRIES,
                    MARKOV_MIN_PATH_LENGTH,
                    MARKOV_ADDITIONAL_TRIES,
                    MARKOV_ARTICULATIVE_PAUSE_LENGTH,
                    
                    D1,
                    D2,
                    HARMONIC_ATTENUATION,
                    
                    INTERPOLATION,
                    FREQ_SCATTER,
                    LENGTH,
                    LENGTH_EXPONENT,
                    LENGTH_EXPONENT_SCATTER,
                    XFADE_LENGTH,
                    
                    PHASE_RATIO1,
                    PHASE_RATIO2
                }};
                
                
                constexpr std::array<ImplParams, 10> params_sweep
                {{
                    GAIN,
                    LOUDNESS_LEVEL,
                    LOUDNESS_COMPENSATION_AMOUNT,
                    LOUDNESS_REF_FREQ_INDEX,
                    
                    INTERPOLATION,
                    
                    LENGTH,
                    LENGTH_EXPONENT,
                    XFADE_LENGTH,
                    
                    LOW_FREQ,
                    HIGH_FREQ
                }};
                
            constexpr auto params_all = make_tuple(params_markov, params_robots, params_sweep);
            
#include "pernamespace.implparams.h"
                
                template<> struct Limits<SEED> {
                    static constexpr auto m = 0;
                    static constexpr auto M = 257; };
                
                template<> struct Limits<GAIN> {
                    static const float m;
                    static const float M; };
                
                template<> struct Limits<LOUDNESS_LEVEL> {
                    static const float m;
                    static const float M; };
                
                template<> struct Limits<LOUDNESS_REF_FREQ_INDEX> {
                    static constexpr auto m = 0;
                    static constexpr auto M = 10; };

                template<> struct Limits<LOUDNESS_COMPENSATION_AMOUNT> : public NormalizedParamLimits {};

            template<> struct Limits<XFADE_LENGTH> {
                static constexpr auto m = 101;
                static constexpr auto M = 2001; };
            
            template<> struct Limits<FREQ_TRANSITION_LENGTH> {
                static constexpr auto m = 1;
                static constexpr auto M = 20001; };
            
            template<> struct Limits<MARKOV_ARTICULATIVE_PAUSE_LENGTH> {
                static constexpr auto m = 0;
                static constexpr auto M = 20001; };
            
            template<> struct Limits<FREQ_SCATTER> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO1> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO2> : public NormalizedParamLimits {};
            template<> struct Limits<LENGTH_EXPONENT_SCATTER> : public NormalizedParamLimits {};

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
            
            template<> struct Limits<LENGTH_EXPONENT> {
                static const float m;
                static const float M; };
            
            template<> struct Limits<MARKOV_START_NODE> {
                static constexpr auto m = 0;
                static constexpr auto M = 2; };
            
            template<> struct Limits<MARKOV_PRE_TRIES> {
                static constexpr auto m = 0;
                static constexpr auto M = 20; };
            
            template<> struct Limits<MARKOV_MIN_PATH_LENGTH> {
                static constexpr auto m = 0;
                static constexpr auto M = 20; };
            
            template<> struct Limits<MARKOV_ADDITIONAL_TRIES> {
                static constexpr auto m = 0;
                static constexpr auto M = 20; };
                
                template<> struct Limits<LOW_FREQ> {
                    static constexpr auto m = 10;
                    static constexpr auto M = 10000; };
                
                template<> struct Limits<HIGH_FREQ> {
                    static constexpr auto m = 20;
                    static constexpr auto M = 40000; };
                
            constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
            constexpr auto size_interleaved = size_interleaved_one_cache_line;

            using SoundEngine = SoundEngine<imajuscule::Logger, UpdateMode::FORCE_SOUND_AT_EACH_UPDATE>;
            using Mode = SoundEngineMode;

            template <
            
            Mode MODE,
            typename Parameters,
            typename ProcessData,
            typename Parent = Impl <
            
            std::get<MODE>(params_all).size(), Parameters, interleaved_buf_t, size_interleaved, ProcessData
            
            >
            
            >
            struct ImplBase : public Parent {
                static constexpr auto n_max_orchestrator_per_channel = 1;

                using Parent::params;
                using Parent::half_tone;
                
            public:
                static std::vector<ParamSpec> const & getParamSpecs() {
                    
                    static std::vector<ParamSpec> params_spec = {
                        {"Seed", Limits<SEED>::m, Limits<SEED>::M},
                        {"Gain", Limits<GAIN>::m, Limits<GAIN>::M},
                        {"[Loudness] Level", Limits<LOUDNESS_LEVEL>::m, Limits<LOUDNESS_LEVEL>::M},
                        {"[Loudness] Compensation", Limits<LOUDNESS_COMPENSATION_AMOUNT>::m, Limits<LOUDNESS_COMPENSATION_AMOUNT>::M},
                        {"[Loudness] Min comp. f. idx", Limits<LOUDNESS_REF_FREQ_INDEX>::m, Limits<LOUDNESS_REF_FREQ_INDEX>::M },
                        {"[Markov] Start node", Limits<MARKOV_START_NODE>::m, Limits<MARKOV_START_NODE>::M},
                        {"[Markov] Num. pre tries", Limits<MARKOV_PRE_TRIES>::m, Limits<MARKOV_PRE_TRIES>::M},
                        {"[Markov] Min path length", Limits<MARKOV_MIN_PATH_LENGTH>::m, Limits<MARKOV_MIN_PATH_LENGTH>::M},
                        {"[Markov] Num. post tries", Limits<MARKOV_ADDITIONAL_TRIES>::m, Limits<MARKOV_ADDITIONAL_TRIES>::M},
                        {"Articulative pause length", Limits<MARKOV_ARTICULATIVE_PAUSE_LENGTH>::m, Limits<MARKOV_ARTICULATIVE_PAUSE_LENGTH>::M},
                         {"Interpolation", itp::interpolation_traversal()},
                        {"Frequency scatter", Limits<FREQ_SCATTER>::m, Limits<FREQ_SCATTER>::M },
                        {"Length", Limits<LENGTH>::m, Limits<LENGTH>::M},
                        {"Length Exponent", Limits<LENGTH_EXPONENT>::m, Limits<LENGTH_EXPONENT>::M},
                        {"Length Exponent Scatter", Limits<LENGTH_EXPONENT_SCATTER>::m, Limits<LENGTH_EXPONENT_SCATTER>::M},
                        {"Crossfade length", static_cast<float>(Limits<XFADE_LENGTH>::m), static_cast<float>(Limits<XFADE_LENGTH>::M) }, // couldn't make it work with nsteps = max-min / 2
                        {"Phase ratio 1", Limits<PHASE_RATIO1>::m, Limits<PHASE_RATIO1>::M},
                        {"Phase ratio 2", Limits<PHASE_RATIO2>::m, Limits<PHASE_RATIO2>::M},
                        {"D1", Limits<D1>::m, Limits<D1>::M },
                        {"D2", Limits<D2>::m, Limits<D2>::M},
                        {"Harmonic attenuation", Limits<HARMONIC_ATTENUATION>::m, Limits<HARMONIC_ATTENUATION>::M},
                        {"Xfade freq", xfade_freq_traversal()},
                        {"Frequency transition length", static_cast<float>(Limits<FREQ_TRANSITION_LENGTH>::m), static_cast<float>(Limits<FREQ_TRANSITION_LENGTH>::M) },
                        {"Frequency Interpolation", itp::interpolation_traversal()},
                        {"[Sweep] Low freq.", static_cast<float>(Limits<LOW_FREQ>::m), static_cast<float>(Limits<LOW_FREQ>::M) },
                        {"[Sweep] High freq.", static_cast<float>(Limits<HIGH_FREQ>::m), static_cast<float>(Limits<HIGH_FREQ>::M) },
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
                
                static std::array<float,21> make_common(int start_node,
                                                        int pre_tries,
                                                        int min_path_length,
                                                        int additionnal_tries,
                                                        int articulative_pause_length,
                                                        itp::interpolation i,
                                                        float freq_scat,
                                                        float length,
                                                        float length_med_exp,
                                                        float length_scale_exp,
                                                        int xfade,
                                                        float phase_ratio1, float phase_ratio2,
                                                        float d1, float d2,
                                                        float harmonic_attenuation) {
                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    A(b);

                    return {{
                        0,
                        normalize<GAIN>(2.f),
                        normalize<LOUDNESS_LEVEL>(30.f),
                        /*normalize<LOUDNESS_COMPENSATION_AMOUNT>*/1.f,
                        /*normalize<LOUDNESS_REF_FREQ_INDEX>*/5,
                        static_cast<float>(start_node),
                        static_cast<float>(pre_tries),
                        static_cast<float>(min_path_length),
                        static_cast<float>(additionnal_tries),
                        static_cast<float>(articulative_pause_length),
                        static_cast<float>(itp_index),
                        /*normalize<FREQ_SCATTER>*/(freq_scat),
                        normalize<LENGTH>(length),
                        normalize<LENGTH_EXPONENT>(length_med_exp),
                        normalize<LENGTH_EXPONENT_SCATTER>(length_scale_exp),
                        normalize<XFADE_LENGTH>(xfade),
                        normalize<PHASE_RATIO1>(phase_ratio1),
                        normalize<PHASE_RATIO2>(phase_ratio2),
                        /*normalize<D1>*/(d1),
                        /*normalize<D2>*/(d2),
                        normalize<HARMONIC_ATTENUATION>(harmonic_attenuation),
                    }};
                }
                
                static Program::ARRAY make_robot(int start_node,
                                                 int pre_tries,
                                                 int min_path_length,
                                                 int additionnal_tries,
                                                 int articulative_pause_length,
                                                 itp::interpolation i,
                                                 float freq_scat,
                                                 float length,
                                                 float length_med_exp,
                                                 float length_scale_exp,
                                                 int xfade,
                                                 float d1, float d2,
                                                 float harmonic_attenuation,
                                                 float phase_ratio1 = 0.f, float phase_ratio2 = 0.f) {
                    auto a = make_common(start_node, pre_tries, min_path_length, additionnal_tries, articulative_pause_length, i, freq_scat, length, length_med_exp, length_scale_exp, xfade, phase_ratio1, phase_ratio2, d1,d2,harmonic_attenuation);
                    Program::ARRAY result;
                    result.resize(std::get<Mode::ROBOTS>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }
                    return result;
                }
                
                static Program::ARRAY make_sweep(itp::interpolation i,
                                                float length,
                                                float length_med_exp,
                                                int xfade,
                                                 float low, float high) {
                    Program::ARRAY result;
                    result.resize(std::get<Mode::SWEEP>(params_all).size());

                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    A(b);

                    result[index(GAIN)] = normalize<GAIN>(20.f);
                    result[index(LOUDNESS_LEVEL)] = normalize<LOUDNESS_LEVEL>(30.f);
                    result[index(LOUDNESS_REF_FREQ_INDEX)] = static_cast<float>(5);
                    result[index(LOUDNESS_COMPENSATION_AMOUNT)] = .8f;
                    result[index(INTERPOLATION)] = static_cast<float>(itp_index);
                    result[index(LENGTH)] = normalize<LENGTH>(length);
                    result[index(LENGTH_EXPONENT)] = normalize<LENGTH_EXPONENT>(length_med_exp);
                    result[index(XFADE_LENGTH)] = normalize<XFADE_LENGTH>(xfade);
                    result[index(LOW_FREQ)] = normalize<LOW_FREQ>(low);
                    result[index(HIGH_FREQ)] = normalize<HIGH_FREQ>(high);
                    
                    return result;
                }
                
                static Program::ARRAY make_markov(int start_node,
                                                  int pre_tries,
                                                  int min_path_length,
                                                  int additionnal_tries,
                                                  itp::interpolation i,
                                                  float freq_scat,
                                                  float length,
                                                  float length_med_exp,
                                                  float length_scale_exp,
                                                  int articulative_pause_length,
                                                  int xfade,
                                                  FreqXfade xfade_freq,
                                                  int freq_xfade,
                                                  itp::interpolation freq_i,
                                                  float phase_ratio1 = 0.f, float phase_ratio2 = 0.f) {
                    auto a = make_common(start_node, pre_tries, min_path_length, additionnal_tries, articulative_pause_length, i, freq_scat, length, length_med_exp, length_scale_exp, xfade, phase_ratio1, phase_ratio2, 0,0,0);
                    Program::ARRAY result;
                    result.resize(std::get<Mode::MARKOV>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }
                    {
                        int idx = 0;
                        auto b = itp::interpolation_traversal().valToRealValueIndex(i, idx);
                        A(b);
                        result[index(FREQ_TRANSITION_INTERPOLATION)] = static_cast<float>(idx);
                    }
                    {
                        int idx = 0;
                        auto b = xfade_freq_traversal().valToRealValueIndex(static_cast<int>(xfade_freq), idx);
                        A(b);
                        result[index(MARKOV_XFADE_FREQ)] = static_cast<float>(idx);
                    }
                    result[index(FREQ_TRANSITION_LENGTH)] = normalize<FREQ_TRANSITION_LENGTH>(freq_xfade);
                    return result;
                }
                
                static Programs const & getPrograms() {
                    if(MODE==Mode::MARKOV) {
                        static ProgramsI ps {{
                            {"Standard & Cute bird",
                                make_markov(0, 0, 1, 0, itp::EASE_INOUT_CIRC, 0.f, 93.f, 2.f, .5f, 1000, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO)
                            },{"Scat bird",
                                make_markov(0, 0, 3, 17, itp::EASE_INOUT_CIRC, 0.015f, 10.f, 2.f, .5f, 1961, 782, FreqXfade::NonTrivial, 1601, itp::EASE_INOUT_EXPO)
                            },{"Rhythmic bird",
                                make_markov(1, 0, 3, 11, itp::EASE_INOUT_CIRC, 0.f, 19.8f, 2.f, 0.f, 1406, 502, FreqXfade::All, 801, itp::EASE_INOUT_EXPO)
                            },{"Slow bird",
                                make_markov(0, 2, 1, 0, itp::EASE_IN_EXPO, 0.f, 73.7f, 2.f, .5f, 1000, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO)
                            },{"BiTone bird",
                                make_markov(1, 0, 2, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 4302, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO)
                            },{"Happy bird",
                                make_markov(1, 0, 4, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 5848, 2001, FreqXfade::No, 6200, itp::EASE_OUT_EXPO)
                            },{"Laughing bird",
                                make_markov(1, 0, 2, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 9672, 1301, FreqXfade::All, 3201, itp::EASE_OUT_EXPO)
                            },{"Talkative bird",
                                make_markov(0, 0, 6, 0, itp::EASE_INOUT_CIRC, 0.12f, 93.3f, 2.f, .5f, 6713, 2201, FreqXfade::NonTrivial, 4401, itp::EASE_OUT_EXPO)
                            }
                        }};
                        return ps.v;
                    }
                    else if(MODE==Mode::ROBOTS) {
                        static ProgramsI ps {{
                            {"Robot 1",
                                make_robot(0, 0, 16, 14, 217, itp::EASE_INOUT_CIRC, 0.f, 108.f, 2.f, 0.f, 234, 6, 10, 0.f, 0.f, 0.f)
                            },{"Communication",
                                make_robot(0, 0, 16, 14, 217, itp::EASE_INOUT_CIRC, 0.f, 10.f, 3.29f, 1.f, 234, 6, 10, 0.f, 0.f, 0.f)
                            },
                        }};
                        return ps.v;
                    }
                    else if(MODE==Mode::SWEEP) {
                        static ProgramsI ps {{
                            {"Sweep 1",
                                make_sweep(itp::LINEAR, 500.f, 5.f, 481, 40.f, 20000.f)
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
                
                static constexpr int index(ImplParams n) {
                    int idx = 0;
                    for(auto p : std::get<MODE>(params_all)) {
                        if(p==n) {
                            return idx;
                        }
                        ++idx;
                    }
                    A(0);
                    return 0;
                }
                
                static constexpr bool has(ImplParams n) {
                    for(auto p : std::get<MODE>(params_all)) {
                        if(p==n) {
                            return true;
                        }
                    }
                    return false;
                }
                
                template<ImplParams N>
                float value() const {
                    return params[index(N)];
                }
                
                template<ImplParams N>
                float denorm() const {
                    return denormalize<N>(params[index(N)]);
                }
                
                template<ImplParams N>
                float norm() const {
                    return normalize<N>(params[index(N)]);
                }
                
            protected:
                template<typename MonoNoteChannel, typename OutputData>
                void onStartNote(float velocity, MonoNoteChannel & c, OutputData & out) {
                    
                    c.elem.engine.set_active(true);
                    c.elem.engine.set_mode(MODE);
                    c.elem.engine.set_channels(c.channels[0], c.channels[0]);

                    auto interp = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + value<INTERPOLATION>())]);
                    
                    c.elem.engine.set_itp(interp);

                    auto ex = denorm<LENGTH_EXPONENT>();
                    A(ex >= 0.f);
                    if(MODE != Mode::SWEEP) {
                        auto variation = denorm<LENGTH_EXPONENT_SCATTER>();
                        A(variation >= 0.f);
                        A(variation <= 1.f);
                        c.elem.engine.set_length_exp(ex * (1.f - variation), ex * (1.f + variation));
                        
                        c.elem.engine.set_freq_scatter(denorm<FREQ_SCATTER>());
                        c.elem.engine.set_phase_ratio1(denorm<PHASE_RATIO1>());
                        c.elem.engine.set_phase_ratio2(denorm<PHASE_RATIO2>());

                        thread_local int seed = 0;
                        if(int i_seed = static_cast<int>(value<SEED>() + .5f)) {
                            seed = i_seed;
                            A(seed <= Limits<SEED>::M);
                            A(seed > Limits<SEED>::m);
                        }
                        else {
                            // seed is 0, we change the seed at each note
                            if(seed == Limits<SEED>::M) {
                                seed = 1;
                            }
                            else {
                                ++seed;
                            }
                        }
                        rng::mersenne().seed(seed);
                    }
                    else {
                        c.elem.engine.set_length_exp(ex, ex);
                    }
                    
                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto f = to_freq(tunedNote-Do_midi, half_tone);
                    c.elem.engine.set_base_freq(f);
                    
                    c.elem.engine.set_length(denorm<LENGTH>());
                    
                    c.elem.engine.set_xfade(get_xfade_length()); // useless in case of Markov (xfade already set in the channel by caller, and this value will not be used)

                    c.elem.setLoudnessParams(value<LOUDNESS_REF_FREQ_INDEX>(),
                                             value<LOUDNESS_COMPENSATION_AMOUNT>(),
                                             denorm<LOUDNESS_LEVEL>());

                    if(MODE == Mode::MARKOV) {
                        c.elem.engine.set_freq_xfade(denorm<FREQ_TRANSITION_LENGTH>());
                        auto interp_freq = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + value<FREQ_TRANSITION_INTERPOLATION>())]);
                        c.elem.engine.set_freq_interpolation(interp_freq);
                        auto xfade_freq = static_cast<FreqXfade>(xfade_freq_traversal().realValues()[static_cast<int>(.5f + value<MARKOV_XFADE_FREQ>())]);

                        c.elem.engine.initialize_markov(out,
                                                        c,
                                                        value<MARKOV_START_NODE>(),
                                                        value<MARKOV_PRE_TRIES>(),
                                                        value<MARKOV_MIN_PATH_LENGTH>(),
                                                        value<MARKOV_ADDITIONAL_TRIES>(),
                                                        SoundEngine::InitPolicy::StartAfresh,
                                                        xfade_freq,
                                                        value<MARKOV_ARTICULATIVE_PAUSE_LENGTH>(),
                                                        denorm<GAIN>());
                    }
                    else if(MODE == Mode::ROBOTS) {
                        c.elem.engine.set_d1(/*denorm<D1>()*/value<D1>());
                        c.elem.engine.set_d2(/*denorm<D2>()*/value<D2>());
                        c.elem.engine.set_har_att(denorm<HARMONIC_ATTENUATION>());
                        c.elem.engine.initialize_robot(out,
                                                       c,
                                                       value<MARKOV_START_NODE>(),
                                                       value<MARKOV_PRE_TRIES>(),
                                                       value<MARKOV_MIN_PATH_LENGTH>(),
                                                       value<MARKOV_ADDITIONAL_TRIES>(),
                                                       SoundEngine::InitPolicy::StartAfresh,
                                                       value<MARKOV_ARTICULATIVE_PAUSE_LENGTH>(),
                                                       denorm<GAIN>());
                    }
                    else if(MODE == Mode::SWEEP) {
                        c.elem.engine.initialize_sweep(out,
                                                       c,
                                                       denorm<LOW_FREQ>(),
                                                       denorm<HIGH_FREQ>(),
                                                       denorm<GAIN>());
                    }
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
                    for(auto & r: ramps) {
                        if(r.isInactive()) {
                            return &r;
                        }
                    }
                    return nullptr;
                }} {}
                
                void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                    for(auto & r : ramps) {
                        r.algo.getOsc().setLoudnessParams(low_index, log_ratio, loudness_level);
                    }
                }
                
                SoundEngine engine;
                
                // 3 because there is 'before', 'current' and the inactive one
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
            
            typename Parent_ = ImplCRTP < nAudioOut, XfadePolicy::UseXfade,
            MonoNoteChannel< 1, EngineAndRamps<SoundEngine> >, false,
            EventIterator, NoteOnEvent, NoteOffEvent, Base >
            
            >
            struct Impl_ : public Parent_
            {
                using Parent = Parent_;
                
                static constexpr auto n_frames_interleaved = size_interleaved / nAudioOut;
                static_assert(n_frames_interleaved * nAudioOut == size_interleaved, ""); // make sure we don't waste space
                
                using Base::interleaved;
                using Base::get_xfade_length;
                
                using Parent::onEvent;
                using Parent::channels;

            public:
                template<typename OutputData>
                void doProcessing (ProcessData& data, OutputData & out)
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
                            this-> template onEvent<WithLock::No>(it, [](auto & c) -> bool {
                                for(auto const & r : c.elem.ramps) {
                                    if(!r.isInactive()) {
                                        return false;
                                    }
                                }
                                // here we know that all elements are inactive
                                // but if the channel has not been closed yet
                                // we cannot use it (if we want to enable that,
                                // we should review the way note on/off are detected,
                                // because it would probably cause bugs)
                                return c.closed();
                            }, out);
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
