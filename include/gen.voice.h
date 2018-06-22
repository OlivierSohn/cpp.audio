namespace imajuscule {
    namespace audio {

        namespace voice {

            enum ImplParams {

                // Common
                PINK_NOISE_LP_GAIN,
                PINK_NOISE_BP_GAIN,
                PINK_NOISE_BR_GAIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MAX,
                CENTER_OCTAVE_MIN_LONG_TERM,
                CENTER_OCTAVE_MAX_LONG_TERM,
                CENTER_SHORT_TERM_RATIO,
                N_SLOW_ITER_SHORT_TERM,
                N_SLOW_ITER_LONG_TERM,
                ORDER_FILTERS,
                SINE_GAIN,
                SEED,
                RANDOM_PAN,
                PAN,

#include "loudness_enum_names.h"

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

                // Birds
                MARKOV_XFADE_FREQ,

                FREQ_TRANSITION_LENGTH,
                FREQ_TRANSITION_INTERPOLATION,

                // Sweep
                LOW_FREQ,
                HIGH_FREQ
            };

            constexpr std::array<ImplParams, 32> params_birds
            {{
                PINK_NOISE_LP_GAIN,
                PINK_NOISE_BP_GAIN,
                PINK_NOISE_BR_GAIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MAX,
                CENTER_OCTAVE_MIN_LONG_TERM,
                CENTER_OCTAVE_MAX_LONG_TERM,
                ORDER_FILTERS,
                SINE_GAIN,

                SEED,

                RANDOM_PAN,
                PAN,

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

            constexpr std::array<ImplParams, 32> params_robots
            {{
                PINK_NOISE_LP_GAIN,
                PINK_NOISE_BP_GAIN,
                PINK_NOISE_BR_GAIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MAX,
                CENTER_OCTAVE_MIN_LONG_TERM,
                CENTER_OCTAVE_MAX_LONG_TERM,
                ORDER_FILTERS,
                SINE_GAIN,

                SEED,

                RANDOM_PAN,
                PAN,

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

            constexpr std::array<ImplParams, 29> params_wind
            {{
                PINK_NOISE_LP_GAIN,
                PINK_NOISE_BP_GAIN,
                PINK_NOISE_BR_GAIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MAX,
                CENTER_OCTAVE_MIN_LONG_TERM,
                CENTER_OCTAVE_MAX_LONG_TERM,
                CENTER_SHORT_TERM_RATIO,
                N_SLOW_ITER_SHORT_TERM,
                N_SLOW_ITER_LONG_TERM,
                ORDER_FILTERS,
                SINE_GAIN,

                SEED,

                RANDOM_PAN,
                PAN,

                GAIN,
                LOUDNESS_LEVEL,
                LOUDNESS_COMPENSATION_AMOUNT,
                LOUDNESS_REF_FREQ_INDEX,

                MARKOV_START_NODE,
                MARKOV_PRE_TRIES,
                MARKOV_MIN_PATH_LENGTH,
                MARKOV_ADDITIONAL_TRIES,

                INTERPOLATION,
                FREQ_SCATTER,
                LENGTH,
                LENGTH_EXPONENT,
                LENGTH_EXPONENT_SCATTER,
                XFADE_LENGTH
            }};

            constexpr std::array<ImplParams, 21> params_sweep
            {{
                PINK_NOISE_LP_GAIN,
                PINK_NOISE_BP_GAIN,
                PINK_NOISE_BR_GAIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MIN,
                PINK_NOISE_BP_OCTAVE_WIDTH_MAX,
                CENTER_OCTAVE_MIN_LONG_TERM,
                CENTER_OCTAVE_MAX_LONG_TERM,
                ORDER_FILTERS,
                SINE_GAIN,

                RANDOM_PAN,
                PAN,

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

            constexpr auto params_all = make_tuple(params_birds, params_robots, params_sweep, params_wind);

#include "pernamespace.implparams.h"

#include "loudness_enum_limits.h"

            template<> struct NoLimits<INTERPOLATION> {static constexpr auto zero = 0;};
            template<> struct NoLimits<LOUDNESS_COMPENSATION_AMOUNT> {static constexpr auto zero = 0;};
            template<> struct NoLimits<RANDOM_PAN> {static constexpr auto zero = 0;};
            template<> struct NoLimits<FREQ_TRANSITION_INTERPOLATION> {static constexpr auto zero = 0;};
            template<> struct NoLimits<MARKOV_XFADE_FREQ> {static constexpr auto zero = 0;};


            template<> struct Limits<ORDER_FILTERS> {
                static constexpr auto m = 1;
                static constexpr auto M = 258; };

            template<> struct Limits<SEED> {
                static constexpr auto m = 0;
                static constexpr auto M = 257; };

            template<> struct Limits<XFADE_LENGTH> {
                static constexpr auto m = 101;
                static constexpr auto M = 2001; };

            template<> struct Limits<FREQ_TRANSITION_LENGTH> {
                static constexpr auto m = 1;
                static constexpr auto M = 20001; };

            template<> struct Limits<MARKOV_ARTICULATIVE_PAUSE_LENGTH> {
                static constexpr auto m = 0;
                static constexpr auto M = 20001; };

            template<> struct Limits<LENGTH_EXPONENT_SCATTER> : public NormalizedParamLimits {};
            template<> struct Limits<PINK_NOISE_LP_GAIN> : public NormalizedParamLimits {};
            template<> struct Limits<PINK_NOISE_BP_GAIN> : public NormalizedParamLimits {};
            template<> struct Limits<PINK_NOISE_BR_GAIN> : public NormalizedParamLimits {};
            template<> struct Limits<FREQ_SCATTER> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO1> : public NormalizedParamLimits {};
            template<> struct Limits<PHASE_RATIO2> : public NormalizedParamLimits {};
            template<> struct Limits<SINE_GAIN> : public NormalizedParamLimits {};

            template<> struct Limits<D1> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };
            template<> struct Limits<D2> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };

            template<> struct Limits<HARMONIC_ATTENUATION> {
                static const float m;
                static const float M; };

            template<> struct Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MIN> {
                static const float m;
                static const float M; };

            template<> struct Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MAX> {
                static const float m;
                static const float M; };

            template<> struct Limits<CENTER_OCTAVE_MIN_LONG_TERM> {
                static const float m;
                static const float M; };

            template<> struct Limits<CENTER_OCTAVE_MAX_LONG_TERM> {
                static const float m;
                static const float M; };

            template<> struct Limits<PAN> {
                static const float m;
                static const float M; };

            template<> struct Limits<CENTER_SHORT_TERM_RATIO> : public NormalizedParamLimits {};
            template<> struct Limits<N_SLOW_ITER_SHORT_TERM> : public NormalizedParamLimits {};
            template<> struct Limits<N_SLOW_ITER_LONG_TERM> : public NormalizedParamLimits {};

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
                static const float m;
                static const float M; };

            template<> struct Limits<HIGH_FREQ> {
                static const float m;
                static const float M; };

            constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
            constexpr auto size_interleaved = size_interleaved_one_cache_line;

            using Mode = SoundEngineMode;


            template<SoundEngineMode>
            struct SetSlowParams {
                template<typename CTRL>
                static void set(CTRL & ctrl, int n_slow_steps_short, int n_slow_steps, float ratio) {
                    Assert(0);
                }
            };

            template<>
            struct SetSlowParams<SoundEngineMode::WIND> {
                template<typename CTRL>
                static void set(CTRL & ctrl, int n_slow_steps_short, int n_slow_steps_long, float ratio) {
                    ctrl.getUnderlyingIter().set_n_slow_steps(n_slow_steps_long);
                    ctrl.set_short_term_noise_rate(n_slow_steps_short);
                    ctrl.set_short_term_noise_amplitude(ratio);
                }
            };

            template<SoundEngineMode>
            struct SetFilterWidths {
                template<typename CTRL>
                static void set(CTRL & ctrl, range<float> const & width_factor_range) {
                    Assert(0);
                }
            };

            template<>
            struct SetFilterWidths<SoundEngineMode::WIND> {
                template<typename CTRL>
                static void set(CTRL & ctrl, range<float> const & width_factor_range) {
                    std::get<1>(ctrl.get()).getOsc().setWidthRange(width_factor_range);
                    std::get<2>(ctrl.get()).getOsc().setWidthRange(width_factor_range);
                }
            };

            template<SoundEngineMode>
            struct ConfigureFilters {
                template<typename CTRL>
                static void configure(CTRL & ctrl, unsigned int n_slow_steps, range<float> const & ra) {
                    Assert(0);
                }
            };

            template<>
            struct ConfigureFilters<SoundEngineMode::WIND> {
                template<typename CTRL>
                static void configure(CTRL & ctrl, unsigned int n_slow_steps, range<float> const & ra) {
                    // those control the width of the band algorithms
                    auto & bpf_width = std::get<1>(ctrl.get()).getOsc().getWidth();
                    auto & bpr_width = std::get<2>(ctrl.get()).getOsc().getWidth();

                    // those control the center frequencies of the band algorithms
                    auto & bpf_center_ctrl = std::get<1>(ctrl.get()).getCtrl();
                    auto & bpr_center_ctrl = std::get<2>(ctrl.get()).getCtrl();

                    bpf_center_ctrl.getUnderlyingIter().set_n_slow_steps(n_slow_steps);
                    bpf_center_ctrl.setFreqRange(ra);
                    bpf_width.getUnderlyingIter().set_n_slow_steps(n_slow_steps);

                    bpr_center_ctrl.getUnderlyingIter().set_n_slow_steps(n_slow_steps);
                    bpr_center_ctrl.setFreqRange(ra);
                    bpr_width.getUnderlyingIter().set_n_slow_steps(n_slow_steps);
                }
            };

            template<SoundEngineMode>
            struct SetGains {
                template<typename CTRL>
                static void set(CTRL & ctrl, std::array<float, 4> arr) {
                    ctrl.setGains(std::array<float, 1>{{
                        arr[3]
                    }});
                }
            };

            template<>
            struct SetGains<SoundEngineMode::WIND> {
                template<typename CTRL>
                static void set(CTRL & ctrl, std::array<float, 4> arr) {
                    ctrl.setGains(std::move(arr));
                }
            };

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
                        {"[1/f Noise] LPF Gain", Limits<PINK_NOISE_LP_GAIN>::m, Limits<PINK_NOISE_LP_GAIN>::M},
                        {"[1/f Noise] BPF Gain", Limits<PINK_NOISE_BP_GAIN>::m, Limits<PINK_NOISE_BP_GAIN>::M},
                        {"[1/f Noise] BRF Gain", Limits<PINK_NOISE_BR_GAIN>::m, Limits<PINK_NOISE_BR_GAIN>::M},
                        {"BPF Width Min", Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>::m, Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>::M},
                        {"BPF Width Max", Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>::m, Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>::M},
                        {"Long Center Min", Limits<CENTER_OCTAVE_MIN_LONG_TERM>::m, Limits<CENTER_OCTAVE_MIN_LONG_TERM>::M},
                        {"Long Center Max", Limits<CENTER_OCTAVE_MAX_LONG_TERM>::m, Limits<CENTER_OCTAVE_MAX_LONG_TERM>::M},
                        {"Short Center Ratio", Limits<CENTER_SHORT_TERM_RATIO>::m, Limits<CENTER_SHORT_TERM_RATIO>::M},
                        {"Iter exp short", Limits<N_SLOW_ITER_SHORT_TERM>::m, Limits<N_SLOW_ITER_SHORT_TERM>::M},
                        {"Iter exp long", Limits<N_SLOW_ITER_LONG_TERM>::m, Limits<N_SLOW_ITER_LONG_TERM>::M},
                        {"Filters Order", Limits<ORDER_FILTERS>::m, Limits<ORDER_FILTERS>::M},
                        {"[Sine] Gain", Limits<SINE_GAIN>::m, Limits<SINE_GAIN>::M},
                        {"Seed", Limits<SEED>::m, Limits<SEED>::M},
                        {"Random pan"},
                        {"Pan", Limits<PAN>::m, Limits<PAN>::M},
#include "loudness_params_specs.h"
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
                        {"[Sweep] Low freq.", Limits<LOW_FREQ>::m, Limits<LOW_FREQ>::M },
                        {"[Sweep] High freq.", Limits<HIGH_FREQ>::m, Limits<HIGH_FREQ>::M },
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

                static std::array<float,35> make_common(int start_node,
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
                                                        float harmonic_attenuation,
                                                        int filter_order,
                                                        float bandpass_width_min,
                                                        float bandpass_width_max
                                                        ) {
                    float gain = 2.f;
                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    Assert(b);

                    return {{
                        0.f,
                        0.f,
                        0.f,
                        normalize<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>(bandpass_width_min),
                        normalize<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>(bandpass_width_max),
                        normalize<CENTER_OCTAVE_MIN_LONG_TERM>(1.f),
                        normalize<CENTER_OCTAVE_MAX_LONG_TERM>(8.f),
                        normalize<CENTER_SHORT_TERM_RATIO>(0.f),
                        /*normalize<N_SLOW_ITER_SHORT_TERM>*/(0.f),
                        /*normalize<N_SLOW_ITER_LONG_TERM>*/(1.f),
                        static_cast<float>(filter_order-Limits<ORDER_FILTERS>::m),
                        1.f,
                        0,
                        static_cast<float>(!true),
                        normalize<PAN>(0.f),
#include "loudness_init_values.h"
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
                    auto a = make_common(start_node, pre_tries, min_path_length, additionnal_tries, articulative_pause_length, i, freq_scat, length, length_med_exp, length_scale_exp, xfade, phase_ratio1, phase_ratio2, d1,d2,harmonic_attenuation, 1, 0.f, 0.f);
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
                    auto a = make_common(0, 0, 0, 0, 0.f, i, 0.f, length, length_med_exp, 0.f, xfade, 0.f, 0.f, 0, 0, 0.f, 20.f, 0.f, 0.f);
                    Program::ARRAY result;
                    result.resize(std::get<Mode::SWEEP>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }

                    result[index(LOW_FREQ)] = normalize<LOW_FREQ>(low);
                    result[index(HIGH_FREQ)] = normalize<HIGH_FREQ>(high);

                    return result;
                }

                static Program::ARRAY make_bird(int start_node,
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
                    auto a = make_common(start_node, pre_tries, min_path_length, additionnal_tries, articulative_pause_length, i, freq_scat, length, length_med_exp, length_scale_exp, xfade, phase_ratio1, phase_ratio2, 0,0,0, 1, 0.f, 0.f);
                    Program::ARRAY result;
                    result.resize(std::get<Mode::BIRDS>(params_all).size());
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
                        Assert(b);
                        result[index(FREQ_TRANSITION_INTERPOLATION)] = static_cast<float>(idx);
                    }
                    {
                        int idx = 0;
                        auto b = xfade_freq_traversal().valToRealValueIndex(static_cast<int>(xfade_freq), idx);
                        Assert(b);
                        result[index(MARKOV_XFADE_FREQ)] = static_cast<float>(idx);
                    }
                    result[index(FREQ_TRANSITION_LENGTH)] = normalize<FREQ_TRANSITION_LENGTH>(freq_xfade);
                    return result;
                }

                static constexpr auto max_n_slow_iter = 100000.f;

                static Program::ARRAY make_noise_wind(int filter_order,
                                                range<float> bp_width,
                                                range<float> bp_center,
                                                float n_slow_iter) {
                    auto a = make_common(0, 0, 6, 0, 0, itp::PROPORTIONAL_VALUE_DERIVATIVE, 0.12f, 93.3f, 2.f, .5f, 2201, 0.f, 0.f, 0,0,0, filter_order, bp_width.getMin(), bp_width.getMax());
                    Program::ARRAY result;
                    result.resize(std::get<Mode::WIND>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }
                    result[index(PINK_NOISE_BP_GAIN)] = 1.f;
                    result[index(PINK_NOISE_BR_GAIN)] = 0.f;
                    result[index(SINE_GAIN)] = 0.f;
                    result[index(CENTER_OCTAVE_MIN_LONG_TERM)] = normalize<CENTER_OCTAVE_MIN_LONG_TERM>(bp_center.getMin()),
                    result[index(CENTER_OCTAVE_MAX_LONG_TERM)] = normalize<CENTER_OCTAVE_MAX_LONG_TERM>(bp_center.getMax()),
                    result[index(N_SLOW_ITER_LONG_TERM)] = std::log(n_slow_iter) / std::log(max_n_slow_iter);

                    return result;
                }

                static Program::ARRAY make_sine_wind(range<float> bp_center,
                                                     float short_center_ratio,
                                                     float n_slow_iter_long_term,
                                                     float n_slow_iter_short_term) {
                    auto a = make_common(0, 0, 6, 0, 0, itp::LINEAR, 0.12f, 93.3f, 2.f, .5f, 2201, 0.f, 0.f, 0,0,0, 1, 0.f, 0.f);
                    Program::ARRAY result;
                    result.resize(std::get<Mode::WIND>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }
                    result[index(LOUDNESS_COMPENSATION_AMOUNT)] = 1.f; // to not have volume discontinuities when freq varies fast
                    result[index(SINE_GAIN)] = 0.01f; // to have the same volume as noise winds
                    result[index(CENTER_OCTAVE_MIN_LONG_TERM)] = normalize<CENTER_OCTAVE_MIN_LONG_TERM>(bp_center.getMin()),
                    result[index(CENTER_OCTAVE_MAX_LONG_TERM)] = normalize<CENTER_OCTAVE_MAX_LONG_TERM>(bp_center.getMax()),
                    result[index(N_SLOW_ITER_LONG_TERM)] = std::log(n_slow_iter_long_term) / std::log(max_n_slow_iter);
                    result[index(N_SLOW_ITER_SHORT_TERM)] = std::log(n_slow_iter_short_term) / std::log(max_n_slow_iter);
                    result[index(CENTER_SHORT_TERM_RATIO)] = short_center_ratio;

                    return result;
                }

                static Program::ARRAY make_mixed_wind(int filter_order,
                                                      range<float> bp_width,
                                                      range<float> bp_center,
                                                      float n_slow_iter) {
                    auto a = make_common(0, 0, 6, 0, 0, itp::PROPORTIONAL_VALUE_DERIVATIVE, 0.12f, 93.3f, 2.f, .5f, 2201, 0.f, 0.f, 0,0,0, filter_order, bp_width.getMin(), bp_width.getMax());
                    Program::ARRAY result;
                    result.resize(std::get<Mode::WIND>(params_all).size());
                    for(int idx = 0; idx<a.size(); ++idx) {
                        auto e = static_cast<ImplParams>(idx);
                        if(!has(e)) {
                            continue;
                        }
                        result[index(e)] = a[idx];
                    }
                    result[index(LOUDNESS_COMPENSATION_AMOUNT)] = 0.f; // to not have volume discontinuities when freq varies fast
                    result[index(PINK_NOISE_BP_GAIN)] = 1.f;
                    result[index(PINK_NOISE_BR_GAIN)] = 0.f;
                    result[index(SINE_GAIN)] = 0.01f;
                    result[index(CENTER_OCTAVE_MIN_LONG_TERM)] = normalize<CENTER_OCTAVE_MIN_LONG_TERM>(bp_center.getMin()),
                    result[index(CENTER_OCTAVE_MAX_LONG_TERM)] = normalize<CENTER_OCTAVE_MAX_LONG_TERM>(bp_center.getMax()),
                    result[index(N_SLOW_ITER_LONG_TERM)] = std::log(n_slow_iter) / std::log(max_n_slow_iter);

                    return result;
                }
                static Programs const & getPrograms() {
                    if constexpr (MODE==Mode::BIRDS) {
                        static ProgramsI ps {{
                            {"Standard & Cute bird",
                                make_bird(0, 0, 1, 0, itp::EASE_INOUT_CIRC, 0.f, 93.f, 2.f, .5f, 1000, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO),
                                {32, 48, 69, 180, 218, 240}
                            },{"Scat bird",
                                make_bird(0, 0, 3, 17, itp::EASE_INOUT_CIRC, 0.015f, 10.f, 2.f, .5f, 1961, 782, FreqXfade::NonTrivial, 1601, itp::EASE_INOUT_EXPO),
                                {4, 5, 23, 26, 34, 48, 58, 68, 73, 74, 75, 80, 85, 88, 109, 116, 124, 125, 131, 141, 146, 165, 181, 205, 213, 214, 227, 232, 249}
                            },{"Rhythmic bird",
                                make_bird(1, 0, 3, 11, itp::EASE_INOUT_CIRC, 0.f, 19.8f, 2.f, 0.f, 1406, 502, FreqXfade::All, 801, itp::EASE_INOUT_EXPO),
                                { 19, 29, 32, 36, 38, 48, 79, 106, 112, 116, 123, 147, 162, 195, 213, 247, 248, 250 }
                            },{"Slow bird",
                                make_bird(0, 2, 1, 0, itp::EASE_IN_EXPO, 0.f, 73.7f, 2.f, .5f, 1000, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO),
                                {63, 70, 83, 91, 110, 160, 197}
                            },{"BiTone bird",
                                make_bird(1, 0, 2, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 4302, 1301, FreqXfade::No, 6200, itp::EASE_OUT_EXPO),
                                { 5, 15, 27, 31, 49, 58, 72, 74, 96, 108, 147, 149, 171, 174, 180, 194, 199, 205, 252},
                            },{"Happy bird 1",
                                make_bird(1, 0, 4, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 5848, 2001, FreqXfade::No, 6200, itp::EASE_OUT_EXPO),
                                {
                                    119, 141, 149, 159, // appel
                                    88,                 // mélancolie
                                    32, 45, 168, 206// bien-être
                                }
                            },{"Happy bird 2",
                                make_bird(1, 0, 4, 0, itp::EASE_IN_EXPO, .414f, 63.9f, 1.19f, 1.f, 5848, 2001, FreqXfade::No, 6200, itp::EASE_OUT_EXPO),
                                {8, 20, 23, 60, 76, 113, 143, 168, 169, 178, 180, 208, 217, 231}
                            },{"Laughing bird",
                                make_bird(1, 0, 2, 0, itp::EASE_IN_EXPO, .414f, 78.6f, 2.f, .5f, 9672, 1301, FreqXfade::All, 3201, itp::EASE_OUT_EXPO),
                                {20, 31, 39, 36, 37, 47, 68, 89, 94, 105, 108, 136, 144, 145, 148, 161, 172, 174, 212, 246, 249}
                            },{"Talkative bird",
                                make_bird(0, 0, 6, 0, itp::EASE_INOUT_CIRC, 0.12f, 93.3f, 2.f, .5f, 6713, 2201, FreqXfade::NonTrivial, 4401, itp::EASE_OUT_EXPO),
                                {9, 28, 33, 38, 53, 54, 83, 114, 117, 122, 131, 162, 168, 171, 187, 196, 216, 220}
                            }
                        }};
                        return ps.v;
                    }
                    else if constexpr (MODE==Mode::ROBOTS) {
                        static ProgramsI ps {{
                            {"R2D2",
                                make_robot(0, 0, 1, 1, 3683, itp::LINEAR, 0.f, 19.8, 2.1f, 0.39f, 234, 6, 12, .98f, 0.f, 0.f)
                            },{"Communication",
                                make_robot(0, 0, 16, 14, 217, itp::EASE_INOUT_CIRC, 0.f, 10.f, 1.89f, 1.f, 234, 6, 10, .98f, 0.f, 0.f)
                            },
                        }};
                        return ps.v;
                    }
                    else if constexpr (MODE==Mode::SWEEP) {
                        static ProgramsI ps {{
                            {"Sweep 1",
                                make_sweep(itp::LINEAR, 73.f, 5.f, 481, 40.f, 20000.f)
                            },{"Fullrange",
                                make_sweep(itp::LINEAR, 500.f, 5.f, 481, 10.f, 20000.f)
                            },
                        }};
                        return ps.v;
                    }
                    else if constexpr (MODE==Mode::WIND) {
                        static ProgramsI ps {{
                            {"Medium wind in trees",
                                make_noise_wind(1, {0.f, 0.f}, {1.f, 8.f}, 100000.f)
                            }, {"Steady wind",
                                make_noise_wind(4, {1.3f, 1.3f}, {5.2f, 5.5f}, 3981.f)
                            }, {"Strong wind",
                                make_noise_wind(4, {3.8f, 3.8f}, {1.f, 8.f}, 100000.f)
                            }, {"Vinyl cracks",
                                make_noise_wind(89, {3.45f, 5.f}, {8.1f, 8.1f}, 33.f)
                            }, {"Small animal eating",
                                make_noise_wind(61, {0.f, 5.f}, {8.1f, 8.1f}, 10.f)
                            }, {"Heavy rain in a car",
                                make_noise_wind(33, {3.45f, 5.f}, {8.1f, 8.1f}, 10.f)
                            }, {"Light rain in a car",
                                make_noise_wind(89, {3.45f, 5.f}, {8.1f, 8.1f}, 10.f)
                            }, {"Heavy rain",
                                make_noise_wind(13, {5.f, 5.f}, {7.8f, 8.f}, 12.5f)
                            }, {"Light rain",
                                make_noise_wind(13, {3.45f, 3.45f}, {8.0f, 8.3f}, 10.f)
                            }, {"Bubbles",
                                make_noise_wind(129, {2.45f, 3.25f}, {4.8f, 8.3f}, 1009.9f)
                            }, {"Earth rumbling",
                                make_noise_wind(30, {1.95f, 5.f}, {2.5f, 3.2f}, 7009.3f)
                            }, {"Sine wind",
                                make_sine_wind({4.6f, 6.8f}, 0.2f, 100000.f, 22.3f )
                            }, {"Kettle whistle pure",
                                make_sine_wind({7.5f, 7.7f}, 0.f, 22.3f, 22.3f)
                            }, {"Kettle whistle mixed",
                                make_mixed_wind(7, {.9f, .9f}, {7.5f, 7.7f}, 316.f)
                            },
                        }};
                        return ps.v;
                    }
                }

            public:

                Program const & getProgram(int i) const override {
                    auto & progs = getPrograms();
                    Assert(i < progs.size());
                    return progs[i];
                }

                int countPrograms() const override {
                    return getPrograms().size();
                }

                static constexpr int index(ImplParams n) {
                    int idx = 0;
                    for(auto p : std::get<MODE>(params_all)) {
                        if(p==n) {
                            return idx;
                        }
                        ++idx;
                    }
                    Assert(0);
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
                    return valueof<N>(params[index(N)]);
                }

                template<ImplParams N>
                float denorm() const {
                    return denormalize<N>(params[index(N)]);
                }

                template<ImplParams N>
                float octaveToFreq() const {
                    constexpr auto lowest_f = 10.f;
                    auto m = denorm<N>();
                    return lowest_f * pow(2.f, m);
                }

                template<ImplParams N1, ImplParams N2>
                range<float> octaveRangeToFreqRange() const {
                    auto m = octaveToFreq<N1>();
                    auto M = octaveToFreq<N2>();
                    if(m > M) {
                        std::swap(m,M);
                    }
                    return { m, M };
                }

            protected:
                template<typename MonoNoteChannel, typename F, typename CS, typename OutputData, typename Chans>
                bool onStartNote(float velocity, MonoNoteChannel & c, F & channelFlag, CS & cs, OutputData & out, Chans & chans) {
                    c.elem.engine.set_channel(*c.channel);

                    {
                        auto interp = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + value<INTERPOLATION>())]);

                        c.elem.engine.set_itp(interp);
                    }

                    auto ex = denorm<LENGTH_EXPONENT>();
                    Assert(ex >= 0.f);
                    if constexpr (MODE != Mode::SWEEP) {
                        auto variation = denorm<LENGTH_EXPONENT_SCATTER>();
                        Assert(variation >= 0.f);
                        Assert(variation <= 1.f);
                        c.elem.engine.set_length_exp(ex * (1.f - variation), ex * (1.f + variation));

                        c.elem.engine.set_freq_scatter(denorm<FREQ_SCATTER>());

                        if constexpr (MODE != Mode::WIND) {
                            c.elem.engine.set_phase_ratio1(denorm<PHASE_RATIO1>());
                            c.elem.engine.set_phase_ratio2(denorm<PHASE_RATIO2>());
                        }

                        thread_local int seed = 0;
                        if(int i_seed = static_cast<int>(value<SEED>() + .5f)) {
                            seed = i_seed;
                            Assert(seed <= Limits<SEED>::M);
                            Assert(seed > Limits<SEED>::m);
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
                        mersenne<SEEDED::Yes>().seed(seed);
                    }
                    else {
                        c.elem.engine.set_length_exp(ex, ex);
                    }

                    {
                        auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                        auto f = to_freq(tunedNote-Do_midi, half_tone);
                        c.elem.engine.set_base_freq(f);
                    }

                    c.elem.engine.set_length(denorm<LENGTH>());

                    c.elem.setLoudnessParams(value<LOUDNESS_REF_FREQ_INDEX>(),
                                             value<LOUDNESS_COMPENSATION_AMOUNT>(),
                                             denorm<LOUDNESS_LEVEL>());
                    c.elem.setFiltersOrder(value<ORDER_FILTERS>());

                    if constexpr (MODE == Mode::WIND)
                    {
                        auto m = denorm<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>();
                        auto M = denorm<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>();
                        auto width_factor_range = range<float>(std::min(m,M),
                                                               std::max(m,M));
                        for(auto & r : c.elem.getRamps()) {
                            SetFilterWidths<MODE>::set(r.algo.getOsc(), width_factor_range);
                        }
                    }
                    SetGains<MODE>::set(c.elem, std::array<float,4>{{
                        denorm<PINK_NOISE_LP_GAIN>(),
                        denorm<PINK_NOISE_BP_GAIN>(),
                        denorm<PINK_NOISE_BR_GAIN>(),
                        denorm<SINE_GAIN>()
                    }});

                    if constexpr (MODE == Mode::WIND) {
                        auto ra = octaveRangeToFreqRange<
                        CENTER_OCTAVE_MIN_LONG_TERM,
                        CENTER_OCTAVE_MAX_LONG_TERM
                        >();

                        // we could omit calling set_n_slow_steps when not in mode WIND,
                        // but we call it with value 1 to be sure that all objects
                        // that we need to call for WIND are called (the value is initialized to
                        // -1 in constructor and makes an assert fail if the method is not called)
                        float n_slow_steps = std::pow(max_n_slow_iter, denorm<N_SLOW_ITER_LONG_TERM>());

                        for(auto & r : c.elem.getRamps()) {
                            auto & mix = r.algo.getOsc();
                            ConfigureFilters<MODE>::configure(mix, n_slow_steps, ra);
                        }
                        auto n_slow_steps_short = std::pow(max_n_slow_iter, denorm<N_SLOW_ITER_SHORT_TERM>());
                        auto ratio = denorm<CENTER_SHORT_TERM_RATIO>();
                        for(auto & f_control : c.elem.engine.getRamps().a) {
                            // f_control controls the frequency of the Mix (has an effect only for sinus and low pass)
                            SetSlowParams<MODE>::set(f_control.get(), n_slow_steps_short, n_slow_steps, ratio);
                            // needs to be after the previous call
                            f_control.get().setFreqRange(ra);
                        }
                    }

                    {
                        float pan;
                        {
                            auto rnd_pan = value<RANDOM_PAN>() ? false : true;
                            if(rnd_pan) {
                                pan = std::uniform_real_distribution<float>{-1.f, 1.f}(mersenne<SEEDED::Yes>());
                            }
                            else {
                                pan = denorm<PAN>();
                            }
                        }

                        if constexpr (MODE == Mode::SWEEP) {
                          return c.elem.engine.initialize_sweep(out,
                                                                chans,
                                                                channelFlag,
                                                                denorm<LOW_FREQ>(),
                                                                denorm<HIGH_FREQ>(),
                                                                pan);
                        }
                        else if constexpr (MODE == Mode::BIRDS) {
                            c.elem.engine.set_freq_xfade(denorm<FREQ_TRANSITION_LENGTH>());
                            auto interp_freq = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + value<FREQ_TRANSITION_INTERPOLATION>())]);
                            c.elem.engine.set_freq_interpolation(interp_freq);
                            auto xfade_freq = static_cast<FreqXfade>(xfade_freq_traversal().realValues()[static_cast<int>(.5f + value<MARKOV_XFADE_FREQ>())]);

                            return c.elem.engine.initialize_birds(out,
                                                                  chans,
                                                                  channelFlag,
                                                                  value<MARKOV_START_NODE>(),
                                                                  value<MARKOV_PRE_TRIES>(),
                                                                  value<MARKOV_MIN_PATH_LENGTH>(),
                                                                  value<MARKOV_ADDITIONAL_TRIES>(),
                                                                  SoundEngineInitPolicy::StartAfresh,
                                                                  xfade_freq,
                                                                  value<MARKOV_ARTICULATIVE_PAUSE_LENGTH>(),
                                                                  pan);
                        }
                        else if constexpr (MODE == Mode::WIND) {
                            return c.elem.engine.initialize_wind(out,
                                                                 chans,
                                                                 channelFlag,
                                                                 value<MARKOV_START_NODE>(),
                                                                 value<MARKOV_PRE_TRIES>(),
                                                                 value<MARKOV_MIN_PATH_LENGTH>(),
                                                                 value<MARKOV_ADDITIONAL_TRIES>(),
                                                                 SoundEngineInitPolicy::StartAfresh,
                                                                 pan);
                        }
                        else if constexpr (MODE == Mode::ROBOTS) {
                          c.elem.engine.set_d1(/*denorm<D1>()*/value<D1>());
                            c.elem.engine.set_d2(/*denorm<D2>()*/value<D2>());
                            c.elem.engine.set_har_att(denorm<HARMONIC_ATTENUATION>());
                            return c.elem.engine.initialize_robot(out,
                                                                  chans,
                                                                  channelFlag,
                                                                  value<MARKOV_START_NODE>(),
                                                                  value<MARKOV_PRE_TRIES>(),
                                                                  value<MARKOV_MIN_PATH_LENGTH>(),
                                                                  value<MARKOV_ADDITIONAL_TRIES>(),
                                                                  SoundEngineInitPolicy::StartAfresh,
                                                                  value<MARKOV_ARTICULATIVE_PAUSE_LENGTH>(),
                                                                  pan);
                        }
                        else {
                          return false;
                        }
                    }
                }

                int32_t get_xfade_length() const {
                    auto d = denorm<XFADE_LENGTH>();
                    // make it odd
                    return 1 + ( static_cast<int>( .5f + d ) / 2 ) * 2;
                }

                float get_gain() const { return denorm<GAIN>(); }

            public:
                void set_gain(float g) {
                    params[index(GAIN)] = normalize<GAIN>(g);
                    Assert(std::abs(get_gain()-g) < .001f);
                }

                void set_loudness_compensation(float c) {
                    params[index(LOUDNESS_COMPENSATION_AMOUNT)] = normalize<LOUDNESS_COMPENSATION_AMOUNT>(c);
                }

                void set_pan(float pan) {
                    params[index(PAN)] = normalize<PAN>(pan);
                }

                void set_random_pan(bool is_random) {
                    params[index(RANDOM_PAN)] = static_cast<float>(!is_random);
                }

                void set_random(bool is_random) {
                    if(!has(SEED)) {
                        return;
                    }
                    // seed param: 0 means random, >= 1 means fixed
                    if(is_random) {
                        constexpr auto seed_random = 0;
                        params[index(SEED)] = static_cast<float>(seed_random);
                    }
                    else {
                        constexpr auto seed_fixed = 1;
                        params[index(SEED)] = static_cast<float>(seed_fixed);
                    }
                }

                void set_seed(int seed) {
                    if(!has(SEED)) {
                        return;
                    }
                    params[index(SEED)] = static_cast<float>(seed);
                }
            };

            template<typename SoundEngine>
            struct EngineAndRamps {
                using audioElt = typename SoundEngine::audioElt;
                using T = typename audioElt::FPT;
                using Buf = audioelement::AEBuffer<T>;
                using buffer_t = std::array<Buf,3>;

                static constexpr auto hasEnvelope = audioElt::hasEnvelope;

                EngineAndRamps(buffer_t&b) :
                ramps{b[0],b[1],b[2]},
              engine{[this](std::function<bool()> const & fShouldKeyRelease)-> Ramps<audioElt> {
                    using namespace imajuscule::audioelement;
                  if(goOn && fShouldKeyRelease()) {
                    onKeyReleased();
                  }
                    Ramps<audioElt> res;
                    // in SoundEngine.playNextSpec (from the rt audio thread),
                    // we onKeyPressed() the inactive ramp and onKeyReleased() the active ramp.
                    for(auto & r: ramps) {
                        auto state = r.algo.getEnveloppe().getState();
                        if(state == EnvelopeState::EnvelopeDone2) {
                            if(goOn) {
                                res.envelopeDone = &r;
                            }
                        }
                        else {
                            res.allDone = false;
                            if(state == EnvelopeState::KeyPressed) {
                                res.keyPressed = &r;
                            }
                        }
                    }
                    return res;
                }} {}

                void setEnvelopeCharacTime(int len) {
                    engine.setEnvelopeCharacTime(len);
                }

                void forgetPastSignals() {
                    // is handled by the sound engine
                }
                bool isEnvelopeFinished() const {
                  if(goOn || engine.get_active()) {
                    return false;
                  }
                  for(auto & r: ramps) {
                      if(!r.isEnvelopeFinished()) {
                          return false;
                      }
                  }
                  return true;
                }
                void onKeyPressed() {
                  engine.set_active(true);
                  goOn = true;
                }
                bool onKeyReleased() {
                  if(goOn) {
                    goOn = false;
                    for(auto & r: ramps) {
                        r.onKeyReleased(); // ignore the result.
                    }
                    // note that the sound engine may already be inactive as a result of the markov chain running out of actions.
                    // this is why we also need 'goOn'.
                    engine.set_active(false);
                    return true;
                  }
                  return false;
                }

                // TODO forward params to the soundengine, let the soudengine do the call at onKeyPressed time.
                void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                    for(auto & r : ramps) {
                        r.algo.getOsc().setLoudnessParams(low_index, log_ratio, loudness_level);
                    }
                }

                // TODO forward params to the soundengine, let the soudengine do the call at onKeyPressed time.
                template<typename T>
                void setGains(T&& gains) {
                    for(auto & r : ramps) {
                        r.algo.getOsc().setGains(std::forward<T>(gains));
                    }
                }

                // TODO forward params to the soundengine, let the soudengine do the call at onKeyPressed time.
                void setFiltersOrder(int order) {
                    for(auto & r : ramps) {
                        r.algo.getOsc().setFiltersOrder(order);
                    }
                }

                auto & getRamps() {
                    return ramps;
                }

                SoundEngine engine; // in onStartNote, this engine is assigned a given channel.

                bool isInactive() const {
                  for(auto const & r: ramps) {
                      if(!r.isInactive()) {
                          return false;
                      }
                  }
                  return true;
                }

                T angle() const { return {}; }

            private:
                // 3 because there is 'before', 'current' and the inactive one
                std::array<audioElt, 3> ramps;
                bool goOn = false;

            public:
                auto const & getRamps() const { return ramps; }
            };

            template<

            AudioOutPolicy outPolicy,
            int nAudioOut,
            Mode MODE,
            bool withNoteOff,

            typename Parameters,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename ProcessData,

            typename Base = ImplBase<MODE, Parameters, ProcessData>,

            typename Parent = ImplCRTP <
            outPolicy,
            nAudioOut, XfadePolicy::UseXfade, // TODO reassess the use of xfades, now that we have enveloppes
            EngineAndRamps<SoundEngine<MODE, nAudioOut, getAtomicity<outPolicy>(), Logger>>,
            withNoteOff,
            EventIterator, NoteOnEvent, NoteOffEvent, Base >
            >

            struct Impl_ : public Parent
            {
                static constexpr auto n_frames_interleaved = size_interleaved / nAudioOut;
                static_assert(n_frames_interleaved * nAudioOut == size_interleaved); // make sure we don't waste space

                // the soundengine enques requests only when the channel queue is empty, so:
                static constexpr auto max_queue_size = MaxQueueSize::One;

                using Base::interleaved;
                using Base::get_xfade_length;

                using Parent::onEvent2;
                using Parent::channels;

                using Event = typename Parent::Event;
                using MonoNoteChannel = typename Parent::MonoNoteChannel;
                static constexpr auto n_channels = Parent::n_channels;
                static constexpr auto xfade_policy = Parent::xfade_policy;

            public:

                template <class... Args>
                Impl_(Args&&... args) : Parent (std::forward<Args>(args)...) {}

                template<typename Out, typename Chans>
                onEventResult onEvent(Event const & e, Out & out, Chans & chans)
                {
                    return onEvent2(e, out, chans);
                }

                template<typename Out, typename Chans>
                void doProcessing (ProcessData& data, Out & out, Chans & chans)
                {
                    static_assert(Out::policy == outPolicy);
                    Assert(data.numSamples);

                    std::array<float *, nAudioOut> outs;
                    Assert(1 == data.numOutputs);
                    Assert(nAudioOut == data.outputs[0].numChannels);
                    for(auto i=0; i<data.outputs[0].numChannels; ++i) {
                        outs[i] = data.outputs[0].channelBuffers32[i];
                    }

                    auto nRemainingFrames = data.numSamples;

                    static_assert((size_interleaved / nAudioOut) * nAudioOut == size_interleaved);

                    auto currentFrame = 0;

                    auto * events = data.inputEvents;
                    Assert( events );

                    EventIterator it(begin(events)), end(end_(events));

                    int nextEventPosition = getNextEventPosition(it, end);
                    Assert(nextEventPosition >= currentFrame);

                    while(nRemainingFrames) {
                        Assert(nRemainingFrames > 0);

                        while(nextEventPosition == currentFrame) {
                            Event e;
                            it.dereference(e);
                            onEvent(e, out, chans);
                            ++it;
                            nextEventPosition = getNextEventPosition(it, end);
                        }
                        Assert(nextEventPosition > currentFrame);

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

                    Assert(nextEventPosition == event_position_infinite); // the events should all have already been processed
                }
            };

        }
    }
} // namespaces
