
namespace imajuscule {
    namespace audio {
        namespace voice {
            static inline float clamp_phase_ratio(float v) {
                if(v > 1.f) {
                    LG(WARN, "clamp phase_ratio '%.1f' to 1.f", v);
                    return 1.f;
                }
                if(v < 0.f) {
                    LG(WARN, "clamp phase_ratio '%.1f' to 0.f", v);
                    return 0.f;
                }
                return v;
            }
            
            enum class Status { OK_CHANGED, OK_STABLE, ERROR };

            template<int nAudioOut, typename Logger>
            struct SoundEngine {
                using ramp = audioelement::FreqRamp<float>;
                using Request = Request<nAudioOut>;

                enum class Mode : unsigned char{
                    DISABLED,
                    ROBOTS,
                    BIRDS,
                    MARKOV
                };
                
                template<typename F>
                SoundEngine(F f) :
                get_inactive_ramp(std::move(f)),
                active(false)
                {}
                
                template<typename OutputData>
                Status update(OutputData & o, bool force = false)
                {
                    static_assert(nAudioOut == OutputData::nOuts, "");
                    if(!this->active) {
                        return Status::OK_STABLE;
                    }
                    if(mode == Mode::DISABLED) {
                        return Status::OK_STABLE;
                    }
                    auto n_frames = static_cast<float>(ms_to_frames(length));
                    if(n_frames <= 0) {
                        Logger::err("length '%f' is too small", length);
                        return Status::ERROR;
                    }
                    
                    if(base_freq <= 0.f) {
                        Logger::err("frequency '%d' should be sctrictly positive", base_freq);
                        return Status::ERROR;
                    }
                    
                    if(mode == Mode::MARKOV) {
                        if(!markov) {
                            markov = std::make_unique<MarkovChain>();
                            
                            auto play = [this, &o](float length, float freq1, float freq2,
                                               float phase_ratio1, float phase_ratio2,
                                               float freq_scatter) {
                                length *= powf(2.f,
                                               std::uniform_real_distribution<float>{1.f, 3.f}(rng::mersenne()));
                                auto n_frames = static_cast<float>(ms_to_frames(length));
                                if(n_frames <= 0) {
                                    Logger::err("zero length");
                                    return false;
                                }
                                
                                auto factor = std::uniform_real_distribution<float>{1.f, 1.f + freq_scatter}(rng::mersenne());
                                freq1 *= factor;
                                freq2 *= factor;
                                
                                auto const stereo_gain = stereo(std::uniform_real_distribution<float>(-1.f, 1.f)(rng::mersenne()));
                                auto volume = MakeVolume::run<nAudioOut>(1.f, stereo_gain);
                                                                
                                if(auto * ramp = get_inactive_ramp()) {
                                    ramp->algo.set(freq1, freq2, n_frames, phase_ratio1 * n_frames, interpolation);
                                    o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume, length }));
                                    if(auto * ramp = get_inactive_ramp()) {
                                        ramp->algo.set(freq1, freq2, n_frames, phase_ratio2 * n_frames, interpolation);
                                        o.playGeneric(c2, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume, length }));
                                    }
                                }
                                return true;
                            };
                            auto &mc= *markov;
                            
                            auto node1 = mc.emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                            });
                            auto node2 = mc.emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                                if(m==Move::ENTER) {
                                    play(length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
                                }
                                else {
                                    play(length, base_freq*2, base_freq*4, phase_ratio1, phase_ratio2, freq_scatter);
                                }
                            });
                            auto node3 = mc.emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                                if(m==Move::ENTER) {
                                    play(length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
                                }
                            });
                            
                            // 3 - < .5 .015 > - 1 - < .015 .5 > - 2
                            // |                                   |
                            //  --------- > 0.885 > ---------------
                            
                            def_markov_transition(node1, node2, 0.5f);
                            def_markov_transition(node2, node1, 0.015f);
                            def_markov_transition(node1, node3, 0.5f);
                            def_markov_transition(node3, node1, 0.015f);
                            def_markov_transition(node3, node2, 0.885f);
                            
                            mc.initialize(0);
                        };
                        
                        auto &mc= *markov;
                        int count_transitions = 0;
                        
                        auto cur = mc.getCurrent();
                        for(int i=0; i<3; ++i) {
                            auto next = mc.step();
                            if(cur != next) {
                                ++count_transitions;
                                cur = next;
                            }
                        }

                        if(force && 0==count_transitions) {
                            while(true) {
                                auto next = mc.step();
                                if(cur != next) {
                                    ++count_transitions;
                                    cur = next;
                                    break;
                                }
                            }
                        }
                        
                        return Status::OK_CHANGED;
                    }
                    
                    if(std::uniform_int_distribution<>{0,2}(rng::mersenne())) {
                        return Status::OK_STABLE;
                    }
                    
                    static float ht = compute_half_tone(1.f);
                    
                    auto vol1 = std::uniform_real_distribution<float>{0.5f,1.5f}(rng::mersenne());
                    auto vol2 = vol1;
                    
                    auto freq1 = std::uniform_real_distribution<float>{base_freq, (1+freq_scatter) * base_freq}(rng::mersenne());
                    auto freq2 = std::uniform_real_distribution<float>{freq1*0.97f, freq1/0.97f}(rng::mersenne());
                    
                    if(!std::uniform_int_distribution<>{0,5}(rng::mersenne())) {
                        freq1 = transpose_frequency(freq1, ht, d1);
                        vol1 *= expt(har_att, d1);
                    }
                    else if(!std::uniform_int_distribution<>{0,5}(rng::mersenne())) {
                        freq2 = transpose_frequency(freq2, ht, d2);
                        vol2 *= expt(har_att, d2);
                    }
                    else {
                        return Status::OK_STABLE;
                    }
                    
                    auto const stereo_gain = stereo(std::uniform_real_distribution<float>(-1.f, 1.f)(rng::mersenne()));
                    auto volume = MakeVolume::run<nAudioOut>(1.f, stereo_gain);
                    
                    for(int i=0; i<2; ++i) {
                        auto & ch = o.editChannel(i?c2:c1);
                        A(!ch.isPlaying());
                        ch.set_xfade(xfade);
                    }
                    
                    if(Mode::BIRDS==mode) {
                        for(int i=0; i<1; ++i) {
                            if(auto * ramp = get_inactive_ramp()) {
                                ramp->algo.set(freq2, freq1, n_frames, phase_ratio1 * n_frames, interpolation);
                                o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol2, length }));
                            }
                            if(auto * ramp = get_inactive_ramp()) {
                                A(c2 != c1);
                                ramp->algo.set(freq2, freq1, n_frames, phase_ratio2 * n_frames, interpolation);
                                // replace c2 by c to have a trill
                                o.playGeneric(c2, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol2, length }));
                            }
                        }
                    }
                    else {
                        if(auto * ramp = get_inactive_ramp()) {
                            ramp->algo.set(freq1, freq1, n_frames, phase_ratio1 * n_frames, interpolation);
                            o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol1, length }));
                        }
                        if(auto * ramp = get_inactive_ramp()) {
                            ramp->algo.set(freq2, freq2, n_frames, phase_ratio1 * n_frames, interpolation);
                            o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol2, length }));
                        }

                        if(!std::uniform_int_distribution<>{0,10}(rng::mersenne())) {
                            if(auto * ramp = get_inactive_ramp()) {
                                auto i = std::uniform_int_distribution<>{0,3}(rng::mersenne());
                                if(i==0) {
                                    freq2 *= 2.f;
                                }
                                else if(i==2) {
                                    freq2 *= 0.5f;
                                }
                                ramp->algo.set(freq2, freq2, n_frames, phase_ratio1 * n_frames, interpolation);

                                // make a helper for that!! if the first term of the pair is wrong, we got +Inf in the signal...
                                // if the first parameter of the request is buffer instead of &buffer[0], it is wrong...
                                o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol2, length }));
                            }
                        }
                        if(!std::uniform_int_distribution<>{0,10}(rng::mersenne())) {
                            if(auto * ramp = get_inactive_ramp()) {
                                ramp->algo.set(freq2, freq1, n_frames, interpolation);
                                o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume*vol2, length }));
                            }
                        }
                    }
                    return Status::OK_CHANGED;
                }
                
                void set_xfade(int xfade_) {
                    xfade = xfade_;
                }
                void set_base_freq(float freq_) {
                    base_freq = freq_;
                }
                void set_freq_scatter(float scatter_) {
                    freq_scatter = scatter_;
                }
                void set_d1(float d1_) {
                    d1 = d1_;
                }
                void set_d2(float d2_) {
                    d2 = d2_;
                }
                void set_mode(Mode m) {
                    mode = m;
                }
                void set_phase_ratio1(float phase_ratio1_) {
                    phase_ratio1 = clamp_phase_ratio(phase_ratio1_);
                }
                void set_phase_ratio2(float phase_ratio2_) {
                    phase_ratio2 = clamp_phase_ratio(phase_ratio2_);
                }
                void set_har_att(float har_att_) {
                    har_att = har_att_;
                    if(har_att > 0.99f) {
                        LG(WARN, "clamp har_att '%.1f' to 0.99f", har_att);
                        har_att = 0.99f;
                    }
                    else if(har_att < 0.f) {
                        LG(WARN, "clamp har_att '%.1f' to 0.f", har_att);
                        har_att = 0.f;
                    }
                }
                void set_length(float length_) {
                    length = length_;
                }
                void set_itp(itp::interpolation i) {
                    if(!itp::intIsReal(i)) {
                        i=itp::LINEAR;
                    }
                    interpolation = i;
                }
                bool get_active() const {
                    return active;
                }
                void set_active(bool b) {
                    active = b;
                }
                
                void set_channels(uint8_t c1, uint8_t c2) {
                    this->c1 = c1;
                    this->c2 = c2;
                }
                
            private:
                bool active : 1;
                Mode mode : 2;
                itp::interpolation interpolation : 5;
                std::function<ramp*(void)> get_inactive_ramp;
                float d1, d2, har_att, length, base_freq, freq_scatter, phase_ratio1, phase_ratio2;
                uint8_t c1, c2;
                int xfade;
                std::unique_ptr<MarkovChain> markov;
            };
            
            enum ImplParams {
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
            
            template<ImplParams e>
            struct Limits;
            
            template<int m, int M>
            constexpr auto do_normalize_i(float v) { return (v-m) / (M-m); }

            template<float const * const m, float const * const M>
            constexpr auto do_normalize_f(float v) { return (v-*m) / (*M-*m); }
            
            template<int m, int M>
            constexpr auto do_denormalize_i(float v) { return m + v * (M-m); }
            
            template<float const * const m, float const * const M>
            constexpr auto do_denormalize_f(float v) { return *m + v * (*M-*m); }
            
            template<ImplParams N>
            struct Normalizer {
                template<typename lim = Limits<N>, int m = lim::m, int M = lim::M>
                static float run(float v)
                {
                    return do_normalize_i<m, M>(v);
                }
                
                template<typename lim = Limits<N>, float const * const m = &lim::m, float const * const M = &lim::M>
                static float run(float v)
                {
                    return do_normalize_f<m, M>(v);
                }
            };
            
            template<ImplParams N>
            struct Denormalizer {
                template<typename lim = Limits<N>, int m = lim::m, int M = lim::M>
                static float run(float v)
                {
                    return do_denormalize_i<m, M>(v);
                }
                
                template<typename lim = Limits<N>, float const * const m = &lim::m, float const * const M = &lim::M>
                static float run(float v)
                {
                    return do_denormalize_f<m, M>(v);
                }
            };

            template<ImplParams N>
            constexpr auto normalize(float v) {
                return Normalizer<N>::run(v);
            }
            
            template<ImplParams N>
            constexpr auto denormalize(float v) {
                return Denormalizer<N>::run(v);
            }

            /*
            struct NormalizedParamLimits {
                static constexpr auto m = 0.f;
                static constexpr auto M = 1.f; };
            */
            
            template<> struct Limits<XFADE_LENGTH> {
                static constexpr auto m = 3;
                static constexpr auto M = 2001; };
            
            template<> struct Limits<FREQ_SCATTER> {
                static const float m;
                static const float M; };
            
            template<> struct Limits<D1> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };
            template<> struct Limits<D2> {
                static constexpr auto m = 0;
                static constexpr auto M = 47; };
            
            template<> struct Limits<PHASE_RATIO1> {
                static const float m;
                static const float M; };
            
            template<> struct Limits<PHASE_RATIO2> {
                static const float m;
                static const float M; };
            
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
                
                static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
                
                static constexpr auto size_interleaved = size_interleaved_one_cache_line;

            public:
                static std::vector<ParamSpec> const & getParamSpecs() {
                    static std::vector<ParamSpec> params_spec = {
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
                
                static Program::ARRAY make(int xfade,
                                           float freq_scat,
                                           float d1, float d2,
                                           float phase_ratio1, float phase_ratio2,
                                           float harmonic_attenuation,
                                           float length,
                                           itp::interpolation i) {
                    return {{
                        normalize<XFADE_LENGTH>(xfade),
                        normalize<FREQ_SCATTER>(freq_scat),
                        normalize<D1>(d1),
                        normalize<D2>(d2),
                        normalize<PHASE_RATIO1>(phase_ratio1),
                        normalize<PHASE_RATIO2>(phase_ratio2),
                        normalize<HARMONIC_ATTENUATION>(harmonic_attenuation),
                        normalize<LENGTH>(length),
                        static_cast<float>(i)
                    }};
                }
                static Programs const & getPrograms() {
                    static ProgramsI ps {{
                        {"",
                        make(401, .0f, 12.f, 24.f, 0.f, 0.f, .95f, 100.f, itp::EASE_INOUT_QUART)
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
                    A(index < getPrograms().size());
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

                using OutputData = outputDataBase<nAudioOut, XfadePolicy::UseXfade, NoOpLock>;
                using SoundEngine = SoundEngine<nAudioOut, imajuscule::Logger>;
                
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
                                                                          return true;
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
#ifndef NDEBUG
                    LG(WARN, "dropped note '%d'", pitch);
#endif
                    return onEventResult::DROPPED_NOTE;
                }
                
                onEventResult startNote(MonoNoteChannel & c, NoteOnEvent const & e) {
                    if(!c.open(out, 1.f, xfade_len())) {
                        return onDroppedNote(e.pitch);
                    }
                    c.elem.engine.set_channels(c.channels[0], c.channels[1]);
                    LG(INFO, "on  %d", e.pitch);

                    c.pitch = e.pitch;
                    c.tuning = e.tuning;
                    
                    c.elem.engine.set_xfade(xfade_len());
                    c.elem.engine.set_base_freq(midi::tuned_note(c.pitch, c.tuning));
                    c.elem.engine.set_freq_scatter(denorm<FREQ_SCATTER>());
                    c.elem.engine.set_d1(denorm<D1>());
                    c.elem.engine.set_d2(denorm<D2>());
                    c.elem.engine.set_mode(SoundEngine::Mode::MARKOV);
                    c.elem.engine.set_phase_ratio1(denorm<PHASE_RATIO1>());
                    c.elem.engine.set_phase_ratio2(denorm<PHASE_RATIO2>());
                    c.elem.engine.set_har_att(denorm<HARMONIC_ATTENUATION>());
                    c.elem.engine.set_length(denorm<LENGTH>());
                    
                    auto interp = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + params[INTERPOLATION])]);
                    
                    c.elem.engine.set_itp(interp);
                    c.elem.engine.set_active(true);
                    
                    c.elem.engine.update(out, true);
                    
                    return onEventResult::OK;
                }
                
            public:
                void allNotesOff() override {
                    for(auto & c : channels) {
                        c.close(out, CloseMode::XFADE_ZERO, xfade_len());
                    }
                }
                
                void allSoundsOff() override {
                    for(auto & c : channels) {
                        c.close(out, CloseMode::NOW);
                    }
                }
                
            private:
                // notes are identified by pitch
                onEventResult noteOff(uint8_t pitch) {
                    LG(INFO, "off %d", pitch);
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
