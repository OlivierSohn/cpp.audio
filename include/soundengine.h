
namespace imajuscule {
    namespace audio {
        
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
        enum class UpdateMode {
            FORCE_SOUND,
            CAN_SKIP_SOUND
        };
        
        template<typename Logger, UpdateMode U = UpdateMode::CAN_SKIP_SOUND>
        struct SoundEngine {
            using ramp = audioelement::FreqRamp<float>;
            
            enum Mode : unsigned char{
                BEGIN=0,
                
                MARKOV = 0,
                ROBOTS,
                BIRDS,
                
                END
            };
            
            static enumTraversal ModeTraversal;
            
            template<typename F>
            SoundEngine(F f) :
            get_inactive_ramp(std::move(f)),
            active(false)
            {}
            
            template<typename OutputData>
            Status update(OutputData & o)
            {
                constexpr bool force = U==UpdateMode::FORCE_SOUND;
                
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                
                static_assert(nAudioOut == OutputData::nOuts, "");
                if(!this->active) {
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
                            
                            if(freq_scatter) {
                                auto factor = 1.f + std::uniform_real_distribution<float>{0.f,freq_scatter}(rng::mersenne());
                                freq1 *= factor;
                                freq2 *= factor;
                            }
                            
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
                
                if(!force && std::uniform_int_distribution<>{0,2}(rng::mersenne())) {
                    return Status::OK_STABLE;
                }
                
                static float ht = compute_half_tone(1.f);
                
                auto vol1 = std::uniform_real_distribution<float>{0.5f,1.5f}(rng::mersenne());
                auto vol2 = vol1;
                
                auto freq1 = std::uniform_real_distribution<float>{base_freq, (1+freq_scatter) * base_freq}(rng::mersenne());
                auto freq2 = std::uniform_real_distribution<float>{freq1*0.97f, freq1/0.97f}(rng::mersenne());
                
                if((force && !std::uniform_int_distribution<>{0,5}(rng::mersenne())) ||
                   (!force && !std::uniform_int_distribution<>{0,5}(rng::mersenne()))) {
                    freq1 = transpose_frequency(freq1, ht, d1);
                    vol1 *= expt(har_att, d1);
                }
                else if(force || !std::uniform_int_distribution<>{0,5}(rng::mersenne())) {
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
        
    }
} // namespaces
