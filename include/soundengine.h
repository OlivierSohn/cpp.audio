
namespace imajuscule {
    namespace audio {
        soundBuffer & getSilence();

        enum class FreqXfade : unsigned char {
            BEGIN,
            
            No=BEGIN,
            NonTrivial,
            All,
            
            END
            };
            
            
        enumTraversal const & xfade_freq_traversal();
            
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
            FORCE_SOUND_AT_EACH_UPDATE, // "virtual instrument" mode
            UPDATE_CAN_BE_SILENT // "script" mode
        };

        enum SoundEngineMode : unsigned char {
            BEGIN=0,
            
            BIRDS = 0,
            ROBOTS,
            SWEEP,
            WIND,
            
            END
        };

        enum SoundEngineInitPolicy : unsigned char {
            StartAtLastPosition,
            StartAfresh
        };

            template<typename ITER>
            struct SoundEngineFreqCtrl {
                using FPT = typename ITER::FPT;
                using T = FPT;
                
                SoundEngineFreqCtrl() {
                    invApproxRange = 1.f / (2.f * ctrl.getAbsMean());
                }
                
                void initializeForRun() {
                    ctrl.initializeForRun();
                }

                void setFiltersOrder(int order) {
                    ctrl.setFiltersOrder(order);
                }
                
                void forgetPastSignals() {
                    ctrl.forgetPastSignals();
                }
                
                void setAngleIncrements(T v) {
                    ctrl.setAngleIncrements(v);
                }
                
                void set_interpolation(itp::interpolation i) {
                    ctrl.set_interpolation(i);
                }
                
                T step() {
                    ctrl.step();
                    auto v = ctrl.imag() * invApproxRange;
                    A(v >= 0.f); // else, use AbsIter
                    constexpr auto fmax = 10000.f;
                    auto exponent = .6f + .4f * v;
                    /*
                    static auto deb = 0;
                    ++deb;
                    if(deb == 10000) {
                        deb = 0;
                        LG(INFO, "exp: %.2f", exponent);
                    }
                     */
                    return powf(fmax, exponent);
                }
                
                auto get_duration_in_samples() const {
                    return std::numeric_limits<int>::max()
                    / 2;  // so that we can add .5f and cast to int
                }
               
                // todo remove and make SoundEngine more generic
                void set(T from_,
                         T to_,
                         T duration_in_samples_,
                         T start_sample,
                         itp::interpolation i) {
                    A(0);
                }
                void set_by_increments(T from_,
                          T to_,
                          T duration_in_samples_,
                          T start_sample,
                          itp::interpolation i) {
                    A(0);
                }
                float getFrom() const {
                    A(0);
                    return 0.f;
                }
                float getTo() const {
                    A(0);
                    return 0.f;
                }

            private:
                float invApproxRange;
                audioelement::Ctrl<ITER> ctrl;
            };
            
            
            template<typename AEAlgo, int ORDER, typename ITER>
            using AsymBandPassAlgo = audioelement::FreqCtrl_<
            audioelement::BandPassAlgo<AEAlgo, audioelement::Ctrl<ITER>, ORDER>,
            SoundEngineFreqCtrl<ITER>
            >;
            
            template<typename AEAlgo, int ORDER, typename ITER>
            using AsymBandRejectAlgo = audioelement::FreqCtrl_<
            audioelement::BandRejectAlgo<AEAlgo, audioelement::Ctrl<ITER>, ORDER>,
            SoundEngineFreqCtrl<ITER>
            >;
            
            template<typename T, SoundEngineMode>
            struct SoundEngineAlgo_ {
                using CTRL = audioelement::LogRamp< typename T::T >;
                using type = audioelement::FreqCtrl_< T, CTRL >;
            };
            
            template<typename T>
            struct SoundEngineAlgo_<T, SoundEngineMode::WIND> {
                using CTRL = SoundEngineFreqCtrl< PinkNoiseIter >;
                using type = audioelement::FreqCtrl_< T, CTRL >;
            };
            
        template<SoundEngineMode M, typename Logger>
        struct SoundEngine {
            static constexpr auto Order = VariableOrder;
            
            using GreyNoiseAlgo = audioelement::soundBufferWrapperAlgo<Sound::GREY_NOISE>;
            using PinkNoiseAlgo = audioelement::soundBufferWrapperAlgo<Sound::PINK_NOISE>;
            
            // should we chose pink or grey here? todo audio tests... maybe the right answer is neither, or pink filtered to model wind
            using Mix = audioelement::Mix <
            audioelement::LowPassAlgo<GreyNoiseAlgo, Order>,
            AsymBandPassAlgo<GreyNoiseAlgo, Order, PinkNoiseIter>,
            AsymBandRejectAlgo<GreyNoiseAlgo, Order, PinkNoiseIter>,
            audioelement::AdjustableVolumeOscillatorAlgo<audioelement::VolumeAdjust::Yes,float>
            >;
            
            using Algo = typename SoundEngineAlgo_<Mix, M>::type;
            
            using audioElt = audioelement::FinalAudioElement< Algo >;
                        
            static enumTraversal ModeTraversal;
            
            template<typename F>
            SoundEngine(F f) :
            get_inactive_ramp(std::move(f)),
            active(false),
            xfade_freq(FreqXfade::No)
            {
                getSilence(); // make sure potential dynamic allocation occurs not under audio lock
            }
            
            template<typename OutputData>
            void play(OutputData & o,
                      float length, float freq1, float freq2,
                      float phase_ratio1, float phase_ratio2,
                      float freq_scatter) {
                length *= powf(2.f,
                               std::uniform_real_distribution<float>{min_exp, max_exp}(rng::mersenne()));
                auto n_frames = static_cast<float>(ms_to_frames(length));
                if(n_frames <= 0) {
                    Logger::err("zero length");
                    return;
                }
                
                auto * current = ramp_specs.get_current();
                
                if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                    if(state_freq == freq1) {
                        // use previous scatter when the markov transitions specify the same base value
                    }
                    else {
                        auto scatter = 1.f + freq_scatter;
                        state_factor = std::uniform_real_distribution<float>{1.f / scatter, scatter}(rng::mersenne());
                    }
                    state_freq = freq2;
                    freq1 *= state_factor;
                    freq2 *= state_factor;
                    
                    ramp_spec->set(freq1, freq2, n_frames, 0, interpolation);
                    if(xfade_freq==FreqXfade::No) {
                        // don't try to solve frequency discontinuity
                        return;
                    }
                    if(current) {
                        // there was a spec before the one we just added...
                        auto from_inc = current->getTo();
                        auto to_inc = ramp_spec->getFrom();
                        auto diff = from_inc - to_inc;
                        if(xfade_freq==FreqXfade::All || diff) {
                            // ... and the new spec creates a frequency discontinuity
                            if(auto * ramp_spec2 = ramp_specs.get_next_ramp_for_build()) {
                                // so we move the new spec one step later
                                *ramp_spec2 = *ramp_spec;
                                // and create a transition
                                if(from_inc == to_inc) {
                                    // comment on next line :  I think this is obsolete,
                                    // now we don't detect the end that way anymore
                                    // (todo remove and test with an instrument that uses that)
                                    from_inc *= 1.00001f; // make sure ramp is non trivial else we cannot detect when it's done
                                }
                                ramp_spec->set(from_inc, to_inc, freq_xfade, 0, freq_interpolation);
                            }
                            else {
                                // just discard it
                                ramp_specs.cancel_last_ramp();
                            }
                        }
                    }
                }
                else {
                    A(0);
                }
            }
            
            template<typename OutputData>
            auto create_markov(OutputData & o) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                auto mc = std::make_unique<MarkovChain>();
                
                auto node1 = mc->emplace([](Move const m, MarkovNode&me, MarkovNode&from_to) {
                });
                auto node2 = mc->emplace([&o, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m==Move::ENTER) {
                        play(o, length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
                    }
                    else {
                        play(o, length, base_freq*2, base_freq*4, phase_ratio1, phase_ratio2, freq_scatter);
                    }
                });
                auto node3 = mc->emplace([&o, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m==Move::ENTER) {
                        play(o, length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
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
                
                return mc;
            }
            
            template<typename OutputData>
            auto create_robot(OutputData & o) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                auto mc = std::make_unique<MarkovChain>();
                
                auto node0 = mc->emplace([this, &o](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m == Move::LEAVE) {
                        auto length = this->length;
                        length *= powf(2.f,
                                       std::uniform_real_distribution<float>{min_exp, max_exp}(rng::mersenne()));
                        auto n_frames = static_cast<float>(ms_to_frames(length));
                        if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                            ramp_spec->set(freq1_robot, freq1_robot, n_frames, phase_ratio1 * n_frames, interpolation);
                        }
                        if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                            ramp_spec->set(freq2_robot, freq2_robot, n_frames, phase_ratio1 * n_frames, interpolation);
                        }
                    }
                });
                auto node1 = mc->emplace([](Move const m, MarkovNode&me, MarkovNode&from_to) {
                });
                auto node2 = mc->emplace([&o, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m==Move::ENTER) {
                        auto length = this->length;
                        length *= powf(2.f,
                                       std::uniform_real_distribution<float>{min_exp, max_exp}(rng::mersenne()));
                        auto n_frames = static_cast<float>(ms_to_frames(length));
                        if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                            // repeat f2...
                            auto i = std::uniform_int_distribution<>{0,3}(rng::mersenne());
                            // ... at same or twice or half freq
                            auto freq2 = freq2_robot;
                            if(i==0) {
                                freq2 *= 2.f;
                            }
                            else if(i==2) {
                                freq2 *= 0.5f;
                            }
                            ramp_spec->set(freq2, freq2, n_frames, phase_ratio1 * n_frames, interpolation);
                        }
                    }
                });
                auto node3 = mc->emplace([&o, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m==Move::ENTER) {
                        auto length = this->length;
                        length *= powf(2.f,
                                       std::uniform_real_distribution<float>{min_exp, max_exp}(rng::mersenne()));
                        auto n_frames = static_cast<float>(ms_to_frames(length));
                        if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                            ramp_spec->set(freq2_robot, freq1_robot, n_frames, phase_ratio1 * n_frames, interpolation);
                        }
                    }
                });
                
                // 0->1 = 1
                // 1->2 = 0.1
                // 2->3 = 0.1 or 2->1 = 1
                // 1->3 = 0.1
                
                //                  3
                //                  |
                //                  ^
                //                 0.1
                //                  ^
                //                  |
                // 0 --- > 1 > ---- 1---\
                //                  |   |
                //                  v   ^
                //                 0.1  1
                //                  v   ^
                //                  |   |
                //                  2---/
                
                def_markov_transition(node0, node1, 1.f);
                def_markov_transition(node1, node2, 0.1f);
                def_markov_transition(node2, node1, 1.f);
                def_markov_transition(node1, node3, 0.1f);
                def_markov_transition(node3, node1, 1.f);
                
                return mc;
            }
            
            template<typename OutputData>
            auto create_sweep(OutputData & o) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                auto mc = std::make_unique<MarkovChain>();
                
                auto node0 = mc->emplace([this, &o](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m == Move::LEAVE) {
                        auto length = this->length;
                        length *= powf(2.f,
                                       std::uniform_real_distribution<float>{min_exp, max_exp}(rng::mersenne()));
                        auto n_frames = static_cast<float>(ms_to_frames(length));
                        if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                            ramp_spec->set(freq1_robot, freq2_robot, n_frames, phase_ratio1 * n_frames, interpolation);
                        }
                    }
                });
                auto node1 = mc->emplace([](Move const m, MarkovNode&me, MarkovNode&from_to) {
                });
                
                def_markov_transition(node0, node1, 1.f);
                
                return mc;
            }
                        
            template<typename OutputData>
            auto create_wind(OutputData & o) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                auto mc = std::make_unique<MarkovChain>();
                
                auto node0 = mc->emplace([this, &o](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                        ramp_spec->set_interpolation(interpolation);
                    }
                });
                auto node1 = mc->emplace([](Move const m, MarkovNode&me, MarkovNode&from_to) {
                });
                
                def_markov_transition(node0, node1, 1.f);
                def_markov_transition(node1, node0, 1.f);
                
                return mc;
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            bool orchestrate(OutputData &out, MonoNoteChannel & c, int max_frame_compute) {
                bool done = false;
                if(onAfterCompute(out, max_frame_compute, done)) {
                    return !done;
                }
                c.template close<WithLock::No>(out, CloseMode::XFADE_ZERO, xfade);
                return false;
            }
            
            // returns false if channel needs to be closed
            template<typename OutputData>
            bool onAfterCompute(OutputData &out, int max_frame_compute, bool & done) {
                done = true;
                if(ramp_specs.done()) {
                    return true; // channel is already closed or closing
                }

                auto & channel = out.editChannel(c1);
                if(!channel.isPlaying()) {
                    return true;
                }
                done = false;
                if(!channel.get_requests().empty()) {
                    return true;
                }
                
                // is current request soon xfading?
                if(channel.get_remaining_samples_count() < max_frame_compute + 1 + channel.get_size_half_xfade())
                {
                    if(state_silence) {
                        state_silence = false;
                        done = !playNextSpec(out);
                    }
                    else {
                        bool const has_silence = articulative_pause_length > 2*channel.get_size_xfade();
                        if(has_silence) {
                            state_silence = true;
                            playSilence(out, getSilence());
                            return true;
                        }
                        done = !playNextSpec(out);
                    }
                }

                return !done;
            }
            
            template<typename OutputData>
            bool playNextSpec(OutputData & out, Volumes<OutputData::nOuts> const * vol = nullptr) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;

                auto & channel = out.editChannel(c1);

                while(auto new_spec = ramp_specs.get_next_ramp_for_run()) {
                    last_ramp_index = 1-last_ramp_index;
                    auto new_ramp = get_inactive_ramp();
                    A(new_ramp); // might be null if length of ramp is too small
                    ramp_[last_ramp_index] = new_ramp;
                    new_ramp->algo.getCtrl() = *new_spec;
                    new_ramp->algo.getCtrl().initializeForRun();
                    
                    if(out.playGenericNoLock(c1,
                                             std::make_pair(std::ref(*new_ramp),
                                                            Request{
                                                                &new_ramp->buffer[0],
                                                                vol? *vol : channel.get_current().volumes * (1.f/Request::chan_base_amplitude),
                                                                static_cast<int>(.5f + new_ramp->algo.getCtrl().get_duration_in_samples())
                                                            })
                                             ))
                    {
                        return true; // channel is playing
                    }
                    // cancel the index change
                    last_ramp_index = 1-last_ramp_index;
                }
                return false; // close channel
            }
            
            template<typename OutputData>
            void playSilence(OutputData & out, soundBuffer & silence) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;

                auto & channel = out.editChannel(c1);
                
                A(articulative_pause_length > 2*channel.get_size_xfade());
                
                auto res = out.playGenericNoLock(c1,
                                                 std::make_pair(std::ref(silence),
                                                                Request{
                                                                    &silence,
                                                                    // to propagate the volume of previous spec to the next spec
                                                                    channel.get_current().volumes * (1.f/Request::chan_base_amplitude),
                                                                    articulative_pause_length
                                                                })
                                                 );
                A(res); // because length was checked
            }
            
            void set_xfade(int xfade_) {
                xfade = xfade_;
            }
            void set_freq_xfade(int xfade_) {
                freq_xfade = xfade_;
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
            
            void set_freq_interpolation(itp::interpolation i) {
                if(!itp::intIsReal(i)) {
                    i=itp::EASE_IN_EXPO;
                }
                freq_interpolation = i;
            }

            void set_length_exp(float min_, float max_) {
                min_exp = min_;
                max_exp = max_;
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
            
            using InitPolicy = SoundEngineInitPolicy;
            
            template<typename OutputData, typename MonoNoteChannel>
            void initialize_sweep(OutputData & o,
                                  MonoNoteChannel & c,
                                  float low, float high,
                                  Volumes<OutputData::nOuts> const & volume) {
                bool initialize = true;
                if(!markov) {
                    markov = create_sweep(o);
                    if(!markov) {
                        return;
                    }
                }
                
                freq1_robot = low;
                freq2_robot = high;
                
                do_initialize(o, c, initialize, 0, 0, 1, 0, articulative_pause_length, volume);
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            void initialize_markov(OutputData & o,
                                   MonoNoteChannel & c,
                                   int start_node, int pre_tries, int min_path_length, int additional_tries, InitPolicy init_policy,
                                   FreqXfade xfade_freq, int articulative_pause_length,
                                   Volumes<OutputData::nOuts> const & volume) {
                this->xfade_freq = xfade_freq;
                
                bool initialize = (!markov) || (init_policy==InitPolicy::StartAfresh);
                if(!markov) {
                    markov = create_markov(o);
                    if(!markov) {
                        return;
                    }
                }
                
                do_initialize(o, c, initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, volume);
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            void initialize_wind(OutputData & o,
                                 MonoNoteChannel & c,
                                 int start_node, int pre_tries, int min_path_length, int additional_tries, InitPolicy init_policy,
                                 Volumes<OutputData::nOuts> const & volume) {
                bool initialize = (!markov) || (init_policy==InitPolicy::StartAfresh);
                if(!markov) {
                    markov = create_wind(o);
                    if(!markov) {
                        return;
                    }
                }
                
                do_initialize(o, c, initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, volume);
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            void initialize_robot(OutputData & o,
                                  MonoNoteChannel & c,
                                  int start_node, int pre_tries, int min_path_length, int additional_tries, InitPolicy init_policy,
                                  int articulative_pause_length,
                                  Volumes<OutputData::nOuts> const & volume) {
                auto scatter = 1.f + freq_scatter;
                freq1_robot = std::uniform_real_distribution<float>{base_freq / scatter, base_freq * scatter}(rng::mersenne());
                freq2_robot = std::uniform_real_distribution<float>{freq1_robot*0.97f, freq1_robot/0.97f}(rng::mersenne());
                
                auto ht = compute_half_tone(1.f);
                
                if(!std::uniform_int_distribution<>{0,1}(rng::mersenne())) {
                    // f1 is shifted up by d1
                    freq1_robot = transpose_frequency(freq1_robot, ht, d1);
                }
                else {
                    // f2 is shifted up by d2
                    freq2_robot = transpose_frequency(freq2_robot, ht, d2);
                }
                
                auto n_frames = static_cast<float>(ms_to_frames(length));
                if(n_frames <= 0) {
                    Logger::err("length '%f' is too small", length);
                    return;
                }
                
                bool initialize = (!markov) || (init_policy==InitPolicy::StartAfresh);
                if(!markov) {
                    markov = create_robot(o);
                    if(!markov) {
                        return;
                    }
                }
                
                do_initialize(o, c, initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, volume);
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            void do_initialize(OutputData & o,
                               MonoNoteChannel & c,
                               bool initialize,
                               int start_node, int pre_tries, int min_path_length, int additional_tries,
                               int articulative_pause_length,
                               Volumes<OutputData::nOuts> const & volume)
            {
                using Request = typename OutputData::Request;

                auto n_frames = static_cast<float>(ms_to_frames(length));
                if(n_frames <= 0) {
                    Logger::err("length '%f' is too small", length);
                    return;
                }
                
                if(base_freq <= 0.f) {
                    Logger::err("frequency '%d' should be sctrictly positive", base_freq);
                    return;
                }

                this->state_freq = 0.f;
                this->state_factor = 0.f;
                
                ramp_specs.reset();
                this->articulative_pause_length = articulative_pause_length;
                state_silence = false;
                
                // running the markov chain will populate ramp_specs
                if(initialize) {
                    markov->initialize(start_node);
                    
                    for(int i=0; i<pre_tries; ++i) {
                        markov->step_normalized<false>();
                    }
                }
                
                for(int i=0; i<min_path_length; ++i) {
                    markov->step_normalized<true>();
                }
                
                for(int i=0; i<additional_tries; ++i) {
                    markov->step<true>();
                }
                
                ramp_specs.finalize();
                
                o.add_orchestrator([&o, &c, this](int max_frame_compute){
                    return orchestrate(o, c, max_frame_compute);
                });
                
                playNextSpec(o, &volume);
            }
            
        private:
            bool active : 1;
            
            itp::interpolation interpolation : 5;
            itp::interpolation freq_interpolation : 5;
            std::function<audioElt*(void)> get_inactive_ramp;
            float d1, d2, har_att, length, base_freq, freq_scatter, phase_ratio1=0.f, phase_ratio2=0.f;
            float min_exp;
            float max_exp;
            
            float state_freq = 0.f;
            float state_factor = 0.f;
            
            float freq1_robot, freq2_robot; // used also for sweep

            uint8_t c1, c2;
            int xfade, freq_xfade;
            int articulative_pause_length;
            
            std::unique_ptr<MarkovChain> markov;
            FreqXfade xfade_freq:2;
            
            std::array<audioElt *, 2> ramp_= {};
            unsigned int last_ramp_index : 1;
            bool state_silence:1;
            
            struct RampSpecs {
                static constexpr auto n_specs = 30;
                static constexpr auto n_bits_iter = relevantBits(n_specs+1);
                static constexpr auto iter_cycle_length = pow2(n_bits_iter);
                
                void reset() {
                    it = -1;
                    end = 0;
                }
                
                bool done() const { return it == end; }
                
                using CTRLS = typename Algo::Ctrl;
                static_assert(std::tuple_size<CTRLS>::value == 1,"multi freq not supported");
                using Ctrl = typename std::tuple_element<0, CTRLS>::type;
                
                Ctrl * get_next_ramp_for_build() {
                    ++it;
                    A(it==end);
                    if(it == n_specs) {
                        --it;
                        return nullptr;
                    }
                    ++end;
                    return &a[it];
                }
                
                void cancel_last_ramp() {
                    --it;
                    --end;
                }
                
                Ctrl * get_current() {
                    if(it >= end) {
                        return nullptr;
                    }
                    return &a[it];
                }
                void finalize() {
                    A(0 == (it+1-end) % iter_cycle_length);
                    it=-1;
                }
                
                Ctrl * get_next_ramp_for_run() {
                    A(it != end);
                    ++it;
                    if(it == end) {
                        return nullptr;
                    }
                    auto ptr_spec = &a[it];
                    return ptr_spec;
                }
                
                using Ctrls = std::array<Ctrl, n_specs>;
                unsigned it : n_bits_iter;
                unsigned end: relevantBits(n_specs+1);
                Ctrls a;
            } ramp_specs;
        };
        
    }
} // namespaces
