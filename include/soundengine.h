
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
            FORCE_SOUND_AT_EACH_UPDATE, // "virtual instrument" mode
            UPDATE_CAN_BE_SILENT // "script" mode
        };
        
        template<typename Logger, UpdateMode U = UpdateMode::UPDATE_CAN_BE_SILENT>
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
            {
                getSilence(); // make sure potential dynamic allocation occur not under audio lock
            }
            
            template<typename OutputData>
            Status update(OutputData & o)
            {
                constexpr bool force = U==UpdateMode::FORCE_SOUND_AT_EACH_UPDATE;
                
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
                    return Status::OK_STABLE;
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
            
            template<typename OutputData>
            auto create_markov(OutputData & o) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;
                auto mc = std::make_unique<MarkovChain>();
                
                auto play = [this, &o](float length, float freq1, float freq2,
                                       float phase_ratio1, float phase_ratio2,
                                       float freq_scatter) {
                    length *= powf(2.f,
                                   std::uniform_real_distribution<float>{1.f, 3.f}(rng::mersenne()));
                    auto n_frames = static_cast<float>(ms_to_frames(length));
                    if(n_frames <= 0) {
                        Logger::err("zero length");
                        return;
                    }
                    
                    if(freq_scatter) {
                        auto factor = 1.f + std::uniform_real_distribution<float>{0.f,freq_scatter}(rng::mersenne());
                        freq1 *= factor;
                        freq2 *= factor;
                    }
                    
                    auto * current = ramp_specs.get_current();
                    
                    if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
                        ramp_spec->set(freq1, freq2, n_frames, 0, interpolation);
                        if(!xfade_freq) {
                            // in this mode, we don't try to solve frequency discontinuity
                            return;
                        }
                        if(current) {
                            // there was a spec before the one we just added...
                            auto from_inc = current->getToIncrements();
                            auto to_inc = ramp_spec->getFromIncrements();
                            auto diff = from_inc - to_inc;
                            if(diff) {
                                // ... and the new spec creates a frequency discontinuity
                                if(auto * ramp_spec2 = ramp_specs.get_next_ramp_for_build()) {
                                    // so we move the new spec one step later
                                    *ramp_spec2 = *ramp_spec;
                                    // and create a transition
                                    ramp_spec->set_by_increments(from_inc, to_inc, freq_xfade, 0, freq_interpolation);
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
                };
                
                auto node1 = mc->emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                });
                auto node2 = mc->emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
                    if(m==Move::ENTER) {
                        play(length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
                    }
                    else {
                        play(length, base_freq*2, base_freq*4, phase_ratio1, phase_ratio2, freq_scatter);
                    }
                });
                auto node3 = mc->emplace_([play, this](Move const m, MarkovNode&me, MarkovNode&from_to) {
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
                
                return mc;
            }
            
            auto & getSilence() {
                static soundBuffer silence{1, 0.f};
                return silence;
            }
            
            template<typename OutputData, typename MonoNoteChannel>
            bool orchestrate(OutputData &out, MonoNoteChannel & c, int max_frame_compute) {
                if(onAfterCompute(out, max_frame_compute)) {
                    return true;
                }
                c.close(out, CloseMode::XFADE_ZERO, xfade);
                return false;
            }
            
            template<typename OutputData>
            bool onAfterCompute(OutputData &out, int max_frame_compute) {
                if(ramp_specs.done()) {
                    return true; // channel is already closed or closing
                }

                typename OutputData::Locking l(out.get_lock());

                auto & channel = out.editChannel(c1);
                if(!channel.isPlaying()) {
                    return true;
                }
                if(!channel.get_requests().empty()) { // I'm not sure this test is usefull, plus it can make a cache miss...
                    return true;
                }
                
                bool has_silence = ( articulative_pause_length > 2*channel.get_size_xfade() );
                
                if(state_silence) {
                    // is silence soon over?
                    if(channel.get_remaining_samples_count() < max_frame_compute + 1 + channel.get_size_half_xfade()) {
                        // yes
                        state_silence = false;
                        return playNextSpec(out);
                    }
                }
                else {
                    // is spec soon over?
                    auto & spec = ramp_[last_ramp_index]->algo.spec;
                    if(ramp_specs.order != (spec.getFromIncrements() < spec.getToIncrements())) {
                        // bounds were swapped, meaning the halfperiod was met.
                        if(has_silence) {
                            state_silence = true;
                            playSilence(out, getSilence());
                            return true;
                        }
                        return playNextSpec(out);
                    }
                }

                return true; // channel is playing
            }
            
            template<typename OutputData>
            bool playNextSpec(OutputData & out) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = Request<nAudioOut>;

                auto & channel = out.editChannel(c1);
                auto & spec = ramp_[last_ramp_index]->algo.spec;

                while(auto new_spec = ramp_specs.get_next_ramp_for_run()) {
                    last_ramp_index = 1-last_ramp_index;
                    auto new_ramp = get_inactive_ramp();
                    A(new_ramp); // might be null if length of ramp is too small
                    ramp_[last_ramp_index] = new_ramp;
                    new_ramp->algo.spec = *new_spec;
                    
                    if(out.playGenericNoLock(c1,
                                             std::make_pair(std::ref(*new_ramp),
                                                            Request{
                                                                &new_ramp->buffer[0],
                                                                channel.get_current().volumes * (1.f/Request::chan_base_amplitude),
                                                                std::numeric_limits<int>::max()
                                                            })
                                             ))
                    {
                        channel.xfade_now();
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
                channel.xfade_now();
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
            
            void set_freq_interpolation(itp::interpolation i) {
                if(!itp::intIsReal(i)) {
                    i=itp::EASE_IN_EXPO;
                }
                freq_interpolation = i;
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
            
            enum InitPolicy : unsigned char {
                StartAtLastPosition,
                StartAfresh
            };
            
            template<typename OutputData, typename MonoNoteChannel>
            void initialize_markov(OutputData & o,
                                   MonoNoteChannel & c,
                                   int start_node, int pre_tries, int min_path_length, int additional_tries, InitPolicy init_policy,
                                   bool xfade_freq, int articulative_pause_length) {
                constexpr auto nAudioOut = OutputData::nOuts;
                using Request = typename OutputData::Request;
                
                this->xfade_freq = xfade_freq;
                ramp_specs.reset();
                this->articulative_pause_length = articulative_pause_length;
                state_silence = false;
                
                bool initialize = (!markov) || (init_policy==InitPolicy::StartAfresh);
                if(!markov) {
                    markov = create_markov(o);
                }
                
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
                auto const stereo_gain = stereo(std::uniform_real_distribution<float>(-1.f, 1.f)(rng::mersenne()));
                auto volume = MakeVolume::run<nAudioOut>(1.f, stereo_gain);
                
                last_ramp_index = 1-last_ramp_index;
                auto & ramp = ramp_[last_ramp_index];
                ramp = get_inactive_ramp();
                A(ramp);
                auto spec = ramp_specs.get_next_ramp_for_run();
                if(!spec) {
                    return;
                }
                o.add_orchestrator([&o, &c, this](int max_frame_compute){
                    return orchestrate(o, c, max_frame_compute);
                });
                
                ramp->algo.spec = *spec;
                if(o.playGeneric(c1, std::make_pair(std::ref(*ramp), Request{&ramp->buffer[0], volume, std::numeric_limits<int>::max() }))) {
                    state_silence = false;
                }
                else {
                    A(0);
                }
            }
            
        private:
            bool active : 1;
            Mode mode : 2;
            itp::interpolation interpolation : 5;
            itp::interpolation freq_interpolation : 5;
            std::function<ramp*(void)> get_inactive_ramp;
            float d1, d2, har_att, length, base_freq, freq_scatter, phase_ratio1, phase_ratio2;
            uint8_t c1, c2;
            int xfade, freq_xfade;
            int articulative_pause_length;
            
            std::unique_ptr<MarkovChain> markov;
            bool xfade_freq:1;
            
            std::array<ramp *, 2> ramp_= {};
            unsigned int last_ramp_index : 1;
            bool state_silence:1;
            
            struct RampSpecs {
                void reset() {
                    it = -1;
                    end = 0;
                }
                
                bool done() const { return it == end; }
                
                using Algo = audioelement::FreqRampAlgo<float>;
                static constexpr auto n_specs = 30;
                
                Algo::Spec * get_next_ramp_for_build() {
                    ++it;
                    A(it==end);
                    if(it == n_specs) {
                        return nullptr;
                    }
                    ++end;
                    return &a[it];
                }
                
                void cancel_last_ramp() {
                    --it;
                    --end;
                }
                
                Algo::Spec * get_current() {
                    if(it >= end) {
                        return nullptr;
                    }
                    return &a[it];
                }
                void finalize() {
                    A(it==end-1);
                    it=-1;
                }
                
                Algo::Spec * get_next_ramp_for_run() {
                    A(it != end);
                    ++it;
                    if(it == end) {
                        return nullptr;
                    }
                    auto ptr_spec = &a[it];
                    order = ptr_spec->getFromIncrements() < ptr_spec->getToIncrements();
                    return ptr_spec;
                }
                
                using Specs = std::array<Algo::Spec, n_specs>;
                unsigned it : relevantBits(n_specs+1);
                unsigned end: relevantBits(n_specs+1);
                Specs a;
                bool order :1;
            } ramp_specs;
        };
        
    }
} // namespaces
