
namespace imajuscule {
    namespace audio {
        namespace cutramp {
            
            /*
             * When cut size is bigger than period, period is automatically adjusted
             *  to cut size + 1
             */
            enum ImplParams {
                // cut
                CUT_ADJUST_FREQ,
                CUT_PERIOD,
                CUT_SIZE,
                CUT_XFADE_AMOUNT,
                
                // mixed
                CUT_ADJUST_RAMP_SIZE,
                
                // ramp
                RAMP_AMOUNT,
                RAMP_INTERPOLATION,
                RAMP_SPEED,
                RAMP_START_FREQ,
                
                NPARAMS // keep last
            };
            
            template<typename T>
            constexpr auto get_middle(T p, T g) {
                static_assert(std::is_integral<T>::value, "");
                return ((p+1)-g)/2;
            }
            
            template<
            int nAudioOut,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename Base
            >
            struct ImplCRTP : public Base {
                
                using Base::get_xfade_length;
                using Base::onStartNote;

                using Event = typename EventIterator::object;

                static constexpr auto n_max_voices = 8;
                
                // notes played in rapid succession can have a common audio interval during xfades
                // even if their noteOn / noteOff intervals are disjoint.
                // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well':
                static constexpr auto n_max_simultaneous_notes_per_voice = 2;
                static constexpr auto n_channels = n_max_voices * n_max_simultaneous_notes_per_voice;

                using MonoNoteChannel = MonoNoteChannel<1, audioelement::FreqRamp<float>>;
                using OutputData = outputDataBase<nAudioOut, XfadePolicy::SkipXfade, NoOpLock, PostProcess::NONE>;

                void allNotesOff() override {
                    auto len =  get_xfade_length();
                    for(auto & c : channels) {
                        c.close(out, CloseMode::XFADE_ZERO, len);
                    }
                }
                
                void allSoundsOff() override {
                    for(auto & c : channels) {
                        c.close(out, CloseMode::NOW);
                    }
                }
                
                onEventResult onEvent(Event const & e) {
                    if(e.type == Event::kNoteOnEvent) {
                        // this case is handled by the wrapper...
                        A(e.noteOn.velocity > 0.f );
                        // ... in case it were not handled by the wrapper we would need to do:
                        // return noteOff(e.noteOn.pitch);
                        
                        if(auto c = editInactiveAudioElementContainer(channels,
                                                                      [](auto & c) -> auto & {
                                                                          return c.elem;
                                                                      })) {
                                                                          return startNote(*c, e.noteOn);
                                                                      }
                        else {
                            return onDroppedNote(e.noteOn.pitch);
                        }
                    }
                    
                    if(e.type == Event::kNoteOffEvent) {
                        return noteOff(e.noteOff.pitch);
                    }
                    return onEventResult::UNHANDLED;
                }
                

            private:
                onEventResult startNote(MonoNoteChannel & c, NoteOnEvent const & e) {
                    if(!c.open(out, 0.f, // initial volume is 0, so that the volume fade starts from 0
                               0)) {
                        return onDroppedNote(e.pitch);
                    }
                    c.pitch = e.pitch;
                    c.tuning = e.tuning;

                    onStartNote(e.velocity, c, out);
                    return onEventResult::OK;
                }

                onEventResult noteOff(uint8_t pitch) {
                    auto len =  get_xfade_length();
                    for(auto & c : channels) {
                        if(c.pitch != pitch) {
                            continue;
                        }
                        if(!c.close(out, CloseMode::XFADE_ZERO, len)) {
                            continue;
                        }
                        // the oscillator is still used for the crossfade,
                        // it will stop being used in the call to openChannel when that channel is finished closing
                        return onEventResult::OK;
                    }
                    // it could be that the corresponding noteOn was skipped
                    // because too many notes where played at that time
                    return onDroppedNote(pitch);
                }
                
            protected:
                onEventResult onDroppedNote(uint8_t pitch) {
#ifndef NDEBUG
                    LG(WARN, "dropped note '%d'", pitch);
#endif
                    return onEventResult::DROPPED_NOTE;
                }
                
                // members
                std::array<MonoNoteChannel, n_channels> channels;
                OutputData out = {n_channels};
            };

            template<typename Parameters, typename ProcessData>
            struct ImplBase {
                using Params = ImplParams;
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                static constexpr auto tune_stretch = 1.f;
                static constexpr auto min_cut_period = 1;
                static constexpr auto max_cut_period = 64;
                static constexpr auto min_ramp_start_freq = 50.f;
                static constexpr auto max_ramp_start_freq = 5000.f;
                static constexpr auto min_ramp_speed = 1.f;
                static constexpr auto max_ramp_speed = 100.f;
                static constexpr auto ramp_speed_normalize(float v) { return (v-min_ramp_speed) / (max_ramp_speed-min_ramp_speed); }
                static constexpr auto ramp_speed_denormalize(float v) { return min_ramp_speed + v*max_ramp_speed; }
                static constexpr auto ramp_start_freq_normalize(float v) { return (v-min_ramp_start_freq) / (max_ramp_start_freq-min_ramp_start_freq); }
                static constexpr auto ramp_start_freq_denormalize(float v) { return min_ramp_start_freq + v*max_ramp_start_freq; }
                
            protected:
                using interleaved_buf_t = cacheline_aligned_allocated::vector<float>;
                
                static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(interleaved_buf_t::value_type);
                
                static constexpr auto size_interleaved = 8 * max_cut_period;
                
                static constexpr auto xfade_len = 401;
                static constexpr auto max_length_ramp_ms = 1000.f;
                
            public:

                static std::vector<ParamSpec> const & getParamSpecs() {
                    static std::vector<ParamSpec> params_spec = {
                        {"[Cut] Adjust osc. freq."},
                        {"[Cut] Period", min_cut_period, max_cut_period},
                        {"[Cut] Size", 0, max_cut_period - 1},
                        {"[Cut] crossfade ratio", 0, max_cut_period},
                        {"[Cut] Adjust ramp size"},
                        {"[Ramp] amount", 0.f, 1.f},
                        {"[Ramp] interpolation", itp::interpolation_traversal()},
                        {"[Ramp] speed", min_ramp_speed, max_ramp_speed},
                        {"[Ramp] start freq.", min_ramp_start_freq, max_ramp_start_freq}
                    };
                    return params_spec;
                }

                static Programs const & getPrograms() {
                    static ProgramsI ps {{
                        {"Clean", // -------------------------     "Cut" presets
                            make_ramp(0.f, itp::LINEAR, min_ramp_speed, min_ramp_start_freq, true, 0)
                        },{"Half-cut",
                            make_ramp(0.f, itp::LINEAR, min_ramp_speed, min_ramp_start_freq, true, max_cut_period/2)
                        }, {"Cut the Ramp", // -------------------------     "Cut & Ramp" presets
                            make_ramp(1.f, itp::EASE_INOUT_CIRC, 69.31f, 50.f, true, 41)
                        },{"Modem", // ---------------------------     "Ramp" presets
                            make_ramp(.33f, itp::EASE_INOUT_EXPO, 36.f)
                        },{"Alarm 0",
                            make_ramp(.35f, itp::EASE_INOUT_EXPO, 7.f)
                        },{"Phone ringing",
                            make_ramp(.34f, itp::EASE_INOUT_EXPO, 20.8f)
                        },{"Motor",
                            make_ramp(.62f, itp::EASE_INOUT_EXPO, 52.48f)
                        },{"Pecking",
                            make_ramp(1.f, itp::EASE_INOUT_EXPO, 100.f)
                        },{"Cat Purr",
                            make_ramp(1.f, itp::EASE_INOUT_CIRC, 55.45f)
                        },{"Alarm 1",
                            make_ramp(.35f, itp::EASE_OUT_CIRC, 10.f)
                        },{"Helicopter 1",
                            make_ramp(1.f,  itp::EASE_OUT_CIRC, 30.f)
                        },{"Alarm 2",
                            make_ramp(.22f, itp::EASE_IN_SINE, 7.f)
                        },{"House alarm",
                            make_ramp(.49f, itp::LINEAR, 3.f)
                        },{"Car alarm",
                            make_ramp(.54f, itp::LINEAR, 16.f)
                        },{"Musical saw",
                            make_ramp(.08f, itp::EASE_INOUT_QUAD, 16.f)
                        },{"Helicopter 2",
                            make_ramp(.94f, itp::EASE_OUT_EXPO, 24.f)
                        },{"Bass 1",
                            make_ramp(.47f, itp::EASE_OUT_EXPO, 74.f)
                        },{"Alarm slow",
                            make_ramp(1.f, itp::EASE_OUT_EXPO, 1.99f, 990.5f)
                        },{"Bass 2",
                            make_ramp(.46f, itp::EASE_IN_QUINT, 77.f)
                        },{"Motor-boat",
                            make_ramp(.79f, itp::EASE_IN_QUINT, 34.f)
                        },{"Bass 3",
                            make_ramp(.79f, itp::EASE_IN_QUINT, 83.f)
                        },{"Helicopter 3",
                            make_ramp(.79f, itp::EASE_OUT_QUINT, 20.f)
                        },{"Phone line",
                            make_ramp(.14f, itp::PROPORTIONAL_VALUE_DERIVATIVE, 81.9f)
                        },{"Krush-like",
                            make_ramp(.02f, itp::PROPORTIONAL_VALUE_DERIVATIVE, 4.96f)
                        },{"Siren",
                            make_ramp(.28f, itp::EASE_IN_SINE, 1.f)
                        }
                    }};
                    
                    return ps.v;
                }
                
                // members
                Parameters params;
                
            protected:
                std::array<interleaved_buf_t, 2> interleaved;
                bool most_recent_buf_idx:1;
                SmoothedFloat<&step_1> ramp_amount, ramp_speed, ramp_start_freq;
                
                Smoothed<unsigned, relevantBits(max_cut_period)> period, gap, xfade;
                decltype(period()) p; // because of optimization for g=0, this value may differ from the period value
                decltype(period()) prev_p; // period of the previoud buffer
                bool adjustFreq : 1;
                bool adjustRamp : 1;
                
                // the first index in "most recent buffer" that contains an out-of-date value
                // 'c' stands for 'current'
                uint8_t c : relevantBits( size_interleaved - 1 );
                float half_tone = compute_half_tone(tune_stretch);
        
                
                // methods
            public:
                ImplBase() :
                most_recent_buf_idx(0)
                , c(0)
                , prev_p(1)
                {}
                
                void initializeSlow() {
                    for(auto & v : interleaved) {
                        v.resize(size_interleaved, 0.f);
                    }
                    params.resize(NPARAMS);
                }
                
                void initializeSteal(ImplBase && o) {
                    interleaved = std::move(o.interleaved);
                    for(auto & v : interleaved) {
                        std::fill(v.begin(), v.end(), 0.f);
                        A(v.size() == size_interleaved);
                    }
                    params = std::move(o.params);
                    A(params.size() == NPARAMS);
                }
                
                void setParameter(int index, float value, int sampleOffset /* not supported yet */) {
                    A(index < params.size());
                    params[index] = value;
                }
                
                void useProgram(int index) {
                    auto & p = getPrograms()[index];
                    for (auto i = 0; i < NPARAMS; i++) {
                        params[i] = p.params[i];
                    }
                }
                
                virtual void doProcessing (ProcessData& data) = 0;
                virtual void allNotesOff() = 0;
                virtual void allSoundsOff() = 0;
                virtual ~ImplBase() = default;
                
            protected:
                int get_xfade_length() { return adjusted(xfade_len); }
                
                template<typename MonoNoteChannel, typename OutputData>
                void onStartNote(float velocity, MonoNoteChannel & c, OutputData & out) {
                    using Request = typename OutputData::Request;
                    
                    auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                    auto freq = to_freq(tunedNote-Do_midi, half_tone);
                    auto channel = c.channels[0];
                    auto len = get_xfade_length();
                    out.setVolume(channel, 1.f, len);
                    
                    if(adjustFreq) {
                        auto r = (p - static_cast<float>(gap())) / p;
                        freq *= r;
                    }
                    auto ramp_size = ms_to_frames(max_length_ramp_ms/ramp_speed_denormalize(ramp_speed()));
                    if(adjustRamp) {
                        ramp_size = adjusted(ramp_size);
                    }
                    auto start_freq = ramp_start_freq_denormalize(ramp_start_freq());
                    auto & osc = c.elem;
                    osc.algo.set(freq - ramp_amount() * (freq-start_freq),
                                 freq,
                                 ramp_size,
                                 0.f,
                                 static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + params[Params::RAMP_INTERPOLATION])]));
                    out.playGeneric(channel,
                                    std::make_pair(std::ref(osc),
                                                   Request{
                                                       &osc.buffer[0],
                                                       velocity,
                                                       // e.noteOn.length is always 0, we must rely on noteOff
                                                       std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
                                                   }));
                }
                
                template<typename T>
                int adjusted(T val) {
                    static_assert(std::is_integral<T>(), "");
                    
                    A(p > gap());
                    return static_cast<int>(.5f + val * p / (p-gap()));
                }
            private:
                static Program::ARRAY make_ramp(float amount, itp::interpolation i, float speed, float start_freq = 50.f,
                                                bool cut_adjust_osc = false, int cut_size = 0) {
                    int itp_index = 0;
                    auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
                    A(b);
                    return {{
                        static_cast<float>(!cut_adjust_osc),
                        max_cut_period-min_cut_period,
                        static_cast<float>(cut_size),
                        max_cut_period,
                        false, // opposite of "adapt ramp size"
                        amount,
                        static_cast<float>(itp_index),
                        ramp_speed_normalize(speed),
                        ramp_start_freq_normalize(start_freq)
                    }};
                }
            };
            
            template<
            
            int nAudioOut,
            typename Parameters,
            typename EventIterator,
            typename NoteOnEvent,
            typename NoteOffEvent,
            typename ProcessData,
            
            typename Base = ImplBase<Parameters, ProcessData>,
            typename Parent = ImplCRTP<nAudioOut, EventIterator, NoteOnEvent, NoteOffEvent, Base>
            
            >
            struct Impl_ : public Parent
            {
                using Programs = imajuscule::audio::Programs;

                using Params = ImplParams;
                using Event = typename EventIterator::object;
                
                static constexpr int NPARAMS = static_cast<int>(Params::NPARAMS);
                static constexpr auto min_cut_period = Base::min_cut_period;
                static constexpr auto max_cut_period = Base::max_cut_period;
                static constexpr auto max_length_ramp_ms = Base::max_length_ramp_ms;
                static constexpr auto size_interleaved = Base::size_interleaved;
                static constexpr auto size_interleaved_one_cache_line = Base::size_interleaved_one_cache_line;
                static constexpr auto xfade_len = Base::xfade_len;
                
                using Base::adjusted;
                using Base::adjustFreq;
                using Base::adjustRamp;
                using Base::c;
                using Base::gap;
                using Base::half_tone;
                using Base::interleaved;
                using Base::most_recent_buf_idx;
                using Base::p;
                using Base::params;
                using Base::period;
                using Base::prev_p;
                using Base::ramp_amount;
                using Base::ramp_speed;
                using Base::ramp_speed_denormalize;
                using Base::ramp_start_freq;
                using Base::ramp_start_freq_denormalize;
                using Base::xfade;

                using Parent::channels;
                using Parent::onEvent;
                using Parent::out;
                
                using typename Parent::MonoNoteChannel;
                
                using Request = Request<nAudioOut>;
                
                //
                //  types
                //
                
                static constexpr auto cut_period_one_cache_line = size_interleaved_one_cache_line / nAudioOut;
                static_assert(cut_period_one_cache_line <= max_cut_period, "");
                
                //
                //  methods
                //
            private:
            private:
                void onEndBufferStepParamChanges() {
                    xfade.step();
                    gap.step();
                    period.step();
                    
                    ramp_speed.step();
                    ramp_amount.step();
                    ramp_start_freq.step();
                }
                
                void compute_state() {
                    p = period();
                    if(0==gap()) {
                        // since g is 0, any positive period value will give the same output
                        // so we chose the one that uses the most out of the cacheline on which
                        // interleaved buffers sit.
                        A(p > 0);
                        p = cut_period_one_cache_line;
                    }
                    else if(gap() >= p) {
                        p = gap()+1;
                    }
                    
                    auto interp = static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + params[RAMP_INTERPOLATION])]);
                    
                    auto rampWasAdjusted = adjustRamp;
                    adjustRamp = params[CUT_ADJUST_RAMP_SIZE]? false: true;
                    auto freqWasAdjusted = adjustFreq;
                    adjustFreq = params[CUT_ADJUST_FREQ]? false : true; // note it's the opposite
                    bool force = false;
                    if((adjustRamp != rampWasAdjusted) ||
                       period.didChange() ||
                       gap.didChange() ||
                       ramp_amount.didChange() ||
                       ramp_speed.didChange() ||
                       ramp_start_freq.didChange()) {
                        force = true;
                    }
                    else if(freqWasAdjusted || adjustFreq) {
                    }
                    else {
                        return;
                    }
                    
                    for(auto& c : channels) {
                        auto & osc = c.elem;
                        if(osc.isInactive()) {
                            continue;
                        }
                        auto tunedNote = midi::tuned_note(c.pitch, c.tuning);
                        auto freq = to_freq(tunedNote-Do_midi, half_tone);
                        
                        if(adjustFreq) {
                            auto r = (p - static_cast<float>(gap())) / p;
                            freq *= r;
                        }
                        if(!force) {
                            // maybe from/to is swapped so we need to test with both ends
                            if(std::abs(freq - angle_increment_to_freq(osc.algo.getToIncrements())) < 0.0001f) {
                                continue;
                            }
                            if(std::abs(freq - angle_increment_to_freq(osc.algo.getFromIncrements())) < 0.0001f) {
                                continue;
                            }
                        }
                        auto ramp_size = ms_to_frames(max_length_ramp_ms/ramp_speed_denormalize(ramp_speed()));
                        if(adjustRamp) {
                            ramp_size = adjusted(ramp_size);
                        }
                        auto start_freq = ramp_start_freq_denormalize(ramp_start_freq());
                        osc.algo.set(freq - ramp_amount() * (freq-start_freq),
                                     freq,
                                     ramp_size,
                                     -1.f,
                                     interp);
                    }
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
                    
                    auto p_ = static_cast<int>(params[CUT_PERIOD] + 0.5f) + min_cut_period;
                    auto g_ = static_cast<int>(params[CUT_SIZE] + 0.5f);
                    auto x_ = static_cast<int>(params[CUT_XFADE_AMOUNT] + 0.5f);
                    
                    A(g_ >= 0);
                    A(p_ >= min_cut_period);
                    A(x_ >= 0);
                    
                    A(g_ < max_cut_period);
                    A(p_ <= max_cut_period);
                    A(x_ <= max_cut_period);
                    
                    bool first = !period.hasValue();
                    
                    period.setTarget(p_);
                    gap.setTarget(g_);
                    xfade.setTarget(x_);
                    
                    auto ra_ = params[RAMP_AMOUNT];
                    auto rs_ = params[RAMP_SPEED];
                    ramp_amount.setTarget(ra_);
                    ramp_speed.setTarget(rs_);
                    ramp_start_freq.setTarget(params[RAMP_START_FREQ]);
                    
                    if(first) {
                        compute_state();
                        A(gap() < p);
                    }
                    
                    auto nRemainingFrames = data.numSamples;
                    auto middle = get_middle(p,gap());
                    
                    static_assert((size_interleaved / nAudioOut) * nAudioOut == size_interleaved, "");
                    
                    auto currentFrame = 0;
                    
                    auto * events = data.inputEvents;
                    A( events );
                    
                    EventIterator it(begin(events)), end(end_(events));
                    
                    int nextEventPosition = getNextEventPosition(it, end);
                    A(nextEventPosition >= currentFrame);
                    
                    while(nRemainingFrames) {
                        A(nRemainingFrames > 0);
                        
                        if(c == p) {
                            c = 0;
                            prev_p = p;
                            most_recent_buf_idx = !most_recent_buf_idx;
                            
                            // we are at the end of the buffer, where xfade ratios are at .5f,
                            // so it's a good location to change xfade amount (for .5f it doesn't change anything)
                            // and a good location to change the period (because of the end of the buffer).
                            onEndBufferStepParamChanges();
                            compute_state();
                            A(gap() < p);
                            middle = get_middle(p,gap());
                        }
                        
                        // keep this loop after onEndBufferStepParamChanges()/compute_state(),
                        // so that new notes have the correct adjusted frequency
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
                        
                        if(gap()) {
                            if(c >= middle) {
                                nFramesToProcess = std::min<int>(nFramesToProcess, p-c);
                            }
                            else if(nFramesToProcess >= middle-c) {
                                if(nFramesToProcess <= (p-c)-gap()) {
                                    nFramesToProcess += gap();
                                }
                                else {
                                    nFramesToProcess = p-c;
                                }
                            }
                        }
                        else {
                            nFramesToProcess = std::min<int>(nFramesToProcess, p-c);
                        }
                        
                        out.step(&interleaved[most_recent_buf_idx][nAudioOut*c], nFramesToProcess);
                        
                        int last = c + nFramesToProcess;
                        
                        if(0 == gap()) {
                            nRemainingFrames -= nFramesToProcess;
                            currentFrame += nFramesToProcess;
                            for(; c < last; ++c) {
                                for(unsigned int i=0; i<nAudioOut; ++i) {
                                    *outs[i] = interleaved[most_recent_buf_idx][nAudioOut*c + i];
                                    ++outs[i];
                                }
                            }
                        }
                        else {
                            auto fp = static_cast<float>(p-gap());
                            auto nRealFramesWritten = 0;
                            A(c < last);
                            auto xfade_amount = xfade() ? (max_cut_period / static_cast<float>(xfade())) : 0.f;
                            
                            while(true) {
                                float const ratio_c = [&] {
                                    if(!xfade()) {
                                        return (c<middle) ? 1.f : 0.f;
                                    }
                                    auto ratio_c = (c<middle) ? (1.f-((middle-c)/fp)) : ((c-(middle+gap()))/fp);
                                    
                                    ratio_c -= 0.5f;
                                    
                                    ratio_c *= xfade_amount;
                                    ratio_c = 0.5f + clamp(ratio_c, -0.5f, 0.5f);
                                    return ratio_c;
                                }();
                                
                                float ratio_b = 1.f - ratio_c;
                                
                                if(c < gap()) {
                                    auto a_ = prev_p+c;
                                    auto b_ = gap() + 1;
                                    int b;
                                    // use prev_p: the period may have changed between previous buffer and this buffer
                                    if(a_ < b_) {
                                        b = prev_p;
                                    }
                                    else {
                                        b = a_ - b_;
                                    }
                                    for(unsigned int i=0; i<nAudioOut; ++i) {
                                        *outs[i] =
                                        ratio_b * interleaved[!most_recent_buf_idx][i + nAudioOut*b] +
                                        ratio_c *  interleaved[most_recent_buf_idx][i + nAudioOut*c];
                                        ++outs[i];
                                    }
                                }
                                else {
                                    auto b = c-gap();
                                    A(b >= 0);
                                    for(unsigned int i=0; i<nAudioOut; ++i) {
                                        *outs[i] =
                                        ratio_b * interleaved[most_recent_buf_idx][i + nAudioOut*b] +
                                        ratio_c * interleaved[most_recent_buf_idx][i + nAudioOut*c];
                                        ++outs[i];
                                    }
                                }
                                ++nRealFramesWritten;
                                
                                ++c;
                                if(c == middle) {
                                    c += gap();
                                }
                                if(c == last) {
                                    break;
                                }
                                A(c < last);
                            }
                            nRemainingFrames -= nRealFramesWritten;
                            currentFrame += nRealFramesWritten;
                        }
                    }
                    
                    A(nextEventPosition == event_position_infinite); // the events should all have already been processed
                }
            };
            
        }
    }
} // namespaces
