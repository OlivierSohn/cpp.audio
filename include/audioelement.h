namespace imajuscule {
    namespace audioelement {
        
        constexpr auto n_frames_per_buffer = 16;

        struct AudioElementBase  {
            AudioElementBase() = default;
            
            // no copy or move because the lambda returned by fCompute() captures 'this'
            AudioElementBase(const AudioElementBase &) = delete;
            AudioElementBase & operator=(const AudioElementBase&) = delete;
            AudioElementBase(AudioElementBase &&) = delete;
            AudioElementBase& operator = (AudioElementBase &&) = delete;
            
            // AudioComponent<float> has a buffer of size 1 cache line
            // AudioComponent<double> has a buffer of size 2 cache lines
            // each of them have 16 frames worth of data in their buffer
            static constexpr auto buffer_alignment = cache_line_n_bytes; // 64 or 32
            
            static constexpr auto index_state = 0;
        };
        
        template<typename T>
        auto & state(T * buffer) { return buffer[AudioElementBase::index_state]; }
        
        // lifecycle :
        // upon creation, state is inactive()
        // when in a queue state is queued()
        // when processed state is a float
        // when done being played state is inactive()
        template<typename T>
        struct AudioElement : public AudioElementBase {
            using buffer_placeholder_t = std::aligned_storage_t<n_frames_per_buffer * sizeof(T), buffer_alignment>;
            static_assert(alignof(buffer_placeholder_t) == buffer_alignment,"");
            
            // state values must be distinct from every possible valid value
            static constexpr auto queued() { return -std::numeric_limits<T>::infinity(); } // in *** at most *** one queue
            static constexpr auto inactive() { return std::numeric_limits<T>::infinity(); }// not active in any queue
            
            
            ////// [AudioElement] beginning of the 1st cache line
            
            union {
                buffer_placeholder_t for_alignment;
                T buffer[n_frames_per_buffer];
            };
            
            ////// [AudioElement<float>] beginning of the 2nd cache line
            ////// [AudioElement<double>] beginning of the 3rd cache line
            
            bool clock_ : 1; // could be removed since we don't have any AudioElement
            // depend on another
            
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            using Tr = NumTraits<T>;
            
            AudioElement() {
                // assert deactivated as it fails on iphone / iphone simulator. I think I need to implement
                // a freelist of blocks of cache line size to get around this issue related to overaligned types.
                //A(0 == reinterpret_cast<unsigned long>(buffer) % buffer_alignment);
                state(buffer) = inactive();
            }
            
            auto getState() const { return state(buffer); }
            constexpr bool isInactive() const { return getState() == inactive(); }
            constexpr bool isActive() const { return getState() != inactive(); }
        };
        
        template<typename ALGO>
        struct FinalAudioElement : public AudioElement<typename ALGO::FPT>{
            using FPT = typename ALGO::FPT;
            static_assert(std::is_floating_point<FPT>::value, "");
            
            template <class... Args>
            FinalAudioElement(Args&&... args) : algo(std::forward<Args>(args)...) {}
            
            ALGO algo;
        };
        
        static inline auto & whiteNoise() {
            static soundBuffer n(soundId{Sound::NOISE, .1f});
            return n;
        }
        
        static inline auto & pinkNoise() {
            static soundBuffer n(soundId{Sound::PINK_NOISE, .1f});
            return n;
        }
        
        
        template<typename ALGO>
        struct VolumeAdjusted {
            using T = typename ALGO::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            
            T real() const { return volume * osc.real(); }
            T imag() const { return volume * osc.imag(); }
            
            T angleIncrements() const { return osc.angleIncrements(); }
            
            VolumeAdjusted() : log_ratio_(1.f), low_index_(0) {}
            
            void setAngleIncrements(T ai) {
                volume = loudness::equal_loudness_volume(angle_increment_to_freq(ai),
                                                         low_index_,
                                                         log_ratio_,
                                                         loudness_level);
                osc.setAngleIncrements(ai);
            }
            
            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                A(low_index >= 0);
                A(low_index < 16);
                low_index_ = low_index;
                A(log_ratio >= 0.f);
                A(log_ratio <= 1.f);
                log_ratio_ = log_ratio;
                this->loudness_level = loudness_level;
            }
            
            void step() { osc.step(); }
        private:
            unsigned int low_index_ : 4;
            float loudness_level;
            float log_ratio_;
            float volume;
            ALGO osc;
        };
        
        // the unit of angle increments is "radian / pi"
        
        template<typename T>
        struct Phased {
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            using Tr = NumTraits<T>;
            
            Phased() = default;
            Phased(T angle_increments) { setAngleIncrements(angle_increments); }
            
            void setAngle( T angle ) { angle_ = angle; }
            T angle() const { return angle_; }
            
            void setAngleIncrements(T v) {
                A(std::abs(v) < Tr::two()); // else need to modulo it
                angle_increments = v;
            }
            T angleIncrements() const { return angle_increments; }
            
            void step() {
                angle_ += angle_increments;
                if(angle_ > Tr::two()) {
                    angle_ -= Tr::two();
                }
                else if(angle_ < Tr::zero()) {
                    angle_ += Tr::two();
                }
            }
            
            protected:
            T angle_ = Tr::zero();
            T angle_increments;
        };
        
        /*
         * Phase controlled oscillator
         */
        template<typename T>
        struct PCOscillatorAlgo : public Phased<T> {
            using Phased<T>::angle_;
            
            PCOscillatorAlgo() = default;
            PCOscillatorAlgo(T angle_increments) : Phased<T>(angle_increments) {}
            
            T real() const { return std::cos(M_PI*angle_); }
            T imag() const { return std::sin(M_PI*angle_); }
        };
        
        template<typename T>
        using PCOscillator = FinalAudioElement<PCOscillatorAlgo<T>>;
  
        template<typename T>
        struct soundBufferWrapperAlgo {            
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");

            soundBufferWrapperAlgo(soundBuffer const & sb) : sb(sb) {}

            T imag() const { return sb[index]; }
            
            void step() {
                ++index;
                if(index == sb.size()) {
                    index = 0;
                }
            }
            
            int index = -1;
            soundBuffer const & sb;
        };
        
        template<typename T>
        struct WhiteNoiseAlgo : public soundBufferWrapperAlgo<T> {
            using FPT = typename soundBufferWrapperAlgo<T>::FPT;
            static_assert(std::is_floating_point<FPT>::value, "");
            WhiteNoiseAlgo() : soundBufferWrapperAlgo<T>(whiteNoise()) {}
        };
        
        template<typename T>
        struct PinkNoiseAlgo : public soundBufferWrapperAlgo<T> {
            using FPT = typename soundBufferWrapperAlgo<T>::FPT;
            static_assert(std::is_floating_point<FPT>::value, "");
            PinkNoiseAlgo() : soundBufferWrapperAlgo<T>(pinkNoise()) {}
        };
        
        template<typename T>
        using WhiteNoise = FinalAudioElement< WhiteNoiseAlgo<T> >;
        
        template<typename T>
        struct SquareAlgo : public Phased<T> {
            using Phased<T>::angle_;
            
            SquareAlgo() = default;
            SquareAlgo(T angle_increments) : Phased<T>(angle_increments) {}
            
            T imag() const { return square(angle_); }
        };

        template<typename T>
        using Square = FinalAudioElement<SquareAlgo<T>>;
        
        /*
         * first pulse happends at angle = 0
         */
        template<typename T>
        struct PulseTrainAlgo : public Phased<T> {
            
            using Tr = NumTraits<T>;
            using Phased<T>::angle_;
            
            PulseTrainAlgo() = default;
            PulseTrainAlgo(T angle_increments, T pulse_width) :
            Phased<T>(angle_increments),
            pulse_width(pulse_width) {
                A(pulse_width >= angle_increments); // else it's always 0
            }
            
            void set(T angle_increments, T pulse_width_) {
                A(pulse_width_ >= angle_increments); // else it's always 0
                this->setAngleIncrements(angle_increments);
                pulse_width = pulse_width_;
            }
            
            T imag() const { return pulse(angle_, pulse_width); }
        private:
            T pulse_width{};
        };
        
        template<typename T>
        using PulseTrain = FinalAudioElement<PulseTrainAlgo<T>>;
        
        
        template<class...AEs>
        struct Mix {
            using T = typename NthTypeOf<0, AEs...>::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            
            static constexpr auto n_aes = sizeof...(AEs);
            
        private:
            std::tuple<AEs...> aes;
            std::array<float, n_aes> gains;
            
        public:
            Mix() { gains.fill( 1.f ); }
            
            auto & get() {
                return aes;
            }
            
            void setGains(decltype(gains) g) { gains = g; }
            
            void step() {
                for_each(aes, [](auto & ae) {
                    ae.step();
                });
            }
            
            void setAngleIncrements(T v) {
                for_each(aes, [v](auto & ae) {
                    ae.setAngleIncrements(v);
                });
            }
            
            T imag() const {
                T sum = 0.f;
                int index = 0;
                // not more than 3 captures to avoid dynamic allocation!
                for_each(aes, [&sum, &index, this] (auto const & ae) {
                    sum += gains[index] * ae.imag();
                    ++index;
                });
                return sum;
            }
            
            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                for_each(aes, [=](auto & ae) {
                    ae.setLoudnessParams(low_index, log_ratio, loudness_level);
                });
            }
        };
        
        template<class...AEs>
        struct Chain {
            using T = typename NthTypeOf<0, AEs...>::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            
        private:
            std::tuple<AEs...> aes;
        };
        
        template<typename AEAlgo, FilterType KIND>
        struct FilterAlgo {
            using T = typename AEAlgo::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value, "");
            
        private:
            AEAlgo audio_element;
            Filter<T, 1, KIND> filter_;
        public:
            void step() {
                audio_element.step();
                auto val = audio_element.imag();
                filter_.feed(&val);
            }
            
            // sets the filter frequency
            void setAngleIncrements(T v) {
                filter_.initWithSampleRate(SAMPLE_RATE, angle_increment_to_freq(v));
            }
            
            T imag() const {
                return *filter_.filtered();
            }
            
            auto & get_element() { return audio_element; }
            auto & filter() { return filter_; }
            
            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {}
        };
        
        template<typename T>
        using LowPassAlgo = FilterAlgo<T, FilterType::LOW_PASS>;
        
        template<typename T>
        using HighPassAlgo = FilterAlgo<T, FilterType::HIGH_PASS>;
        
        template<typename AEAlgo>
        struct BandPassAlgo {
            using FPT = typename AEAlgo::FPT;
            using T = FPT;
            using Tr = NumTraits<T>;

            void setWidthFactor(T w) {
                width_factor = w;
            }
            void setAngleIncrements(T inc) {
                A(inc >= 0.f);
                
                auto inv_width_factor = Tr::one() / width_factor;
                auto low = inc * inv_width_factor;
                auto high = inc * width_factor;
                
                getHP().setAngleIncrements(low);
                getLP().setAngleIncrements(high);
                
                // gain compensation to have an equal power of the central frequency for all widths
                {
                    compensation = 1 + (inv_width_factor * inv_width_factor);
#ifndef NDEBUG
                    // verify accuracy of above simplification
                    
                    // inc / low == width_factor
                    // inc / high == 1 / width_factor
                    auto inv_sq_mag_hp = get_inv_square_filter_magnitude<FilterType::HIGH_PASS>(width_factor * width_factor);
                    auto inv_sq_mag_lp = get_inv_square_filter_magnitude<FilterType::LOW_PASS >(inv_width_factor * inv_width_factor);
                    auto inv_mag = sqrt(inv_sq_mag_hp * inv_sq_mag_lp);
                    auto original_compensation = inv_mag;
                    A(std::abs(original_compensation - compensation) / (original_compensation + compensation) < FLOAT_EPSILON);
#endif
                }
            }

            void step() { cascade.step(); }
            
            T imag() const { return compensation * cascade.imag(); }
            
            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {}
        private:
            HighPassAlgo<LowPassAlgo<AEAlgo>> cascade;
            T compensation;
            T width_factor;
            
            auto & getHP() { return cascade; }
            auto & getLP() { return cascade.get_element(); }
        };
        
        template<typename T>
        using LPWhiteNoiseAlgo = LowPassAlgo<WhiteNoiseAlgo<T>>;
        
        template<typename T>
        using LPWhiteNoise = FinalAudioElement<LPWhiteNoiseAlgo<T>>;
        
        template<typename AEAlgo>
        using LowPass = FinalAudioElement<LowPassAlgo<AEAlgo>>;
        
        enum class eNormalizePolicy {
            FAST,
            ACCURATE
        };

        template<typename T, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
        struct OscillatorAlgo {
            using Tr = NumTraits<T>;
            using FPT = T;
            
            constexpr OscillatorAlgo(T angle_increments) { setAngleIncrements(angle_increments); }
            constexpr OscillatorAlgo() : mult(Tr::one(), Tr::zero()) {}
            
            void setAngle(T f) {
                cur = polar(static_cast<T>(M_PI)*f);
            }
            void setAngleIncrements(T f) {
                mult = polar(static_cast<T>(M_PI)*f);
            }
            
            void step() {
                cur *= mult;
                if(NormPolicy == eNormalizePolicy::FAST) {
                    approx_normalize(); // to fix iterative error accumulation... if it is costly it could be done less frequently
                }
                else {
                    normalize();
                }
            }
            
            T real() const { return cur.real(); }
            T imag() const { return cur.imag(); }
            
            T angle() const { return arg(cur)/M_PI; }
            T angleIncrements() const { return arg(mult)/M_PI; }
            
            private:
            complex<T> cur = {Tr::one(), Tr::zero()};
            complex<T> mult;
            
            void approx_normalize() {
                // http://dsp.stackexchange.com/questions/971/how-to-create-a-sine-wave-generator-that-can-smoothly-transition-between-frequen
                
                cur *= Tr::half() * (Tr::three() - norm(cur));
            }
            
            void normalize() {
                cur /= abs(cur);
            }
        };
        
        template<typename T, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
        using Oscillator = FinalAudioElement<OscillatorAlgo<T, NormPolicy>>;
        
        // 1 : lowest resolution
        // 0 : highest resolution
        template<typename T>
        static constexpr T resolution(range<T> const & r) {
            A(!r.empty());
            A(r.getMax() != NumTraits<T>::zero());
            return r.delta() / r.getMax();
        }

        enum class VolumeAdjust {
            Yes,
            No
        };
        
        template<typename>
        struct FreqRampAlgo_;

        template<typename T>
        struct FreqRampAlgoSpec {
            
            template<typename> friend class FreqRampAlgo_;
            
            using Tr = NumTraits<T>;
            using lut = std::array<T, itp::interpolation::INTERPOLATION_UPPER_BOUND>;

            //static lut lut_interpolation_proportionality;
            
            // offsets because we use the value at the beginning of the timestep
            static constexpr auto increasing_integration_offset = 0;
            static constexpr auto decreasing_integration_offset = 1;

            FreqRampAlgoSpec() : cur_sample(Tr::zero()), from{}, to{}, duration_in_samples{}
            {}

            void set(T from_,
                     T to_,
                     T duration_in_samples_,
                     T start_sample,
                     itp::interpolation i = itp::LINEAR) {
                set_by_increments(freq_to_angle_increment(from_),
                                  freq_to_angle_increment(to_),
                                  duration_in_samples_,
                                  start_sample,
                                  i);
            }
            
            void set_by_increments(T from_increments,
                                   T to_increments,
                                   T duration_in_samples_,
                                   T start_sample,
                                   itp::interpolation i = itp::LINEAR) {
                if(start_sample >= Tr::zero()) {
                    cur_sample = start_sample;
                }
                else {
                    // if start_sample < 0 we adapt it to be at the same ratio in the new range
                    // and we adapt bounds order to the existing bounds
                    A(duration_in_samples);
                    cur_sample *= duration_in_samples_ / duration_in_samples;
                    if(from_increments > to_increments) {
                        if(from < to) {
                            std::swap(from_increments, to_increments);
                        }
                    }
                    else if(from > to) {
                        std::swap(from_increments, to_increments);
                    }
                }
                from = from_increments;
                to = to_increments;
                duration_in_samples = duration_in_samples_;
                
                C = get_linear_proportionality_constant();
                                
                A(duration_in_samples > 0);
                interp.setInterpolation(i);
            }
            
            T do_step(T proportionality) {
                if(cur_sample + .5f > duration_in_samples) {
                    cur_sample = Tr::zero();
                    std::swap(from, to);
                }
                // we call get_unfiltered_value instead of get_value because we ensure:
                A(cur_sample <= duration_in_samples);
                auto f_result = interp.get_unfiltered_value(cur_sample, duration_in_samples, from, to);
                // Taking the value at cur_sample means taking the value at the beginning of the step.
                // The width of the step depends on that value so if we had taken the value in the middle or at the end of the step,
                // not only would the value be different, but also the step width!
                
                // we could take the value in the middle and adjust "value + step width" accordingly
                
                // linear interpolation for parameter
                auto f = from + (to-from) * (cur_sample + .5f) / duration_in_samples;
                
                
                cur_sample += proportionality * f;
                return f_result;
            }

            T step() {
                return do_step(C);
            }
            
            T step_calibrate(T proportionality) {
                return do_step(proportionality * C);
            }
            
            T getFromIncrements() const { return from; }
            T getToIncrements() const { return to; }

            T get_duration_in_samples() const { return duration_in_samples; }
            
            /*
            static lut compute_lut() {
                lut l;
                for(auto i=0; i<itp::interpolation::INTERPOLATION_UPPER_BOUND; ++i) {
                    auto val = safe_cast<itp::interpolation>(i);
                    if(itp::intIsReal(val)) {
                        l[i] = calibrate_constant(val);
                    }
                    else {
                        l[i] = 0;
                    }
                }
                return l;
            }
            
            static constexpr T calibrate_constant(itp::interpolation const i) {
                LG(INFO, "-------------------");
                LG(INFO, "%s", itp::interpolationInfo(i));
                LG(INFO, "-------------------");
                
                // no more than 3 captures else dynamic allocation occurs!
                auto f = [i](T const proportionality, T const duration) -> int {
                    // avoid 0 in the ramp range (for interpolations that are exponential)
                    constexpr auto fLow = NumTraits<T>::one();
                    constexpr auto fHigh = NumTraits<T>::two();
                    
                    FreqRampAlgoSpec<T> spec;
                    spec.set_by_increments<AdjustProportionality::No>(fLow,
                                                                      fHigh,
                                                                      duration,
                                                                      NumTraits<T>::zero(),
                                                                      i);

                    // assuming an ascending ramp (else we need to substract integration_offset)
                    A(fLow < fHigh);
                    return steps_to_swap(spec, proportionality);
                };

                enum TestNature {
                    NotStrict,
                    Strict
                };
                
                std::array<TestNature, 2> a_tests {{ Strict, NotStrict }};
                std::array<range<T>, 2> ranges;
                
                range<T> r {
                    NumTraits<T>::zero(),
                    NumTraits<T>::two()
                };

                int idx = -1;
                for(auto test_nature : a_tests) {
                    ++idx;

                    auto test = [test_nature](T const proportionality, T const duration, auto f) {
                        auto n_steps = f(proportionality, duration);
                        return (n_steps > duration) || ((test_nature == TestNature::NotStrict) && n_steps == duration);
                    };
                    

                    // 16 corresponds to one second of sound
                    // and tests for linear interpolation showed that 17 is the first exponent where numerical errors
                    // become a problem
                    constexpr auto n_max_exp = 16;
                    unsigned int exp = 1;
                    
                    A(!r.empty());
                    
                    logRange(r);
                    
                    while(exp <= n_max_exp) {
                        auto const duration = static_cast<T>(pow2(exp));
                        LG(INFO, "-------- %2d --------", exp);
                        
                        // when range doesn't contain the wanted value
                        // we translate and double the delta
                        {
                            bool done = false;
                            while(r.getMin() && !test(r.getMin(), duration, f)) {
                                // range is too high
                                r = {
                                    r.getMin() - 2 * r.delta(),
                                    r.getMin()
                                };
                                if(r.getMin() < 0) {
                                    r.setMin(0);
                                }
                                LG(INFO, "<<<");
                                logRange(r);
                                done = true;
                            }
                            if(!done) {
                                while(test(r.getMax(), duration, f)) {
                                    // range is too low
                                    r = {
                                        r.getMax(),
                                        r.getMax() + 2 * r.delta()
                                    };
                                    LG(INFO, "                >>>");
                                    logRange(r);
                                }
                            }
                        }
                        
                        // now range contains the wanted value
                        
                        // for the last exponent, we continue even if the resolution is smaller than
                        // the estimated error
                        while(
                              (exp == n_max_exp) ||
                              resolution(r) > NumTraits<T>::one() / duration
                              )
                        {
                            auto center = r.getCenter();
                            if(center == r.getMin()) {
                                break;
                            }
                            
                            if(test(center, duration, f)) {
                                LG(INFO, "                  >");
                                r = {center, r.getMax()};
                            }
                            else {
                                LG(INFO, "<");
                                r = {r.getMin(), center};
                            }
                            logRange(r);
                        }
                        
                        ++exp;
                    }
                    
                    ranges[idx] = r;
                }

                logRange(ranges[0]);
                logRange(ranges[1]);
               
                return (ranges[1].getMin() + ranges[0].getMax()) / 2;
            }*/
            
        private:
            NormalizedInterpolation<T> interp;
            T from, to, cur_sample;
            T duration_in_samples;
            T C;
            
            // do not make it public : if bounds are swapped afterwards, using this value can lead to bugs
            T get_linear_proportionality_constant() const {
                // we want to achieve the same effect as PROPORTIONAL_VALUE_DERIVATIVE
                // but without paying the cost of one 'expf' call per audio frame :
                // to achieve the same effect we add to cur_sample a value proportionnal to
                // the current frequency. the factor of proportionnality is adjusted to match
                // the wanted duration_in_samples
                A(from > 0);
                A(to > 0);
                // else computation cannot be done
                
                return (to==from) ? 1.f : -std::log(from/to) / (to-from);
            }
        };
        
        template<VolumeAdjust V, typename T>
        struct OscillatorAlgo_;
        
        template<typename T>
        struct OscillatorAlgo_<VolumeAdjust::No, T> {
            using type = OscillatorAlgo<T>;
        };
        
        template<typename T>
        struct OscillatorAlgo_<VolumeAdjust::Yes, T> {
            using type = VolumeAdjusted<OscillatorAlgo<T>>;
        };
        
        template<VolumeAdjust V, typename T>
        using AdjustableVolumeOscillatorAlgo = typename OscillatorAlgo_<V,T>::type;
        
        template<typename T>
        static int steps_to_swap(FreqRampAlgoSpec<T> & spec, T proportionality = 1.f) {
            bool order = spec.getToIncrements() < spec.getFromIncrements();
            
            int count = 0;
            while(order == spec.getToIncrements() < spec.getFromIncrements()) {
                spec.step_calibrate( proportionality );
                ++count;
            }
            --count; // because swap occurs at the beginning of step method, before current step is modified
            return count;
        }

        template<typename ALGO>
        struct FreqRampAlgo_ {
            using T = typename ALGO::FPT;
            using FPT = T;
            
            using Spec = FreqRampAlgoSpec<T>;
            using Tr = NumTraits<T>;

            static_assert(std::is_same<T,float>::value, "non-float interpolation is not supported");
            
            void set(T from_,
                     T to_,
                     T duration_in_samples_,
                     T start_sample = Tr::zero(),
                     itp::interpolation i = itp::LINEAR)
            {
                spec.set(from_, to_, duration_in_samples_, start_sample, i);
            }
            
            void step() {
                auto current_freq = spec.step();
                
                osc.setAngleIncrements(current_freq);
                osc.step();
            }
            
            T real() const { return osc.real(); }
            T imag() const { return osc.imag(); }
            
            auto & getOsc() { return osc; }
            
            Spec spec;
        private:
            ALGO osc;
        };
        
        template<typename T, VolumeAdjust V>
        using FreqRampOscillatorAlgo_ = FreqRampAlgo_<AdjustableVolumeOscillatorAlgo<V,T>>;
        
        template<typename T>
        using FreqRampAlgo = FreqRampOscillatorAlgo_<T, VolumeAdjust::Yes>;
        
        template<typename T>
        using FreqRamp = FinalAudioElement<FreqRampAlgo<T>>;
        

        template<typename T>
        using FreqRampLPWhiteNoiseAlgo_ = FreqRampAlgo_<LPWhiteNoiseAlgo<T>>;
        
        template<typename T>
        using FreqRampLPWhiteNoise = FinalAudioElement<FreqRampAlgo_<LPWhiteNoiseAlgo<T>>>;

        template<typename A1, typename A2>
        struct RingModulationAlgo {
            using T = typename A1::FPT;
            using FPT = T;
            
            static_assert(std::is_same<typename A1::FPT, typename A2::FPT>::value, ""); // else choice for T is arbitrary
            
            using Tr = NumTraits<T>;
            
            RingModulationAlgo() = default;
            
            void set(T angle_increments1, T angle_increments2, bool reset = true) {
                osc1.setAngleIncrements(angle_increments1);
                if(reset) {
                    osc1.setAngle(0.f);
                }
                osc2.setAngleIncrements(angle_increments2);
                if(reset) {
                    osc1.setAngle(0.f);
                }
            }
            
            void step() {
                osc1.step();
                osc2.step();
            }
            
            T real() const { return osc1.real() * osc2.real(); }
            T imag() const { return osc1.imag() * osc2.imag(); }
            
            auto & get_element_1() { return osc1; }
            auto & get_element_2() { return osc2; }
        private:
            A1 osc1;
            A2 osc2;
        };
        
        template<typename A1, typename A2>
        using RingModulation = FinalAudioElement<RingModulationAlgo<A1,A2>>;
        
        /*
         * returns false when the buffer is done being used
         */
        using ComputeFunc = std::function<bool(bool)>;
        
        template<typename T>
        struct FCompute {
            template<typename U=T>
            static auto get(U & ae)
            -> std::enable_if_t<
            IsDerivedFrom<U, AudioElementBase>::Is,
            ComputeFunc
            >
            {
                return [&ae](bool sync_clock) {
                    using AE = AudioElement<typename T::FPT>;
                    auto & e = safe_cast<AE&>(ae);
                    
                    if(e.getState() == AE::inactive()) {
                        // Issue : if the buffer just got marked inactive,
                        // but no new AudioElementCompute happends
                        // and from the main thread someone acquires this and queues it,
                        // it will have 2 lambdas because the first lambda will never have seen the inactive state.
                        // However the issue is not major, as the 2 lambdas have a chance to be removed
                        // the next time
                        return false;
                    }
                    if(likely(e.getState() != AE::queued())) {
                        if(sync_clock == e.clock_) {
                            return true;
                        }
                    }
                    e.clock_ = sync_clock;
                    for(auto & v : e.buffer) {
                        ae.algo.step();
                        v = ae.algo.imag();
                    }
                    A(e.getState() != AE::queued());
                    A(e.getState() != AE::inactive());
                    return true;
                };
            }
            
            template<typename U=T>
            static auto get(U & e)
            -> std::enable_if_t<
            !IsDerivedFrom<U, AudioElementBase>::Is,
            ComputeFunc
            >
            {
                return {};
            }
        };
        
        template<typename T>
        ComputeFunc fCompute(T & e) {
            return FCompute<T>::get(e);
        }
        
    } // NS audioelement
} // NS imajuscule
