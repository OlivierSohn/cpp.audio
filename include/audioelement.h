namespace imajuscule {
    namespace audioelement {

        constexpr auto n_frames_per_buffer = 16;

        // AudioComponent<float> has a buffer of size 1 cache line
        // AudioComponent<double> has a buffer of size 2 cache lines
        // each of them have 16 frames worth of data in their buffer
        static constexpr auto buffer_alignment = cache_line_n_bytes; // 64 or 32

        static constexpr auto index_state = 0;

        template<typename T>
        auto & state(T * buffer) { return buffer[index_state]; }

      // lifecycle :
      // upon creation, state is inactive()
      // when in a queue state is queued()
      // when processed state is a float
      // when done being played state is inactive()
      template<typename T>
      struct AEBuffer {
        AEBuffer() {
          // assert deactivated as it fails on iphone / iphone simulator. I think I need to implement
          // a freelist of blocks of cache line size to get around this issue related to overaligned types.
          //Assert(0 == reinterpret_cast<unsigned long>(buffer) % buffer_alignment);
          state(buffer) = inactive();
        }

        // no copy or move because the lambda returned by fCompute() captures 'this'
        AEBuffer(const AEBuffer &) = delete;
        AEBuffer & operator=(const AEBuffer&) = delete;
        AEBuffer(AEBuffer &&) = delete;
        AEBuffer& operator = (AEBuffer &&) = delete;

        using buffer_placeholder_t = std::aligned_storage_t<n_frames_per_buffer * sizeof(T), buffer_alignment>;
        static_assert(alignof(buffer_placeholder_t) == buffer_alignment);

        // state values must be distinct from every possible valid value
        static constexpr auto queued() { return -std::numeric_limits<T>::infinity(); } // in *** at most *** one queue
        static constexpr auto inactive() { return std::numeric_limits<T>::infinity(); }// not active in any queue

        ////// [AEBuffer] beginning of the 1st cache line
        
        union {
          // 'for_alignment' is the largest member, hence it defines the minimal alignment of 'AudioElement' and its derived classes
          buffer_placeholder_t for_alignment;
          T buffer[n_frames_per_buffer];
        };
        
        ////// [AEBuffer<float>] beginning of the 2nd cache line
        ////// [AEBuffer<double>] beginning of the 3rd cache line

        constexpr bool isInactive() const { return getState() == inactive(); }
        auto getState() const { return state(buffer); }
      };
      
        template<typename ALGO>
        struct FinalAudioElement {
            static constexpr auto hasEnvelope = ALGO::hasEnvelope;
            static constexpr bool computable = true;
            using FPT = typename ALGO::FPT;
            static_assert(std::is_floating_point<FPT>::value);
            using buffer_t = AEBuffer<FPT>;

            // no copy or move because the lambda returned by fCompute() captures 'this'
            FinalAudioElement(const FinalAudioElement &) = delete;
            FinalAudioElement & operator=(const FinalAudioElement&) = delete;
            FinalAudioElement(FinalAudioElement &&) = delete;
            FinalAudioElement& operator = (FinalAudioElement &&) = delete;

            bool isEnvelopeFinished() const {
              return algo.isEnvelopeFinished();
            }

            template <class... Args>
            FinalAudioElement(buffer_t & b, Args&&... args) : buffer(&b), algo(std::forward<Args>(args)...) {}

            void forgetPastSignals() {
                algo.forgetPastSignals();
            }
            void setEnvelopeCharacTime(int len) {
              algo.setEnvelopeCharacTime(len);
            }
            void onKeyPressed() {
                algo.onKeyPressed();
            }
            bool onKeyReleased() {
                return algo.onKeyReleased();
            }

            FPT angle() const { return algo.angle(); }

            constexpr bool isInactive() const { return buffer->isInactive(); }
            auto getState() const { return buffer->getState(); }

            AEBuffer<FPT> * buffer;
            bool clock_ : 1;
            ALGO algo;
        };


        enum class EnvelopeState : unsigned char {
            KeyPressed // the envelope is raising or sustained
          , KeyReleased // the envelope is closing
          , EnvelopeDone1 // the envelope has closed, but the last buffer contains samples where the enveloppe was still opened.
          , EnvelopeDone2 // the envelope has closed, the last buffer contains only samples where the enveloppe was closed.
        };

        // Similar to RingModulationAlgo except the modulator has equal real and imaginary parts.
        template <typename ALGO, typename Envelope>
        struct Envelopped {
          static constexpr auto hasEnvelope = true;
          using FPT = typename ALGO::FPT;
          static_assert(std::is_same<typename ALGO::FPT, typename Envelope::FPT>::value);

          void forgetPastSignals() {
            env.forgetPastSignals();
            algo.forgetPastSignals();
          }
          void setEnvelopeCharacTime(int len) {
            env.setEnvelopeCharacTime(len);
          }

          void step() {
              env.step();
              algo.step();
          }

          void setAngle(FPT a) {
              algo.setAngle(a);
          }
          FPT angle() const { return algo.angle(); }

          void setAngleIncrements(FPT a) {
              algo.setAngleIncrements(a);
          }

          FPT real() const { return algo.real() * env.value(); }
          FPT imag() const { return algo.imag() * env.value(); }

          bool isEnvelopeFinished() const {
              return env.getState() == EnvelopeState::EnvelopeDone2;
          }
          void onKeyPressed() {
              env.onKeyPressed();
          }
          bool onKeyReleased() {
              return env.onKeyReleased();
          }

          auto & getOsc() { return algo.getOsc(); }
          auto & getAlgo() { return algo; }
          auto const & getEnveloppe() const { return env; }
          auto & editEnveloppe() { return env; }

          void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
              algo.setLoudnessParams(low_index, log_ratio, loudness_level);
          }

        private:
          Envelope env;
          ALGO algo;
        };

        enum class EnvelopeRelease : unsigned char {
          WaitForKeyRelease,
          ReleaseAfterDecay
        };

        template <typename Base>
        struct EnvelopeCRT : public Base {
            using FPT = typename Base::FPT;
            using T = FPT;
            static constexpr EnvelopeRelease Release = Base::Release;

            using Base::startPressed;
            using Base::stepPressed;
            using Base::getReleaseItp;
            using Base::getReleaseTime;
            using Base::isAfterAttackBeforeSustain;

            void forgetPastSignals() {
                state = EnvelopeState::EnvelopeDone2;
                _value = static_cast<T>(0);
                counter = 0;
            }

            void step() {
                ++counter;
                switch(state) {
                    case EnvelopeState::KeyPressed:
                    {
                      auto maybeV = stepPressed(counter);
                      if(maybeV) {
                        _value = *maybeV;
                      }
                      else {
                        onKeyReleased();
                        step(); // recursion, and note that we won't infinite loop because
                                // onKeyReleased() changes the state.
                      }
                    }
                        break;
                    case EnvelopeState::KeyReleased:
                        _value = itp::interpolate(getReleaseItp()
                                                  , static_cast<T>(counter)
                                                  , _topValue
                                                  , - _topValue
                                                  , getReleaseTime());
                        if(counter >= getReleaseTime()) {
                            state = EnvelopeState::EnvelopeDone1;
                            counter = 0;
                            _value = static_cast<T>(0);
                        }
                        break;
                    case EnvelopeState::EnvelopeDone1:
                        if(counter > n_frames_per_buffer) { // to be sure that all non-zero computed signals
                            // were used
                            state = EnvelopeState::EnvelopeDone2;
                            counter = 0;
                        }
                        [[fallthrough]];
                    case EnvelopeState::EnvelopeDone2:
                        _value = static_cast<T>(0);
                        break;
                }
            }

            T value () const {
                return _value;
            }

            // can be called from the main thread
            void onKeyPressed() {
                state = EnvelopeState::KeyPressed;
                counter = 0;
                startPressed();
            }

            // can be called from the main thread
            bool onKeyReleased() {
                if(state == EnvelopeState::KeyPressed) {
                    if(0 == counter) {
                        // the key was pressed, but immediately released, so we skip the note.
                        state = EnvelopeState::EnvelopeDone2;
                    }
                    else {
                        state = EnvelopeState::KeyReleased;
                        _topValue = _value;
                        counter = 0;
                    }
                    return true;
                }
                return false;
            }

            EnvelopeState getState() const {
                return state;
            }

            bool afterAttackBeforeSustain() const {
              return isAfterAttackBeforeSustain(counter);
            }

        private:
            // between 0 and 1.
            FPT _value = static_cast<T>(0);
            FPT _topValue = static_cast<T>(0); // the value when the key was released

            // 'state' will be set to 'KeyReleased' when the key is released.
            // reads / writes to this are atomic so we don't need to lock.
            EnvelopeState state = EnvelopeState::EnvelopeDone2;
            int32_t counter = 0;
        };

        template <typename T, itp::interpolation AttackItp, itp::interpolation ReleaseItp>
        struct SimpleEnvelopeBase {
          static_assert(itp::intIsReal(AttackItp));
          static_assert(itp::intIsReal(ReleaseItp));
          static_assert(AttackItp != itp::PROPORTIONAL_VALUE_DERIVATIVE);
          static_assert(ReleaseItp != itp::PROPORTIONAL_VALUE_DERIVATIVE);

          using FPT = T;
          using Param = int;
          static constexpr auto Release = EnvelopeRelease::WaitForKeyRelease;

          static constexpr auto normalizedMinDt = 100;

            // len is in samples
            void setEnvelopeCharacTime(int len) {
                C = std::max(len,normalizedMinDt);
            }
        private:
          int32_t C = normalizedMinDt;

        protected:

          void startPressed() {}

          Optional<T> stepPressed (int32_t counter) const {
              return std::min
                  (static_cast<T>(1)
                   ,itp::interpolate( AttackItp
                                    , static_cast<T>(counter)
                                    , static_cast<T>(0)
                                    , static_cast<T>(1)
                                    , C));
          }

          int32_t getReleaseTime() const { return C; }

          static constexpr itp::interpolation getReleaseItp() { return ReleaseItp; }

          bool isAfterAttackBeforeSustain(int32_t counter) const {
            return counter >= 0 && counter < C;
          }
        };

        template <typename T, itp::interpolation AttackItp, itp::interpolation ReleaseItp>
        using SimpleEnvelope = EnvelopeCRT < SimpleEnvelopeBase <T, AttackItp, ReleaseItp> >;

        template <typename T>
        using SimpleLinearEnvelope = SimpleEnvelope < T, itp::LINEAR, itp::LINEAR >;

        /* This inner state describes states when the outer state is 'KeyPressed'. */
        enum class AHD : unsigned char {
            Attacking,
            Holding,
            Decaying
        };

        inline Optional<AHD> rotateAHD(AHD s) {
            switch(s) {
                case AHD::Attacking:
                    return AHD::Holding;
                case AHD::Holding:
                    return AHD::Decaying;
                case AHD::Decaying:
                    return {};
                default:
                    Assert(0);
                    return {};
            }
        }

        struct AHDSR {
            int32_t attack;
            itp::interpolation attackItp;
            int32_t hold;
            int32_t decay;
            itp::interpolation decayItp;
            int32_t release;
            itp::interpolation releaseItp;
            float sustain;
        };

        inline bool operator < (AHDSR const& l, AHDSR const& r)
        {
            return
            std::make_tuple(l.attack,l.attackItp,l.hold,l.decay,l.decayItp,l.release,l.releaseItp,l.sustain) <
            std::make_tuple(r.attack,r.attackItp,r.hold,r.decay,r.decayItp,r.release,r.releaseItp,r.sustain);
        }

        /* The AHDSR envelope is like an ADSR envelope, except we allow to hold the value after the attack:

         Attack, decay and release interpolations are specified via parameters.

           | a |h| d |           |r|
               ___                                      < 1
              .   .
             .      ._____________                      < s
            .                     .
         ___                       ___________________  < 0
           ^                     ^ ^             ^
           |                     | envelopeDone1 envelopeDone2    <- state changes
           keyPressed            keyReleased                      <- state changes
           attacking                                              <- inner pressed state changes
               holding                                            <- inner pressed state changes
                 decaying                                         <- inner pressed state changes
                    sustaining                                    <- inner pressed state changes

         */
        template <typename T, EnvelopeRelease Rel>
        struct AHDSREnvelopeBase {
            using FPT = T;
            using Param = AHDSR;
            static constexpr auto Release = Rel;

            void setEnvelopeCharacTime(int len) {
                // we just ignore this, but it shows that the design is not optimal:
                //   we should unify 'setEnvelopeCharacTime' with 'setAHDSR'
                //Assert(0 && "ADSR enveloppes cannot be set using setEnvelopeCharacTime");
            }

            // fast moog attacks are 1ms according to
            // cf. https://www.muffwiggler.com/forum/viewtopic.php?t=65964&sid=0f628fc3793b76de64c7bceabfbd80ff
            // so we set the max normalized enveloppe velocity to 1ms (i.e the time to go from 0 to 1)
            static constexpr auto normalizedMinDt = SAMPLE_RATE/1000;
            static_assert(normalizedMinDt > 0);

            void setAHDSR(AHDSR const & s) {
                int32_t _A = s.attack;
                int32_t _H = s.hold;
                int32_t _D = s.decay;
                int32_t _R = s.release;
                auto S = clamp (s.sustain, static_cast<T>(0), static_cast<T>(1));
                SMinusOne = S - static_cast<T>(1);
                A = std::max( _A, normalizedMinDt );
                H = std::max( _H, 0 );
                D = std::max( _D, static_cast<int>(static_cast<T>(normalizedMinDt) * (static_cast<T>(1) - S)));
                R = std::max( _R, static_cast<int>(static_cast<T>(normalizedMinDt) * S));
                attackItp = s.attackItp;
                decayItp = s.decayItp;
                releaseItp = s.releaseItp;
            }

        protected:
            void startPressed() {
                ahdState = AHD::Attacking;
                onAHDStateChange();
            }

            Optional<T> stepPressed(int) {
                if(ahdState) {
                    stepAHD();
                    ++ahdCounter;
                }
                if(!ahdState) {
                    if constexpr (Rel == EnvelopeRelease::WaitForKeyRelease) {
                      return static_cast<T>(1) + SMinusOne;
                    }
                    else {
                      return {};
                    }
                }
                else {
                    T from, toMinusFrom;
                    switch(get_value(ahdState)) {
                        case AHD::Attacking:
                            from = static_cast<T>(0);
                            toMinusFrom = static_cast<T>(1);
                            break;
                        case AHD::Holding:
                            from = static_cast<T>(1);
                            toMinusFrom = static_cast<T>(0);
                            break;
                        case AHD::Decaying:
                            from = static_cast<T>(1);
                            toMinusFrom = SMinusOne;
                            break;
                        default:
                            toMinusFrom = from = static_cast<T>(0);
                            Assert(0);
                            break;
                    }

                    return itp::interpolate(
                        getInterpolation()
                      , static_cast<T>(ahdCounter)
                      , from
                      , toMinusFrom
                      , getMaxCounterForAHD());
                }
            }

            int32_t getReleaseTime() const { return R; }

            itp::interpolation getReleaseItp() const { return releaseItp; }

            bool isAfterAttackBeforeSustain(int) const {
              return static_cast<bool>(ahdState);
            }

        private:
            int32_t ahdCounter = 0;
            int32_t A = normalizedMinDt;
            int32_t H = 0;
            int32_t D = normalizedMinDt;
            int32_t R = normalizedMinDt;
            itp::interpolation attackItp;
            itp::interpolation decayItp;
            itp::interpolation releaseItp;
            T SMinusOne = static_cast<T>(-0.5);

            // this inner state is taken into account only while the outer state is KeyPressed:
            Optional<AHD> ahdState;

            void onAHDStateChange() {
                ahdCounter = 0;
                if(!ahdState) {
                    return;
                }
                if(0==getMaxCounterForAHD()) {
                    stepAHD();
                }
            }

            void stepAHD() {
                Assert(ahdState);
                auto maxCounter = getMaxCounterForAHD();
                Assert(ahdCounter >= 0);
                if (ahdCounter < maxCounter) {
                    return;
                }
                ahdState = rotateAHD(get_value(ahdState));
                onAHDStateChange();
            }

            int32_t getMaxCounterForAHD() const {
                switch(get_value(ahdState)) {
                    case AHD::Attacking:
                        return A;
                    case AHD::Holding:
                        return H;
                    case AHD::Decaying:
                        return D;
                    default:
                        Assert(0);
                        return 0;
                }
            }

            itp::interpolation getInterpolation() const {
                switch(get_value(ahdState)) {
                    case AHD::Attacking:
                        return attackItp;
                    case AHD::Holding:
                        return itp::LINEAR;
                    case AHD::Decaying:
                        return decayItp;
                    default:
                        Assert(0);
                        return itp::LINEAR;
                }
            }
        };

        template <typename T, EnvelopeRelease Rel>
        using AHDSREnvelope = EnvelopeCRT < AHDSREnvelopeBase <T, Rel> >;

        template <typename ALGO>
        using SimplyEnveloped = Envelopped<ALGO,SimpleLinearEnvelope<typename ALGO::FPT>>;

        template<typename ALGO>
        struct VolumeAdjusted {
            static constexpr auto hasEnvelope = ALGO::hasEnvelope;
            using T = typename ALGO::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value);

            T real() const { return volume * osc.real(); }
            T imag() const { return volume * osc.imag(); }

            T angleIncrements() const { return osc.angleIncrements(); }
            T angle() const { return osc.angle(); }

            VolumeAdjusted() : log_ratio_(1.f), low_index_(0) {}

            void forgetPastSignals() {
                osc.forgetPastSignals();
            }
            void setEnvelopeCharacTime(int len) {
              osc.setEnvelopeCharacTime(len);
            }
            void onKeyPressed() {
                osc.onKeyPressed();
            }
            bool onKeyReleased() {
                return osc.onKeyReleased();
            }

            void setFiltersOrder(int order) {
                osc.setFiltersOrder(order);
            }

            void setAngleIncrements(T ai) {
                volume = loudness::equal_loudness_volume(angle_increment_to_freq(ai),
                                                         low_index_,
                                                         log_ratio_,
                                                         loudness_level);
                osc.setAngleIncrements(ai);
            }

            void setAngle(T ai) {
                osc.setAngle(ai);
            }

            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                Assert(low_index >= 0);
                Assert(low_index < 16);
                low_index_ = low_index;
                Assert(log_ratio >= 0.f);
                Assert(log_ratio <= 1.f);
                log_ratio_ = log_ratio;
                this->loudness_level = loudness_level;
            }

            void step() { osc.step(); }

            bool isEnvelopeFinished() const {
              return osc.isEnvelopeFinished();
            }

        private:
            uint32_t low_index_ : 4;
            float loudness_level;
            float log_ratio_;
            float volume;
            ALGO osc;
        };

        // the unit of angle increments is "radian / pi"

        template<typename T>
        struct Phased {
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value);
            using Tr = NumTraits<T>;

            Phased() = default;
            Phased(T angle_increments) { setAngleIncrements(angle_increments); }

            void setAngle( T angle ) { angle_ = angle; }
            T angle() const { return angle_; }

            void setAngleIncrements(T v) {
                Assert(std::abs(v) < Tr::two()); // else need to modulo it
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
            static constexpr auto hasEnvelope = false;
            using Phased<T>::angle_;

            PCOscillatorAlgo() = default;
            PCOscillatorAlgo(T angle_increments) : Phased<T>(angle_increments) {}

            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }

            T real() const { return std::cos(static_cast<T>(M_PI) * angle_); }
            T imag() const { return std::sin(static_cast<T>(M_PI) * angle_); }
        };

        template<typename T>
        using PCOscillator = FinalAudioElement<PCOscillatorAlgo<T>>;

        template<Sound::Type SOUND>
        struct soundBufferWrapperAlgo {
            static constexpr auto hasEnvelope = false;
            using F_GET_BUFFER = FGetBuffer<SOUND>;
            using T = soundBuffer::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value);

            soundBufferWrapperAlgo() {
                F_GET_BUFFER().getAbsMean(); // just to initialize the static in it
            }

            void forgetPastSignals() {
            }
            void setEnvelopeCharacTime(int len) {
                Assert(0);
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }

            T imag() const { return sb[index]; }

            void step() {
                ++index;
                if(index == sb.size()) {
                    index = 0;
                }
            }

            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }

            int index = -1;
            soundBuffer const & sb = F_GET_BUFFER()();
        };

        template<typename T>
        using WhiteNoise = FinalAudioElement< soundBufferWrapperAlgo<Sound::NOISE> >;

        template<typename T>
        struct SquareAlgo : public Phased<T> {
            static constexpr auto hasEnvelope = false;
            using Phased<T>::angle_;

            SquareAlgo() = default;
            SquareAlgo(T angle_increments) : Phased<T>(angle_increments) {}

            T imag() const { return square(angle_); }

            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }
        };

        template<typename Envel>
        using Square = FinalAudioElement<Envelopped<SquareAlgo<typename Envel :: FPT>,Envel>>;

        /*
         * first pulse happends at angle = 0
         */
        template<typename T>
        struct PulseTrainAlgo : public Phased<T> {
            static constexpr auto hasEnvelope = false;

            using Tr = NumTraits<T>;
            using Phased<T>::angle_;

            PulseTrainAlgo() = default;
            PulseTrainAlgo(T angle_increments, T pulse_width) :
            Phased<T>(angle_increments),
            pulse_width(pulse_width) {
                Assert(pulse_width >= angle_increments); // else it's always 0
            }

            void forgetPastSignals() {
            }

            void set(T angle_increments, T pulse_width_) {
                Assert(pulse_width_ >= angle_increments); // else it's always 0
                this->setAngleIncrements(angle_increments);
                pulse_width = pulse_width_;
            }

            T imag() const { return pulse(angle_, pulse_width); }

            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }
        private:
            T pulse_width{};
        };

        template<typename T>
        using PulseTrain = FinalAudioElement<PulseTrainAlgo<T>>;


        template<class...AEs>
        struct Mix {
            static constexpr auto hasEnvelope = false; // TODO all hasEnvelope AEs ?
            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }

            using T = typename NthTypeOf<0, AEs...>::FPT;
            using FPT = T;
            static_assert(std::is_floating_point<FPT>::value);

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

            void setFiltersOrder(int order) {
                for_each(aes, [order](auto & ae) {
                    ae.setFiltersOrder(order);
                });
            }

            void forgetPastSignals() {
                for_each(aes, [](auto & ae) {
                    ae.forgetPastSignals();
                });
            }
            void setEnvelopeCharacTime(int len) {
                for_each(aes, [len](auto & ae) {
                    ae.setEnvelopeCharacTime(len);
                });
            }

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
            static_assert(std::is_floating_point<FPT>::value);

        private:
            std::tuple<AEs...> aes;
        };

        template<int ORDER, typename T>
        struct InternalFilterFPTFromOrder {
            using type = double; // for order >= 2 and 0(adjustable), we use double
        };

        template<typename T>
        struct InternalFilterFPTFromOrder<1,T> {
            using type = T;
        };

        template<typename AEAlgo, FilterType KIND, int ORDER>
        struct FilterAlgo {
            static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
            using T = typename AEAlgo::FPT;
            using FPT = T;
            using FilterFPT = typename InternalFilterFPTFromOrder<ORDER, FPT>::type;
            static_assert(std::is_floating_point<FPT>::value);

            bool isEnvelopeFinished() const {
              return audio_element.isEnvelopeFinished();
            }
            void onKeyPressed() {
                return audio_element.onKeyPressed();
            }
            bool onKeyReleased() {
                return audio_element.onKeyReleased();
            }

        private:
            AEAlgo audio_element;
            Filter<FilterFPT, 1, KIND, ORDER> filter_;
        public:
            void forgetPastSignals() {
                filter_.forgetPastSignals();
                audio_element.forgetPastSignals();
            }
            void setEnvelopeCharacTime(int len) {
                audio_element.setEnvelopeCharacTime(len);
            }

            void setFiltersOrder(int order) {
                filter_.setOrder(order);
            }

            void step() {
                audio_element.step();
                FilterFPT val = static_cast<FilterFPT>(audio_element.imag());
                filter_.feed(&val);
            }

            // sets the filter frequency
            void setAngleIncrements(T v) {
                filter_.initWithAngleIncrement(v);
            }

            T imag() const {
                return *filter_.filtered();
            }

            auto & get_element() { return audio_element; }
            auto & get_element() const { return audio_element; }
            auto & filter() { return filter_; }

            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {}
        };

        template<typename T, int ORDER>
        using LowPassAlgo = FilterAlgo<T, FilterType::LOW_PASS, ORDER>;

        template<typename T, int ORDER>
        using HighPassAlgo = FilterAlgo<T, FilterType::HIGH_PASS, ORDER>;

        template<typename AEAlgo, int ORDER>
        struct BandPassAlgo_ {
            static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
            using FPT = typename AEAlgo::FPT;
            using T = FPT;
            static constexpr auto low_index = 0;
            static constexpr auto high_index = 1;

            void setCompensation(T sq_inv_width_factor) {
                // gain compensation to have an equal power of the central frequency for all widths
                compensation = expt<ORDER>(1 + sq_inv_width_factor);
#ifndef NDEBUG
                // verify accuracy of above simplification

                // inc / low == width_factor
                // inc / high == 1 / width_factor
                auto inv_sq_mag_hp = get_inv_square_filter_magnitude<FilterType::HIGH_PASS>(1/sq_inv_width_factor);
                auto inv_sq_mag_lp = get_inv_square_filter_magnitude<FilterType::LOW_PASS >(sq_inv_width_factor);
                auto inv_mag = sqrt(inv_sq_mag_hp * inv_sq_mag_lp);
                auto original_compensation = inv_mag;
                original_compensation = expt<ORDER>(original_compensation);
                Assert(std::abs(original_compensation - compensation) / (original_compensation + compensation) < FLOAT_EPSILON);
#endif
            }

            T imag() const {
                return compensation * cascade.imag();
            }

            bool isEnvelopeFinished() const {
              return cascade.isEnvelopeFinished();
            }
            void onKeyPressed() {
                return cascade.onKeyPressed();
            }
            bool onKeyReleased() {
                return cascade.onKeyReleased();
            }

        protected:
            T compensation;
        private:
            HighPassAlgo<LowPassAlgo<AEAlgo, ORDER>, ORDER> cascade;
        protected:

            auto & getHP() { return cascade; }
            auto & getLP() { return cascade.get_element(); }

            void onWidthFactor(float inv_width_factor) {
                setCompensation(inv_width_factor * inv_width_factor);
            }

            void doStep() {
                cascade.step();
            }
        };

        template<typename AEAlgo, int ORDER>
        struct BandRejectAlgo_ {
            static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
            using FPT = typename AEAlgo::FPT;
            using T = FPT;

            T imag() const { return lp.imag() + hp.imag(); }

            bool isEnvelopeFinished() const {
              return lp.isEnvelopeFinished() && hp.isEnvelopeFinished();
            }
            void onKeyPressed() {
                lp.onKeyPressed();
                hp.onKeyPressed();
            }
            bool onKeyReleased() {
                auto res = lp.onKeyReleased();
                auto res2 = hp.onKeyReleased();
                return res || res2;
            }

        private:
            LowPassAlgo<AEAlgo, ORDER> lp;
            HighPassAlgo<AEAlgo, ORDER> hp;
        protected:
            using Tr = NumTraits<T>;

            static constexpr auto compensation = Tr::one();

            static constexpr auto low_index = 1;
            static constexpr auto high_index = 0;

            auto & getHP() { return hp; }
            auto & getLP() { return lp; }

            void onWidthFactor(float) const {}

            void doStep() {
                lp.step();
                hp.step();
            }
        };

        template<typename AEAlgoWidth, typename Base>
        struct BandAlgo_ : public Base {
            using FPT = typename Base::FPT;
            using T = FPT;
            using Tr = NumTraits<T>;
            using AEWidth = AEAlgoWidth;
            using Base::compensation;
            using Base::low_index;
            using Base::high_index;
            using Base::onWidthFactor;
            using Base::getHP;
            using Base::getLP;
            using Base::doStep;


            void forgetPastSignals() {
                getHP().forgetPastSignals();
                getLP().forgetPastSignals();
                width.forgetPastSignals();
            }

            void setFiltersOrder(int order) {
                getHP().setFiltersOrder(order);
                getLP().setFiltersOrder(order);
            }

            void setWidthRange(range<float> const & r) {
                width_range = r;
            }

            void setAngleIncrements(T inc) {
                Assert(inc >= 0.f);
                increment = inc;
            }

            void step() {
                width.step();
                T width_factor = pow(Tr::two(), width_range.getAt(std::abs(width.imag())));

                Assert(width_factor);
                auto inv_width_factor = Tr::one() / width_factor;
                auto low  = increment * inv_width_factor;
                auto high = increment * width_factor;

                onWidthFactor(inv_width_factor);
                doSetAngleIncrements({{low, high}});

                doStep();
            }

            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) const {}

            AEWidth & getWidth() { return width; }

        protected:
            T increment;
            AEAlgoWidth width;
            range<float> width_range;


            void doSetAngleIncrements(std::array<T, 2> incs) {
                /*
                static auto deb = 0;
                ++deb;
                if(deb == 10000) {
                    deb = 0;
                    LG(INFO, "%.2f %.2f",
                       angle_increment_to_freq(incs[low_index]),
                       angle_increment_to_freq(incs[high_index]));
                }
                 */
                getHP().setAngleIncrements(incs[low_index]);
                getLP().setAngleIncrements(incs[high_index]);
            }
        };

        template<typename AEAlgo, typename AEAlgoWidth, int ORDER>
        using BandPassAlgo = BandAlgo_<AEAlgoWidth, BandPassAlgo_<AEAlgo, ORDER>>;

        template<typename AEAlgo, typename AEAlgoWidth, int ORDER>
        using BandRejectAlgo = BandAlgo_<AEAlgoWidth, BandRejectAlgo_<AEAlgo, ORDER>>;

        template<typename T>
        struct LoudnessCompensationFilter {
            LoudnessCompensationFilter(unsigned int fft_length, unsigned int NumTaps) :
            filter(imajuscule::loudness::getLoudnessCompensationFIRCoefficients<T>(fft_length, NumTaps))
            {}

            void step(T val) {
                filter.step(val);
            }

            T get() const { return filter.get(); }

            auto size() const { return filter.size(); }
        private:
            FIRFilter<T> filter;
        };

        template<typename T, int ORDER>
        using LPWhiteNoiseAlgo = LowPassAlgo<soundBufferWrapperAlgo<Sound::NOISE>, ORDER>;

        template<typename T, int ORDER>
        using LPWhiteNoise = FinalAudioElement<LPWhiteNoiseAlgo<T, ORDER>>;

        template<typename AEAlgo, int ORDER>
        using LowPass = FinalAudioElement<LowPassAlgo<AEAlgo, ORDER>>;

        enum class eNormalizePolicy : unsigned char {
            FAST,
            ACCURATE
        };

        template<typename T, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
        struct OscillatorAlgo {
            static constexpr auto hasEnvelope = false;
            using Tr = NumTraits<T>;
            using FPT = T;

            constexpr OscillatorAlgo(T angle_increments) { setAngleIncrements(angle_increments); }
            constexpr OscillatorAlgo() : mult(Tr::one(), Tr::zero()) {}

            void forgetPastSignals() {
            }
            void setEnvelopeCharacTime(int len) {
                Assert(0);
            }
            void onKeyPressed() {
                Assert(0);
            }
            bool onKeyReleased() {
                Assert(0);
                return false;
            }

            void setFiltersOrder(int order) const {
            }

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

            bool isEnvelopeFinished() const {
              Assert(0);
              return false;
            }

            private:
            complex<T> cur = {Tr::one(), Tr::zero()};
            complex<T> mult;

            void approx_normalize() {
                // http://dsp.stackexchange.com/questions/971/how-to-create-a-sine-wave-generator-that-can-smoothly-transition-between-frequen

                cur *= Tr::half() * (Tr::three() - norm(cur));
            }

            void normalize() {
                cur *= 1/abs(cur);
            }
        };

        template<typename Envel, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
        using Oscillator = FinalAudioElement<Envelopped<OscillatorAlgo<typename Envel::FPT, NormPolicy>, Envel>>;

        template<typename T>
        struct LogRamp {
            static_assert(std::is_same<T,float>::value, "non-float interpolation is not supported");

            using Tr = NumTraits<T>;
            using FPT = T;

            // offsets because we use the value at the beginning of the timestep
            static constexpr auto increasing_integration_offset = 0;
            static constexpr auto decreasing_integration_offset = 1;

            LogRamp() : cur_sample(Tr::zero()), from{}, to{}, duration_in_samples{}
            {}

            void forgetPastSignals() {}

            void setFreqRange(range<float> const &) const { Assert(0); } // use set instead
            void set_interpolation(itp::interpolation) const {Assert(0);} // use set instead
            void set_n_slow_steps(unsigned int) const { Assert(0); }

            auto & getUnderlyingIter() { Assert(0); return *this; }

            void set(T from_increments,
                     T to_increments,
                     T duration_in_samples_,
                     T start_sample,
                     itp::interpolation i) {
                if(start_sample >= Tr::zero()) {
                    cur_sample = start_sample;
                }
                else {
                    // if start_sample < 0 we adapt it to be at the same ratio in the new range
                    // and we adapt bounds order to the existing bounds
                    Assert(duration_in_samples);
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

                Assert(duration_in_samples > 0);
                interp.setInterpolation(i);
            }

            T do_step(T proportionality) {
                if(cur_sample + .5f > duration_in_samples) {
                    cur_sample = Tr::zero();
                    std::swap(from, to);
                }

                // we call get_unfiltered_value instead of get_value because we ensure:
                Assert(cur_sample <= duration_in_samples);
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

            T getFrom() const { return from; }
            T getTo() const { return to; }

            T get_duration_in_samples() const { return duration_in_samples; }

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
                Assert(from > 0);
                Assert(to > 0);
                // else computation cannot be done

                return (to==from) ? 1.f : -std::log(from/to) / (to-from);
            }
        };

        /*
         * std::abs(<value>)
         */
        template<typename Iterator>
        struct AbsIter {
            using FPT = typename Iterator::FPT;

            void initializeForRun() {
                it.initializeForRun();
            }

            void operator ++() {
                ++it;
            }

            auto operator *() const {
                return std::abs(*it);
            }

            float getAbsMean() const { return it.getAbsMean(); }
        private:
            Iterator it;
        };

        /*
         * Slow down iteration by an integer factor and chose interpolation.
         */
        template<typename Iterator>
        struct SlowIter {
            using FPT = typename Iterator::FPT;

            using uint_steps = int32_t;

            SlowIter(itp::interpolation interp) : interp(interp) {
            }
            SlowIter() : SlowIter(itp::LINEAR) {
            }

            void set_interpolation(itp::interpolation i) { interp.setInterpolation(i); }
            void set_n_slow_steps(uint_steps n) {
                if(n == n_steps) {
                    return;
                }
                if(slow_it) {
                    // adapt the iterator
                    Assert(n_steps);
                    auto ratio = (slow_it + .5f) / static_cast<float>(n_steps);
                    Assert(ratio < 1.f);
                    Assert(ratio > 0.f);
                    slow_it = ratio * static_cast<float>(n);
                    if(slow_it == n) {
                        onMajorStep();
                    }
                }
                n_steps = n;
                Assert(n_steps > 0);
            }

            void initializeForRun() {
                it.initializeForRun();
                onMajorStep();
            }

            bool increment() {
                ++slow_it;
                if(slow_it < n_steps) {
                    return false;
                }
                onMajorStep();
                return true;
            }

            void operator ++() {
                increment();
            }

            float operator *() const {
                // todo verify that this is inlined (probably not...)
                // for performance we may need to have a NormalizedInterpolation class templated for interp
                Assert(n_steps >= 1);
                return interp.get_unfiltered_value(slow_it, n_steps, prev, *it);
            }

            bool isDiminishing() const {
                return prev > *it;
            }

            bool isAugmenting() const {
                return !isDiminishing();
            }

            auto const & getUnderlyingIterator() const { return it; }

            float getAbsMean() const { return it.getAbsMean(); }

        private:
            uint_steps n_steps = -1;
            uint_steps slow_it = 0;
            NormalizedInterpolation<> interp;
            float prev;
            Iterator it;

            void onMajorStep() {
                slow_it = 0;
                prev = *it;
                ++it;
//                LG(INFO, "++it:");
//                it.log();
            }

        };

        /*
         * Makes ascending iteration faster than descending.
         */
        template<typename UnderlyingIt, int SCALE_UP = 3>
        struct WindFreqIter {
            template <class... Args>
            WindFreqIter(Args&&... args) : it(std::forward<Args>(args)...) {}

            void initializeForRun() {
                it.initializeForRun();
            }

            void operator ++() {
                auto const n = [this]() {
                    if(it.isDiminishing()) {
                        return 1;
                    }
                    return SCALE_UP;
                }();

                for(int i=0; i<n; ++i) {
                    if(it.increment()) {
                        return;
                    }
                }
            }

            auto operator *() const {
                return *it;
            }

            float getAbsMean() const { return it.getAbsMean(); }

            auto & getUnderlyingIter() {
                return it;
            }

        private:
            UnderlyingIt it;
        };

        template<typename ITER>
        struct Ctrl {
            using FPT = typename ITER::FPT;
            using T = FPT;
            using Tr = NumTraits<T>;

            void forgetPastSignals() {
                it.initializeForRun();
            }
            void setFiltersOrder(int ) const {}
            void setAngleIncrements(T ) const {}

            void step() { ++it; }
            T imag() const { return *it; }

            auto & getUnderlyingIter() {
                return it.getUnderlyingIter();
            }

            float getAbsMean() const { return it.getAbsMean(); }
        private:
            WindFreqIter<ITER> it = { itp::LINEAR };
        };

        enum class VolumeAdjust : unsigned char{
            Yes,
            No
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
        static int steps_to_swap(LogRamp<T> & spec) {
            bool order = spec.getTo() < spec.getFrom();

            int count = 0;
            while(order == spec.getTo() < spec.getFrom()) {
                spec.step();
                ++count;
            }
            --count; // because swap occurs at the beginning of step method, before current step is modified
            return count;
        }

        template<typename ALGO, typename CTRLS, int N>
        struct SetFreqs {
            void operator()(CTRLS & ctrls, ALGO & osc) {
                constexpr auto sz = std::tuple_size<CTRLS>::value;
                std::array<typename ALGO::FPT, sz> increments;
                int i = 0;
                for_each(ctrls, [&i, &increments] (auto & c) {
                    increments[i] = freq_to_angle_increment(c.step());
                    ++i;
                });
                osc.setAngleIncrements(std::move(increments));
            }
        };

        template<typename ALGO, typename CTRLS>
        struct SetFreqs<ALGO, CTRLS, 1> {
            void operator()(CTRLS & ctrls, ALGO & osc) {
                auto f = std::get<0>(ctrls).step();
                osc.setAngleIncrements(freq_to_angle_increment(f));
            }
        };

        template<typename ALGO, typename CTRLS>
        void setFreqs(CTRLS & ctrls, ALGO & osc) {
            SetFreqs<ALGO,CTRLS, std::tuple_size<CTRLS>::value>{}(ctrls, osc);
        }

        template<typename ALGO, typename ...CTRLS>
        struct FreqCtrl_ {
            static constexpr auto hasEnvelope = ALGO::hasEnvelope;
            using Ctrl = std::tuple<CTRLS...>;
            using T = typename ALGO::FPT;
            using FPT = T;

            using Tr = NumTraits<T>;

            T angle() const { return {}; }

            void setLoudnessParams(int low_index, float log_ratio, float loudness_level) {
                osc.setLoudnessParams(low_index, log_ratio, loudness_level);
            }

            void setFiltersOrder(int order) {
                for_each(ctrls, [order](auto & c) {
                    c.setFiltersOrder(order);
                });
                osc.setFiltersOrder(order);
            }

            void forgetPastSignals() {
                for_each(ctrls, [](auto & c) {
                    c.forgetPastSignals();
                });
                osc.forgetPastSignals();
            }

            void setAngleIncrements(T v) {
                for_each(ctrls, [v](auto & c) {
                    c.setAngleIncrements(v);
                });
                // not for osc : it will be done in step()
            }

            void step() {
                setFreqs(ctrls, osc);
                osc.step();
            }

            T real() const { return osc.real(); }
            T imag() const { return osc.imag(); }

            auto & getOsc() { return osc; }

            auto & getCtrl() {
                assert(std::tuple_size<Ctrl>::value == 1);
                return std::get<0>(ctrls);
            }

            bool isEnvelopeFinished() const {
                return osc.isEnvelopeFinished();
            }
            void setEnvelopeCharacTime(int len) {
                osc.setEnvelopeCharacTime(len);
            }
            void onKeyPressed() {
                osc.onKeyPressed();
            }
            bool onKeyReleased() {
                return osc.onKeyReleased();
            }

        private:
            Ctrl ctrls;
            ALGO osc;
        };

        template<typename T, VolumeAdjust V>
        using FreqRampOscillatorAlgo_ = FreqCtrl_<AdjustableVolumeOscillatorAlgo<V,T>, LogRamp<T>>;

        template<typename T>
        using FreqRampAlgo = FreqRampOscillatorAlgo_<T, VolumeAdjust::Yes>;

        template<typename Envel>
        using FreqRamp = FinalAudioElement<Envelopped<FreqRampAlgo<typename Envel::FPT>, Envel>>;

        template<typename T, int ORDER>
        using FreqRampLPWhiteNoiseAlgo_ = FreqCtrl_<LPWhiteNoiseAlgo<T, ORDER>, LogRamp<T>>;

        template<typename T, int ORDER>
        using FreqRampLPWhiteNoise = FinalAudioElement<FreqRampLPWhiteNoiseAlgo_<T, ORDER>>;


        template<typename A1, typename A2>
        struct RingModulationAlgo {
            static constexpr auto hasEnvelope = A1::hasEnvelope || A2::hasEnvelope;
            using T = typename A1::FPT;
            using FPT = T;

            static_assert(std::is_same<typename A1::FPT, typename A2::FPT>::value); // else choice for T is arbitrary

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

            void forgetPastSignals() {
                osc1.forgetPastSignals();
                osc2.forgetPastSignals();
            }

            void step() {
                osc1.step();
                osc2.step();
            }

            T real() const { return osc1.real() * osc2.real(); }
            T imag() const { return osc1.imag() * osc2.imag(); }

            auto & get_element_1() { return osc1; }
            auto & get_element_2() { return osc2; }

            bool isEnvelopeFinished() const {
                if(A1::hasEnvelope && osc1.isEnvelopeFinished()) {
                    return true;
                }
                if(A2::hasEnvelope && osc2.isEnvelopeFinished()) {
                    return true;
                }
                if(!A1::hasEnvelope && !A2::hasEnvelope) {
                    Assert(0);
                }
                return false;
            }
            void onKeyPressed() {
                if(A1::hasEnvelope) {
                    osc1.onKeyPressed();
                }
                if(A2::hasEnvelope) {
                    osc2.onKeyPressed();
                }
                if(!A1::hasEnvelope && !A2::hasEnvelope) {
                    Assert(0);
                }
            }
            bool onKeyReleased() {
                bool res = false;
                if(A1::hasEnvelope) {
                    res = osc1.onKeyReleased();
                }
                if(A2::hasEnvelope) {
                    res = osc2.onKeyReleased() || res;
                }
                if(!A1::hasEnvelope && !A2::hasEnvelope) {
                    Assert(0);
                }
                return res;
            }

        private:
            A1 osc1;
            A2 osc2;
        };

        template<typename A1, typename A2>
        using RingModulation = FinalAudioElement<RingModulationAlgo<A1,A2>>;

      template <typename T>
      auto fCompute(T&e) {
        return [&e](bool sync_clock) {
          using AE = AEBuffer<typename T::FPT>;

          if(e.getState() == AE::inactive()) {
            // Issue : if the buffer just got marked inactive,
            // but no new AudioElementCompute happends
            // and from the main thread someone acquires this and queues it,
            // it will have 2 lambdas because the first lambda will never have seen the inactive state.
            // However the issue is not major, as the 2 lambdas have a chance to be removed
            // the next time
            if constexpr (T::hasEnvelope) {
              // Temporary, to fix soundengine, where the channel xfade is used instead of the enveloppe xfade:
              // this resets the eveloppe state to Done2, and then lets the audioelement be reused
              // in subsequent markov move.
              e.forgetPastSignals();
            }
            return false;
          }
          if(likely(e.getState() != AE::queued())) {
            if(sync_clock == e.clock_) {
              // we alredy did compute this step.
              return true;
            }
          }
          e.clock_ = sync_clock;
          for(auto & v : e.buffer->buffer) {
            e.algo.step();
            v = e.algo.imag();
          }
          Assert(e.getState() != AE::queued());
          Assert(e.getState() != AE::inactive());
          if constexpr (T::hasEnvelope) {
            // it is important that isEnvelopeFinished() returns true only one buffer cycle after
            // the real enveloppe end, to prevent "race conditions".
            return !e.algo.isEnvelopeFinished();
          }
          else {
            return true;
          }
        };
      }
      
    } // NS audioelement
} // NS imajuscule
