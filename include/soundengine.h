namespace imajuscule::audio {

enum class FreqXfade : unsigned char {
  BEGIN,

  No=BEGIN,
  NonTrivial,
  All,

  END
};

enumTraversal const & xfade_freq_traversal();

} // NS

namespace imajuscule::audio::audioelement {

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

      void set_sample_rate(int s) {
        ctrl.set_sample_rate(s);
      }

      void setAngleIncrementsRange(range<float> const & r) {
        Assert(r.getMin() > 0);
        Assert(r.getMax() > 0);
        Assert(r.getMin() <= r.getMax());
        this->log_increment_min = std::log(r.getMin());
        this->log_increment_max = std::log(r.getMax());
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

      T getAngleIncrementFrom() const {
        return 0;
      }

      T step() {
        ctrl.step();
        auto v = ctrl.imag() * invApproxRange;
        Assert(v >= 0.f); // else, use AbsIter
        // v is roughly between 0 and 1, but could be bigger than 1.
        // when v is 0, we want increment_min,
        // when v is 1, we want increment_max,
        // and an exponential interpolation between them
        Assert(log_increment_max >= log_increment_min);
        T const log_increment = log_increment_min + (log_increment_max - log_increment_min) * v;
        return std::exp(log_increment);
      }

      auto & getUnderlyingIter() { return ctrl.getUnderlyingIter(); }

    private:
      T invApproxRange;
      T log_increment_max{}, log_increment_min{};
      audioelement::Ctrl<ITER> ctrl;
    };

    template<typename CTRL, typename NOISE_ITER>
    struct ShortTermNoiseAdderCtrl {
      using T = typename CTRL::FPT;
      using FPT = T;
      using Tr = NumTraits<T>;

      void set_sample_rate(int s) {
        ctrl.set_sample_rate(s);
        noise.set_sample_rate(s);
      }

      void setAngleIncrementsRange(range<float> const & r) {
        ctrl.setAngleIncrementsRange(r);
      }

      void forgetPastSignals() {
        ctrl.forgetPastSignals();
        noise.initializeForRun();
      }

      void set_short_term_noise_amplitude(float f) {
        noise_amplitude = f;
      }

      void set_short_term_noise_rate(float f) {
        ratio = f;
      }

      T step() {
        auto long_term_angle_increment = ctrl.step();
        Assert(long_term_angle_increment > 0.f);
        Assert(ratio >= 0.f);
        // keep short term noise rate inv. proportional to long term frequency
        noise.set_n_slow_steps(1 + ratio / long_term_angle_increment);
        ++noise;
        // TODO optimize this pow:
        return long_term_angle_increment * std::pow(Tr::two(), *noise * noise_amplitude);
      }

      auto get_duration_in_samples() const {
        return std::numeric_limits<int>::max()
        / 2;  // so that we can add .5f and cast to int
      }

      auto & getUnderlyingIter() { return ctrl.getUnderlyingIter(); }

      // todo remove and make SoundEngine more generic
      void set(T from_,
               T to_,
               T duration_in_samples_,
               T start_sample,
               itp::interpolation i) {
        Assert(0);
      }
      float getAngleIncrementFrom() const {
        return 0.f;
      }
      float getTo() const {
        Assert(0);
        return 0.f;
      }

    private:
      T noise_amplitude;
      T ratio = -1.f;
      CTRL ctrl;
      NOISE_ITER noise;
    };

    // 'setAngleIncrements' of BandAlgo is controlled by SoundEngineFreqCtrl

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
      using CTRL = audioelement::LogRamp< typename T::FPT >;
      using type = audioelement::FreqCtrl_< T, CTRL >;
    };

    template<typename T>
    struct SoundEngineAlgo_<T, SoundEngineMode::WIND> {
      using CTRL = ShortTermNoiseAdderCtrl < SoundEngineFreqCtrl< audioelement::SlowIter<audioelement::AbsIter< PinkNoiseIter>> >, audioelement::SlowIter< PinkNoiseIter> >;
      using type = audioelement::FreqCtrl_< T, CTRL >;
    };

    static constexpr auto Order = VariableOrder;
    using GreyNoiseAlgo = audioelement::soundBufferWrapperAlgo<Sound::GREY_NOISE>;
    using PinkNoiseAlgo = audioelement::soundBufferWrapperAlgo<Sound::PINK_NOISE>;

    template<SoundEngineMode M>
    struct MixOf {
      using type =
      audioelement::Mix
      <
      audioelement::LoudnessAdjustableVolumeOscillatorAlgo<audioelement::LoudnessVolumeAdjust::Yes, double>
      >;
    };

    template<> struct MixOf<SoundEngineMode::WIND> {
      using type = audioelement::Mix
      <
      audioelement::LowPassAlgo<PinkNoiseAlgo, Order>,
      AsymBandPassAlgo<PinkNoiseAlgo, Order, audioelement::SlowIter<audioelement::AbsIter<PinkNoiseIter>>>,
      AsymBandRejectAlgo<PinkNoiseAlgo, Order, audioelement::SlowIter<audioelement::AbsIter<PinkNoiseIter>>>,
      audioelement::LoudnessAdjustableVolumeOscillatorAlgo<audioelement::LoudnessVolumeAdjust::Yes, double>
      >;
    };

    template<typename T>
    struct Ramps {
      T * keyPressed = nullptr;
      T * envelopeDone = nullptr;
    };

    /*
     Uses a markov graph to create randomized succession of ramps specs.
     **/
    template<
    SoundEngineMode M,
    int nOuts,
    Atomicity A,
    typename Logger,
    typename Mix = typename MixOf<M>::type
    >
    struct SoundEngine {
      using Algo = typename SoundEngineAlgo_<Mix, M>::type;

      using FPT = typename Algo::FPT;
      
      using MonoAudioElt =
      audioelement::VolumeAdjusted<
      audioelement::Enveloped <
      Algo,
      audioelement::AHDSREnvelope<A, FPT, audioelement::EnvelopeRelease::WaitForKeyRelease>
      >
      >;
      
      using StereoAudioElt = audioelement::StereoPanned<MonoAudioElt>;
      
      using audioElt = std::conditional_t<nOuts == 1, MonoAudioElt, StereoAudioElt>;      

      static constexpr auto hasEnvelope = true;
      static constexpr auto baseVolume = audioElt::baseVolume;
      static constexpr auto isMonoHarmonic = audioElt::isMonoHarmonic; // this is an approximation : during a xfade we may have 2 different harmonics
      static constexpr int count_channels = audioElt::count_channels;

      static constexpr auto atomicity = A;
      using oddOnTraits = maybeAtomic<atomicity,unsigned int>;
      using oddOnType = typename oddOnTraits::type;

      static enumTraversal ModeTraversal;

      SoundEngine()
      : xfade_freq(FreqXfade::No)
      , rt_active(false) // TODO we should refactor this : oddOn should have 3 states : SoonKeyPressed, RTActive, Finished
      {
        oddOnTraits::write(oddOn, 0, std::memory_order_relaxed);
      }

      Ramps<audioElt> get_ramps() {
        using namespace imajuscule::audio::audioelement;
        Ramps<audioElt> res;
        // in SoundEngine.playNextSpec (from the rt audio thread),
        // we onKeyPressed() the inactive ramp and onKeyReleased() the active ramp.
        for(auto & r: ramps) {
          auto state = r.getEnvelope().getRelaxedState();
          if(state == EnvelopeState::EnvelopeDone2) {
            if(goOn()) {
              res.envelopeDone = &r;
            }
          }
          else {
            if(state == EnvelopeState::KeyPressed) {
              res.keyPressed = &r;
            }
          }
        }
        return res;
      }

      auto &       editEnvelope()       { return *this; }
      auto const & getEnvelope()  const { return *this; }

      bool tryAcquire() {
        unsigned int cur = getOddOn();
        while(1) {
          if(!isEnvelopeFinished_internal(cur)) {
            return false;
          }
          if(tryIncrementOddOn(cur)) {
            // we took ownership.
            return true;
          }
          // another thread took ownership ...
          if(is_odd(cur)) {
            return false;
          }
          // ... but the other thread released ownership already, so we can retry.
        }
      }

      bool acquireStates() const {
        unsigned int cur = getOddOn();
        return is_odd(cur);
      }

      void forgetPastSignals() {
        remaining_silence_steps = 0;
        start_angle = 0;
      }
      bool isEnvelopeRTActive() const {
        return rt_active && !isEnvelopeFinished();
      }
      bool isEnvelopeSoonKeyPressed() const {
        return !rt_active && !isEnvelopeFinished_internal(getOddOn());
      }
      bool isEnvelopeFinished() const {
        return isEnvelopeFinished_internal(getOddOn());
      }
      void onKeyPressed(int32_t delay) { // TODO use delay
        // we don't increment 'oddOn' here, or 'engine.set_active(true)' : it has been done in 'tryAcquire'.
        Assert(!rt_active);
        rt_active = true;
      }
      bool canHandleExplicitKeyReleaseNow(int32_t delay) const { // TODO use delay
        return goOn();
      }
      void onKeyReleased(int32_t delay) {
        rt_active = false; // TODO we should delay this
        if(auto i = goOn()) {
          stop(i); // TODO we should delay the stop
          for(auto & r: ramps) {
            r.editEnvelope().onKeyReleased(delay);
          }
        }
      }

      void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
        for(auto & r : ramps) {
          r.getOsc().setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
        }
      }

      template<typename T>
      void setGains(T&& gains) {
        for(auto & r : ramps) {
          r.getVolumeAdjustment().getOsc().getOsc().setGains(std::forward<T>(gains));
        }
      }

      void setFiltersOrder(int order) {
        for(auto & r : ramps) {
          r.getVolumeAdjustment().getOsc().getOsc().setFiltersOrder(order);
        }
      }

      void setStartAngle(FPT a) {
        start_angle = a;
      }

      FPT angle() const {
        for (auto const & r : ramps) {
          switch (r.getEnvelope().getRelaxedState()) {
            case EnvelopeState::KeyReleased:
            case EnvelopeState::KeyPressed:
              return r.angle();
            case EnvelopeState::SoonKeyPressed:
            case EnvelopeState::EnvelopeDone1:
            case EnvelopeState::EnvelopeDone2:
              break;
          }
        }
        return {};
      }

      void setAngleIncrements(FPT ai) {
        base_freq = angle_increment_to_freq(ai, sample_rate);
      }
      
      FPT angleIncrements() const {
        for (auto const & r : ramps) {
          switch (r.getEnvelope().getRelaxedState()) {
            case EnvelopeState::KeyReleased:
            case EnvelopeState::KeyPressed:
              return r.angleIncrements();
            case EnvelopeState::SoonKeyPressed:
            case EnvelopeState::EnvelopeDone1:
            case EnvelopeState::EnvelopeDone2:
              break;
          }
        }
        return {};
      }

      void step_algos() {
        for (auto & r : ramps) {
          switch (r.getEnvelope().getRelaxedState()) {
            case EnvelopeState::KeyReleased:
            case EnvelopeState::KeyPressed:
            case EnvelopeState::SoonKeyPressed:
            case EnvelopeState::EnvelopeDone1:
              r.step();
              break;
            case EnvelopeState::EnvelopeDone2:
              break;
          }
        }
      }

      
      template<int C = nOuts>
      std::enable_if_t<C == 1, FPT>
      imag() const {
        FPT v{};
        for (auto & r : ramps) {
          switch (r.getEnvelope().getRelaxedState()) {
            case EnvelopeState::KeyReleased:
            case EnvelopeState::KeyPressed:
              v += r.imag();
              break;
            case EnvelopeState::SoonKeyPressed:
            case EnvelopeState::EnvelopeDone1:
            case EnvelopeState::EnvelopeDone2:
              break;
          }
        }
        return v;
      }
      
      template<int C = nOuts>
      std::enable_if_t<(C > 1), FPT>
      imag(int i) const {
        FPT v{};
        for (auto & r : ramps) {
          switch (r.getEnvelope().getRelaxedState()) {
            case EnvelopeState::KeyReleased:
            case EnvelopeState::KeyPressed:
              v += r.imag(i);
              break;
            case EnvelopeState::SoonKeyPressed:
            case EnvelopeState::EnvelopeDone1:
            case EnvelopeState::EnvelopeDone2:
              break;
          }
        }
        return v;
      }
      
    private:
      // 3 because there is 'before', 'current' and the inactive one
      std::array<audioElt, 3> ramps;

      bool isEnvelopeFinished_internal (unsigned int state) const {
        if(is_odd(state)) {
          return false;
        }
        for(auto & r: ramps) {
          if(r.getEnvelope().isEnvelopeRTActive()) {
            return false;
          }
        }
        return true;
      }

    public:
      auto & getRamps() { return ramps; }
      auto const & getRamps() const { return ramps; }

      void set_sample_rate(int s) {
        sample_rate = s;
        for (auto & r : ramps) {
          r.set_sample_rate(sample_rate);
        }
        for (auto & r : ramp_specs.a) {
          r.get().set_sample_rate(sample_rate);
        }
      }

    private:
      void play(float length, float freq1, float freq2,
                float phase_ratio1, float phase_ratio2,
                float freq_scatter) {
        Assert(sample_rate);
        length *= powf(2.f,
                       std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
        auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
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
            state_factor = std::uniform_real_distribution<float>{1.f / scatter, scatter}(mersenne<SEEDED::Yes>());
          }
          state_freq = freq2;
          freq1 *= state_factor;
          freq2 *= state_factor;

          ramp_spec->get().setup(freq_to_angle_increment(freq1, sample_rate),
                                 freq_to_angle_increment(freq2, sample_rate),
                                 n_frames,
                                 0,
                                 interpolation);
          ramp_spec->silenceFollows(true);
          ramp_spec->setVolume(1.f);
          if(xfade_freq==FreqXfade::No) {
            // don't try to solve frequency discontinuity
            return;
          }
          if(current) {
            // there was a spec before the one we just added...
            auto from_inc = current->get().getAngleIncrementTo();
            auto to_inc = ramp_spec->get().getAngleIncrementFrom();
            auto diff = from_inc - to_inc;
            if(xfade_freq==FreqXfade::All || diff) {
              // ... and the new spec creates a frequency discontinuity
              if(auto * ramp_spec2 = ramp_specs.get_next_ramp_for_build()) {
                // so we move the new spec one step later
                *ramp_spec2 = *ramp_spec;
                // and create a transition
                if(from_inc == to_inc) {
                  from_inc *= 1.00001f; // make sure ramp is non trivial else we cannot detect when it's done
                }
                ramp_spec->get().setup(from_inc, to_inc, freq_xfade, 0, freq_interpolation);
                ramp_spec->silenceFollows(true);
                ramp_spec->setVolume(1.f);
              }
              else {
                // just discard it
                ramp_specs.cancel_last_ramp();
              }
            }
          }
        }
        else {
          Assert(0);
        }
      }

    public:
      auto create_birds() {
        auto mc = std::make_unique<MarkovChain>();

        auto node1 = mc->emplace([](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
        });
        auto node2 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m==MarkovMove::ENTER_NODE) {
            play(length, base_freq*4, base_freq*3, phase_ratio1, phase_ratio2, freq_scatter);
          }
          else {
            play(length, base_freq*2, base_freq*4, phase_ratio1, phase_ratio2, freq_scatter);
          }
        });
        auto node3 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m==MarkovMove::ENTER_NODE) {
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

      auto create_robot() {
        auto mc = std::make_unique<MarkovChain>();

        auto node0 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m == MarkovMove::LEAVE_NODE) {
            auto length = this->length;
            length *= powf(2.f,
                           std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
            auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().setup(freq_to_angle_increment(freq1_robot, sample_rate),
                                     freq_to_angle_increment(freq1_robot, sample_rate),
                                     n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol1);
              ramp_spec->silenceFollows(false);
            }
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().setup(freq_to_angle_increment(freq2_robot, sample_rate),
                                     freq_to_angle_increment(freq2_robot, sample_rate),
                                     n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }
          }
        });
        auto node1 = mc->emplace([](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
        });
        auto node2 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m==MarkovMove::ENTER_NODE) {
            auto length = this->length;
            length *= powf(2.f,
                           std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
            auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().setup(freq_to_angle_increment(freq2_robot, sample_rate),
                                     freq_to_angle_increment(freq2_robot, sample_rate),
                                     n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }

            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              auto inc = freq_to_angle_increment(midi.transpose_frequency(freq2_robot, 2), sample_rate);
              ramp_spec->get().setup(inc, inc, n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              auto inc = freq_to_angle_increment(midi.transpose_frequency(freq2_robot, 4), sample_rate);
              ramp_spec->get().setup(inc, inc, n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }
          }
        });
        auto node3 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m==MarkovMove::ENTER_NODE) {
            constexpr auto slide_length_scale = 2.f;
            auto length = slide_length_scale * this->length;
            length *= powf(2.f,
                           std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
            auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().setup(freq_to_angle_increment(freq2_robot, sample_rate),
                                     freq_to_angle_increment(freq1_robot, sample_rate),
                                     n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(std::min(vol2, vol1));
              ramp_spec->silenceFollows(true);
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
        //                 0.2
        //                  ^
        //                  |
        // 0 --- > 1 > ---- 1---\
        //                  |   |
        //                  v   ^
        //                 0.2  .1
        //                  v   ^
        //                  |   |
        //                  2---/

        def_markov_transition(node0, node1, 1.f);
        def_markov_transition(node1, node2, 0.2f);
        def_markov_transition(node2, node1, 0.1f);
        def_markov_transition(node1, node3, 0.2f);
        def_markov_transition(node3, node1, 1.f);

        return mc;
      }

      auto create_sweep() {
        auto mc = std::make_unique<MarkovChain>();

        auto node0 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(m == MarkovMove::LEAVE_NODE) {
            auto length = this->length;
            length *= powf(2.f,
                           std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
            auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().setup(freq_to_angle_increment(freq1_robot, sample_rate),
                                     freq_to_angle_increment(freq2_robot, sample_rate),
                                     n_frames, phase_ratio1 * n_frames, interpolation);
            }
          }
        });
        auto node1 = mc->emplace([](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
        });

        def_markov_transition(node0, node1, 1.f);

        return mc;
      }

      auto create_wind() {
        auto mc = std::make_unique<MarkovChain>();

        auto node0 = mc->emplace([this](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
          if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
            ramp_spec->get().getUnderlyingIter().set_interpolation(interpolation);
          }
        });
        auto node1 = mc->emplace([](MarkovMove const m, MarkovNode&me, MarkovNode&from_to) {
        });

        def_markov_transition(node0, node1, 1.f);
        def_markov_transition(node1, node0, 1.f);

        return mc;
      }

      void step() {
        orchestrate_algos();
        step_algos();
      }

    private:
      void orchestrate_algos() {
        if (ramp_specs.done()) {
          return;
        }

        // there are some more specs to run

        auto cur = ramp_specs.get_current();

        if(auto const * pressed = get_ramps().keyPressed) {
          int const remaining_steps_to_release = pressed->getEnvelope().countStepsToRelease();
          if (remaining_steps_to_release > 0) {
            return;
          }

          if (remaining_steps_to_release == 0) {
            if (cur && cur->getSilenceFollows()) {
              remaining_silence_steps = articulative_pause_length;
            }
          }
        }

        if (cur && cur->getSilenceFollows()) {
          if (remaining_silence_steps > 0) {
            --remaining_silence_steps;
            return;
          }
        }

        playNextSpec();
      }

      void playNextSpec() {
        auto rampsStatus = get_ramps();
        Assert(!goOn() || rampsStatus.envelopeDone); // might be null if length of ramp is too small ?
        auto new_ramp = rampsStatus.envelopeDone;
        if(!new_ramp) {
          return;
        }
        auto new_spec = ramp_specs.get_next_ramp_for_run();
        if(!new_spec) {
          return;
        }
        new_ramp->getVolumeAdjustment().getOsc().getAlgo().getCtrl() = new_spec->get();
        new_ramp->forgetPastSignals();
        new_ramp->set_sample_rate(sample_rate);
        new_ramp->setStartAngle(start_angle);
        new_ramp->getVolumeAdjustment().setVolumeTarget(new_spec->volume());
        new_ramp->editEnvelope().setAHDSR(AHDSR{xfade_len, itp::LINEAR,0,0, itp::LINEAR,xfade_len,itp::LINEAR,1.}, sample_rate);
        if(!new_ramp->editEnvelope().tryAcquire()) {
          Assert(0);
        }

        int const time_to_release =
          static_cast<int>(.5f + new_ramp->getVolumeAdjustment().getOsc().getAlgo().getCtrl().get_duration_in_samples()) - xfade_len;
        Assert(time_to_release >= 0);

        new_ramp->editEnvelope().onKeyPressed(0);
        new_ramp->editEnvelope().onKeyReleased(std::max(time_to_release, 0));
     }

    public:
      void setEnvelopeCharacTime(int len) {
        xfade_len = len;
      }
      void set_freq_xfade(int xfade_) {
        freq_xfade = xfade_;
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

      unsigned int goOn() const {
        unsigned int i = getOddOn();
        if(is_odd(i)) {
          return i;
        }
        return 0;
      }

      unsigned int getOddOn() const {
        return oddOnTraits::read(oddOn, std::memory_order_acquire);
      }

      void stop(unsigned int i) {
        oddOnTraits::write(oddOn, i+1, std::memory_order_release);
      }

      bool tryIncrementOddOn(unsigned int & cur) {
        return oddOnTraits::compareExchangeStrong(oddOn,
                                                  cur,
                                                  cur+1,
                                                  std::memory_order_acq_rel);
      }

      bool initialize_sweep(float low, float high) {
        bool initialize = true;
        if(!markov) {
          markov = create_sweep();
          if(!markov) {
            return false;
          }
        }

        freq1_robot = low;
        freq2_robot = high;

        return do_initialize(initialize, 0, 0, 1, 0, articulative_pause_length);
      }

      bool initialize_birds(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy, FreqXfade xfade_freq, int articulative_pause_length) {
        this->xfade_freq = xfade_freq;

        bool initialize = (!markov) || (init_policy==SoundEngineInitPolicy::StartAfresh);
        if(!markov) {
          markov = create_birds();
          if(!markov) {
            return false;
          }
        }

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length);
      }

      bool initialize_wind(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy) {
        bool initialize = (!markov) || (init_policy==SoundEngineInitPolicy::StartAfresh);
        if(!markov) {
          markov = create_wind();
          if(!markov) {
            return false;
          }
        }

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length);
      }

      bool initialize_robot(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy, int articulative_pause_length) {
        auto scatter = 1.f + freq_scatter;
        constexpr auto detune = 0.985f;
        freq1_robot = std::uniform_real_distribution<float>{base_freq / scatter, base_freq * scatter}(mersenne<SEEDED::Yes>());
        freq2_robot = std::uniform_real_distribution<float>{freq1_robot*detune, freq1_robot/detune}(mersenne<SEEDED::Yes>());

        vol1 = vol2 = 1.f;

        if(!std::uniform_int_distribution<>{0,1}(mersenne<SEEDED::Yes>())) {
          // f1 is shifted up by d1
          freq1_robot = midi.transpose_frequency(freq1_robot, d1);
          vol1 = std::pow(har_att, d1);
        }
        else {
          // f2 is shifted up by d2
          freq2_robot = midi.transpose_frequency(freq2_robot, d2);
          vol2 = std::pow(har_att, d2);
        }

        auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
        if(n_frames <= 0) {
          Logger::err("length '%f' is too small", length);
          return false;
        }

        bool initialize = (!markov) || (init_policy==SoundEngineInitPolicy::StartAfresh);
        if(!markov) {
          markov = create_robot();
          if(!markov) {
            return false;
          }
        }

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length);
      }

      bool do_initialize(bool initialize, int start_node, int pre_tries, int min_path_length, int additional_tries,
                    int articulative_pause_length)
      {
        auto n_frames = static_cast<float>(ms_to_frames(length, sample_rate));
        if(n_frames <= 0) {
          Logger::err("length '%f' is too small", length);
          return false;
        }

        if(base_freq <= 0.f) {
          Logger::err("frequency '%d' should be sctrictly positive", base_freq);
          return false;
        }

        this->state_freq = 0.f;
        this->state_factor = 0.f;

        ramp_specs.reset();
        this->articulative_pause_length = articulative_pause_length;

        // running the markov chain will populate ramp_specs
        if(initialize) {
          markov->initialize(start_node);

          for(int i=0; i<pre_tries; ++i) {
            markov->step_normalized<ExecuteLambdas::No>(rand_0_1());
          }
        }

        for(int i=0; i<min_path_length; ++i) {
          markov->step_normalized<ExecuteLambdas::Yes>(rand_0_1());
        }

        for(int i=0; i<additional_tries; ++i) {
          markov->step<ExecuteLambdas::Yes>(rand_0_1());
        }

        ramp_specs.finalize();

        return true;
      }

    private:
      Midi midi;
      oddOnType oddOn; // even values mean off, odd values mean on.
      bool rt_active;

      itp::interpolation interpolation : 5;
      itp::interpolation freq_interpolation : 5;

      float d1, d2, har_att, length, base_freq, freq_scatter, phase_ratio1=0.f, phase_ratio2=0.f;
      float min_exp;
      float max_exp;

      float state_freq = 0.f;
      float state_factor = 0.f;

      float freq1_robot, freq2_robot; // used also for sweep
      float vol1 = 1.f, vol2 = 1.f;

      FPT start_angle=0;
      int xfade_len;
      int freq_xfade;
      int articulative_pause_length;

      int sample_rate=0;

      std::unique_ptr<MarkovChain> markov;
      FreqXfade xfade_freq:2;

      int remaining_silence_steps = 0;

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
        struct Ctrl {
          bool getSilenceFollows() const { return silenceAfter; }

          void silenceFollows(bool b) { silenceAfter = b; }

          void setVolume(float vol) {
            Assert(vol > 0.f);
            this->vol = vol;
          }

          float volume() const {
            Assert(vol > 0.f);
            return vol;
          }

          using Type = typename std::tuple_element<0, CTRLS>::type;
          Type & get() { return ctrl; }
        private:
          Type ctrl;
          float vol = 1.f;
          bool silenceAfter = true;
        };

        Ctrl * get_next_ramp_for_build() {
          ++it;
          Assert(it==end);
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
        Ctrl const * get_current() const {
          if(it >= end) {
            return nullptr;
          }
          return &a[it];
        }

        void finalize() {
          Assert(0 == (it+1-end) % iter_cycle_length);
          it=-1;
        }

        Ctrl * get_next_ramp_for_run() {
          Assert(it != end);
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

      static float rand_0_1() { return std::uniform_real_distribution<float>{0.f, 1.f}(mersenne<SEEDED::Yes>()); }
    public:
      auto & getRampsSpecs() { return ramp_specs; }
    };

  } // NS imajuscule::audio
