
namespace imajuscule::audio {
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

    // to optimize things, should this class work in increments instead of frequencies?
    template<typename ITER>
    struct SoundEngineFreqCtrl {
      using FPT = typename ITER::FPT;
      using T = FPT;

      void setFreqRange(range<float> const & r) {
        this->fmax = r.getMax();
        Assert(this->fmax);
        invApproxRange = 1.f / (2.f * ctrl.getAbsMean());
        x = std::log(r.getMin()) / std::log(fmax);
        Assert(x > 0.f);
        Assert(x <= 1.f);
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

      T step() {
        Assert(fmax && x);
        ctrl.step();
        auto v = ctrl.imag() * invApproxRange;
        Assert(v >= 0.f); // else, use AbsIter
        auto exponent = x + (1.f-x) * v;
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

      auto & getUnderlyingIter() { return ctrl.getUnderlyingIter(); }

    private:
      float invApproxRange;
      float fmax = 0.f, x = 0.f;
      audioelement::Ctrl<ITER> ctrl;
    };

    template<typename CTRL, typename NOISE_ITER>
    struct ShortTermNoiseAdderCtrl {
      using T = typename CTRL::FPT;
      using FPT = T;
      using Tr = NumTraits<T>;

      void setFreqRange(range<float> const & r) {
        ctrl.setFreqRange(r);
      }

      void forgetPastSignals() {
        ctrl.forgetPastSignals();
        noise.initializeForRun();
      }

      void set_short_term_noise_amplitude(float f) {
        noise_amplitude = f;
      }

      void set_short_term_noise_rate(float f) {
        ratio = 20000.f * f;
      }

      T step() {
        auto long_term_freq = ctrl.step();
        Assert(long_term_freq > 0.f);
        Assert(ratio >= 0.f);
        // keep short term noise rate inv. proportional to long term frequency
        noise.set_n_slow_steps(1 + ratio / long_term_freq);
        ++noise;
        return long_term_freq * std::pow(Tr::two(), *noise * noise_amplitude);
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
      void set_by_increments(T from_,
                             T to_,
                             T duration_in_samples_,
                             T start_sample,
                             itp::interpolation i) {
        Assert(0);
      }
      float getFrom() const {
        Assert(0);
        return 0.f;
      }
      float getTo() const {
        Assert(0);
        return 0.f;
      }

    private:
      T noise_amplitude;
      // we want approx. 1000 for 10000Hx
      T ratio = .1f;
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
      audioelement::AdjustableVolumeOscillatorAlgo<audioelement::VolumeAdjust::Yes, float>
      >;
    };

    template<> struct MixOf<SoundEngineMode::WIND> {
      using type = audioelement::Mix
      <
      audioelement::LowPassAlgo<PinkNoiseAlgo, Order>,
      AsymBandPassAlgo<PinkNoiseAlgo, Order, audioelement::SlowIter<audioelement::AbsIter<PinkNoiseIter>>>,
      AsymBandRejectAlgo<PinkNoiseAlgo, Order, audioelement::SlowIter<audioelement::AbsIter<PinkNoiseIter>>>,
      audioelement::AdjustableVolumeOscillatorAlgo<audioelement::VolumeAdjust::Yes, float>
      >;
    };

    template<typename T>
    struct Ramps {
      T * keyPressed = nullptr;
      T * envelopeDone = nullptr;
    };

    template<
    SoundEngineMode M,
    int nOuts,
    Atomicity A,
    typename Logger,
    typename Mix = typename MixOf<M>::type
    >
    struct SoundEngine {

      using Algo = typename SoundEngineAlgo_<Mix, M>::type;
      using Request = Request<A, nOuts>;

      using audioElt = audioelement::FinalAudioElement< audioelement::SimplyEnveloped < A, Algo > >;
      using Chan = Channel<A, nOuts, XfadePolicy::UseXfade, MaxQueueSize::One>;

      static constexpr auto atomicity = A;
      using oddOnTraits = maybeAtomic<atomicity,unsigned int>;
      using oddOnType = typename oddOnTraits::type;

      static enumTraversal ModeTraversal;

    public:

      template<typename F>
      SoundEngine(F f) :
      get_ramps(std::move(f)),
      xfade_freq(FreqXfade::No)
      {
        getSilence(); // make sure potential dynamic allocation occurs not under audio lock
        oddOnTraits::write(oddOn, 0, std::memory_order_relaxed);
      }

      void play(float length, float freq1, float freq2,
                float phase_ratio1, float phase_ratio2,
                float freq_scatter) {
        length *= powf(2.f,
                       std::uniform_real_distribution<float>{min_exp, max_exp}(mersenne<SEEDED::Yes>()));
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
            state_factor = std::uniform_real_distribution<float>{1.f / scatter, scatter}(mersenne<SEEDED::Yes>());
          }
          state_freq = freq2;
          freq1 *= state_factor;
          freq2 *= state_factor;

          ramp_spec->get().set(freq1, freq2, n_frames, 0, interpolation);
          ramp_spec->silenceFollows(true);
          ramp_spec->setVolume(1.f);
          if(xfade_freq==FreqXfade::No) {
            // don't try to solve frequency discontinuity
            return;
          }
          if(current) {
            // there was a spec before the one we just added...
            auto from_inc = current->get().getTo();
            auto to_inc = ramp_spec->get().getFrom();
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
                ramp_spec->get().set(from_inc, to_inc, freq_xfade, 0, freq_interpolation);
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
            auto n_frames = static_cast<float>(ms_to_frames(length));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().set(freq1_robot, freq1_robot, n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol1);
              ramp_spec->silenceFollows(false);
            }
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().set(freq2_robot, freq2_robot, n_frames, phase_ratio1 * n_frames, interpolation);
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
            auto n_frames = static_cast<float>(ms_to_frames(length));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().set(freq2_robot, freq2_robot, n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }
            auto ht = compute_half_tone(1.f);
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              auto f = transpose_frequency(freq2_robot, ht, 2);
              ramp_spec->get().set(f, f, n_frames, phase_ratio1 * n_frames, interpolation);
              ramp_spec->setVolume(vol2);
              ramp_spec->silenceFollows(true);
            }
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              auto f = transpose_frequency(freq2_robot, ht, 4);
              ramp_spec->get().set(f, f, n_frames, phase_ratio1 * n_frames, interpolation);
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
            auto n_frames = static_cast<float>(ms_to_frames(length));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().set(freq2_robot, freq1_robot, n_frames, phase_ratio1 * n_frames, interpolation);
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
            auto n_frames = static_cast<float>(ms_to_frames(length));
            if(auto * ramp_spec = ramp_specs.get_next_ramp_for_build()) {
              ramp_spec->get().set(freq1_robot, freq2_robot, n_frames, phase_ratio1 * n_frames, interpolation);
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

      // returns false if channel needs to be closed
      template<typename Chans>
      bool orchestrate(Chans & chans, int max_frame_compute, Chan & channel) {
        if(ramp_specs.done()) {
          return false;
        }

        if(channel.isPlaying() || !channel.get_requests().empty()) {
          return true;
        }

        // is current request soon xfading?
        // TODO today, we rely on channel xfades for xfades. But we should have channels
        // with no xfade, and just use enveloppes (when one enveloppe starts releasing,
        // the other starts raising : hence we need 2 channels instead of one.).
        if(channel.get_remaining_samples_count() >= max_frame_compute + 1 + channel.get_size_half_xfade()) {
          return true;
        }
        if(state_silence) {
          state_silence = false;
        }
        else {
          bool const has_silence = articulative_pause_length > 2*channel.get_size_xfade();
          if(has_silence) {
            if(auto cur = ramp_specs.get_current()) {
              if(cur->silenceFollows()) {
                state_silence = true;
                playSilence(channel);
                return true;
              }
            }
          }
        }
        return playNextSpec(chans, channel);
      }

      template<typename Chans>
      bool playNextSpec(Chans & chans, Chan & channel) {
        while(true) {
          auto rampsStatus = get_ramps();
          if(auto * prevRamp = rampsStatus.keyPressed) {
            prevRamp->algo.editEnvelope().onKeyReleased();
          }
          auto new_ramp = rampsStatus.envelopeDone;
          if(!new_ramp) {
            Assert(rampsStatus.envelopeDone); // might be null if length of ramp is too small ?
            return true;
          }
          auto new_spec = ramp_specs.get_next_ramp_for_run();
          if(!new_spec) {
            break;
          }
          new_ramp->algo.getAlgo().getCtrl() = new_spec->get();
          new_ramp->algo.forgetPastSignals();
          new_ramp->algo.editEnvelope().setEnvelopeCharacTime(xfade_len);
          if(!new_ramp->algo.editEnvelope().tryAcquire()) {
            Assert(0);
          }
          new_ramp->algo.editEnvelope().onKeyPressed();

          auto v = MakeVolume::run<nOuts>(1.f, pan) * (new_spec->volume()/Request::chan_base_amplitude);
          // note that by design (see code of caller), the channel request queue is empty at this point
          // no lock : the caller is responsible for taking the out lock
          if(chans.playComputableNoLock(channel, new_ramp->fCompute(),
                                        Request{
                                          &new_ramp->buffer->buffer[0],
                                          v,
                                          // TODO this should probably be reworked, as now, the enveloppe is responsible for fading out.
                                          static_cast<int>(.5f + new_ramp->algo.getAlgo().getCtrl().get_duration_in_samples())
                                        }))
          {
            return true;
          }
        }
        // release all keys

        while(auto * prevRamp = get_ramps().keyPressed) {
          prevRamp->algo.editEnvelope().onKeyReleased();
        }
        return false;
      }

      void playSilence(Chan & channel) {
        Assert(articulative_pause_length > 2*channel.get_size_xfade());

        if(auto * prevRamp = get_ramps().keyPressed) {
          prevRamp->algo.editEnvelope().onKeyReleased();
        }
        // note that by design (see code of caller), the channel request queue is empty at this point
        // no lock : the caller is responsible for taking the out lock
        bool res = channel.addRequest({
          &getSilence(),
          // to propagate the volume of previous spec to the next spec
          channel.get_current().volumes * (1.f/Request::chan_base_amplitude),
          articulative_pause_length
        });

        Assert(res); // because length was checked
      }

      void setEnvelopeCharacTime(int len) {
        xfade_len = len;
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

      bool initialize_sweep(float low, float high, float pan) {
        bool initialize = true;
        if(!markov) {
          markov = create_sweep();
          if(!markov) {
            return false;
          }
        }

        freq1_robot = low;
        freq2_robot = high;

        return do_initialize(initialize, 0, 0, 1, 0, articulative_pause_length, pan);
      }

      bool initialize_birds(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy, FreqXfade xfade_freq, int articulative_pause_length, float pan) {
        this->xfade_freq = xfade_freq;

        bool initialize = (!markov) || (init_policy==SoundEngineInitPolicy::StartAfresh);
        if(!markov) {
          markov = create_birds();
          if(!markov) {
            return false;
          }
        }

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, pan);
      }

      bool initialize_wind(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy, float pan) {
        bool initialize = (!markov) || (init_policy==SoundEngineInitPolicy::StartAfresh);
        if(!markov) {
          markov = create_wind();
          if(!markov) {
            return false;
          }
        }

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, pan);
      }

      bool initialize_robot(int start_node, int pre_tries, int min_path_length, int additional_tries,
        SoundEngineInitPolicy init_policy, int articulative_pause_length, float pan) {
        auto scatter = 1.f + freq_scatter;
        constexpr auto detune = 0.985f;
        freq1_robot = std::uniform_real_distribution<float>{base_freq / scatter, base_freq * scatter}(mersenne<SEEDED::Yes>());
        freq2_robot = std::uniform_real_distribution<float>{freq1_robot*detune, freq1_robot/detune}(mersenne<SEEDED::Yes>());

        vol1 = vol2 = 1.f;

        auto ht = compute_half_tone(1.f);

        if(!std::uniform_int_distribution<>{0,1}(mersenne<SEEDED::Yes>())) {
          // f1 is shifted up by d1
          freq1_robot = transpose_frequency(freq1_robot, ht, d1);
          vol1 = pow(har_att, d1);
        }
        else {
          // f2 is shifted up by d2
          freq2_robot = transpose_frequency(freq2_robot, ht, d2);
          vol2 = pow(har_att, d2);
        }

        auto n_frames = static_cast<float>(ms_to_frames(length));
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

        return do_initialize(initialize, start_node, pre_tries, min_path_length, additional_tries, articulative_pause_length, pan);
      }

      bool do_initialize(bool initialize, int start_node, int pre_tries, int min_path_length, int additional_tries,
                    int articulative_pause_length, float pan)
      {
        auto n_frames = static_cast<float>(ms_to_frames(length));
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
        state_silence = true;

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

        this->pan = pan;
        return true;
      }

      template<typename Chans>
      std::function<bool(Chans&,int)> getOrchestrator(Chan & channel)
      {
        return [this, &channel](Chans & chans, int max_frame_compute){
          auto i = goOn();
          if(0==i) {
            return false;
          }

          auto res = orchestrate(chans, max_frame_compute, channel);
          if(!res) {
            tryIncrementOddOn(i);
          }
          return res;
        };
      }

    private:
      oddOnType oddOn; // even values mean off, odd values mean on.

      itp::interpolation interpolation : 5;
      itp::interpolation freq_interpolation : 5;
      std::function< Ramps<audioElt>() > get_ramps;
      float d1, d2, har_att, length, base_freq, freq_scatter, phase_ratio1=0.f, phase_ratio2=0.f;
      float min_exp;
      float max_exp;

      float state_freq = 0.f;
      float state_factor = 0.f;

      float freq1_robot, freq2_robot; // used also for sweep
      float vol1 = 1.f, vol2 = 1.f;
      float pan;

      int xfade_len;
      int freq_xfade;
      int articulative_pause_length;

      std::unique_ptr<MarkovChain> markov;
      FreqXfade xfade_freq:2;

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
        struct Ctrl {
          bool silenceFollows() const { return silenceAfter; }

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

      float rand_0_1() const { return std::uniform_real_distribution<float>{0.f, 1.f}(mersenne<SEEDED::Yes>());}
    public:
      auto & getRamps() { return ramp_specs; }
    };

  } // NS imajuscule::audio
