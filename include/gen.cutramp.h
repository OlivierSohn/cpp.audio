namespace imajuscule::audio::cutramp {

  /*
   * When cut size is bigger than period, period is automatically adjusted
   *  to cut size + 1
   */
  enum ImplParams {

#include "loudness_enum_names.h"

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

#include "pernamespace.implparams.h"

#include "loudness_enum_limits.h"

  template<typename T>
  constexpr auto get_middle(T p, T g) {
    static_assert(std::is_integral<T>::value);
    return ((p+1)-g)/2;
  }

  constexpr auto max_cut_period = 64;
  constexpr auto size_interleaved = 8 * max_cut_period;

  template<
  typename Parameters,
  typename ProcessData,
  typename InterleavedBuffer = std::array<interleaved_buf_t, 2>,
  typename Parent = Impl<ImplParams::NPARAMS, Parameters, InterleavedBuffer, size_interleaved, ProcessData>
  >
  struct ImplBase : public Parent {
    using Params = ImplParams;
    static constexpr auto NPARAMS = static_cast<int>(Params::NPARAMS);
    static constexpr auto n_max_orchestrator_per_channel = 0;
    static constexpr auto min_cut_period = 1;
    static constexpr auto min_ramp_start_freq = 50.f;
    static constexpr auto max_ramp_start_freq = 5000.f;
    static constexpr auto min_ramp_speed = 1.f;
    static constexpr auto max_ramp_speed = 100.f;

    // todo replace w. normalize / denorm
    static constexpr auto ramp_speed_normalize(float v) { return (v-min_ramp_speed) / (max_ramp_speed-min_ramp_speed); }
    static constexpr auto ramp_speed_denormalize(float v) { return min_ramp_speed + v*max_ramp_speed; }
    static constexpr auto ramp_start_freq_normalize(float v) { return (v-min_ramp_start_freq) / (max_ramp_start_freq-min_ramp_start_freq); }
    static constexpr auto ramp_start_freq_denormalize(float v) { return min_ramp_start_freq + v*max_ramp_start_freq; }

    using Parent::params;
    using Parent::half_tone;

  protected:
    using interleaved_buf_t = InterleavedBuffer;

    static constexpr auto size_interleaved_one_cache_line = cache_line_n_bytes / sizeof(typename interleaved_buf_t::value_type);
    static constexpr auto xfade_len = 401;
    static constexpr auto max_length_ramp_ms = 1000.f;

  public:

    static std::vector<ParamSpec> const & getParamSpecs() {
      static std::vector<ParamSpec> params_spec = {
#include "loudness_params_specs.h"
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
  protected:
    bool most_recent_buf_idx:1;
    SmoothedFloat<&step_1> ramp_amount, ramp_speed, ramp_start_freq;

    Smoothed<unsigned, relevantBits(max_cut_period)> period, gap, xfade;
    decltype(period()) p; // because of optimization for g=0, this value may differ from the period value
    decltype(period()) prev_p; // period of the previoud buffer
    bool adjustFreq : 1;
    bool adjustRamp : 1;

    // the first index in "most recent buffer" that contains an out-of-date value
    uint32_t c : relevantBits( size_interleaved - 1 );

    // methods
  public:
    ImplBase() :
    most_recent_buf_idx(0)
    , c(0)
    , prev_p(1)
    {}

    Program const & getProgram(int i) const override {
      auto & progs = getPrograms();
      Assert(i < progs.size());
      return progs[i];
    }

    int countPrograms() const override { return getPrograms().size(); }

    static constexpr int index(ImplParams n) {
      return static_cast<int>(n);
    }

    template<ImplParams N>
    float value() const {
      return params[index(N)];
    }

    template<ImplParams N>
    float denorm() const {
      return denormalize<N>(params[index(N)]);
    }

    float set_gain(float g) {
      params[index(GAIN)] = normalize<GAIN>(g);
      assert(get_gain() == g);
    }

    void set_pan(float pan) const {}

    void set_random(bool b) const {}

    void set_loudness_compensation(float c) {
      params[index(LOUDNESS_COMPENSATION_AMOUNT)] = normalize<LOUDNESS_COMPENSATION_AMOUNT>(c);
    }

    void set_random_pan(bool is_random) const {}

    void set_seed(int seed) {}

  protected:
    int32_t get_xfade_length() { return adjusted(xfade_len); }

    float get_gain() const { return denorm<GAIN>(); }

    template<typename Element>
    bool setupAudioElement(float freq, Element & e)
    {
      if(adjustFreq) {
        auto r = (p - static_cast<float>(gap())) / p;
        freq *= r;
      }
      auto ramp_size = ms_to_frames(max_length_ramp_ms/ramp_speed_denormalize(ramp_speed()));
      if(adjustRamp) {
        ramp_size = adjusted(ramp_size);
      }
      auto start_freq = ramp_start_freq_denormalize(ramp_start_freq());
      e.algo.getOsc().setLoudnessParams(value<LOUDNESS_REF_FREQ_INDEX>(),
                                          value<LOUDNESS_COMPENSATION_AMOUNT>(),
                                          denorm<LOUDNESS_LEVEL>());

      e.algo.getAlgo().getCtrl().set(freq - ramp_amount() * (freq-start_freq),
                                       freq,
                                       ramp_size,
                                       0.f,
                                       static_cast<itp::interpolation>(itp::interpolation_traversal().realValues()[static_cast<int>(.5f + params[Params::RAMP_INTERPOLATION])]));
      return true;
    }

    template<typename Chans, typename MonoNoteChannel, typename CS>
    std::function<bool(Chans&,int)> onStartNote(MonoNoteChannel & c, CS & cs)
    {
      setPhase(c,cs);
      return {};
    }

    template<typename T>
    int adjusted(T val) {
      static_assert(std::is_integral<T>());

      Assert(p > gap());
      return static_cast<int>(.5f + val * p / (p-gap()));
    }
  private:
    static Program::ARRAY make_ramp(float amount, itp::interpolation i, float speed, float start_freq = 50.f,
                                    bool cut_adjust_osc = false, int cut_size = 0, float gain = 2.f) {
      int itp_index = 0;
      auto b = itp::interpolation_traversal().valToRealValueIndex(i, itp_index);
      Assert(b);
      return {{
#include "loudness_init_values.h"
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
  AudioOutPolicy outPolicy,
  int nAudioOut,
  typename Parameters,
  typename EventIterator,
  typename NoteOnEvent,
  typename NoteOffEvent,
  typename ProcessData,

  typename Base = ImplBase<Parameters, ProcessData>,

  typename Parent = ImplCRTP< outPolicy, nAudioOut, XfadePolicy::SkipXfade,
  audioelement::FreqRamp<audioelement::SimpleLinearEnvelope<getAtomicity<outPolicy>(),float>>, true,
  EventIterator, NoteOnEvent, NoteOffEvent, Base>

  >
  struct Impl_ : public Parent {
    static constexpr auto min_cut_period = Base::min_cut_period;
    static constexpr auto max_length_ramp_ms = Base::max_length_ramp_ms;
    static constexpr auto size_interleaved = Base::size_interleaved;
    static constexpr auto size_interleaved_one_cache_line = Base::size_interleaved_one_cache_line;
    static constexpr auto max_queue_size = MaxQueueSize::One;

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
    using Parent::onEvent2;

    using Event = typename Parent::Event;
    using MonoNoteChannel = typename Parent::MonoNoteChannel;
    static constexpr auto n_channels = Parent::n_channels;
    static constexpr auto xfade_policy = Parent::xfade_policy;

    static constexpr auto cut_period_one_cache_line = size_interleaved_one_cache_line / nAudioOut;
    static_assert(cut_period_one_cache_line <= max_cut_period);

  public:
    template <class... Args>
    Impl_(Args&&... args) : Parent (std::forward<Args>(args)...) {}

    template<typename Out, typename Chans>
    onEventResult onEvent(Event const & e, Out & out, Chans & chans)
    {
      static_assert(Out::policy == outPolicy);
      // in case this is called before the first doProcessing:
      if(!period.hasValue()) {
        updateParams();
      }
      return onEvent2(e, out, chans, {});
    }

    void updateParams()
    {
      auto p_ = static_cast<int>(params[CUT_PERIOD] + 0.5f) + min_cut_period;
      auto g_ = static_cast<int>(params[CUT_SIZE] + 0.5f);
      auto x_ = static_cast<int>(params[CUT_XFADE_AMOUNT] + 0.5f);

      Assert(g_ >= 0);
      Assert(p_ >= min_cut_period);
      Assert(x_ >= 0);

      Assert(g_ < max_cut_period);
      Assert(p_ <= max_cut_period);
      Assert(x_ <= max_cut_period);

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
        Assert(gap() < p);
      }
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

      updateParams();

      auto nRemainingFrames = data.numSamples;
      auto middle = get_middle(p,gap());

      static_assert((size_interleaved / nAudioOut) * nAudioOut == size_interleaved);

      auto currentFrame = 0;

      auto * events = data.inputEvents;
      Assert( events );

      EventIterator it(begin(events)), end(end_(events));

      int nextEventPosition = getNextEventPosition(it, end);
      Assert(nextEventPosition >= currentFrame);

      while(nRemainingFrames) {
        Assert(nRemainingFrames > 0);

        if(c == p) {
          c = 0;
          prev_p = p;
          most_recent_buf_idx = !most_recent_buf_idx;

          // we are at the end of the buffer, where xfade ratios are at .5f,
          // so it's a good location to change xfade amount (for .5f it doesn't change anything)
          // and a good location to change the period (because of the end of the buffer).
          onEndBufferStepParamChanges();
          compute_state();
          Assert(gap() < p);
          middle = get_middle(p,gap());
        }

        // keep this loop after onEndBufferStepParamChanges()/compute_state(),
        // so that new notes have the correct adjusted frequency
        while(nextEventPosition == currentFrame) {
          Event e;
          it.dereference(e);
          this->onEvent(e, out, chans);
          ++it;
          nextEventPosition = getNextEventPosition(it, end);
        }
        Assert(nextEventPosition > currentFrame);

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

        out.step(&interleaved[most_recent_buf_idx][nAudioOut*c], nFramesToProcess, 0);

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
          Assert(c < last);
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
              Assert(b >= 0);
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
            Assert(c < last);
          }
          nRemainingFrames -= nRealFramesWritten;
          currentFrame += nRealFramesWritten;
        }
      }

      Assert(nextEventPosition == event_position_infinite); // the events should all have already been processed
    }

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
        Assert(p > 0);
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

      static_assert(outPolicy == AudioOutPolicy::Slave ||
                    outPolicy == AudioOutPolicy::MasterGlobalLock);
      for(auto it = channels.seconds(), end = channels.seconds_end(); it!= end; ++it) {
        auto & c = *it;
        auto & osc = c.elem;
        if(osc.isInactive()) {
          continue;
        }
        auto const & tunedNote = channels.corresponding(c);
        auto freq = to_freq(tunedNote.getValue()-Do_midi, half_tone);

        if(adjustFreq) {
          auto r = (p - static_cast<float>(gap())) / p;
          freq *= r;
        }
        auto & ctrl = osc.algo.getAlgo().getCtrl();
        if(!force) {
          // maybe from/to is swapped so we need to test with both ends
          if(std::abs(freq - ctrl.getTo()) < 0.0001f) {
            continue;
          }
          if(std::abs(freq - ctrl.getFrom()) < 0.0001f) {
            continue;
          }
        }
        auto ramp_size = ms_to_frames(max_length_ramp_ms/ramp_speed_denormalize(ramp_speed()));
        if(adjustRamp) {
          ramp_size = adjusted(ramp_size);
        }
        auto start_freq = ramp_start_freq_denormalize(ramp_start_freq());
        ctrl.set(freq - ramp_amount() * (freq-start_freq),
                 freq,
                 ramp_size,
                 -1.f,
                 interp);
      }
    }
  };

} // NS imajuscule::audio::cutramp
