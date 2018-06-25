
#ifndef NDEBUG
#define DO_LOG_MIDI 1
#else
#define DO_LOG_MIDI 0
#endif

#if DO_LOG_MIDI
#define MIDI_LG( x , y, ...) LG( x , y ,##__VA_ARGS__)
#else
#define MIDI_LG(...)
#endif

namespace imajuscule::audio {
  using interleaved_buf_t = a64::vector<float>;

  template<typename T>
  struct ForEach;

  template<>
  struct ForEach<interleaved_buf_t> {
    template<typename F>
    static void run(interleaved_buf_t & b, F f) {
      f(b);
    };
  };

  template<int N>
  struct ForEach<std::array<interleaved_buf_t, N>> {
    template<typename F>
    static void run(std::array<interleaved_buf_t, N> & a, F f) {
      for(auto & b : a) {
        f(b);
      }
    };
  };

  template<typename InterleavedBuffer, typename F>
  void forEach(InterleavedBuffer & b, F f) {
    ForEach<InterleavedBuffer>::run(b, f);
  }

  template<

  int NParams,
  typename Parameters,
  typename InterleavedBuffer,
  int SizeInterleaved,
  typename ProcessData_

  >
  struct Impl {
    using ProcessData = ProcessData_;

    static constexpr auto tune_stretch = 1.f;

    virtual Program const & getProgram(int i) const = 0;
    virtual int countPrograms() const = 0;

    virtual ~Impl() = default;

    static constexpr auto NPARAMS = NParams;
    static constexpr auto size_interleaved = SizeInterleaved;

    void initializeSlow() {
      forEach(interleaved, [](interleaved_buf_t & v) {
        v.resize(size_interleaved, 0.f);
      });
      params.resize(NPARAMS);
    }

    void initializeSteal(Impl && o) {
      interleaved = std::move(o.interleaved);
      forEach(interleaved, [](interleaved_buf_t & v) {
        std::fill(v.begin(), v.end(), 0.f);
        Assert(v.size() == size_interleaved);
      });
      params = std::move(o.params);
      Assert(params.size() == NPARAMS);
    }

    void setParameter(int index, float value, int sampleOffset = 0 /* not supported yet */) {
      Assert(index < params.size());
      params[index] = value;
    }

    void useProgram(int index) {
      auto const & p = getProgram(index);
      MIDI_LG(INFO, "with program %d of %d", index, countPrograms());
      Assert(p.params.size() == NPARAMS);
      for (auto i = 0; i < NPARAMS; i++) {
        params[i] = p.params[i];
      }
    }

  protected:
    Parameters params;
    InterleavedBuffer interleaved;
    float half_tone = compute_half_tone(tune_stretch);

  };

  // When another oscillator with the same frequency exists,
  // we synchronize the phase to avoid cancellations.
  //
  // Else, we randomize the phase to keep some nice randomness.
  struct Phase {
    Phase(bool deterministic, float value) : deterministic(deterministic), value(value) {}
    bool isDeterministic() const { return deterministic; }
    float getDeterministicValue() const { Assert(deterministic); return value; }
  private:
    bool deterministic;
    float value;
  };
  static inline Phase mkDeterministicPhase(float v) { return Phase{true,v}; }
  static inline Phase mkNonDeterministicPhase() { return Phase{false,{}}; }

  template<typename T>
  void setAlgoPhase (Phase const & phase, T&algo) {
    auto angle =
    phase.isDeterministic() ?
    phase.getDeterministicValue() :
    std::uniform_real_distribution<float>{-1.f, 1.f}(mersenne<SEEDED::Yes>());
    algo.getOsc().setAngle(angle);
  }

  template<
  AudioOutPolicy outPolicy,
  int nOuts,
  XfadePolicy xfade_policy_,
  typename AE,
  bool handle_note_off,
  typename EventIterator,
  typename NoteOnEvent,
  typename NoteOffEvent,
  typename Base,
  int n_max_voices = 8

  >
  struct ImplCRTP : public Base {

    static constexpr auto nAudioOut = nOuts;
    static constexpr auto xfade_policy = xfade_policy_;
    static constexpr Atomicity atomicity = getAtomicity<outPolicy>();

    using Element = AE;
    using MonoNoteChannel = MonoNoteChannel<Element, Channel<atomicity, nOuts, xfade_policy_, MaxQueueSize::One>>;

    using Base::get_xfade_length;
    using Base::get_gain;
    using Base::onStartNote;
    using Base::setupAudioElement;

    using Event = typename EventIterator::object;

    static constexpr auto n_channels_per_note = 1;

    // notes played in rapid succession can have a common audio interval during xfades
    // even if their noteOn / noteOff intervals are disjoint.
    // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well'.
    // TODO if the release time is long, and notes are consecutive and short, this number
    // should be increased.
    static constexpr auto n_max_simultaneous_notes_per_voice = 2;
    static constexpr auto n_channels = n_channels_per_note * n_max_voices * n_max_simultaneous_notes_per_voice;

  protected:
    // members
    float half_tone = compute_half_tone(tune_stretch); // TODO remove duplicates, use constexpr version : https://github.com/elbeno/constexpr/blob/master/src/include/cx_math.h
    LocalPairArray<TunedPitch,MonoNoteChannel, n_channels> channels;

    static constexpr auto tune_stretch = 1.f;

  public:

    template <typename A>
    ImplCRTP(std::array<A, n_channels> & as) :
    channels(TunedPitch{}, as)
    {}

    template<typename ChannelsT>
    bool initialize(ChannelsT & chans) {
      for(auto & c : seconds(channels)) {
        // using WithLock::No : since we own all these channels and they are not playing, we don't need
        // to take the audio lock.
        if(!c.template open<WithLock::No>(chans, 0.f)) {
          return false;
        }
      }
      return true;
    }

    ~ImplCRTP() {
      finalize();
    }

    void finalize() {
      for(auto & c : seconds(channels)) {
        if(c.channel == nullptr) {
          continue;
        }
        c.reset();
        c.channel = nullptr;
      }
    }

    // returns true if at least one channel has an active enveloppe.
    bool areEnvelopesFinished() const {
      for(auto const & c : seconds(channels)) {
        if(!c.elem.isEnvelopeFinished()) {
          return false;
        }
      }
      return true;
    }

    // Note: Logic Audio Express 9 calls this when two projects are opened and
    // the second project starts playing, so we are not in an "initialized" state.
    template<typename Chans>
    void allNotesOff(Chans & chans) {
      MIDI_LG(INFO, "all notes off");
      chans.enqueueOneShot([this](auto &){
        for(auto & c : seconds(channels)) {
          c.elem.onKeyReleased();
        }
      });
    }

    template<typename Chans>
    void allSoundsOff(Chans & chans) {
      MIDI_LG(INFO, "all sounds off");
      chans.enqueueOneShot([this](auto &){
        for(auto & c : seconds(channels)) {
          c.elem.onKeyReleased();
        }
      });
    }

    template <typename F>
    void forEachElems(F f) {
      for(auto & c : seconds(channels)) {
        f(c.elem);
      }
    }

    template<typename Out, typename Chans>
    onEventResult onEvent2(Event const & e, Out & out, Chans & chans)
    {
      using Request = typename Chans::Request;
      static_assert(Out::policy == outPolicy);
      if(e.type == Event::kNoteOnEvent)
      {
        Assert(e.noteOn.velocity > 0.f ); // this case is handled by the wrapper... else we need to do a noteOff
        MIDI_LG(INFO, "on  %d", e.noteOn.pitch);
        MonoNoteChannel * channel = nullptr;

        // 1. [with maybe-lock]
        //      reserve a channel by maybe-CAS the envelope State Finished2 -> SoonKeyPressed

        {
          typename Out::LockFromNRT L(out.get_lock());

          for(auto & c : seconds(channels)) {
            if(!c.elem.tryAcquire()) {
              continue;
            }
            Assert(!c.elem.isEnvelopeFinished()); // because we just acquired it.
            channel = &c;
            break;
          }
        }

        if(!channel) {
          // note that we could sleep and retry or yield and retry.
          return onDroppedNote(e.noteOn.pitch);
        }
        auto & c = *channel;

        // 2. [without maybe-lock]
        //      set the tunedpitch
        //      setup the channel (dynamic allocations allowed for soundengine)

        TunedPitch tp{e.noteOn.pitch, e.noteOn.tuning};
        channels.corresponding(*channel) = tp;

        c.elem.setEnvelopeCharacTime(get_xfade_length());

        auto freq = to_freq(tp.getValue()-Do_midi, half_tone);
        // setupAudioElement is allowed to be slow, allocate / deallocate memory, etc...
        // because it's not running in the audio realtime thread.
        if(!setupAudioElement(freq, c.elem)) {
          MIDI_LG(ERR,"setupAudioElement failed");
          return onDroppedNote(e.noteOn.pitch);
        }

        Assert(!c.elem.isEnvelopeFinished());

        // 3. [with maybe-lock]
        //      register the maybe-oneshot that does
        //        - maybe phase cancellation avoidance (in onStartNote)
        //        - either
        //          - register the compute and trigger onKeyPress, or
        //          - register the orchestrator (in onStartNote)

        {
          typename Out::LockFromNRT L(out.get_lock());

          chans.enqueueOneShot([this, &c, velocity = e.noteOn.velocity](Chans & chans){
            // unqueue the (potential) previous request, else an assert fails
            // when we enqueue the next request, because it's already queued.
            c.reset();
            if constexpr (xfade_policy == XfadePolicy::UseXfade) {
              c.channel->set_xfade(get_xfade_length());
            }
            c.channel->setVolume(get_gain());

            if constexpr (std::remove_reference_t<decltype(c.elem)>::computable) {
              // The caller is responsible for growing the channel request queue if needed
              if(!chans.playComputableNoLock(*c.channel,
                                             c.elem.fCompute(),
                                             Request{
                                               &c.elem.buffer->buffer[0],
                                               velocity,
                                               // e.noteOn.length is always 0, we must rely on noteOff
                                               std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
                                             }))
              {
                return;
              }
            }
            if(auto orchestrator = this -> template onStartNote<Chans>(c,channels)) {
              chans.add_orchestrator(orchestrator);
            }
          });
        }
      }
      else if(e.type == Event::kNoteOffEvent)
      {
        if(!handle_note_off) { // TODO remove handle_note_off, redundant with autorelease notion?
          MIDI_LG(INFO, "off (ignored) %d", e.noteOff.pitch);
          // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
          // but close method sets the channel to -1 so it's impossible to fade it to zero
          // afterwards using close(), so instead we don't do anything here
          return onEventResult::OK;
        }
        MIDI_LG(INFO, "off %d", e.noteOff.pitch);
        TunedPitch tp{e.noteOff.pitch, e.noteOff.tuning};
        {
          typename Out::LockFromNRT L(out.get_lock());
          static_assert(sizeof(decltype(tp))<=sizeof(void*)); // ensure that the std::function won't dynamically allocate / deallocate
          chans.enqueueOneShot([this, tp](auto &){
            // We can have multiple notes with the same pitch, and different durations.
            // Hence, here we just close the first opened channel with matching pitch.
            for(auto &tunedPitch : firsts(channels)) {
              if (tunedPitch != tp) {
                continue;
              }
              auto & c = channels.corresponding(tunedPitch);
              if(!c.elem.canHandleExplicitKeyReleaseNow()) {
                continue;
              }
              c.elem.onKeyReleased();
              return;
            }
            // The corresponding noteOn was skipped,
            // because too many notes were being played at the same time
          });
        }
      }
      else {
        return onEventResult::UNHANDLED;
      }
      return onEventResult::OK;
    }

  protected:
    onEventResult onDroppedNote(uint8_t pitch) {
      MIDI_LG(WARN, "dropped note '%d'", pitch);
      return onEventResult::DROPPED_NOTE;
    }
  };

  template<typename T>
  struct Wrapper {
    static constexpr auto n_channels = T::n_channels;
    static constexpr auto with_lock = imajuscule::WithLock::No;

    using ProcessData = typename T::ProcessData;

    using OutputData = outputDataBase<
    Channels<T::nAudioOut, T::xfade_policy, T::max_queue_size, AudioOutPolicy::Slave>
    >;

    template <class... Args>
    Wrapper(int nOrchestratorsMax, Args&&... args) :
    plugin(std::forward<Args>(args)...),
    out{
      GlobalAudioLock<AudioOutPolicy::Slave>::get(),
      n_channels,
      nOrchestratorsMax
    }
    {
      dontUseConvolutionReverbs(out);
      plugin.template initialize(out.getChannels());
    }

    void doProcessing (ProcessData& data) {
      return plugin.doProcessing(data, out, out.getChannels());
    }

    void allNotesOff() {
      return plugin.template allNotesOff(out.getChannels());
    }

    void allSoundsOff() {
      return plugin.template allSoundsOff(out.getChannels());
    }

    OutputData out;
    T plugin;
  };

  template<typename MonoNoteChannel, typename CS>
  void setPhase(MonoNoteChannel & c, CS & cs)
  {
    auto & tp = cs.corresponding(c);
    auto phase = mkNonDeterministicPhase();
    auto & thisAlgo = c.elem.algo;
    for(auto &p : firsts(cs)) {
      if((&p == &tp) || (p != tp)) {
        // same channel, or different frequency.
        continue;
      }
      // We found a matching TunedPitch
      auto & otherChannel = cs.corresponding(p);
      Assert(&otherChannel != &c);
      // To prevent phase cancellation, the phase of the new note will be
      // coherent with the phase of any active channel that plays a note at the same frequency.
      if(otherChannel.elem.isEnvelopeFinished()) {
        continue;
      }
      auto & otherAlgo = otherChannel.elem.algo;
      phase = mkDeterministicPhase(otherAlgo.angle());
      break;
    }
    setAlgoPhase(phase, thisAlgo);
  }

} // NS imajuscule::audio
