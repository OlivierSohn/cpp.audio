
#ifdef IMJ_LOG_MIDI
#  define MIDI_LG( x , y, ...) LG( x , y ,##__VA_ARGS__)
#else
#  define MIDI_LG(...)
#endif

namespace imajuscule::audio {

// Whether to synchronize the start phase with the currently playing oscillators of same frequency,
// to avoid phase cancellation (used mainly for constant frequency oscillators)
enum class SynchronizePhase {
  Yes,
  No
};
// If the phase has not been synchronized with another oscillator, defines the phase initialization
enum class DefaultStartPhase {
  Zero,
  Random
};

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


// key = source, value = maybe delay
std::unordered_map<uint64_t, std::optional<uint64_t>> & midiDelays();

uint64_t & maxMIDIJitter();

template<SynchronizePhase Sync, DefaultStartPhase Phase, typename ElementMidi, typename CS>
void setPhase(ElementMidi & e, CS & cs)
{
  auto & thisAlgo = e.elem;
  if constexpr (Sync == SynchronizePhase::Yes) {
    auto & tp = cs.corresponding(e);
    for(auto &p : firsts(cs)) {
      if((&p == &tp) || (p != tp)) {
        // same channel, or different frequency.
        continue;
      }
      auto & otherChannel = cs.corresponding(p);
      Assert(&otherChannel != &e);
      // To prevent phase cancellation, the phase of the new note will be
      // coherent with the phase of any active channel that plays a note at the same frequency.
      if(!otherChannel.elem.getEnvelope().isEnvelopeRTActive()) {
        continue;
      }
      thisAlgo.getOsc().synchronizeAngles(otherChannel.elem.getOsc());
      return;
    }
  }
  if constexpr (Phase == DefaultStartPhase::Zero) {
    thisAlgo.getOsc().setAngle(0);
  } else if constexpr (Phase == DefaultStartPhase::Random) {
    thisAlgo.getOsc().setAngle(std::uniform_real_distribution<float>{-1.f, 1.f}(mersenne<SEEDED::Yes>()));
  } else {
    Assert(0);
  }
}

struct NoopElementInitializer {
  template<typename Element>
  void operator()(Element const &) {
  }
};

enum class SynthState {
  ComputeNotRegistered,
  ComputeRegistered,
  WaitingForComputeUnregistration
};

template<
AudioOutPolicy outPolicy,
int nOuts,
XfadePolicy xfade_policy_,
typename AE,
SynchronizePhase Sync,
DefaultStartPhase Phase,
bool handle_note_off,
typename EventIterator,
typename Base,
int n_max_voices = 8,
typename ElementInitializer = NoopElementInitializer
>
struct ImplCRTP : public Base {

  static constexpr auto nAudioOut = nOuts;
  static constexpr auto xfade_policy = xfade_policy_;
  static constexpr Atomicity atomicity = getAtomicity<outPolicy>();

  using Element = AE;
  struct ElemMidiDelay {
    Element elem;
    Optional<MIDITimestampAndSource> midiDelay;
  };

  using Base::get_xfade_length;
  using Base::get_gain;
  using Base::setupAudioElement;

  // notes played in rapid succession can have a common audio interval during xfades
  // even if their noteOn / noteOff intervals are disjoint.
  // n_max_simultaneous_notes_per_voice controls the possibility to support that 'well'.
  // TODO if the release time is long, and notes are consecutive and short, this number
  // should be increased.
  static constexpr auto n_max_simultaneous_notes_per_voice = 2;
  // assumes a single channel is used per note
  static constexpr auto n_channels = n_max_voices * n_max_simultaneous_notes_per_voice;

protected:

  // We use this datastructure because we generally iterate on the indexes,
  // and once we find out match, we go to the corresponding channel. But instead we could simply use:
  //std::array<std::optional<NoteId>, n_channels> index_to_noteid;
  //std::array<ElemMidiDelay, n_channels> channels;

  LocalPairArray<std::optional<NoteId>, ElemMidiDelay, n_channels> channels;

private:
  std::atomic<SynthState> state = SynthState::ComputeNotRegistered;
  std::optional<ElementInitializer> synchronous_element_initializer;
  std::atomic_int count_acquire_race_errors{0};
  static_assert(decltype(count_acquire_race_errors)::is_always_lock_free);

public:

  void setSynchronousElementInitializer(ElementInitializer const & i) {
    synchronous_element_initializer = i;
  }
  std::optional<ElementInitializer> const & getSynchronousElementInitializer() const {
    return synchronous_element_initializer;
  }

  int getAndResetAcquireRaceErrors() {
    int res = count_acquire_race_errors;
    count_acquire_race_errors -= res;
    return res;
  }

  ImplCRTP()
  : channels()
  {}

  template<typename ChannelsT>
  bool initialize(ChannelsT & chans) {
    midiDelays(); // to allocate the memory for the container
    Assert(state == SynthState::ComputeNotRegistered);
    chans.enqueueOneShot([this, &chans](auto &, auto){
      if (!chans.registerSimpleCompute([this](double * buf, int frames){
        return compute(buf, frames);
      })) {
        throw std::runtime_error("failed to register compute");
      }
    });
    state = SynthState::ComputeRegistered;
    return true;
  }

  template<typename ChannelsT>
  void finalize(ChannelsT & chans) {
    Assert(state == SynthState::ComputeRegistered);
    allNotesOff(chans);
    state = SynthState::WaitingForComputeUnregistration;
    // block until the registered compute function returned false (to be removed from the context queue)
    while(state == SynthState::WaitingForComputeUnregistration) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  bool allEnvelopesFinished() const {
    for(auto const & c : seconds(channels)) {
      if(!c.elem.getEnvelope().isEnvelopeFinished()) {
        return false;
      }
    }
    return true;
  }

  bool someEnvelopesRTActive() const {
    for(auto const & c : seconds(channels)) {
      if(c.elem.getEnvelope().isEnvelopeRTActive()) {
        return true;
      }
    }
    return false;
  }

  // Note: Logic Audio Express 9 calls this when two projects are opened and
  // the second project starts playing, so we are not in an "initialized" state.
  template<typename Chans>
  void allNotesOff(Chans & chans) {
    MIDI_LG(INFO, "all notes off");
    chans.enqueueOneShot([this](auto &, auto){
      for(auto & c : seconds(channels)) {
        c.elem.editEnvelope().onKeyReleased(0);
      }
    });
  }

  template<typename Chans>
  void allSoundsOff(Chans & chans) {
    MIDI_LG(INFO, "all sounds off");
    chans.enqueueOneShot([this](auto &, auto){
      for(auto & c : seconds(channels)) {
        c.elem.editEnvelope().onKeyReleased(0);
      }
    });
  }

  template <typename F>
  void forEachRTActiveElem(F f) {
    for(auto &c : seconds(channels)) {
      if (c.elem.getEnvelope().isEnvelopeRTActive()) {
        f(c.elem);
      }
    }
  }
  template <typename F>
  void forEachElem(F f) {
    for(auto & c : seconds(channels)) {
      f(c.elem);
    }
  }

private:
  bool compute(double * buffer,
               int n_frames) {
    bool silence = true;
    for(auto &c : seconds(channels)) {
      if (!c.elem.getEnvelope().isEnvelopeRTActive()) {
        continue;
      }
      silence = false;
      for (int i=0; i!=n_frames; ++i) {
        c.elem.step();
        // the call above can make the element become "not RT active", but the transition "not RT active" -> "RT active" happens in this thread (the RT thread) so it is safe.
        if (!c.elem.getEnvelope().isEnvelopeRTActive()) {
          break;
        }
        for (int j = 0; j<nAudioOut; ++j) {
          if constexpr (nAudioOut > 1 && Element::count_channels == nAudioOut) {
            buffer[nAudioOut * i + j] += c.elem.imag(j);
          } else {
            buffer[nAudioOut * i + j] += c.elem.imag();
          }
        }
      }
    }
    if (!silence || state != SynthState::WaitingForComputeUnregistration) {
      return true;
    }
    state = SynthState::ComputeNotRegistered;
    return false;
  }

public:
  template<typename Out, typename Chans>
  onEventResult onEvent(int const sample_rate,
                        Event const & e,
                        Out & out,
                        Chans & chans,
                        std::optional<MIDITimestampAndSource> const & maybeMidiTimeAndSource)
  {
    static_assert(Out::policy == outPolicy);
    switch(e.type) {
      case EventType::NoteOn:
      {
        Assert(e.noteOn.velocity > 0.f ); // this case is handled by the wrapper... else we need to do a noteOff
        MIDI_LG(INFO, "on  %d freq %f", e.noteid.noteid, e.noteOn.frequency);
        ElemMidiDelay * channel = nullptr;

        // 1. [with maybe-lock]
        //      reserve a channel by maybe-CAS the envelope State Finished2 -> SoonKeyPressed

        {
          typename Out::LockFromNRT L(out.get_lock());

          for(auto & c : seconds(channels)) {
            if(!c.elem.editEnvelope().tryAcquire()) {
              continue;
            }
            Assert(!c.elem.getEnvelope().isEnvelopeFinished()); // because we just acquired it.
            Assert(c.elem.getEnvelope().isEnvelopeSoonKeyPressed()); // because we just acquired it.
            channel = &c;
            break;
          }
        }

        if(!channel) {
          return onDroppedNote(e);
        }
        auto & c = *channel;

        // 2. [without maybe-lock]
        //      set the noteid
        //      setup the channel (dynamic allocations allowed for soundengine)

        channels.corresponding(*channel) = e.noteid;

        c.elem.forgetPastSignals(); // this does _not_ touch the envelope
        c.elem.set_sample_rate(sample_rate);
        c.elem.getVolumeAdjustment().setVolumeTarget(Element::baseVolume * e.noteOn.velocity);

        // called before setupAudioElement so that we can setup before setting the angle increment in setupAudioElement
        if (synchronous_element_initializer) {
          (*synchronous_element_initializer)(c.elem);
        }

        // setupAudioElement is allowed to be slow, allocate / deallocate memory, etc...
        // because it's not running in the audio realtime thread.
        if(!setupAudioElement(e.noteOn.frequency, c.elem, sample_rate)) {
          MIDI_LG(ERR,"setupAudioElement failed");
          // we let the noteoff reset the envelope state.
          return onDroppedNote(e);
        }

        Assert(!c.elem.getEnvelope().isEnvelopeFinished());
        Assert(c.elem.getEnvelope().isEnvelopeSoonKeyPressed());

        // 3. [with maybe-lock]
        //      register the maybe-oneshot that does
        //        - maybe phase cancellation avoidance (in setPhase)
        //        - register the compute and trigger onKeyPress

        {
          typename Out::LockFromNRT L(out.get_lock());

          chans.enqueueOneShot([this,
                                &c,
                                sample_rate,
                                maybeMidiTimeAndSource
                                // pannedVol, // TODO how to handle panning? systematic use of Panner<>?
                                ]
                               (auto &, uint64_t curTimeNanos){
            // TODO take get_gain() into account globally, when writing the out audio buffer for this synth

            // we issue this call to make sure that the 'tryAcquire' effect will be visible in this thread.
            if(unlikely(!c.elem.editEnvelope().acquireStates())) {
              // error : we did 'tryAcquire' but now, it's not acquired anymore!
              ++count_acquire_race_errors;
              return;
            }

            setPhase<Sync, Phase>(c, channels);

            if(maybeMidiTimeAndSource) {
              Assert(curTimeNanos);
              auto srcKey = maybeMidiTimeAndSource->getSourceKey();
              uint64_t midiTimeNanos = maybeMidiTimeAndSource->getNanosTime();

              /*
               * We introduce an artificial MIDI delay to avoid jitter.
               */

              auto & mayDelay = midiDelays()[srcKey];
              {
                auto const margin = maxMIDIJitter();
                uint64_t const candidateDelay = margin + (curTimeNanos - midiTimeNanos);
                if(!mayDelay) {
                  mayDelay = candidateDelay;
                } else {
                  // a delay is already registered.
                  if(cyclic_unsigned_dist(candidateDelay, *mayDelay) > 2 * (margin + 100000)) {
                    // The change is significant enough, so we apply the change.
                    // The previous delay was maybe determined based
                    // on events that were generated while the program was starting,
                    // hence their timings were off.
                    mayDelay = candidateDelay;
                  }
                }
              }
              auto const delayNanos = *mayDelay;

              uint64_t const targetNanos = midiTimeNanos + delayNanos;

              if(targetNanos < curTimeNanos) {
                // we're late.
                c.elem.editEnvelope().onKeyPressed(0);
                c.midiDelay = {{delayNanos + (curTimeNanos - targetNanos), srcKey}};
              } else {
                // we're on time.
                c.elem.editEnvelope().onKeyPressed(nanoseconds_to_frames(targetNanos-curTimeNanos,
                                                                         sample_rate));
                c.midiDelay = {{delayNanos, srcKey}};
              }
            } else {
              c.elem.editEnvelope().onKeyPressed(0);
              c.midiDelay = {};
            }
          });
        }
      }
        break;
      case EventType::NoteOff:
      {
        if(!handle_note_off) { // TODO remove handle_note_off, redundant with autorelease notion?
          MIDI_LG(INFO, "off (ignored) %d", e.noteid.noteid);
          // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
          // but close method sets the channel to -1 so it's impossible to fade it to zero
          // afterwards using close(), so instead we don't do anything here
          return onEventResult::OK;
        }
        MIDI_LG(INFO, "off %d", e.noteid.noteid);
        typename Out::LockFromNRT L(out.get_lock());
        chans.enqueueOneShot([this,
                              noteid = e.noteid,
                              sample_rate,
                              maybeMidiTimeAndSource](auto &, uint64_t curTimeNanos){
          for(auto &i : firsts(channels)) { // TODO this is linear complexity in number of channels, can we do constant complexity with a unordered_map? Is it better? (we have not so many channels so we need to benchamark...)
            if (!i || *i != noteid) {
              continue;
            }
            auto & c = channels.corresponding(i);

            if(maybeMidiTimeAndSource) {
              Assert(curTimeNanos);
              Assert(c.midiDelay);

              auto d = *c.midiDelay;

              Assert(d.getSourceKey() == maybeMidiTimeAndSource->getSourceKey());

              uint64_t targetNanos = maybeMidiTimeAndSource->getNanosTime() + d.getNanosTime();

              auto delay = (targetNanos < curTimeNanos) ? 0 : nanoseconds_to_frames(targetNanos-curTimeNanos,
                                                                                    sample_rate);
              if(c.elem.getEnvelope().canHandleExplicitKeyReleaseNow(delay)) {
                c.elem.editEnvelope().onKeyReleased(delay);
              }
            } else {
              Assert(!c.midiDelay);
              if(c.elem.getEnvelope().canHandleExplicitKeyReleaseNow(0)) {
                c.elem.editEnvelope().onKeyReleased(0);
              }
            }
            return;
          }
          // The corresponding noteOn was skipped,
          // because too many notes were being played at the same time
        });
      }
        break;
      case EventType::NoteChange:
      {
        MIDI_LG(INFO, "change %d", e.noteid.noteid);
        {
          typename Out::LockFromNRT L(out.get_lock());
          chans.enqueueOneShot([this,
                                volume = Element::baseVolume * e.noteChange.changed_velocity,
                                noteid = e.noteid,
                                increments = freq_to_angle_increment(e.noteChange.changed_frequency, sample_rate)](auto &,
                                                                                                                   uint64_t){
            for(auto &i : firsts(channels)) {
              if (!i || *i != noteid) {
                continue;
              }
              auto & c = channels.corresponding(i);
              c.elem.getVolumeAdjustment().setVolumeTarget(volume);
              c.elem.setAngleIncrements(increments);
            }
          });
        }
      }
        break;
    }
    return onEventResult::OK;
  }

protected:
  onEventResult onDroppedNote(Event const & e) const {
    MIDI_LG(WARN, "dropped note %d at %f Hz", e.noteid.noteid, e.noteOn.frequency);
    return onEventResult::DROPPED_NOTE;
  }
};

template<typename T, ReverbType ReverbT>
struct Wrapper {
  static constexpr auto n_channels = T::n_channels;
  static constexpr auto with_lock = WithLock::No;

  using ProcessData = typename T::ProcessData;

  using OutputData = SimpleAudioOutContext<
  T::nAudioOut,
  AudioOutPolicy::Slave
  >;

  Wrapper(int nOrchestratorsMax, int sampleRate)
  : plugin()
  , out{
    GlobalAudioLock<AudioOutPolicy::Slave>::get(),
    n_channels,
    nOrchestratorsMax
  }
  , sample_rate(sampleRate)
  {
    plugin.template initialize(out);
  }

  void doProcessing (ProcessData& data) {
    return plugin.doProcessing(sample_rate, data, out, out);
  }

  void allNotesOff() {
    return plugin.template allNotesOff(out);
  }

  void allSoundsOff() {
    return plugin.template allSoundsOff(out);
  }

  OutputData out;
  T plugin;
  int sample_rate;
};

} // NS imajuscule::audio
