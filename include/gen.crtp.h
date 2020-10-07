
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

template<SynchronizePhase Sync, DefaultStartPhase Phase, typename MonoNoteChannel, typename CS>
void setPhase(MonoNoteChannel & c, CS & cs)
{
  auto & thisAlgo = c.elem.algo;
  if constexpr (Sync == SynchronizePhase::Yes) {
    auto & tp = cs.corresponding(c);
    for(auto &p : firsts(cs)) {
      if((&p == &tp) || (p != tp)) {
        // same channel, or different frequency.
        continue;
      }
      auto & otherChannel = cs.corresponding(p);
      Assert(&otherChannel != &c);
      // To prevent phase cancellation, the phase of the new note will be
      // coherent with the phase of any active channel that plays a note at the same frequency.
      if(otherChannel.elem.getEnvelope().isEnvelopeFinished()) {
        continue;
      }
      thisAlgo.getOsc().synchronizeAngles(otherChannel.elem.algo.getOsc());
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
  
  LocalPairArray<std::optional<NoteId>, MonoNoteChannel, n_channels> channels;
  
private:
  std::atomic_int count_acquire_race_errors{0};
  static_assert(decltype(count_acquire_race_errors)::is_always_lock_free);
  
public:
  
  int getAndResetAcquireRaceErrors() {
    int res = count_acquire_race_errors;
    count_acquire_race_errors -= res;
    return res;
  }
  
  template <typename A>
  ImplCRTP(std::array<A, n_channels> & as)
  : channels(std::optional<NoteId>{}, as)
  {}
  
  template<typename ChannelsT>
  bool initialize(ChannelsT & chans) {
    midiDelays(); // to allocate the memory for the container
    
    for(auto & c : seconds(channels)) {
      // using WithLock::No : since we own these channels and they are not playing,
      // we don't need to take the audio lock.
      if(!c.template open<WithLock::No>(chans, 0.f)) {
        return false;
      }
    }
    return true;
  }
  
  template<typename ChannelsT>
  void finalize(ChannelsT & chans) {
    for(auto & c : seconds(channels)) {
      if(c.channel == nullptr) {
        continue;
      }
      // using WithLock::No : since we own these channels and they are not playing anymore,
      // we don't need to take the audio lock.
      c.template close<WithLock::No>(chans);
      c.channel = nullptr;
    }
  }
  
  // returns true if at least one channel has an active enveloppe.
  bool areEnvelopesFinished() const {
    for(auto const & c : seconds(channels)) {
      if(!c.elem.getEnvelope().isEnvelopeFinished()) {
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
  void forEachElems(F f) {
    for(auto & c : seconds(channels)) {
      f(c.elem);
    }
  }
  
  template<typename Out, typename Chans>
  onEventResult onEvent(int const sample_rate,
                        Event const & e,
                        Out & out,
                        Chans & chans,
                        std::optional<MIDITimestampAndSource> const & maybeMidiTimeAndSource)
  {
    using Request = typename Chans::Request;
    static_assert(Out::policy == outPolicy);
    switch(e.type) {
      case EventType::NoteOn:
      {
        Assert(e.noteOn.velocity > 0.f ); // this case is handled by the wrapper... else we need to do a noteOff
        MIDI_LG(INFO, "on  %d", e.noteOn.pitch);
        MonoNoteChannel * channel = nullptr;
        
        // 1. [with maybe-lock]
        //      reserve a channel by maybe-CAS the envelope State Finished2 -> SoonKeyPressed
        
        {
          typename Out::LockFromNRT L(out.get_lock());
          
          for(auto & c : seconds(channels)) {
            if(!c.elem.editEnvelope().tryAcquire()) {
              continue;
            }
            Assert(!c.elem.getEnvelope().isEnvelopeFinished()); // because we just acquired it.
            channel = &c;
            break;
          }
        }
        
        if(!channel) {
          // note that we could sleep and retry or yield and retry.
          return onDroppedNote(e);
        }
        auto & c = *channel;
        int32_t channel_index = channels.index(c);
        
        // 2. [without maybe-lock]
        //      set the noteid
        //      setup the channel (dynamic allocations allowed for soundengine)
        
        channels.corresponding(*channel) = e.noteid;
        
        c.elem.forgetPastSignals(); // this does _not_ touch the envelope
        c.elem.algo.set_sample_rate(sample_rate);
        c.elem.algo.setVolumeTarget(e.noteOn.velocity);
        int const xfade_frames_length = static_cast<int>(0.5f + (get_xfade_length() * sample_rate));
        
        // setupAudioElement is allowed to be slow, allocate / deallocate memory, etc...
        // because it's not running in the audio realtime thread.
        Volumes<nAudioOut> pannedVol;
        if(!setupAudioElement(e.noteOn.frequency, c.elem, sample_rate, pannedVol)) {
          MIDI_LG(ERR,"setupAudioElement failed");
          // we let the noteoff reset the envelope state.
          return onDroppedNote(e);
        }
        
        Assert(!c.elem.getEnvelope().isEnvelopeFinished());
        
        // 3. [with maybe-lock]
        //      register the maybe-oneshot that does
        //        - maybe phase cancellation avoidance (in setPhase)
        //        - register the compute and trigger onKeyPress
        
        {
          typename Out::LockFromNRT L(out.get_lock());
          
          chans.enqueueOneShot([this,
                                channel_index,
                                pannedVol,
                                xfade_frames_length]
                               (Chans & chans, uint64_t){
            // unqueue the (potential) previous request, else an assert fails
            // when we enqueue the next request, because it's already queued.
            auto & c = channels.seconds()[channel_index];
            c.reset();
            c.channel->setVolume(get_gain());
            if constexpr (xfade_policy == XfadePolicy::UseXfade) {
              c.channel->set_xfade((xfade_frames_length%2) + 1); // make it odd
            }
            
            // we issue this call to make sure that the 'tryAcquire' effect will be visible in this thread.
            if(unlikely(!c.elem.editEnvelope().acquireStates())) {
              // error : we did 'tryAcquire' but now, it's not acquired anymore!
              ++count_acquire_race_errors;
              return;
            }
            
            // The caller is responsible for growing the channel request queue if needed
            if(!chans.playComputableNoLock(*c.channel,
                                           c.elem.fCompute(),
                                           Request{
              &c.elem.buffer->buffer[0],
              pannedVol,
              // e.noteOn.length is always 0, so we wait for noteOff event to know the duration
              std::numeric_limits<decltype(std::declval<Request>().duration_in_frames)>::max()
            }))
            {
              // there is currently no error reporting from the realtime thread.
              return;
            }
            setPhase<Sync, Phase>(c, channels);
          });
          
          chans.enqueueOneShot([&c,
                                sample_rate,
                                maybeMidiTimeAndSource](Chans & chans, uint64_t curTimeNanos){
            if(maybeMidiTimeAndSource) {
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
          MIDI_LG(INFO, "off (ignored) %f", e.ref_frequency.getFrequency());
          // the initial implementation was using CloseMode::WHEN_DONE_PLAYING for that case
          // but close method sets the channel to -1 so it's impossible to fade it to zero
          // afterwards using close(), so instead we don't do anything here
          return onEventResult::OK;
        }
        MIDI_LG(INFO, "off %f", e.ref_frequency.getFrequency());
        typename Out::LockFromNRT L(out.get_lock());
        chans.enqueueOneShot([this,
                              noteid = e.noteid,
                              sample_rate,
                              maybeMidiTimeAndSource](auto &, uint64_t curTimeNanos){
          for(auto &i : firsts(channels)) {
            if (!i || *i != noteid) {
              continue;
            }
            auto & c = channels.corresponding(i);
            
            if(maybeMidiTimeAndSource) {
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
        MIDI_LG(INFO, "change %d", e.noteChange.ref_pitch);
        {
          typename Out::LockFromNRT L(out.get_lock());
          chans.enqueueOneShot([this,
                                volume = e.noteChange.changed_velocity,
                                noteid = e.noteid,
                                increments = freq_to_angle_increment(e.noteChange.changed_frequency, sample_rate)](auto &,
                                                                                                                   uint64_t){
            for(auto &i : firsts(channels)) {
              if (!i || *i != noteid) {
                continue;
              }
              auto & c = channels.corresponding(i);
              c.elem.algo.setVolumeTarget(volume);
              c.elem.algo.setAngleIncrements(increments);
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
    MIDI_LG(WARN, "dropped note '%f' Hz", e.ref_frequency.getFrequency);
    return onEventResult::DROPPED_NOTE;
  }
};

template<typename T, ReverbType ReverbT>
struct Wrapper {
  static constexpr auto n_channels = T::n_channels;
  static constexpr auto with_lock = WithLock::No;
  
  using ProcessData = typename T::ProcessData;
  
  using OutputData = outputDataBase<
  Channels<T::nAudioOut, T::xfade_policy, T::max_queue_size, AudioOutPolicy::Slave>,
  ReverbT
  >;
  
  template <class... Args>
  Wrapper(int sampleRate, int nOrchestratorsMax, Args&&... args)
  : plugin(std::forward<Args>(args)...)
  , out{
    GlobalAudioLock<AudioOutPolicy::Slave>::get(),
    n_channels,
    nOrchestratorsMax
  }
  , sample_rate(sampleRate)
  {
    dontUseConvolutionReverbs(out, sample_rate);
    plugin.template initialize(out.getChannels());
  }
  
  void doProcessing (ProcessData& data) {
    return plugin.doProcessing(sample_rate, data, out, out.getChannels());
  }
  
  void allNotesOff() {
    return plugin.template allNotesOff(out.getChannels());
  }
  
  void allSoundsOff() {
    return plugin.template allSoundsOff(out.getChannels());
  }
  
  OutputData out;
  T plugin;
  int sample_rate;
};

} // NS imajuscule::audio
