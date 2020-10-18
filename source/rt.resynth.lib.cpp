
namespace imajuscule::audio {

namespace audioelement {

template <typename T>
using ResynthElement =
StereoPanned<
VolumeAdjusted<
Enveloped<
FreqCtrl_<
SineOscillatorAlgo<T, eNormalizePolicy::FAST>,
InterpolatedFreq<T>
>,
AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
>
>
>
;

template <typename T>
using VocoderCarrierElement =
VolumeAdjusted<
Enveloped<
FreqCtrl_<
FOscillatorAlgo<T, FOscillator::SAW, OscillatorUsage::FilteredByLoudnessAdaptedSound>, // Use a saw as carrier to have a wide spectrum
InterpolatedFreq<T>
>,
AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
>
>
;

} // NS audioelement

namespace rtresynth {

enum class InitializationType {
  NewNote,
  ExistingNote
};

inline
audioelement::AHDSR
mkAHDSR(int sample_rate,
        float attack_seconds,
        float hold_seconds,
        float decay_seconds,
        float release_seconds,
        float sustain_level) {
  auto seconds_to_frames = [sample_rate](float secs){
    return ms_to_frames(1000.f * secs,
                        sample_rate);
  };
  return audioelement::AHDSR{
    seconds_to_frames(attack_seconds), itp::LINEAR,
    seconds_to_frames(hold_seconds),
    seconds_to_frames(decay_seconds), itp::LINEAR,
    seconds_to_frames(release_seconds), itp::LINEAR,
    sustain_level
  };
}

template<typename T>
struct ResynthElementInitializer {
  ResynthElementInitializer(int const sample_rate,
                            int const stride,
                            float const stereo_spread,
                            audioelement::AHDSR const & a)
  : sample_rate(sample_rate)
  , stride(stride)
  , stereo_spread_(stereo_spread)
  , ahdsr(a)
  {}
  
  void operator()(audioelement::ResynthElement<T> & e,
                  InitializationType t = InitializationType::NewNote) const {
    // envelope (now that we track the volume, we only need a minimal envelope)
    
    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    
    
    // limit the speed of volume adjustment:
    
    e.getVolumeAdjustment().setMaxFilterIncrement(2. / static_cast<double>(stride));
    
    // frequency control
    
    e.getVolumeAdjustment().getOsc().getAlgo().getCtrl().setup(stride,
                                                               itp::LINEAR);
    
    // panning (except for note changes)
    
    if (t==InitializationType::NewNote) {
      e.setup(stereo(stereo_spread_ * std::uniform_real_distribution<float>{-1.f,1.f}(mersenne<SEEDED::No>())));
    }
  }
  
  bool operator ==(ResynthElementInitializer const& o) const {
    return
    std::make_tuple(sample_rate, stride, stereo_spread_, ahdsr) ==
    std::make_tuple(o.sample_rate, o.stride, o.stereo_spread_, o.ahdsr);
  }
  bool operator !=(ResynthElementInitializer const& o) const {
    return !this->operator ==(o);
  }
  
private:
  int sample_rate;
  int stride;
  float stereo_spread_;
  audioelement::AHDSR ahdsr;
};


template<typename T>
struct VocoderCarrierElementInitializer {
  VocoderCarrierElementInitializer(int const sample_rate,
                                   audioelement::AHDSR const & a)
  : sample_rate(sample_rate)
  , ahdsr(a)
  {}
  
  void operator()(audioelement::VocoderCarrierElement<T> & e,
                  InitializationType t = InitializationType::NewNote) const {
    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    e.getVolumeAdjustment().getOsc().getAlgo().getCtrl().setup(100,
                                                               itp::LINEAR);
  }
  
  bool operator ==(VocoderCarrierElementInitializer const& o) const {
    return
    std::make_tuple(sample_rate, ahdsr) ==
    std::make_tuple(o.sample_rate, o.ahdsr);
  }
  bool operator !=(VocoderCarrierElementInitializer const& o) const {
    return !this->operator ==(o);
  }
  
private:
  int sample_rate;
  audioelement::AHDSR ahdsr;
};


constexpr int nAudioOut = 2;
constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

using Synth = sine::Synth <
audioEnginePolicy
, nAudioOut
, XfadePolicy::SkipXfade
, audioelement::ResynthElement<double>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator
, 127  // lots of voices
, ResynthElementInitializer<double>
>;

using SynthVocoderCarier = sine::Synth <
audioEnginePolicy
, 1 // mono
, XfadePolicy::SkipXfade
, audioelement::VocoderCarrierElement<double>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator
, 127  // lots of voices
, VocoderCarrierElementInitializer<double>
>;


using PostImpl = AudioPostPolicyImpl<nAudioOut, ReverbType::Realtime_Synchronous, audioEnginePolicy>;

using Ctxt = Context<
AudioPlatform::PortAudio,
Features::InAndOut,
SimpleAudioOutContext<
nAudioOut,
audioEnginePolicy,
PostImpl
>
>;


bool constexpr logs = false;

inline
void synthesize_sounds(Midi const & midi,
                       int64_t const analysis_frame_idx,
                       int const sample_rate,
                       MIDITimestampAndSource const & miditime,
                       int const future_stride,
                       float const gain_analysis,
                       float const stereo_spread,
                       audioelement::AHDSR const & ahdsr,
                       std::vector<PitchVolume> const & autotuned_pitches,
                       std::vector<int> const & autotuned_pitches_idx_sorted_by_perceived_loudness,
                       std::vector<std::optional<int>> const & pitch_changes,
                       std::vector<bool> const & continue_playing,
                       Synth & synth,
                       Ctxt & ctxt,
                       int64_t & next_noteid,
                       std::vector<PlayedNote> & played_pitches,
                       NonRealtimeAnalysisFrame & analysis_data,
                       std::atomic_int & dropped_note_on) {
  {
    ResynthElementInitializer<double> initializer{sample_rate, future_stride, stereo_spread, ahdsr};
    if (!synth.getSynchronousElementInitializer() || *synth.getSynchronousElementInitializer() != initializer) {
      // This will apply the new params for new notes:
      synth.setSynchronousElementInitializer(initializer);
      
      // This will apply the new params for currently played notes
      ctxt.getStepper().enqueueOneShot([&synth,
                                        initializer](auto &, auto){
        synth.forEachRTActiveElem([initializer](auto & e) {
          initializer(e, InitializationType::ExistingNote);
        });
      });
      
      // If we are unlucky, we missed the notes that are in "SoonKeyPressed" state.
    }
  }
  
  // issue "note off" events
  {
    int idx = -1;
    for (auto play : continue_playing) {
      ++idx;
      if (!play) {
        auto res = synth.onEvent(sample_rate,
                                 mkNoteOff(played_pitches[idx].noteid),
                                 ctxt.getStepper(),
                                 ctxt.getStepper(),
                                 miditime);
        if (logs) std::cout << analysis_frame_idx << ": XXX pitch " << played_pitches[idx].midi_pitch << " " << res << std::endl;
        if (res != onEventResult::OK) {
          throw std::logic_error("dropped note off");
        }
        analysis_data.try_push_note_off(played_pitches[idx]);
      }
    }
  }
  
  // issue "note change" and "note on" events
  for (int idx : autotuned_pitches_idx_sorted_by_perceived_loudness) {
    std::optional<int> const & pitch_change = pitch_changes[idx];
    
    double const new_pitch = autotuned_pitches[idx].midipitch;
    float const new_freq = midi.midi_pitch_to_freq(new_pitch);
    
    // Divide by reduceUnadjustedVolumes because in our case eventhough the oscillator is not adjusted wrt loudness,
    // its volume comes from a real sound so the loudness is correct
    float const volume = gain_analysis * autotuned_pitches[idx].volume / audioelement::reduceUnadjustedVolumes;
    
    if (pitch_change) {
      PlayedNote & played = played_pitches[*pitch_change];
      
      auto res = synth.onEvent(sample_rate,
                               mkNoteChange(played.noteid,
                                            volume,
                                            new_freq),
                               ctxt.getStepper(),
                               ctxt.getStepper(),
                               miditime);
      if (res != onEventResult::OK) {
        throw std::logic_error("dropped note change");
      }
      if (logs) std::cout << analysis_frame_idx << ": pitch " << played.midi_pitch << " newpitch " << new_pitch << " Vol " << volume << " " << res << std::endl;
      
      played.cur_freq = new_freq;
      played.midi_pitch = new_pitch;
      played.cur_velocity = volume;
      
      analysis_data.try_push_note_change(played);
    } else {
      if (volume <= 0) {
        continue;
      }
      ++next_noteid;
      NoteId const note_id{next_noteid};
      auto const res = synth.onEvent(sample_rate,
                                     mkNoteOn(note_id,
                                              new_freq,
                                              volume),
                                     ctxt.getStepper(),
                                     ctxt.getStepper(),
                                     miditime);
      if (logs) std::cout << analysis_frame_idx << ": pitch " << new_pitch << " vol " << volume << " " << res << std::endl;
      
      PlayedNote new_note{
        analysis_frame_idx,
        note_id,
        new_pitch,
        new_freq,
        volume
      };
      if (res == onEventResult::OK) {
        played_pitches.push_back(new_note);
        analysis_data.try_push_note_on(played_pitches.back());
      } else {
        ++dropped_note_on;
        analysis_data.try_push_note_on_dropped(new_note);
      }
    }
  }
  
  analysis_data.try_push(NonRealtimeAnalysisFrame::EndOfFrame{
    analysis_frame_idx,
    std::chrono::microseconds(static_cast<int>(1000000. * static_cast<double>(future_stride)/sample_rate))
  }, played_pitches);
}

struct RtResynth {
private:
  
  static double constexpr minInLatency  = 0.000001;
  static double constexpr minOutLatency = 0.000001;
  
  static PitchReductionMethod constexpr pitch_method =
  PitchReductionMethod::PonderateByVolume;
  static VolumeReductionMethod constexpr volume_method =
  VolumeReductionMethod::SumVolumes;
  
  // producer : Audio input callback
  // consumer : analysis thread
  using QueueItem = std::variant<InputSample,CountDroppedFrames>;
  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ QueueItem,
  /* A = */ std::allocator<QueueItem>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  
public:
  
  RtResynth();
  
  ~RtResynth();

  void init(int const sample_rate);
  
  void teardown();
  
  // These methods can be used with no need to reinitialize
  
  void setInputDelaySeconds(float f) {
    if (f < 0) {
      throw std::invalid_argument("input_delay_seconds is too small");
    }
    input_delay_seconds = f;
  }
  
  void setWindowCenterStrideSeconds(float f) {
    if (f < 0) {
      throw std::invalid_argument("window_center_stride_seconds is too small");
    }
    window_center_stride_seconds = f;
  }
  
  void setWindowSizeSeconds(float f) {
    if (f < 0) {
      throw std::invalid_argument("window_size_seconds is too small");
    }
    window_size_seconds = f;
  }
  
  void setMinVolume(float f) {
    if (f < 0) {
      throw std::invalid_argument("min_volume is too small");
    }
    min_volume = f;
  }
  
  void setNearbyDistanceTones(float f) {
    if (nearby_distance_tones < 0) {
      throw std::invalid_argument("nearby_distance_tones is too small");
    }
    nearby_distance_tones = f;
  }
  
  void setMaxTrackPitches(float f) {
    if (f < 0) {
      throw std::invalid_argument("max_track_pitches is too small");
    }
    max_track_pitches = f;
  }
  
  float getInputDelaySeconds() const {
    return input_delay_seconds;
  }
  float getWindowSizeSeconds() const {
    return window_size_seconds;
  }
  float getEffectiveWindowSizeSeconds(int sample_rate) const {
    return std::max(getWindowSizeSeconds(),
                    2.f/sample_rate);
  }
  float getWindowCenterStrideSeconds() const {
    return window_center_stride_seconds;
  }
  float getEffectiveWindowCenterStrideSeconds(int sample_rate) const {
    if (!sample_rate) {
      return 0.f;
    }
    return std::max(getWindowCenterStrideSeconds(),
                    1.f/sample_rate);
  }
  float getMinVolume() const {
    return min_volume;
  }
  float getNearbyDistanceTones() const {
    return nearby_distance_tones;
  }
  float getMaxTrackPitches() const {
    return max_track_pitches;
  }
  
  void storeAudioInputQueueFillRatio(float f) {
    input_queue_fill_ratio = f;
  }
  
  float getAudioInputQueueFillRatio() const {
    return input_queue_fill_ratio;
  }
  int countDroppedInputFrames() const {
    int count = 0;
    if (input_2_analysis_queue) {
      count += input_2_analysis_queue->countDroppedInputFrames();
    }
    if (input_2_vocoder_queue) {
      count += input_2_vocoder_queue->countDroppedInputFrames();
    }
    return 0;
  }
  
  int countDroppedNoteOns() const {
    return dropped_note_on;
  }
  
  int countFailedComputeInsertions() const {
    return ctxt.getStepper().countFailedComputeInsertions();
  }
  int countRetriedOneshotInsertions() const {
    return ctxt.getStepper().countRetriedOneshotInsertions();
  }
  
  std::optional<float> getDurationProcess() {
    float const f = dt_process_seconds;
    if (f < 0) {
      return {};
    }
    return f;
  }
  std::optional<float> getDurationStep() {
    float const f = dt_step_seconds;
    if (f < 0) {
      return {};
    }
    return f;
  }
  std::optional<float> getDurationCopy() {
    float const f = dt_copy_seconds;
    if (f < 0) {
      return {};
    }
    return f;
  }
  
  NonRealtimeAnalysisFrame const & getAnalysisData() const {
    return analysis_data;
  }
  
  float getHarmonizePreAutotune() const {
    return pitch_harmonize_pre_autotune;
  }
  void setHarmonizePreAutotune(float f) {
    pitch_harmonize_pre_autotune = f;
  }
  float getHarmonizePostAutotune() const {
    return pitch_harmonize_post_autotune;
  }
  void setHarmonizePostAutotune(float f) {
    pitch_harmonize_post_autotune = f;
  }
  
  float getPitchShiftPreAutotune() const {
    return pitch_shift_pre_autotune;
  }
  void setPitchShiftPreAutotune(float f) {
    pitch_shift_pre_autotune = f;
  }
  float getPitchShiftPostAutotune() const {
    return pitch_shift_post_autotune;
  }
  void setPitchShiftPostAutotune(float f) {
    pitch_shift_post_autotune = f;
  }
  
  
  bool getUseAutotune() const {
    return use_autotune;
  }
  void setUseAutotune(bool t) {
    use_autotune = t;
  }
  
  float getAutotunePitchTolerance() const {
    return autotune_tolerance_pitches;
  }
  void setAutotunePitchTolerance(float f) {
    autotune_tolerance_pitches = f;
  }
  int getAutotuneMaxPitch() const {
    return autotune_max_pitch;
  }
  void setAutotuneMaxPitch(int f) {
    autotune_max_pitch = f;
  }
  int getAutotuneFactor() const {
    return autotune_factor;
  }
  void setAutotuneFactor(int f) {
    autotune_factor = f;
  }
  AutotuneType getAutotuneType() const {
    return autotune_type;
  }
  void setAutotuneType(AutotuneType f) {
    autotune_type = f;
  }
  MusicalScaleMode getAutotuneMusicalScaleMode() const {
    return autotune_musical_scale_mode;
  }
  void setAutotuneMusicalScaleMode(MusicalScaleMode f) {
    autotune_musical_scale_mode = f;
  }
  Note getAutotuneMusicalScaleRoot() const {
    return autotune_musical_scale_root_note;
  }
  void setAutotuneMusicalScaleRoot(Note f) {
    autotune_musical_scale_root_note = f;
  }
  AutotuneChordFrequencies getAutotuneChordFrequencies() const {
    return autotune_chord_frequencies;
  }
  void setAutotuneChordFrequencies(AutotuneChordFrequencies f) {
    autotune_chord_frequencies = f;
  }
  
  int getAutotuneRootTranspose() const {
    return autotune_root_note_halftones_transpose;
  }
  void setAutotuneRootTranspose(int t) {
    autotune_root_note_halftones_transpose = t;
  }
  uint64_t getAutotuneBitChord() const {
    return autotune_bit_chord;
  }
  void setAutotuneBitChord(uint64_t t) {
    autotune_bit_chord = t;
  }
  
  bool autotuneBitChordHasNote(Note const n) const {
    uint64_t const value = to_underlying(n);
    Assert(value < 64);
    uint64_t mask = 1 << value;
    return static_cast<bool>(mask & autotune_bit_chord.load());
  }
  void autotuneBitChordSetNote(Note const n, bool const active) {
    uint64_t const value = to_underlying(n);
    Assert(value < 64);
    uint64_t mask = 1 << value;
    if (active) {
      autotune_bit_chord |= mask;
    } else {
      autotune_bit_chord &= ~mask;
    }
  }
  float getStereoSpread() const {
    return stereo_spread;
  }
  void setStereoSpread(float f) {
    stereo_spread = f;
  }

  float getEnvAttackSeconds() const {
    return env_attack_seconds;
  }
  void setEnvAttackSeconds(float f) {
    env_attack_seconds = f;
  }
  float getEnvHoldSeconds() const {
    return env_hold_seconds;
  }
  void setEnvHoldSeconds(float f) {
    env_hold_seconds = f;
  }
  float getEnvDecaySeconds() const {
    return env_decay_seconds;
  }
  void setEnvDecaySeconds(float f) {
    env_decay_seconds = f;
  }
  float getEnvReleaseSeconds() const {
    return env_release_seconds;
  }
  void setEnvReleaseSeconds(float f) {
    env_release_seconds = f;
  }
  float getEnvSustainLevel() const {
    return env_sustain_level;
  }
  void setEnvSustainLevel(float f) {
    env_sustain_level = f;
  }

  float getDirectVoiceVolume() const {
    return voice_volume;
  }
  void setDirectVoiceVolume(float f) {
    voice_volume = f;
  }
  
  float getVocoderVolume() const {
    return vocoder_volume;
  }
  void setVocoderVolume(float f) {
    vocoder_volume = f;
  }
  
  float getAnalysisVolume() const {
    return analysis_volume;
  }
  void setAnalysisVolume(float f) {
    analysis_volume = f;
  }
  
  float getInputStreamCpuLoad() const {
    return input.getStreamCpuLoad();
  }
  float getOutputStreamCpuLoad() const {
    return ctxt.getStreamCpuLoad();
  }

  float getCompressionFactor() const {
    return ctxt.getStepper().getPost().getLimiter().getTargetCompressionLevel();
  }
  
private:
  Ctxt ctxt;
  Synth synth;
  SynthVocoderCarier vocoder_carrier;
  Midi midi;
  
  std::atomic_bool thread_resynth_active{false};
  std::unique_ptr<std::thread> thread_resynth;
  
  MetaQueue<Queue> * input_2_analysis_queue = nullptr;
  MetaQueue<Queue> * input_2_vocoder_queue = nullptr;
  
  ReadQueuedSampleSource<Queue, Ctxt> read_queued_input;
  
  Vocoder vocoder;
  
  Input<AudioPlatform::PortAudio, Queue> input;
  
  NonRealtimeAnalysisFrame analysis_data;
  int64_t analysis_frame_idx = 0;
  
  cyclic<float> delayed_input;
  std::atomic<float> input_delay_seconds = 0.f;
  std::atomic<float> window_size_seconds = 0.1814f;
  std::atomic<float> window_center_stride_seconds = 0.09f;
  std::atomic<float> min_volume = 0.0001;
  std::atomic<float> nearby_distance_tones = 0.4;
  std::atomic<float> max_track_pitches = 1.;
  std::atomic<float> autotune_tolerance_pitches = 100.;
  std::atomic<float> pitch_shift_pre_autotune = 0.f;
  std::atomic<float> pitch_shift_post_autotune = 0.f;
  std::atomic<float> pitch_harmonize_pre_autotune = 0.f;
  std::atomic<float> pitch_harmonize_post_autotune = 0.f;
  std::atomic<float> stereo_spread = 1.f;
  std::atomic<float> env_attack_seconds = 0.f;
  std::atomic<float> env_hold_seconds = 0.f;
  std::atomic<float> env_decay_seconds = 0.f;
  std::atomic<float> env_release_seconds = 0.f;
  std::atomic<float> env_sustain_level = 1.f;
  static_assert(std::atomic<float>::is_always_lock_free);
  
  std::atomic<AutotuneType> autotune_type = AutotuneType::MusicalScale;
  static_assert(std::atomic<AutotuneType>::is_always_lock_free);
  std::atomic_bool use_autotune = false;
  std::atomic_int autotune_max_pitch = 150;
  std::atomic_int autotune_factor = 2;
  static_assert(std::atomic_int::is_always_lock_free);
  std::atomic<MusicalScaleMode> autotune_musical_scale_mode = MusicalScaleMode::Major;
  static_assert(std::atomic<MusicalScaleMode>::is_always_lock_free);
  std::atomic<Note> autotune_musical_scale_root_note = Note::Do;
  static_assert(std::atomic<Note>::is_always_lock_free);
  std::atomic_int autotune_root_note_halftones_transpose = 0;
  std::atomic<AutotuneChordFrequencies> autotune_chord_frequencies = AutotuneChordFrequencies::Harmonics;
  std::atomic<uint64_t> autotune_bit_chord = 0b10010001; // least significant beat = lower pitch
  std::atomic<float> voice_volume = 1.f;
  std::atomic<float> vocoder_volume = 0.f;
  std::atomic<float> analysis_volume = 1.f;
  
  std::vector<float> allowed_pitches;
  
  int64_t next_noteid = 0;
  
  using Sample = double;
  using SampleVector = std::vector<Sample>;
  using SampleVectorConstIterator = SampleVector::const_iterator;
  SampleVector samples;
  std::vector<double> half_window;
  a64::vector<double> work_vector_signal;
  typename fft::RealFBins_<fft::Fastest, double, a64::Alloc>::type work_vector_freqs;
  
  FrequenciesSqMag<double> frequencies_sqmag;
  std::vector<FreqMag<double>> freqmags;
  std::vector<PitchVolume> freqmags_data;
  std::vector<PitchInterval> pitch_intervals;
  std::vector<PitchVolume> reduced_pitches, autotuned_pitches, pitches_tmp;
  std::vector<float> autotuned_pitches_perceived_loudness;
  std::vector<int> autotuned_pitches_idx_sorted_by_perceived_loudness;
  std::vector<PlayedNote> played_pitches;
  std::vector<std::optional<int>> pitch_changes;
  std::vector<bool> continue_playing; // indexed like 'played_pitches'
  
  std::atomic<float> dt_process_seconds = -1.f;
  std::atomic<float> dt_step_seconds = -1.f;
  std::atomic<float> dt_copy_seconds = -1.f;
  std::atomic<float> input_queue_fill_ratio = 0.f;
  std::atomic_int dropped_note_on = 0;
  
  void analysis_thread(int const sample_rate);
  
  void setDurations(std::optional<float> duration_process_seconds,
                    std::optional<float> duration_step_seconds,
                    std::optional<float> duration_copy_seconds);
  
  void step (int const sample_rate,
             std::vector<FreqMag<double>> const & fs,
             MIDITimestampAndSource const & miditime,
             int const future_stride);
  
  std::function<std::optional<float>(float const)>
  mkAutotuneFunction();
};


RtResynth::RtResynth()
: synth()
, ctxt(GlobalAudioLock<audioEnginePolicy>::get(),
       Synth::n_channels * 4 /* one shot */,
       1 // fft-based synth
       + 1    //vocoder
       )
, input(4,
        4)
{
  frequencies_sqmag.frequencies_sqmag.reserve(200);
  freqmags.reserve(200);
  freqmags_data.reserve(200);
  pitch_intervals.reserve(200);
  reduced_pitches.reserve(200);
  autotuned_pitches.reserve(200);
  pitches_tmp.reserve(200);
  autotuned_pitches_perceived_loudness.reserve(200);
  autotuned_pitches_idx_sorted_by_perceived_loudness.reserve(200);
  played_pitches.reserve(200);
  pitch_changes.reserve(200);
  continue_playing.reserve(200);
  
  allowed_pitches.reserve(2*max_audible_midi_pitch);
  
#ifndef NDEBUG
  testAutotune();
#endif
}

RtResynth::~RtResynth() {
  teardown();
}

void
RtResynth::init(int const sample_rate) {
  if (sample_rate <= 0) {
    throw std::invalid_argument("sample_rate is too small");
  }
  
  teardown();
  Assert(!thread_resynth_active);
  
  if (!ctxt.doInit(minOutLatency, sample_rate)) {
    throw std::runtime_error("ctxt init failed");
  }
  
  if (!synth.initialize(ctxt.getStepper())) {
    throw std::logic_error("failed to initialize synth");
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  input.Init(sample_rate,
             minInLatency);
  
  input_2_analysis_queue = input.add_queue(sample_rate); // one second of input can fit in the queue

  if (input.getSampleRate() != ctxt.getSampleRate()) {
    throw std::runtime_error("in and out sample rates mismatch");
  }

  {
    int const size = sample_rate * 4. * std::max(input.getInputLatencySeconds(),
                                                 ctxt.getOutputLatencySeconds());
    std::cout << "Vocoder queue size = " << size << std::endl;
    input_2_vocoder_queue = input.add_queue(size);
  }

  read_queued_input.set(input_2_vocoder_queue->queue
#ifndef NDEBUG
                        , ctxt
#endif
                        );
  
  vocoder.initialize([this]() -> std::optional<std::pair<double, double>> {
    std::optional<double> mod = read_queued_input();
    if (!mod) {
      return {};
    }
    double carrier_val{};
    vocoder_carrier.compute(&carrier_val,
                            1);
    return std::make_pair(*mod, carrier_val);
  },
                     ctxt,
                     voice_volume,
                     vocoder_volume);
  vocoder_carrier.setSynchronousElementInitializer(VocoderCarrierElementInitializer<double>(
    sample_rate,
    mkAHDSR(sample_rate,
            env_attack_seconds,
            env_hold_seconds,
            env_decay_seconds,
            env_release_seconds,
            env_sustain_level)
  ));

  vocoder_carrier.onEvent(sample_rate,
                          mkNoteOn(NoteId{1},
                                   100.,
                                   0.9),
                          ctxt.getStepper(),
                          ctxt.getStepper(),
                          {});
  
  thread_resynth_active = true;
  thread_resynth = std::make_unique<std::thread>([this,
                                                  sample_rate](){
    analysis_thread(sample_rate);
  });
}

void
RtResynth::teardown() {
  if (!thread_resynth_active) {
    return;
  }
  // stop queue consumers
  thread_resynth_active = false;
  thread_resynth->join();
  thread_resynth.reset();
  
  vocoder.finalize(ctxt.getStepper());
  
  // finalize synth
  synth.finalize(ctxt.getStepper());
  
  Assert(input_2_analysis_queue);
  input.remove_queue(input_2_analysis_queue);
  input_2_analysis_queue = nullptr;
  
  Assert(input_2_vocoder_queue);
  input.remove_queue(input_2_vocoder_queue);
  input_2_vocoder_queue = nullptr;
  
  input.Teardown();
  
  ctxt.doTearDown();
}


void
RtResynth::analysis_thread(int const sample_rate) {
  auto process = [this](int const sample_rate,
                        SampleVectorConstIterator from,
                        SampleVectorConstIterator to,
                        std::vector<double> const & half_window,
                        FrequenciesSqMag<double> & frequencies_sqmag,
                        std::vector<FreqMag<double>> & freqmags) {
    int constexpr zero_padding_factor = 1;
    int constexpr windowed_signal_stride = 1;
    
    findFrequenciesSqMag<fft::Fastest>(from,
                                       to,
                                       windowed_signal_stride,
                                       half_window,
                                       zero_padding_factor,
                                       work_vector_signal,
                                       work_vector_freqs,
                                       frequencies_sqmag);
    
    extractLocalMaxFreqsMags(sample_rate / windowed_signal_stride,
                             frequencies_sqmag,
                             SqMagToDb<double>(),
                             freqmags);
  };
  
  int end = 0;
  
  auto init_data = [this,
                    sample_rate
                    ](bool const force){
    int const input_delay_frames = input_delay_seconds * sample_rate;
    
    // we need an even window size
    int const window_size = 2 * std::max(1,
                                         static_cast<int>(0.5 * window_size_seconds * sample_rate));
    bool reinit_samples = force;
    if (force || static_cast<int>(delayed_input.size()) != input_delay_frames + 1) {
      delayed_input.resize(input_delay_frames + 1); // this also resets the elements to zero
      reinit_samples = true;
    }
    {
      Assert(0 == window_size%2);
      if (static_cast<int>(half_window.size()) != window_size/2) {
        half_gaussian_window<double>(4, window_size/2, half_window);
        normalize_window(half_window);
        Assert(static_cast<int>(half_window.size()) == window_size/2);
        reinit_samples = true;
      }
    }
    if (reinit_samples) {
      samples.clear();
      samples.resize(window_size, {});
    }
    return std::make_pair(input_delay_frames, window_size);
  };
  
  // force reinitialization
  auto [input_delay_frames, window_size] = init_data(true);
  
  constexpr uint64_t midi_src_id = 0;
  
  double const nanos_pre_frame = 1. / static_cast<double>(sample_rate);
  uint64_t count_queued_frames = 0;
  uint64_t local_count_dropped_input_frames = 0;
  
  int ignore_frames = 0;
  
  while (thread_resynth_active) {
    QueueItem var;
    while (input_2_analysis_queue->queue.try_pop(var)) {
      if (std::holds_alternative<CountDroppedFrames>(var)) {
        auto count = std::get<CountDroppedFrames>(var).count;
        local_count_dropped_input_frames += count;
        ignore_frames -= count;
        if (!thread_resynth_active) {
          return;
        }
        continue;
      }
      Assert(std::holds_alternative<InputSample>(var));
      ++count_queued_frames;
      if (input_delay_frames > 1) {
        delayed_input.feed(std::get<InputSample>(var).value);
      }
      if (ignore_frames > 0) {
        --ignore_frames;
        continue;
      }
      if (input_delay_frames > 1) {
        samples[end] = *delayed_input.cycleEnd();
      } else {
        samples[end] = std::get<InputSample>(var).value;
      }
      ++ end;
      if (end == window_size) {
        end = 0;
        if (!thread_resynth_active) {
          return;
        }
        
        std::optional<float>
        duration_process_seconds,
        duration_step_seconds,
        duration_copy_seconds;
        
        {
          std::optional<profiling::CpuDuration> dt;
          {
            profiling::ThreadCPUTimer timer(dt);
            
            process(sample_rate,
                    samples.begin(),
                    samples.begin() + window_size,
                    half_window,
                    frequencies_sqmag,
                    freqmags);
          }
          if (dt) duration_process_seconds = dt->count() / 1000000.;
        }
        
        int const window_center_stride = std::max(1,
                                                  static_cast<int>(0.5f + window_center_stride_seconds * sample_rate));
        {
          std::optional<profiling::CpuDuration> dt;
          {
            profiling::ThreadCPUTimer timer(dt);
            step(sample_rate,
                 freqmags,
                 MIDITimestampAndSource((count_queued_frames + local_count_dropped_input_frames) * nanos_pre_frame,
                                        midi_src_id),
                 window_center_stride); // we pass the future stride, on purpose
          }
          if (dt) duration_step_seconds = dt->count() / 1000000.;
        }
        
        std::tie(input_delay_frames, window_size) = init_data(false);
        
        {
          std::optional<profiling::CpuDuration> dt;
          {
            profiling::ThreadCPUTimer timer(dt);
            int const windowoverlapp = window_size - window_center_stride;
            if (windowoverlapp >= 0) {
              const int offset = window_size-windowoverlapp;
              for (; end<windowoverlapp; ++end) {
                samples[end] = samples[end + offset];
              }
              ignore_frames = 0;
            } else {
              ignore_frames = -windowoverlapp;
            }
          }
          if (dt) duration_copy_seconds = dt->count() / 1000000.;
        }
        
        setDurations(duration_process_seconds,
                     duration_step_seconds,
                     duration_copy_seconds);
        
        storeAudioInputQueueFillRatio(input_2_analysis_queue->queue.was_size() / static_cast<float>(input_2_analysis_queue->queue.capacity()));
      }
    }
    std::this_thread::yield(); // should we sleep?
  }
}

void
RtResynth::setDurations(std::optional<float> duration_process_seconds,
                        std::optional<float> duration_step_seconds,
                        std::optional<float> duration_copy_seconds) {
  auto set = [](std::optional<float> const & secs, std::atomic<float> & atom) {
    if (secs) {
      atom = *secs;
    } else {
      atom = -1.f;
    }
  };
  set(duration_process_seconds, dt_process_seconds);
  set(duration_step_seconds, dt_step_seconds);
  set(duration_copy_seconds, dt_copy_seconds);
}

void
RtResynth::step (int const sample_rate,
                 std::vector<FreqMag<double>> const & fs,
                 MIDITimestampAndSource const & miditime,
                 int const future_stride) {
  ++analysis_frame_idx;
  
  frequencies_to_pitches(midi,
                         fs,
                         freqmags_data);
  
  aggregate_pitches(nearby_distance_tones,
                    freqmags_data,
                    pitch_intervals);
  
  reduce_pitches(pitch_method,
                 volume_method,
                 min_volume,
                 pitch_intervals,
                 reduced_pitches);
  
  shift_pitches(pitch_shift_pre_autotune,
                reduced_pitches);
  
  harmonize_pitches(pitch_harmonize_pre_autotune,
                    pitches_tmp,
                    reduced_pitches);
  
  autotune_pitches(autotune_max_pitch.load(),
                   autotune_tolerance_pitches,
                   mkAutotuneFunction(),
                   reduced_pitches,
                   autotuned_pitches);
  
  shift_pitches(pitch_shift_post_autotune,
                autotuned_pitches);
  
  harmonize_pitches(pitch_harmonize_post_autotune,
                    pitches_tmp,
                    autotuned_pitches);
  
  track_pitches(max_track_pitches,
                autotuned_pitches,
                played_pitches,
                pitch_changes,
                continue_playing);
  
  // 60 dB SPL is a normal conversation, see https://en.wikipedia.org/wiki/Sound_pressure#Sound_pressure_level
  int constexpr loudness_idx = loudness::phons_to_index(60.f);
  order_pitches_by_perceived_loudness([loudness_idx](PitchVolume const & pv) {
    return
    pv.volume /
    loudness::equal_loudness_volume_db(loudness::pitches,
                                       pv.midipitch,
                                       loudness_idx);
  },
                                      autotuned_pitches,
                                      autotuned_pitches_perceived_loudness,
                                      autotuned_pitches_idx_sorted_by_perceived_loudness);
  
  synthesize_sounds(// constant args:
                    midi,
                    analysis_frame_idx,
                    sample_rate,
                    miditime,
                    future_stride,
                    analysis_volume,
                    stereo_spread,
                    mkAHDSR(sample_rate,
                            env_attack_seconds,
                            env_hold_seconds,
                            env_decay_seconds,
                            env_release_seconds,
                            env_sustain_level),
                    autotuned_pitches,
                    autotuned_pitches_idx_sorted_by_perceived_loudness,
                    pitch_changes,
                    continue_playing,
                    // mutable args:
                    synth,
                    ctxt,
                    next_noteid,
                    played_pitches,
                    analysis_data,
                    dropped_note_on);
  
  remove_dead_notes(continue_playing,
                    played_pitches);
  
  sort_by_current_pitch(played_pitches);
}

std::function<std::optional<float>(float const)>
RtResynth::mkAutotuneFunction() {
  if (!use_autotune) {
    return [](float const v) -> std::optional<float> {return {v};};
  }
  switch(autotune_type) {
    case AutotuneType::Chord:
    {
      int offset = half_tones_distance(Note::Do,
                                       autotune_musical_scale_root_note);
      if (offset < 0) {
        offset += num_halftones_per_octave;
      }
      offset += autotune_root_note_halftones_transpose;
      // the lowest bit is C4+offset
      int constexpr C_pitch = A_pitch + half_tones_distance(Note::La,
                                                            Note::Do) + num_halftones_per_octave;
      int const root_pitch = offset + C_pitch;
      std::bitset<64> const chord{autotune_bit_chord.load()};
      allowed_pitches.clear();
      
      bool single = false;
      switch(autotune_chord_frequencies) {
        case AutotuneChordFrequencies::SingleFreq:
          single = true;
        case AutotuneChordFrequencies::OctavePeriodic:
        {
          int const octaveMin = single ? 0:-5;
          int const octaveMax = single ? 0:5;
          for (int octave = octaveMin; octave <= octaveMax; ++octave) {
            int const add = num_halftones_per_octave * octave;
            for (int i=0, sz = static_cast<int>(chord.size()); i < sz; ++i) {
              if (chord[i]) {
                allowed_pitches.push_back(root_pitch + i + add);
              }
            }
          }
          break;
        }
        case AutotuneChordFrequencies::Harmonics:
        {
          constexpr int n_harmo = 36;
          static constexpr std::array<double, n_harmo> harmonic_pitch_add = compute_harmonic_pitch_adds<n_harmo>(ConstexprMidi());
          for (int harmo = 0; harmo < n_harmo; ++harmo) {
            for (int i=0, sz = static_cast<int>(chord.size()); i < sz; ++i) {
              if (chord[i]) {
                // positive harmonic
                allowed_pitches.push_back(harmonic_pitch_add[harmo] + root_pitch + i);
                // negative harmonic
                allowed_pitches.push_back(-harmonic_pitch_add[harmo] + root_pitch + i);
              }
            }
          }
          break;
        }
      }
      
      std::sort(allowed_pitches.begin(),
                allowed_pitches.end());
      
      return [this](float const pitch)-> std::optional<float> {
        if (float * p = find_closest_pitch(pitch, allowed_pitches, [](float p){ return p; })) {
          return *p;
        }
        return {};
      };
    }
    case AutotuneType::FixedSizeIntervals:
    {
      int offset = half_tones_distance(Note::Do,
                                       autotune_musical_scale_root_note);
      if (offset < 0) {
        offset += num_halftones_per_octave;
      }
      offset += autotune_root_note_halftones_transpose;
      allowed_pitches.clear();
      allowed_pitches.push_back(offset);
      int const factor = autotune_factor.load();
      Assert(factor >= 0);
      if (factor) {
        for (int val = offset - factor; val > 0; val -= factor) {
          allowed_pitches.push_back(val);
        }
        for (int val = offset + factor; val < max_audible_midi_pitch; val += factor) {
          allowed_pitches.push_back(val);
        }
      }
      
      std::sort(allowed_pitches.begin(),
                allowed_pitches.end());
      
      return [this](double pitch)-> std::optional<float> {
        if (float * p = find_closest_pitch(pitch, allowed_pitches, [](float p){ return p; })) {
          return *p;
        }
        return {};
      };
    }
    case AutotuneType::MusicalScale:
      const auto * scale = &getMusicalScale(autotune_musical_scale_mode);
      return [scale,
              root_pitch = A_pitch +
              autotune_root_note_halftones_transpose +
              half_tones_distance(Note::La,
                                  autotune_musical_scale_root_note)](float const pitch) {
        return scale->closest_pitch<float>(root_pitch, pitch);
      };
  }
}

} // NS resynth
} // NS imajuscule::audio
