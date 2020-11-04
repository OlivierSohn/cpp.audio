
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
UnityGainMix<
VolumeAdjusted<
soundBufferWrapperAlgo<Sound::NOISE>
>,
VolumeAdjusted<
FOscillatorAlgo<T, FOscillator::SAW, OscillatorUsage::FilteredByLoudnessAdaptedSound>
>,
VolumeAdjusted<
FOscillatorAlgo<T, FOscillator::TRIANGLE, OscillatorUsage::FilteredByLoudnessAdaptedSound>
>,
VolumeAdjusted<
FOscillatorAlgo<T, FOscillator::SQUARE, OscillatorUsage::FilteredByLoudnessAdaptedSound>
>,
VolumeAdjusted<
SineOscillatorAlgo<T>
>,
VolumeAdjusted<
PulseTrainAlgo<T>
>
>,
InterpolatedFreq<T>
>,
AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
>,
BaseVolumeDef::One // because it's used as a carrier
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
      e.setStereoGain(stereo(stereo_spread_ * std::uniform_real_distribution<float>{-1.f,1.f}(mersenne<SEEDED::No>())));
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
                                   audioelement::AHDSR const & a,
                                   float const noise_vol,
                                   float const saw_vol,
                                   float const triangle_vol,
                                   float const square_vol,
                                   float const sine_vol,
                                   float const pulse_vol,
                                   float const pulsewidth)
  : sample_rate(sample_rate)
  , ahdsr(a)
  , noise_volume(noise_vol)
  , saw_volume(saw_vol)
  , triangle_volume(triangle_vol)
  , square_volume(square_vol)
  , sine_volume(sine_vol)
  , pulse_volume(pulse_vol)
  , pulse_width(pulsewidth)
  {}
  
  void operator()(audioelement::VocoderCarrierElement<T> & e,
                  InitializationType t = InitializationType::NewNote) const {
    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    e.getVolumeAdjustment().getOsc().getAlgo().getCtrl().setup(100,
                                                               itp::LINEAR);
    auto & oscs = e.getVolumeAdjustment().getOsc().getAlgo().getOsc().get();
    
    std::get<0>(oscs).setVolumeTarget(noise_volume);
    std::get<1>(oscs).setVolumeTarget(saw_volume);
    std::get<2>(oscs).setVolumeTarget(triangle_volume);
    std::get<3>(oscs).setVolumeTarget(square_volume);
    std::get<4>(oscs).setVolumeTarget(sine_volume);
    std::get<5>(oscs).setVolumeTarget(pulse_volume);

    std::get<5>(oscs).getOsc().getAlgo().setPulseWidth(pulse_width);
  }
  
  bool operator ==(VocoderCarrierElementInitializer const& o) const {
    return
    std::make_tuple(sample_rate, ahdsr, noise_volume, saw_volume, triangle_volume, square_volume, sine_volume, pulse_volume, pulse_width) ==
    std::make_tuple(o.sample_rate, o.ahdsr, o.noise_volume, o.saw_volume, o.triangle_volume, o.square_volume, o.sine_volume, o.pulse_volume, o.pulse_width);
  }
  bool operator !=(VocoderCarrierElementInitializer const& o) const {
    return !this->operator ==(o);
  }
  
private:
  int sample_rate;
  audioelement::AHDSR ahdsr;
  float noise_volume;
  float saw_volume;
  float triangle_volume;
  float square_volume;
  float sine_volume;
  float pulse_volume;
  float pulse_width;
};


constexpr int nAudioOut = 2;

using Synth = sine::Synth <
nAudioOut
, audioelement::ResynthElement<double>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, HandleNoteOff::Yes
, 127  // lots of voices
, ResynthElementInitializer<double>
>;

using SynthVocoderCarier = sine::Synth <
1 // mono
, audioelement::VocoderCarrierElement<double>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, HandleNoteOff::Yes
, 127  // lots of voices
, VocoderCarrierElementInitializer<double>
>;

constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

using Ctxt = Context<
AudioPlatform::PortAudio,
Features::InAndOut
>;

using FullDuplexCtxt = FullDuplexContext<
AudioPlatform::PortAudio
>;

using Stepper = SimpleAudioOutContext<
nAudioOut,
audioEnginePolicy
>;


bool constexpr logs = false;

template<typename AStepper, typename ASynth, typename Initializer>
void apply_initializer_to_current_and_future_notes(AStepper & stepper,
                                                   ASynth & synth,
                                                   Initializer const & initializer) {
  if (!synth.getSynchronousElementInitializer() || *synth.getSynchronousElementInitializer() != initializer) {
    // This will apply the new params for new notes:
    synth.setSynchronousElementInitializer(initializer);
    
    // This will apply the new params for currently played notes
    stepper.enqueueOneShot([&synth,
                            initializer](auto &, auto){
      synth.forEachRTActiveElem([initializer](auto & e) {
        initializer(e.elem, InitializationType::ExistingNote);
      });
    });
    
    // If we are unlucky, we missed the notes that are in "SoonKeyPressed" state.
  }
}

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
                       Stepper & stepper,
                       int64_t & next_noteid,
                       std::vector<PlayedNote> & played_pitches,
                       NonRealtimeAnalysisFrame & analysis_data,
                       std::atomic_int & dropped_note_on) {
  {
    ResynthElementInitializer<double> initializer{
      sample_rate,
      future_stride,
      stereo_spread,
      ahdsr
    };
    apply_initializer_to_current_and_future_notes(stepper,
                                                  synth,
                                                  initializer);
  }
  
  // issue "note off" events
  {
    int idx = -1;
    for (auto play : continue_playing) {
      ++idx;
      if (!play) {
        auto res = synth.onEvent(sample_rate,
                                 mkNoteOff(played_pitches[idx].noteid),
                                 stepper,
                                 stepper,
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
                               stepper,
                               stepper,
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
                                     stepper,
                                     stepper,
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

enum class Duplex {
  Half, // one thread receives audio input, another thread writes audio output
  Full, // the same thread receives audio input and writes audio output
};

struct RtResynth {
private:
  // each midi time source needs to have a distinct identity:
  enum class MidiSource {
    Analysis,
    MidiInput
  };

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

  using MidiQueueItem = folly::Function<void(midi::PortMidi &)>;
  using MidiQueue = atomic_queue::AtomicQueueB2<
  /* T = */ MidiQueueItem,
  /* A = */ std::allocator<MidiQueueItem>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;

public:
  // Mode::Realtime
  RtResynth(int const sample_rate);

  // Mode::Offline
  RtResynth(RtResynthOfflineJob & job);

private:
  enum class Mode {
    Realtime,
    Offline
  };
  RtResynth(Mode m,
            int const sample_rate,
            Postprocessing p);

public:
  ~RtResynth();

  void init();
  
  void teardown();
  
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
  
  // we need an even window size because we use 2 half windows to represent a full window
  int getEvenWindowSizeFrames() const {
    return 2 * std::max(1,
                        sample_rate ? static_cast<int>(0.5 * window_size_seconds * sample_rate) : 0);
  }
  float getWindowSizeSeconds() const {
    return window_size_seconds;
  }
  float getEffectiveWindowSizeSeconds() const {
    if (!sample_rate) {
      return 0.f;
    }
    return std::max(getWindowSizeSeconds(),
                    2.f/sample_rate);
  }
  float getWindowCenterStrideSeconds() const {
    return window_center_stride_seconds;
  }
  float getEffectiveWindowCenterStrideSeconds() const {
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
  void storeAudioOutputQueueFillRatio(float f) {
    output_queue_fill_ratio = f;
  }
  
  float getAudioInputQueueFillRatio() const {
    return input_queue_fill_ratio;
  }
  float getAudioOutputQueueFillRatio() const {
    return output_queue_fill_ratio;
  }
  int countDroppedOutputFrames() const {
    int count = 0;
    if (output_2_analysis_queue) {
      count += output_2_analysis_queue->countDroppedInputFrames();
    }
    return 0;
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
    return stepper.countFailedComputeInsertions();
  }
  int countRetriedOneshotInsertions() const {
    return stepper.countRetriedOneshotInsertions();
  }
  
  std::optional<float> getDurationFft() {
    return periodic_fft.getDurationFft();
  }
  std::optional<float> getDurationCopy() {
    return periodic_fft.getDurationCopy();
  }
  std::optional<float> getDurationExtract() {
    float const f = dt_extract_seconds;
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
  
  NonRealtimeAnalysisFrame const & getAnalysisData() const {
    return analysis_data;
  }
  
  Vocoder const & getVocoder() const { return vocoder; }

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
    updateVocoderCarrierInitializer();
  }
  float getEnvHoldSeconds() const {
    return env_hold_seconds;
  }
  void setEnvHoldSeconds(float f) {
    env_hold_seconds = f;
    updateVocoderCarrierInitializer();
  }
  float getEnvDecaySeconds() const {
    return env_decay_seconds;
  }
  void setEnvDecaySeconds(float f) {
    env_decay_seconds = f;
    updateVocoderCarrierInitializer();
  }
  float getEnvReleaseSeconds() const {
    return env_release_seconds;
  }
  void setEnvReleaseSeconds(float f) {
    env_release_seconds = f;
    updateVocoderCarrierInitializer();
  }
  float getEnvSustainLevel() const {
    return env_sustain_level;
  }
  void setEnvSustainLevel(float f) {
    env_sustain_level = f;
    updateVocoderCarrierInitializer();
  }

  float getVocoderCarrierNoiseVolume() const {
    return vocoder_carrier_noise_volume;
  }
  void setVocoderCarrierNoiseVolume(float f) {
    vocoder_carrier_noise_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierSawVolume() const {
    return vocoder_carrier_saw_volume;
  }
  void setVocoderCarrierSawVolume(float f) {
    vocoder_carrier_saw_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierTriangleVolume() const {
    return vocoder_carrier_triangle_volume;
  }
  void setVocoderCarrierTriangleVolume(float f) {
    vocoder_carrier_triangle_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierSquareVolume() const {
    return vocoder_carrier_square_volume;
  }
  void setVocoderCarrierSquareVolume(float f) {
    vocoder_carrier_square_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierSineVolume() const {
    return vocoder_carrier_sine_volume;
  }
  void setVocoderCarrierSineVolume(float f) {
    vocoder_carrier_sine_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierPulseVolume() const {
    return vocoder_carrier_pulse_volume;
  }
  void setVocoderCarrierPulseVolume(float f) {
    vocoder_carrier_pulse_volume = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderCarrierPulseWidth() const {
    return vocoder_carrier_pulse_width;
  }
  void setVocoderCarrierPulseWidth(float f) {
    vocoder_carrier_pulse_width = f;
    updateVocoderCarrierInitializer();
  }
  float getVocoderEnvFollowerCutoffRatio() const {
    return vocoder_env_follower_cutoff_ratio;
  }
  void setVocoderEnvFollowerCutoffRatio(float f) {
    vocoder_env_follower_cutoff_ratio = f;
  }
  float getVocoderModulatorWindowSizeSeconds() const {
    return vocoder_modulator_window_size_seconds;
  }
  void setVocoderModulatorWindowSizeSeconds(float f) {
    vocoder_modulator_window_size_seconds = f;
  }
  float getVocoderStrideSeconds() const {
    return vocoder_stride_seconds;
  }
  void setVocoderStrideSeconds(float f) {
    vocoder_stride_seconds = f;
  }
  int getVocoderBandsCount() const {
    return vocoder_count_bands;
  }
  void setVocoderBandsCount(int f) {
    vocoder_count_bands = f;
  }
  float getVocoderMinFreq() const {
    return vocoder_min_freq;
  }
  void setVocoderMinFreq(float f) {
    vocoder_min_freq = f;
  }
  float getVocoderMaxFreq() const {
    return vocoder_max_freq;
  }
  void setVocoderMaxFreq(float f) {
    vocoder_max_freq = f;
  }

  float getPitchWheelMultiplier() const {
    return pitch_wheel_multiplier;
  }
  void setPitchWheelMultiplier(float f) {
    pitch_wheel_multiplier = f;
  }
  
  float getDirectVoiceVolume() const {
    return voice_volume;
  }
  void setDirectVoiceVolume(float f) {
    voice_volume = f;
  }
  float getCarrierVolume() const {
    return carrier_volume;
  }
  void setCarrierVolume(float f) {
    carrier_volume = f;
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
  
  float getAnalysisInputGain() const {
    return analysis_input_gain;
  }
  void setAnalysisInputGain(float f) {
    analysis_input_gain = f;
  }
  float getAnalysisFeedbackGain() const {
    return analysis_output_feedback_gain;
  }
  void setAnalysisFeedbackGain(float f) {
    analysis_output_feedback_gain = f;
  }
  float getAnalysisFeedbackDelay() const {
    return output_delay_seconds;
  }
  void setAnalysisFeedbackDelay(float f) {
    output_delay_seconds = f;
  }
  
  float getInputStreamCpuLoad() const {
    if (mode == Duplex::Half) {
      return input.getStreamCpuLoad();
    } else if (mode == Duplex::Full) {
      return fullduplex.getStreamCpuLoad();
    }
    return -1.f;
  }
  float getOutputStreamCpuLoad() const {
    if (mode == Duplex::Half) {
      return ctxt.getStreamCpuLoad();
    } else if (mode == Duplex::Full) {
      return fullduplex.getStreamCpuLoad();
    }
    return -1.f;
  }

  float getCompressionFactor() const {
    return limiter.getTargetCompressionLevel();
  }
  
  void saveAsPreset(Preset &) const;
  void restorePreset(Preset const&);

  std::optional<MidiName> getMidiInput();
  void setMidiInput(std::optional<MidiName> const &);
  std::vector<std::optional<MidiName>> listMidiInputs();

private:
  Mode run_mode;
  std::optional<Duplex> mode;
  FullDuplexCtxt fullduplex;
  Ctxt ctxt;

  Stepper stepper;
  Limiter<double> limiter;
  Synth synth;
  SynthVocoderCarier vocoder_carrier;
  NoteIdsGenerator vocoder_carrier_noteids;
  Midi midi;
  int sample_rate = 0;
  
  std::atomic_bool thread_resynth_active{false};
  std::unique_ptr<std::thread> thread_resynth;
  
  AudioBufferAggregator<Queue> analysis_input_aggregator;
  
  MetaQueue<Queue> * input_2_analysis_queue = nullptr;
  MetaQueue<Queue> * output_2_analysis_queue = nullptr;

  // Only for Duplex::Half mode:
  MetaQueue<Queue> * input_2_vocoder_queue = nullptr;
  ReadQueuedSampleSource<Queue> read_queued_input;
  
  // Only for Duplex::Full mode:
  const float * direct_input = nullptr;
  
  Vocoder vocoder;
  
  AudioInput<AudioPlatform::PortAudio> input;

  AudioBufferPubSub<Queue> input_pubsub;
  AudioBufferPubSub<Queue> output_pubsub;

  NonRealtimeAnalysisFrame analysis_data;
  int64_t analysis_frame_idx = 0;
  
  //
  // Begin params
  //
  std::atomic<float> pitch_wheel_multiplier = 2.f;
  
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
  
  std::atomic<float> vocoder_carrier_noise_volume = 0.f;
  std::atomic<float> vocoder_carrier_saw_volume = 0.f;
  std::atomic<float> vocoder_carrier_triangle_volume = 0.f;
  std::atomic<float> vocoder_carrier_square_volume = 1.f;
  std::atomic<float> vocoder_carrier_sine_volume = 0.f;
  std::atomic<float> vocoder_carrier_pulse_volume = 0.f;
  std::atomic<float> vocoder_carrier_pulse_width = 0.01f;
  std::atomic<float> vocoder_env_follower_cutoff_ratio = 1.f/20.f;
  std::atomic<float> vocoder_modulator_window_size_seconds = 0.10f;
  std::atomic<float> vocoder_stride_seconds = 0.005f;
  std::atomic<int> vocoder_count_bands = 5;
  std::atomic<float> vocoder_min_freq = 100.f;
  std::atomic<float> vocoder_max_freq = 20000.f;
  
  std::atomic<float> voice_volume = 0.f;
  std::atomic<float> carrier_volume = 0.1f;
  std::atomic<float> vocoder_volume = 0.f;
  std::atomic<float> analysis_volume = 0.f;
  
  std::atomic<float> analysis_input_gain = 1.f;
  std::atomic<float> analysis_output_feedback_gain = 0.f;
  std::atomic<float> output_delay_seconds = 1.f;
  //
  // End params
  //
  
  cyclic<float> output_delay;
  
  std::vector<float> allowed_pitches;
  
  int64_t next_noteid = 0;
  
  PeriodicFFT<SqMagFftOperation<Window::Gaussian, double>> periodic_fft;
    
  std::vector<FreqMag<double>> freqmags;
  std::vector<PitchVolume> freqmags_data;
  std::vector<PitchInterval> pitch_intervals;
  std::vector<PitchVolume> reduced_pitches, autotuned_pitches, pitches_tmp;
  std::vector<float> autotuned_pitches_perceived_loudness;
  std::vector<int> autotuned_pitches_idx_sorted_by_perceived_loudness;
  std::vector<PlayedNote> played_pitches;
  std::vector<std::optional<int>> pitch_changes;
  std::vector<bool> continue_playing; // indexed like 'played_pitches'
  
  std::atomic<float> dt_extract_seconds = -1.f;
  std::atomic<float> dt_step_seconds = -1.f;
  std::atomic<float> input_queue_fill_ratio = 0.f;
  std::atomic<float> output_queue_fill_ratio = 0.f;
  std::atomic_int dropped_note_on = 0;
  
  std::atomic_bool midi_thread_active = true;
  std::unique_ptr<std::thread> midi_thread;
  std::mutex midi_mutex;
  std::unique_ptr<midi::PortMidi> portmidi;
  
  MidiQueue midi_queue;
  
  std::atomic_bool preset_autosave_thread_active;
  std::unique_ptr<std::thread> preset_autosave_thread;
  
  uint64_t analysis_frames_counter = 0;
  
  RtResynthOfflineJob * job = nullptr;

  static constexpr char * autosave_preset_file = "autosave.json";
public:
  static constexpr char * default_preset_file = "default.json";
private:
  
  void init_post(Postprocessing p);
  
  RecordF input_func();
  RecordF input_func_with_direct_input();
  PlayF output_func();
  
  void init_analysis();
  void analyze_until_input_starvation();
  
  void step (std::vector<FreqMag<double>> const & fs,
             MIDITimestampAndSource const & miditime,
             int const future_stride);
  
  std::function<std::optional<float>(float const)>
  mkAutotuneFunction();
  
  void onInputMidiEvent(std::optional<uint64_t> const & time_ms,
                        midi::Event const & e);

  void updateVocoderCarrierInitializer();
  
  // must be called from the same thread that has previously writen 'direct_input'
  std::optional<std::pair<double, SampleContinuity>> read_direct_input() {
    Assert(direct_input);
    double const val = *direct_input;
    ++direct_input;
    return std::make_pair(val,
                          SampleContinuity::Yes);
  }
};


RtResynth::RtResynth(Mode runmode,
                     int const samplerate,
                     Postprocessing p)
: run_mode(runmode)
, sample_rate(samplerate)
, synth()
, stepper(GlobalAudioLock<audioEnginePolicy>::get(),
          Synth::n_channels * 4 /* one shot */,
          1 // fft-based synth
          + 1    //vocoder
          )
, input_pubsub(4, // num oneshots max
               4) // num queues max
, output_pubsub(4, // num oneshots max
                4) // num queues max
, periodic_fft(pow2(14))
, vocoder_carrier_noteids(150)
, midi_queue(10) // 10 should be enough
{
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

  init_post(p);
  
  switch(run_mode) {
    case Mode::Realtime:
    {
      portmidi = std::make_unique<midi::PortMidi>([this](midi::Event const & midievent, std::optional<uint64_t> time_ms){
        onInputMidiEvent(time_ms, midievent);
      });
      
      // Save "default" preset
      Preset p;
      saveAsPreset(p);
      try {
        writeToJsonFile(default_preset_file,
                        p);
      } catch (std::exception const & e) {
        std::cerr << "writePresetToFile : " << e.what() << std::endl;
      }
      
      // Restore autosave preset
      {
        Preset p;
        try {
          readFromJsonFile(autosave_preset_file,
                           p);
        } catch (std::exception const & e) {
          std::cerr << "readPresetFromFile : " << e.what() << std::endl;
        }
        restorePreset(p);
      }
      
      // Periodically save autosave preset
      preset_autosave_thread_active = true;
      preset_autosave_thread = std::make_unique<std::thread>([this](){
        while(preset_autosave_thread_active) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          Preset p;
          saveAsPreset(p);
          try {
            writeToJsonFile(autosave_preset_file,
                            p);
          } catch (std::exception const & e) {
            std::cerr << "writePresetToFile : " << e.what() << std::endl;
          }
        }
      });
      
      // generate the static sounbBuffer for noises, to avoid a delay on the first played note
      getPinkNoise(sample_rate);
      getPinkNoiseAbsMean(sample_rate);
      getWhiteNoise(sample_rate);
      getWhiteNoiseAbsMean(sample_rate);
    }
    case Mode::Offline:
      break;
  }

#ifndef NDEBUG
  testAutotune();
#endif
}

RtResynth::RtResynth(int const samplerate)
: RtResynth(Mode::Realtime,
            samplerate,
            Postprocessing::Limit)
{}

// we run the job in the constructor
RtResynth::RtResynth(RtResynthOfflineJob & j)
: RtResynth(Mode::Offline,
            j.get_samplerate(),
            j.getPostprocessing())
{
  Assert(j.count_outputs() == nAudioOut);
  
  restorePreset(j.getPreset());
  
  this->job = &j;

  init();
  
  RecordF input_f = input_func_with_direct_input();
  PlayF output_f = output_func();
  int zero_since = 0;

  double const nanoseconds_increment = 1000000000. / sample_rate;
  int64_t frame = 0;
  do {
    float voice;
    j.read_voice(voice);
    
    input_f(&voice,
            1);
    
    analyze_until_input_starvation();
    
    // TODO use doubles to keep the 64 bits internal precision
    float output[nAudioOut];
    output_f(output,
             1,
             static_cast<uint64_t>(nanoseconds_increment * frame));
    ++frame;
    
    j.write_output(output,
                   nAudioOut);
    
    double sumAbs{};
    for (auto o : output) {
      sumAbs += std::abs(o);
    }
    if (sumAbs < 0.000001) {
      ++zero_since;
    } else {
      zero_since = 0;
    }
  } while (j.has_more_voice() ||
           j.has_more_carrier() ||
           zero_since > sample_rate);
}

RtResynth::~RtResynth() {
  teardown();

  preset_autosave_thread_active = false;
  if (preset_autosave_thread) {
    preset_autosave_thread->join();
  }
}


void RtResynth::init_post(Postprocessing p) {
  std::vector<postProcessFunc> v;
  switch(p) {
    case Postprocessing::None:
      break;
    case Postprocessing::Limit:
      v.emplace_back([this](double * buf, int const nFrames, int const blockSize) {
        for (int i=0; i<nFrames; ++i) {
          CArray<nAudioOut, double> a{buf + i*nAudioOut};
          limiter.feedOneFrame(a);
        }
        // by now, the signal is compressed and limited
      });
      break;
  }
  
  v.emplace_back([this](double * buf, int const nFrames, int const blockSize) {
    output_pubsub.update_rt_listeners();
    for (int i=0; i<nFrames; ++i) {
      // make a mono version of the signal
      double sum {};
      for (int j=0; j<nAudioOut; ++j) {
        sum += buf[i*nAudioOut + j];
      }
      output_pubsub.try_publish_sample(sum);
    }
  });

  stepper.getPost().set_post_processors(std::move(v));
}

RecordF RtResynth::input_func() {
  return [this](const float * buf, int nFrames){
    input_pubsub.try_publish_buffer(buf,
                                    nFrames);
  };
}

RecordF RtResynth::input_func_with_direct_input() {
  return[this](const float * buf, int nFrames){
    direct_input = buf;
    input_pubsub.try_publish_buffer(buf,
                                    nFrames);
  };
}

PlayF RtResynth::output_func() {
  return [this,
          nanos_per_audioelement_buffer = static_cast<uint64_t>(0.5f +
                                                                audio::nanos_per_frame<float>(sample_rate) *
                                                                static_cast<float>(audio::audioelement::n_frames_per_buffer))]
  (SAMPLE *outputBuffer,
   int nFrames,
   uint64_t const tNanos){
    stepper.step(outputBuffer,
                 nFrames,
                 tNanos,
                 nanos_per_audioelement_buffer);
  };
}


void
RtResynth::init() {
  teardown();
  Assert(!thread_resynth_active);
  
  if (run_mode == Mode::Realtime) {
    if (fullduplex.Init(sample_rate,
                        minInLatency,
                        1,
                        input_func_with_direct_input(),
                        minOutLatency,
                        nAudioOut,
                        output_func())) {
      if (sample_rate != fullduplex.getSampleRate()) {
        throw std::runtime_error("sample rate mismatch");
      }
      mode = Duplex::Full;
    } else {
      // no fullduplex, fallback to separate buffers
      if (!ctxt.doInit(minOutLatency,
                       sample_rate,
                       nAudioOut,
                       output_func())) {
        throw std::runtime_error("ctxt init failed");
      }
      
      // This sleep is necessary, else no sound is audible
      // (maybe a portaudio bug?)
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      
      if (!input.Init(input_func(),
                      sample_rate,
                      minInLatency)) {
        throw std::runtime_error("input init failed");
      }
      if (sample_rate != ctxt.getSampleRate()) {
        throw std::runtime_error("sample rate mismatch");
      }
      if (input.getSampleRate() != sample_rate) {
        throw std::runtime_error("in and out sample rates mismatch");
      }
      mode = Duplex::Half;
    }
  } else {
    // for offline mode we use full duplex
    mode = Duplex::Full;
  }
  
  if (!synth.initialize(stepper)) {
    throw std::logic_error("failed to initialize synth");
  }

  input_2_analysis_queue = input_pubsub.add_listener(sample_rate); // one second of input can fit in the queue
  output_2_analysis_queue = output_pubsub.add_listener(sample_rate); // one second of input can fit in the queue
  analysis_input_aggregator.add_stream(input_2_analysis_queue->queue);
  analysis_input_aggregator.add_stream(output_2_analysis_queue->queue);

  if (mode == Duplex::Half) {
    Assert(ctxt.Initialized());
    Assert(input.Initialized());
    int size = sample_rate * 4. * std::max(input.getInputLatencySeconds(),
                                           ctxt.getOutputLatencySeconds());
    std::cout << "Vocoder queue size = " << size << std::endl;
    input_2_vocoder_queue = input_pubsub.add_listener(size);

    read_queued_input.set(input_2_vocoder_queue->queue
#ifndef NDEBUG
                          , ctxt.asyncLogger()
#endif
                          );
  }

  vocoder.initialize(stepper,
                     sample_rate,
                     [this] () -> Vocoder::Params{
    return
    {
      {
        voice_volume.load(),
        carrier_volume.load(),
        vocoder_volume.load()
      },
      {
        vocoder_env_follower_cutoff_ratio.load(),
        vocoder_modulator_window_size_seconds.load(),
        vocoder_stride_seconds.load(),
        vocoder_count_bands.load(),
        vocoder_min_freq.load(),
        vocoder_max_freq.load()
      }
    };
  },
                     [this] () -> std::pair<std::optional<std::pair<double, SampleContinuity>>, std::optional<std::pair<double, SampleContinuity>>> {
    std::optional<std::pair<double, SampleContinuity>> mod;
    if (auto const m = mode) {
      mod =
      (*m == Duplex::Half) ?
      read_queued_input() :
      read_direct_input();
    }
    
    double carrier_val{};
    if (run_mode == Mode::Realtime) {
      vocoder_carrier.compute(&carrier_val,
                              1);
    } else {
      Assert(job);
      job->read_carrier(carrier_val);
    }
        
    return std::make_pair(mod,
                          std::make_pair(carrier_val,
                                         SampleContinuity::Yes));
  });
  
  updateVocoderCarrierInitializer();

  if (portmidi) {
    midi_thread_active = true;
    midi_thread = std::make_unique<std::thread>([this](){
      midi::listen_to_midi_input(midi_thread_active,
                                 midi_queue,
                                 *portmidi);
    });
  }

  {
    init_analysis();
    
    if (run_mode == Mode::Realtime) {
      thread_resynth_active = true;
      thread_resynth = std::make_unique<std::thread>([this](){
        while (thread_resynth_active) {
          analyze_until_input_starvation();
          std::this_thread::yield(); // should we sleep?
        }
      });
    }
  }
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
  
  midi_thread_active = false;
  if (midi_thread) {
    midi_thread->join();
    midi_thread.reset();
  }
  
  vocoder.finalize(stepper);
  synth.finalize(stepper);
  
  analysis_input_aggregator.reset();

  if (input_2_analysis_queue) {
    input_pubsub.remove_listener(input_2_analysis_queue);
    input_2_analysis_queue = nullptr;
  }
  
  if (output_2_analysis_queue) {
    output_pubsub.remove_listener(output_2_analysis_queue);
    output_2_analysis_queue = nullptr;
  }
  
  if (input_2_vocoder_queue) {
    input_pubsub.remove_listener(input_2_vocoder_queue);
    input_2_vocoder_queue = nullptr;
  }
  
  if (input.Initialized()) {
    if (!input.Teardown()) {
      throw std::runtime_error("input teardown failed");
    }
  }
  if (ctxt.Initialized()) {
    ctxt.doTearDown();
  }
  if (fullduplex.Initialized()) {
    fullduplex.Teardown();
  }
  mode.reset();
}

void RtResynth::updateVocoderCarrierInitializer() {
  VocoderCarrierElementInitializer<double> initializer(sample_rate,
                                                       mkAHDSR(sample_rate,
                                                               env_attack_seconds,
                                                               env_hold_seconds,
                                                               env_decay_seconds,
                                                               env_release_seconds,
                                                               env_sustain_level),
                                                       vocoder_carrier_noise_volume.load(),
                                                       vocoder_carrier_saw_volume.load(),
                                                       vocoder_carrier_triangle_volume.load(),
                                                       vocoder_carrier_square_volume.load(),
                                                       vocoder_carrier_sine_volume.load(),
                                                       vocoder_carrier_pulse_volume.load(),
                                                       vocoder_carrier_pulse_width.load()
                                                       );
  apply_initializer_to_current_and_future_notes(stepper,
                                                vocoder_carrier,
                                                initializer);
}

void RtResynth::onInputMidiEvent(std::optional<uint64_t> const &time_ms,
                                 midi::Event const & e) {
  if (std::holds_alternative<midi::NoteOn>(e)) {
    Assert(time_ms); // the only case we have no time is for AllNotesOff
    // Portmidi time is in milliseconds, we convert it to nanoseconds.
    uint64_t const time_nanos = 1000000 * *time_ms;
    midi::NoteOn const & on = std::get<midi::NoteOn>(e);
    Assert(on.velocity);
    vocoder_carrier.onEvent(sample_rate,
                            mkNoteOn(vocoder_carrier_noteids.NoteOnId(on.key),
                                     midi.midi_pitch_to_freq(on.key),
                                     on.velocity / 127.f),
                            stepper,
                            stepper,
                            MIDITimestampAndSource{time_nanos, to_underlying(MidiSource::MidiInput)});
  } else if (std::holds_alternative<midi::NoteOff>(e)) {
    Assert(time_ms); // the only case we have no time is for AllNotesOff
    // Portmidi time is in milliseconds, we convert it to nanoseconds.
    uint64_t const time_nanos = 1000000 * *time_ms;
    midi::NoteOff const & off = std::get<midi::NoteOff>(e);
    vocoder_carrier.onEvent(sample_rate,
                            mkNoteOff(vocoder_carrier_noteids.NoteOffId(off.key)),
                            stepper,
                            stepper,
                            MIDITimestampAndSource{time_nanos, to_underlying(MidiSource::MidiInput)});
  } else if (std::holds_alternative<midi::AllNotesOff>(e)) {
    // send explicit note-offs for every note currently playing
    for (auto const & [key, note_id] : vocoder_carrier_noteids) {
      vocoder_carrier.onEvent(sample_rate,
                              mkNoteOff(note_id),
                              stepper,
                              stepper,
                              {});
    }
    vocoder_carrier_noteids.clear();
    vocoder_carrier.allNotesOff(stepper); // this is redundant, but... "just in case"
  } else if (std::holds_alternative<midi::KeyPressure>(e)) {
    std::cout << "todo use keypressure" << std::endl;
  } else if (std::holds_alternative<midi::ChannelPressure>(e)) {
    std::cout << "todo use channelpressure" << std::endl;
  } else if (std::holds_alternative<midi::PitchWheel>(e)) {
    float const half_tone_offset = pitch_wheel_multiplier * std::get<midi::PitchWheel>(e).getCenteredValue();
    float const factor = std::pow(midi.getHalfToneRatio(),
                                  half_tone_offset);
    // we apply the effect on both synths.
    vocoder_carrier.onAngleIncrementMultiplier(stepper,
                                               factor);
    synth.onAngleIncrementMultiplier(stepper,
                                     factor);
  }
}

void RtResynth::init_analysis() {
  double const nanos_per_frame = 1. / static_cast<double>(sample_rate);
  uint64_t local_count_dropped_input_frames = 0;
  
  periodic_fft.setLambdas([this]() {return getEvenWindowSizeFrames(); },
                          [this]() {return std::max(1,
                                                    static_cast<int>(0.5f + getWindowCenterStrideSeconds() * sample_rate)); },
                          [this, nanos_per_frame, &local_count_dropped_input_frames]
                          (int const window_center_stride,
                           FrequenciesSqMag<double> const & frequencies_sqmag) {
    if (!thread_resynth_active) {
      return;
    }
    
    {
      std::optional<float> duration_extract_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        
        extractLocalMaxFreqsMags(sample_rate / decltype(periodic_fft)::FftOp::windowed_signal_stride,
                                 frequencies_sqmag,
                                 SqMagToDb<double>(),
                                 freqmags);
      }
      if (dt) duration_extract_seconds = dt->count() / 1000000.;
      setDuration(duration_extract_seconds, dt_extract_seconds);
    }
    
    {
      std::optional<float> duration_step_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        
        step(freqmags,
             MIDITimestampAndSource((analysis_frames_counter + local_count_dropped_input_frames) * nanos_per_frame,
                                    to_underlying(MidiSource::Analysis)),
             window_center_stride); // we pass the future stride, on purpose
      }
      if (dt) duration_step_seconds = dt->count() / 1000000.;
      setDuration(duration_step_seconds, dt_step_seconds);
    }
    
    storeAudioOutputQueueFillRatio(output_2_analysis_queue->queue.was_size() / static_cast<float>(output_2_analysis_queue->queue.capacity()));
    storeAudioInputQueueFillRatio(input_2_analysis_queue->queue.was_size() / static_cast<float>(input_2_analysis_queue->queue.capacity()));
  });
}

void RtResynth::analyze_until_input_starvation() {
  while (analysis_input_aggregator.try_pop()) {
    float sum{};
    bool dropped = false;
    int idx = -1;
    for (auto const & var : analysis_input_aggregator.get_data()) {
      ++idx;
      if (std::holds_alternative<CountDroppedFrames>(var)) {
        dropped = true;
      } else {
        Assert(std::holds_alternative<InputSample>(var));
        if (idx == 0) {
          sum += analysis_input_gain * std::get<InputSample>(var).value;
        } else {
          int const sz = std::max(0,
                                  static_cast<int>(0.5f + output_delay_seconds * sample_rate));
          if (static_cast<int>(output_delay.size()) != sz) {
            output_delay.resize(sz); // this also zeroes
          }
          if (sz) {
            auto value = *output_delay.cycleEnd();
            sum += analysis_output_feedback_gain * value;
            output_delay.feed(std::get<InputSample>(var).value);
          } else {
            sum += analysis_output_feedback_gain * std::get<InputSample>(var).value;
          }
        }
      }
    }
    if (dropped) {
      std::cout << "stop analysis thread : frames dropped" << std::endl;
      return;
      /*
       if (!thread_resynth_active) {
       return;
       }
       periodic_fft.on_dropped_frames(std::optional<int>{});
       analysis_input_aggregator.resync();
       continue;
       */
    }
    ++analysis_frames_counter;
    periodic_fft.feed(sum);
  }
}

void
RtResynth::step (std::vector<FreqMag<double>> const & fs,
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
                    stepper,
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


std::optional<MidiName> RtResynth::getMidiInput(){
  if (!midi_thread_active) {
    return {};
  }
  std::optional<MidiName> res;
  bool called = false;
  if (!midi_queue.try_push([this, &res, &called](midi::PortMidi & pm){
    std::lock_guard l(midi_mutex);
    res = pm.get_input_stream();
    called = true;
  })) {
    std::cerr << "getMidiInput : midi queue full" << std::endl;
    return {};
  }
  while(true) {
    std::this_thread::yield();
    std::lock_guard l(midi_mutex);
    if (called) {
      return res;
    }
  }
}

void RtResynth::setMidiInput(std::optional<MidiName> const & n){
  if (!midi_thread_active) {
    return;
  }
  bool called = false;
  if (!midi_queue.try_push([this, &n, &called](midi::PortMidi & pm){
    std::lock_guard l(midi_mutex);
    if (n) {
      if (n != pm.get_input_stream()) {
        pm.open_input_stream(*n);
      }
    } else {
      pm.close_input_stream();
    }
    called = true;
  })) {
    std::cerr << "setMidiInput : midi queue full" << std::endl;
    return;
  }
  while(true) {
    std::this_thread::yield();
    std::lock_guard l(midi_mutex);
    if (called) {
      return;
    }
  }
}

std::vector<std::optional<MidiName>> RtResynth::listMidiInputs(){
  Assert(portmidi);
  
  std::vector<MidiName> names = portmidi->list_input_devices_names();
  std::vector<std::optional<MidiName>> res;
  res.reserve(names.size() + 1);
  res.push_back({});
  for (auto const & n : names) {
    res.push_back(n);
  }
  return res;
}


void RtResynth::saveAsPreset(Preset & p) const {
  p.clear();
  
  p.set("pitch_wheel_multiplier",
        pitch_wheel_multiplier);
  
  p.set("window_size_seconds",
        window_size_seconds);
  p.set("window_center_stride_seconds",
        window_center_stride_seconds);
  p.set("min_volume",
        min_volume);
  p.set("nearby_distance_tones",
        nearby_distance_tones);
  p.set("max_track_pitches",
        max_track_pitches);
  p.set("autotune_tolerance_pitches",
        autotune_tolerance_pitches);
  p.set("pitch_shift_pre_autotune",
        pitch_shift_pre_autotune);
  p.set("pitch_shift_post_autotune",
        pitch_shift_post_autotune);
  p.set("pitch_harmonize_pre_autotune",
        pitch_harmonize_pre_autotune);
  p.set("pitch_harmonize_post_autotune",
        pitch_harmonize_post_autotune);
  p.set("stereo_spread",
        stereo_spread);
  
  p.set("env_attack_seconds",
        env_attack_seconds);
  p.set("env_hold_seconds",
        env_hold_seconds);
  p.set("env_decay_seconds",
        env_decay_seconds);
  p.set("env_release_seconds",
        env_release_seconds);
  p.set("env_sustain_level",
        env_sustain_level);
  
  p.set("autotune_type",
        autotune_type.load());
  p.set("use_autotune",
        use_autotune);
  p.set("autotune_max_pitch",
        autotune_max_pitch);
  p.set("autotune_factor",
        autotune_factor);
  p.set("autotune_musical_scale_mode",
        autotune_musical_scale_mode.load());
  p.set("autotune_musical_scale_root_note",
        autotune_musical_scale_root_note.load());
  p.set("autotune_root_note_halftones_transpose",
        autotune_root_note_halftones_transpose);
  p.set("autotune_chord_frequencies",
        autotune_chord_frequencies.load());
  p.set("autotune_bit_chord",
        autotune_bit_chord);
  
  p.set("vocoder_carrier_noise_volume",
        vocoder_carrier_noise_volume);
  p.set("vocoder_carrier_saw_volume",
        vocoder_carrier_saw_volume);
  p.set("vocoder_carrier_triangle_volume",
        vocoder_carrier_triangle_volume);
  p.set("vocoder_carrier_square_volume",
        vocoder_carrier_square_volume);
  p.set("vocoder_carrier_sine_volume",
        vocoder_carrier_sine_volume);
  p.set("vocoder_carrier_pulse_volume",
        vocoder_carrier_pulse_volume);
  p.set("vocoder_carrier_pulse_width",
        vocoder_carrier_pulse_width);
  p.set("vocoder_env_follower_cutoff_ratio",
        vocoder_env_follower_cutoff_ratio);
  p.set("vocoder_modulator_window_size_seconds",
        vocoder_modulator_window_size_seconds);
  p.set("vocoder_stride_seconds",
        vocoder_stride_seconds);
  p.set("vocoder_count_bands",
        vocoder_count_bands);
  p.set("vocoder_min_freq",
        vocoder_min_freq);
  p.set("vocoder_max_freq",
        vocoder_max_freq);
  
  p.set("voice_volume",
        voice_volume);
  p.set("carrier_volume",
        carrier_volume);
  p.set("vocoder_volume",
        vocoder_volume);
  p.set("analysis_volume",
        analysis_volume);

  p.set("analysis_input_gain",
        analysis_input_gain);
  p.set("analysis_output_feedback_gain",
        analysis_output_feedback_gain);
  p.set("output_delay_seconds",
        output_delay_seconds);
}

void RtResynth::restorePreset(Preset const& p) {
  p.read("pitch_wheel_multiplier",
         pitch_wheel_multiplier);
  
  p.read("window_size_seconds",
         window_size_seconds);
  p.read("window_center_stride_seconds",
         window_center_stride_seconds);
  p.read("min_volume",
         min_volume);
  p.read("nearby_distance_tones",
         nearby_distance_tones);
  p.read("max_track_pitches",
         max_track_pitches);
  p.read("autotune_tolerance_pitches",
         autotune_tolerance_pitches);
  p.read("pitch_shift_pre_autotune",
         pitch_shift_pre_autotune);
  p.read("pitch_shift_post_autotune",
         pitch_shift_post_autotune);
  p.read("pitch_harmonize_pre_autotune",
         pitch_harmonize_pre_autotune);
  p.read("pitch_harmonize_post_autotune",
         pitch_harmonize_post_autotune);
  p.read("stereo_spread",
         stereo_spread);
  
  p.read("env_attack_seconds",
         env_attack_seconds);
  p.read("env_hold_seconds",
         env_hold_seconds);
  p.read("env_decay_seconds",
         env_decay_seconds);
  p.read("env_release_seconds",
         env_release_seconds);
  p.read("env_sustain_level",
         env_sustain_level);
  
  p.read("autotune_type",
         autotune_type);
  p.read("use_autotune",
         use_autotune);
  p.read("autotune_max_pitch",
         autotune_max_pitch);
  p.read("autotune_factor",
         autotune_factor);
  p.read("autotune_musical_scale_mode",
         autotune_musical_scale_mode);
  p.read("autotune_musical_scale_root_note",
         autotune_musical_scale_root_note);
  p.read("autotune_root_note_halftones_transpose",
         autotune_root_note_halftones_transpose);
  p.read("autotune_chord_frequencies",
         autotune_chord_frequencies);
  p.read("autotune_bit_chord",
         autotune_bit_chord);
  
  p.read("vocoder_carrier_noise_volume",
         vocoder_carrier_noise_volume);
  p.read("vocoder_carrier_saw_volume",
         vocoder_carrier_saw_volume);
  p.read("vocoder_carrier_triangle_volume",
         vocoder_carrier_triangle_volume);
  p.read("vocoder_carrier_square_volume",
         vocoder_carrier_square_volume);
  p.read("vocoder_carrier_sine_volume",
         vocoder_carrier_sine_volume);
  p.read("vocoder_carrier_pulse_volume",
         vocoder_carrier_pulse_volume);
  p.read("vocoder_carrier_pulse_width",
         vocoder_carrier_pulse_width);
  p.read("vocoder_env_follower_cutoff_ratio",
         vocoder_env_follower_cutoff_ratio);
  p.read("vocoder_modulator_window_size_seconds",
         vocoder_modulator_window_size_seconds);
  p.read("vocoder_stride_seconds",
         vocoder_stride_seconds);
  p.read("vocoder_count_bands",
         vocoder_count_bands);
  p.read("vocoder_min_freq",
         vocoder_min_freq);
  p.read("vocoder_max_freq",
         vocoder_max_freq);
  
  p.read("voice_volume",
         voice_volume);
  p.read("carrier_volume",
         carrier_volume);
  p.read("vocoder_volume",
         vocoder_volume);
  p.read("analysis_volume",
         analysis_volume);
  
  p.read("analysis_input_gain",
         analysis_input_gain);
  p.read("analysis_output_feedback_gain",
         analysis_output_feedback_gain);
  p.read("output_delay_seconds",
         output_delay_seconds);
  
  updateVocoderCarrierInitializer();
}

} // NS resynth
} // NS imajuscule::audio
