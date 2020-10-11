
namespace imajuscule::audio {

namespace audioelement {

template <typename T>
using ResynthFinalElement =
FinalAudioElement<
 VolumeAdjusted<
  Enveloped<
   FreqCtrl_<
    OscillatorAlgo<T, eNormalizePolicy::FAST>,
    InterpolatedFreq<T>
   >,
   AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
  >
 >
>;

} // NS audioelement

namespace rtresynth {

template<typename T>
struct ResynthElementInitializer {
  ResynthElementInitializer(int const sample_rate,
                            int const stride)
  : sample_rate(sample_rate)
  , stride(stride)
  {}

  void operator()(audioelement::ResynthFinalElement<T> & e) const {
    // envelope (now that we track the volume, we only need a minimal envelope)
    
    // TODO try the following effect via volume tracking only (not evelope):
    // make the volume decrease slower than in reality (notion of meta envelope)
    
    e.algo.editEnvelope().setAHDSR(audioelement::AHDSR{
      0, itp::LINEAR,
      0,
      0, itp::LINEAR,
      0, itp::LINEAR,
      1.f
    }, sample_rate);
    
    // limit the speed of volume adjustment:
    
    e.algo.setMaxFilterIncrement(2. / static_cast<double>(stride));
    
    // frequency control
    
    e.algo.getOsc().getAlgo().getCtrl().setup(stride,
                                              itp::LINEAR);
  }
  
private:
  int sample_rate;
  int stride;
};

template <typename Ctxt, typename T>
using synthOf = sine::Synth < // the name of the namespace is misleading : it can handle all kinds of oscillators
Ctxt::policy
, Ctxt::nAudioOut
, XfadePolicy::SkipXfade
, audioelement::ResynthFinalElement<T>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator
, 127  // lots of voices
, ResynthElementInitializer<T>
>;

struct RtResynth {
private:
  static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

  using AllChans = ChannelsVecAggregate< 2, audioEnginePolicy >;
  
  using NoXFadeChans = typename AllChans::NoXFadeChans;
  using XFadeChans = typename AllChans::XFadeChans;
  
  using ChannelHandler = outputDataBase< AllChans, ReverbType::Realtime_Synchronous >;
  
  using Ctxt = AudioOutContext<
  ChannelHandler,
  Features::JustOut,
  AudioPlatform::PortAudio
  >;
  
  static double constexpr minInLatency  = 0.000001;
  static double constexpr minOutLatency = 0.000001;
  static bool constexpr logs = false;
  static bool constexpr print_changes = false;
    
  static PitchReductionMethod constexpr pitch_method =
  PitchReductionMethod::PonderateByVolume;
  static VolumeReductionMethod constexpr volume_method =
  VolumeReductionMethod::SumVolumes;
    
  using Synth = synthOf<Ctxt, double>;
  
  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  
  // producer : Audio input callback
  // consumer : analysis thread
  struct InputSample {
    float value;
  };
  struct CountDroppedFrames {
    int count = 0;
  };
  using QueueItem = std::variant<InputSample,CountDroppedFrames>;
  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ QueueItem,
  /* A = */ std::allocator<QueueItem>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  
public:
  
  
  RtResynth()
  : synth(buffers)
  , channel_handler(ctxt.getChannelHandler()) {
    auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                                std::min(n_mnc,
                                                                                                         static_cast<int>(std::numeric_limits<uint8_t>::max())));
    channels = &channels_;
    
    frequencies_sqmag.frequencies_sqmag.reserve(200);
    freqmags.reserve(200);
    freqmags_data.reserve(200);
    pitch_intervals.reserve(200);
    reduced_pitches.reserve(200);
    autotuned_pitches.reserve(200);
    autotuned_pitches_perceived_loudness.reserve(200);
    autotuned_pitches_idx_sorted_by_perceived_loudness.reserve(200);
    played_pitches.reserve(200);
    pitch_changes.reserve(200);
    continue_playing.reserve(200);
    played_notes.resize(200);
#ifndef NDEBUG
    testAutotune();
#endif
  }
  
  ~RtResynth() {
    teardown();    
  }
  
  void init(int const sample_rate = 88200) {
    if (sample_rate <= 0) {
      throw std::invalid_argument("sample_rate is too small");
    }
    
    teardown();
    Assert(!thread_resynth_active);
    
    if (!synth.initialize(*channels)) {
      throw std::logic_error("failed to initialize synth");
    }
    
    if (!ctxt.Init(sample_rate, minOutLatency)) {
      throw std::runtime_error("ctxt init failed");
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    input_queue = std::make_unique<Queue>(sample_rate); // one second of input can fit in the queue
    
    if (!input.Init([this](const float * buf, int nFrames){
      if (!thread_resynth_active) {
        return;
      }
      if (pending_dropped_frames.count) {
        if (unlikely(!input_queue->try_push(pending_dropped_frames))) {
          int const nLocalDroppedFrames = nFrames;
          pending_dropped_frames.count += nLocalDroppedFrames;
          count_dropped_input_frames += nLocalDroppedFrames;
          return;
        }
        pending_dropped_frames.count = 0;
      }
      for (int i=0; i<nFrames; ++i) {
        if (unlikely(!input_queue->try_push(InputSample{buf[i]}))) {
          int const nLocalDroppedFrames = nFrames-i;
          pending_dropped_frames.count += nLocalDroppedFrames;
          count_dropped_input_frames += nLocalDroppedFrames;
          return;
        }
      }
    },
                    sample_rate,
                    minInLatency)) {
      throw std::runtime_error("input init failed");
    }
    
    thread_resynth_active = true;
    thread_resynth = std::make_unique<std::thread>([this,
                                                    sample_rate](){
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
      
      n = 0;
      
      std::optional<int> stride_for_synth_config;
      auto step = [sample_rate,
                   &stride_for_synth_config,
                   this
                   ](std::vector<FreqMag<double>> const & fs, MIDITimestampAndSource const & miditime, int const future_stride) {
        ++n;
        
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
        
        std::function<double(double const)> autotune;
        switch(autotune_type) {
          case AutotuneType::None :
            autotune = [](double v) {return v;};
            break;
          case AutotuneType::FixedSizeIntervals:
            autotune = [factor = this->autotune_factor.load()](double pitch) -> double {
              if (!factor) {
                return pitch;
              } else {
                int const discrete_pitch = static_cast<int>(pitch + 0.5);
                return static_cast<double>(factor * (discrete_pitch / factor));
              }
            };
            break;
          case AutotuneType::MusicalScale:
            const auto * scale = &getMusicalScale(autotune_musical_scale_mode);
            autotune = [scale,
                        root_pitch = A_pitch + half_tones_distance(Note::La,
                                                                   autotune_musical_scale_root_note)](double const pitch) {
              return scale->closest_pitch(root_pitch, pitch);
            };
            break;
        }
        autotune_pitches(autotune,
                         reduced_pitches,
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
        
        // reconfigure synth if needed
        
        if (!stride_for_synth_config || *stride_for_synth_config != future_stride) {
          stride_for_synth_config = future_stride;

          ResynthElementInitializer<double> initializer{sample_rate, future_stride};
          
          // This will apply the new params for new notes:
          synth.setSynchronousElementInitializer(initializer);
          
          // This will apply the new params for currently played notes
          channels->enqueueOneShot([this,
                                    initializer](auto & chans, auto){
            synth.forEachActiveElem([initializer](auto & e) {
              initializer(e);
            });
          });
        }
        
        bool changed = false;
        
        // issue "note off" events
        {
          int idx = -1;
          for (auto play : continue_playing) {
            ++idx;
            if (!play) {
              auto res = synth.onEvent(sample_rate,
                                       mkNoteOff(played_pitches[idx].noteid),
                                       channel_handler,
                                       *channels,
                                       miditime);
              if (logs) std::cout << n << ": XXX pitch " << played_pitches[idx].midi_pitch << " " << res << std::endl;
              if (res != onEventResult::OK) {
                throw std::logic_error("dropped note off");
              }
              analysis_data.try_push_note_off(played_pitches[idx]);
              changed = true;
            }
          }
        }
        
        // issue "note change" and "note on" events
        for (int idx : autotuned_pitches_idx_sorted_by_perceived_loudness) {
          std::optional<int> const & pitch_change = pitch_changes[idx];
          
          changed = true;
          
          double const new_pitch = autotuned_pitches[idx].midipitch;
          float const new_freq = midi.midi_pitch_to_freq(new_pitch);
          
          float const volume = autotuned_pitches[idx].volume;
          
          if (pitch_change) {
            PlayedNote & played = played_pitches[*pitch_change];
            
            auto res = synth.onEvent(sample_rate,
                                     mkNoteChange(played.noteid,
                                                  volume,
                                                  new_freq),
                                     channel_handler,
                                     *channels,
                                     miditime);
            if (res != onEventResult::OK) {
              throw std::logic_error("dropped note change");
            }
            if (logs) std::cout << n << ": pitch " << played.midi_pitch << " newpitch " << new_pitch << " Vol " << volume << " " << res << std::endl;
            
            played.cur_freq = new_freq;
            played.midi_pitch = new_pitch;
            played.cur_velocity = volume;
            
            analysis_data.try_push_note_change(played);
          } else {
            static int noteid = 0;
            ++noteid;
            NoteId const note_id{noteid};
            auto const res = synth.onEvent(sample_rate,
                                           mkNoteOn(note_id,
                                                    new_freq,
                                                    volume),
                                           channel_handler,
                                           *channels,
                                           miditime);
            if (logs) std::cout << n << ": pitch " << new_pitch << " vol " << volume << " " << res << std::endl;
            
            PlayedNote new_note{
              n,
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
        
        remove_dead_notes(continue_playing, played_pitches);
        
        sort_by_current_pitch(played_pitches);
        
        analysis_data.try_push(NonRealtimeAnalysisFrame::EndOfFrame{
          n,
          std::chrono::microseconds(static_cast<int>(1000000. * static_cast<double>(future_stride)/sample_rate))
        }, played_pitches);
        
        if (changed && print_changes) {
          print(played_pitches);
        }
      };
      
      auto init_data = [this,
                        sample_rate
                        ](bool const force){
        int const input_delay_frames = input_delay_seconds * sample_rate;
        
        // we need an even window size
        int const window_size = 2 * std::max(1,
                                             static_cast<int>(0.5 * window_size_seconds * sample_rate));
        bool reinit_samples = force;
        if (force || delayed_input.size() != input_delay_frames + 1) {
          delayed_input.resize(input_delay_frames + 1); // this also resets the elements to zero
          reinit_samples = true;
        }
        {
          Assert(0 == window_size%2);
          if (half_window.size() != window_size/2) {
            half_gaussian_window<double>(4, window_size/2, half_window);
            normalize_window(half_window);
            Assert(half_window.size() == window_size/2);
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
        while (input_queue->try_pop(var)) {
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
                step(freqmags,
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
            
            storeAudioInputQueueFillRatio(input_queue->was_size() / static_cast<float>(input_queue->capacity()));
          }
        }
        std::this_thread::yield(); // should we sleep?
      }
    });
  }
  
  void teardown() {
    if (!thread_resynth_active) {
      return;
    }
    thread_resynth_active = false;
    thread_resynth->join();
    thread_resynth.reset();
    if (!input.Teardown()) {
      throw std::runtime_error("input teardown failed");
    }
    input_queue.reset();
    
    ctxt.onApplicationShouldClose();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ctxt.TearDown();
    
    synth.finalize(*channels);
  }
  
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
    return count_dropped_input_frames;
  }
  
  int countDroppedNoteOns() const {
    return dropped_note_on;
  }
  
  int countFailedComputeInsertions() const {
    return channels->countFailedComputeInsertions();
  }
  int countRetriedOneshotInsertions() const {
    return channels->countRetriedOneshotInsertions();
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
private:
  Ctxt ctxt;
  ChannelHandler & channel_handler;
  NoXFadeChans * channels;
  std::array<mnc_buffer,n_mnc> buffers;
  Synth synth;
  Midi midi;
  
  AudioInput<AudioPlatform::PortAudio> input;
  
  std::atomic_bool thread_resynth_active{false};
  std::unique_ptr<std::thread> thread_resynth;
  std::unique_ptr<Queue> input_queue;
  CountDroppedFrames pending_dropped_frames;
  std::atomic_int count_dropped_input_frames = 0;
  static_assert(decltype(count_dropped_input_frames)::is_always_lock_free);
  
  NonRealtimeAnalysisFrame analysis_data;
  
  cyclic<float> delayed_input;
  std::atomic<float> input_delay_seconds = 0.f;
  std::atomic<float> window_size_seconds = 0.1814f;
  std::atomic<float> window_center_stride_seconds = 0.09f;
  std::atomic<float> min_volume = 0.0001;
  std::atomic<float> nearby_distance_tones = 0.4;
  std::atomic<float> max_track_pitches = 1.;
  static_assert(std::atomic<float>::is_always_lock_free);

  std::atomic<AutotuneType> autotune_type = AutotuneType::None;
  static_assert(std::atomic<AutotuneType>::is_always_lock_free);
  std::atomic_int autotune_factor = 2;
  static_assert(std::atomic_int::is_always_lock_free);
  std::atomic<MusicalScaleMode> autotune_musical_scale_mode = MusicalScaleMode::Major;
  static_assert(std::atomic<MusicalScaleMode>::is_always_lock_free);
  std::atomic<Note> autotune_musical_scale_root_note = Note::Do;
  static_assert(std::atomic<Note>::is_always_lock_free);

  int64_t n;
  
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
  std::vector<PitchVolume> reduced_pitches, autotuned_pitches;
  std::vector<float> autotuned_pitches_perceived_loudness;
  std::vector<int> autotuned_pitches_idx_sorted_by_perceived_loudness;
  std::vector<PlayedNote> played_pitches;
  std::vector<std::optional<int>> pitch_changes;
  std::vector<bool> continue_playing; // indexed like 'played_pitches'
  std::vector<PlayedNote> played_notes;
  
  std::atomic<float> dt_process_seconds = -1.f;
  std::atomic<float> dt_step_seconds = -1.f;
  std::atomic<float> dt_copy_seconds = -1.f;
  std::atomic<float> input_queue_fill_ratio = 0.f;
  std::atomic_int dropped_note_on = 0;
  
  void setDurations(std::optional<float> duration_process_seconds,
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
  
};

} // NS resynth
} // NS imajuscule::audio
