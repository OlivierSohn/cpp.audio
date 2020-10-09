
/* Backlog:
 
 -------
 Bug fix
 -------
 
 We do the setup asynchronously, there is a race condition because in onEvent, we setAngleIncrement, whereas
 the corresponding setup may not have been executed yet.
 so we should do:
 . async action for notes that are currently playing only (based on our state vector)
 . sync action for new notes

 synth.setSynchronousElementInitializer(action) {
   init_func = action;
 }
 ::onEvent {
   if (init_func) {
     init_func(element);
   }
   setupAudioelement(element)
 }

 ----------------
 Creative effects
 ----------------

 - make autotune available in the UI (for now, use evenly spaced authorized pitches)
 
 - make an arpegiating effect where current frequencies are played individually in
 sequence from bottom to top.
 varying parameters :
 . arpegiating speed
 . arpegiating overlap
 . gap
 . sustain
 . one-way / 2-ways / 2-ways-bounds-exclusive
 ---
 ...---
 ......---
 or
 ---
 .---
 ..---
 or
 ---
 ......---
 ............---
 or
 --- --- ---  (gap = 1, one-way)
 .-- .-- .--
 ..- ..- ..-
 or
 --- ..- ---  (gap = 1, 2-ways)
 .-- .-- .--
 ..- --- ..-
 or
 ---..----   (gap = 0, 2-ways)
 .--.--.--
 ..----..-
 or
 --..--.     (gap = 0, 2-ways-bounds-exclusive)
 .-.-.-.
 ..--..-
 or
 --.--.--.  (gap = 1, sustain=2, one-way-infinite)
 .--.--.--
 -.--.--.-

 We can detect which notes begin together, to have the notion of chord, or simply consider that all playing notes are part of the same chord
 
 - control AHDSR parameters in real time

 - an interesting effect occurs when reinjecting a delayed version of the output in the input,
 it was discovered by using the "external" feedback from speakers to mic, and by delaying the input signal by one second.
 to implement "internal" feedback, we need additional parameters:
 feedback volume:
 feedback delay:
 and a queue where the producer is the audio output thread, the consumer is the analysis thread.
 The analysis thread reads from both the input and the output queue.
 If the latencies for in and out are different, the sizes of the queues should be augmented by the number of samples corresponding to the max latency

 - customize resynthesys:
 Add a curve 0..1 -> 0..1 to change (compress / expand) volumes
 Add a filter notion : "volume(freq)" to allow to low-pass or high-pass for example

 --
 UI
 --
 
 - use log scales for ui params
 analysis period,
 analysis window size,
 min volume,
 pitch interval,
 pitch tracking
 
 - Display in UI :
 FFT size in frames,
 window size in frames,
 stride in frames
 
 ------
 Others
 ------
 
 - Find a way to that verify notechanges, noteon, noteoff take midi time into account
 (output buffers have a limited size so we can't easily test by using long output buffers)
 
 - (micro optimization) 'samples' has size = "window size"
 this choice was made to improve memory locality but
 when stride is very small, copying the overlap after each processing
 can become significant : for a stride = 1, for each audio frame, we copy "window size" audio frames!
 if 'samples' had twice its size, we would copy, on average, a single frame per audio frame which is much more reasonable.

 --------------
 Peak detection
 --------------
 
 - Improve accuracy of low frequency detection, using small and large ffts:
 small ffts will be used for high frequencies (good temporal accuracy)
 and large ffts will be used for low frequencies (poor temporal accuracy)
 
 - there is a method where using the derivative of the audio signal we get a more precise fourier transform :
 https://hal.archives-ouvertes.fr/hal-00308047/document
 However I'm not sure it will help, in the paper they don't seem to know about quadratic interpolation for peak finding?
 
 - when detecting peaks, to have better precision on close peaks, we could iteratively remove the highest peak contribution from the spectrum.
 But it's not an easy task : when 2 peaks are close, they interfere with each other so ideally we should iterativelly optimize the pair of peaks.
 cf. TEST(ParabollaGaussian, test)
 
 Then we can use these parameters:
 
 static PitchReductionMethod constexpr pitch_method =
 PitchReductionMethod::PonderateByVolume;
 static VolumeReductionMethod constexpr volume_method =
 VolumeReductionMethod::SumVolumes;
 
 - write unit tests for frequency detection:
 . generate a signal with known frequencies / amplitudes.
 . for every type of window, detect peaks
 

 */



/*
 -----------------
 Development Notes
 -----------------

 - We are using portaudio HEAD that fixes a bug when the same device is used as input and output
 (the bug was that buffer sizes had to be identical)
 
 */



/*
 --------------------------------------
 About jitter (and how we cope with it)
 --------------------------------------
 
 Here is what is likely to happen with threads jitter, using an example.
 
 To simplify, we will assume that each audio input callback call handles 'widow_center_stride' input samples,
 and that the input signal contains 4 events (0, 1, 2, 3) which are exactly 'window_center_stride' apart, like so:
 
 input signal: 0--1--2--3--
 
 We have 3 threads running: the audio input thread, the processing thread and the audio output thread.
 The audio input thread writes in a queue read by the process thread, and
 the process thread writes in a queue read by the audio output thread.
 
 In an ideal world, all threads run at the same regular intervals, with no overlap:
 
 audio in thread :    0--      1--      2--      3--       // we represent both the input signal and the time it takes for the input thread to run
 process thread  :       ---      ---      ---      ---    // we represent the time it takes for the thread to run
 audio out thread:          0--      1--      2--      3-- // we represent both the output signal and the time it takes for the output thread to run
 
 -> output signal : 0--1--2--3--
 
 In this ideal case, the events maintain their timings. But let's see how we can diverge from the ideal case:
 
 A. We can imagine that the 'process' thread has jitter:
 
 audio in thread :    0--      1--      2--      3--
 process thread  :       ---                ---     ---
 audio out thread:          0--      ---      1-2      3--
 
 -> output signal : 0-----1-23--
 
 B. We can imagine that the 'process' thread runs less often that the audio threads:
 
 audio in thread :    0--      1--      2--      3--
 process thread  :       ---               ---
 audio out thread:          0--      ---      12-      3--
 
 -> output signal : 0-----12-3--
 
 C. we can imagine that the audio in thread has jitter:
 
 audio in thread :    0--      1--            2--3--
 process thread  :       ---      ---      ---      ---
 audio out thread:          0--      1--      ---      23-
 
 -> output signal : 0--1-----23-
 
 D. we can imagine that the audio out thread has jitter:
 
 audio in thread :    0--      1--      2--      3--
 process thread  :       ---      ---      ---      ---
 audio out thread:          0--      1--            2-3---
 
 -> output signal : 0--1--2-3---
 
 In the real world, it is likely that A., B. C. and D. happen simulataneously.
 
 To cope with this, we use midi timestamps to accurately
 trigger the relevant actions at the right time in the audio out thread.
 */


namespace imajuscule::audio {

struct PitchVolume {
  double midipitch;
  double volume;
};

void frequencies_to_pitches(Midi const & midi,
                            std::vector<FreqMag<double>> const & fs,
                            std::vector<PitchVolume> & res) {
  res.clear();
#ifndef NDEBUG
  double freq = std::numeric_limits<double>::lowest();
#endif
  for (auto const & f : fs) {
#ifndef NDEBUG
    Assert(freq < f.freq); // verify invariant
    freq = f.freq;
#endif
    if (auto pitch = midi.frequency_to_midi_pitch(f.freq)) {
      res.push_back({
        *pitch,
        DbToMag<double>()(f.mag_db)
      });
    }
  }
}

enum class PitchReductionMethod {
  IntervalCenter,
  MaxVolume,
  PonderateByVolume
};

enum class VolumeReductionMethod{
  MaxVolume,
  SumVolumes
};

class PitchInterval {
  double min_pitch, max_pitch;
  double maxVolumePitch;
  double maxVolume{};
  double sumProductsPitchVolume{};
  double sumVolumes{};

 public:
  PitchInterval(PitchVolume const & pv)
  : min_pitch(pv.midipitch)
  , max_pitch(pv.midipitch)
  , maxVolumePitch(pv.midipitch)
  {
    aggregate(pv);
  }

  double minPitch() const {
    return min_pitch;
  }
  double maxPitch() const {
    return max_pitch;
  }

  void extend(PitchVolume const & pv) {
    min_pitch = std::min(min_pitch,
                         pv.midipitch);
    max_pitch = std::max(max_pitch,
                         pv.midipitch);
    aggregate(pv);
  }

  double getPitch(PitchReductionMethod m) const {
    switch(m) {
      case PitchReductionMethod::IntervalCenter:
        return 0.5 * (min_pitch + max_pitch);
      case PitchReductionMethod::MaxVolume:
        return maxVolumePitch;
      case PitchReductionMethod::PonderateByVolume:
        Assert(sumVolumes);
        return sumProductsPitchVolume / sumVolumes;
    }
  }

  double getVolume(VolumeReductionMethod m) const {
    switch(m) {
      case VolumeReductionMethod::MaxVolume:
        return maxVolume;
      case VolumeReductionMethod::SumVolumes:
        return sumVolumes;
    }
  }

private:
  void aggregate(PitchVolume const & pv) {
    sumVolumes += pv.volume;
    sumProductsPitchVolume += pv.midipitch * pv.volume;

    if (maxVolume < pv.volume) {
      maxVolume = pv.volume;
      maxVolumePitch = pv.midipitch;
    }
  }
};


template<typename T>
T diameter(T a, T b, T c) {
  return std::max({a, b, c}) - std::min({a, b, c});
}

// We aggregate nearby pitches to keep the number of channels needed to a reasonable amount, especially when handling percusive sounds
// which have a lot of pitch peaks.

// - Aggregation phase:
// We have a rule to aggregate nearby pitches ('nearby_distance', in number of tones):
// for example, with nearby_distance = 3 half tones,
//   .....  could lead to 2 pitches:
//   A..B.  or a single pitch:
//   ..A..  depending on where we start analyzing. So we have a notion of pitch interval, and analyze pitches in a monotonic order:
//          we "aggregate" frequencies in the last interval, and when aggregating a frequency would leand to an interval with
//          a diameter bigger than nearby_distance, we create a new interval.
// The output of this phase is a list of pitch intervals where each interval is a list of pitch + amplitude.
void aggregate_pitches(double const nearby_distance_tones,
                       // invariant : ordered by pitch
                       std::vector<PitchVolume> const & pitch_volumes,
                       // invariant : ordered by pitch
                       std::vector<PitchInterval> & pitch_intervals) {
  pitch_intervals.clear();
  pitch_intervals.reserve(pitch_volumes.size());
  
  std::optional<PitchInterval> cur;
#ifndef NDEBUG
  double pitch = std::numeric_limits<double>::lowest();
  int idx = -1;
#endif
  for (auto const & pv : pitch_volumes) {
#ifndef NDEBUG
    ++idx;
    Assert(pitch < pv.midipitch); // verify invariant
    pitch = pv.midipitch;
#endif
    if (cur && (diameter(cur->minPitch(),
                         cur->maxPitch(),
                         pv.midipitch) > nearby_distance_tones)) {
      pitch_intervals.push_back(*cur);
      cur.reset();
    }
    if (!cur) {
      cur = {pv};
    } else {
      cur->extend(pv);
    }
  }

  if (cur) {
    pitch_intervals.push_back(*cur);
    cur.reset();
  }
}

// - Reduction phase:
// each interval will be reduced to a single pitch and amplitude. The amplitude will be the sum of amplitudes,
// the pitch can be either the pitch of max amplitude, or the center of the interval, or a ponderation of the pitches by amplitudes.
void reduce_pitches(PitchReductionMethod const pitch_method,
                    VolumeReductionMethod const volume_method,
                    double const min_volume,
                    std::vector<PitchInterval> const & pitch_intervals,
                    std::vector<PitchVolume> & reduced_pitches) {
  reduced_pitches.clear();
  reduced_pitches.reserve(pitch_intervals.size());
  for (auto const & i : pitch_intervals) {
    double const vol = i.getVolume(volume_method);
    if (vol < min_volume) {
      continue;
    }
    reduced_pitches.push_back({
      i.getPitch(pitch_method),
      vol
    });
  }
}

// - Autotune phase:
// the reduced frequency will be changed to the closest allowed frequency (in pitch space).
// the amplitudes of frequencies that fall in the same closest frequency will be added
template<typename Autotune>
void autotune_pitches(Autotune pitch_transform,
                      // invariant : ordered by pitch
                      std::vector<PitchVolume> const & input,
                      // invariant : ordered by pitch
                      std::vector<PitchVolume> & output) {
  output.clear();
  output.reserve(input.size());
#ifndef NDEBUG
  double pitch = std::numeric_limits<double>::lowest();
#endif
  for (auto const & pv : input) {
#ifndef NDEBUG
    Assert(pitch < pv.midipitch); // verify invariant
    pitch = pv.midipitch;
#endif
    double const transformedPitch = pitch_transform(pv.midipitch);
    if (!output.empty() && (std::abs(output.back().midipitch - transformedPitch) < 0.0001)) {
      output.back().volume += pv.volume;
    } else {
      output.push_back({transformedPitch, pv.volume});
    }
  }
}


// A note currently played
struct PlayedNote {
  // The id of the analysis frame at which the note was started
  int64_t note_on_frame_id;
  
  // the identifier of the played note
  NoteId noteid;

  // the current midi_pitch
  double midi_pitch;

  // the current frequency
  float cur_freq;
  
  // the current volume
  // Not currently used during analysis, but could be used during matching :
  // if the volume is currently low and the now volume is much higher, we could trigger a noteon
  float cur_velocity;
};

// - Tracking phase:
// we will determine whether the frequencies are noteon or notechange, based on currently runing oscillators.
// we need a "max_track_pitches" upperbound for the distance between pitches that can be associated.
// the caller must take into account the period of the analysis, and adapt 'max_track_pitches' accordingly
void track_pitches(double const max_track_pitches,
                   // invariant : ordered by pitch
                   std::vector<PitchVolume> const & new_pitches,
                   // invariant : ordered by pitch (current pitch)
                   std::vector<PlayedNote> const & played_pitches,
                   // indexes homogenous to indexes of 'new_pitches',
                   // values  homogenous to indexes of 'played_pitches'
                   // invariant : all non-empty elements are distinct
                   std::vector<std::optional<int>> & pitch_changes,
                   std::vector<bool> & continue_playing) {
  pitch_changes.clear();
  pitch_changes.resize(new_pitches.size());

  continue_playing.clear();
  continue_playing.resize(played_pitches.size(), false);
  
  int idx = -1;
#ifndef NDEBUG
  double pitch2 = std::numeric_limits<double>::lowest();
  for (auto const & p : played_pitches) {
    Assert(pitch2 <= p.midi_pitch);
    pitch2 = p.midi_pitch;
  }
  double pitch = std::numeric_limits<double>::lowest();
#endif
  auto const begin = played_pitches.begin();
  auto it = played_pitches.begin();
  auto const end = played_pitches.end();
  for (auto const & newPv : new_pitches) {
    ++idx;
#ifndef NDEBUG
    Assert(pitch < newPv.midipitch); // verify invariant
    pitch = newPv.midipitch;
#endif
    for (; it != end; ++it) {
      if (it->midi_pitch < newPv.midipitch - max_track_pitches) {
        continue;
      }
      if (it->midi_pitch <= newPv.midipitch + max_track_pitches) {
        std::size_t const idx_played = std::distance(begin, it);
        pitch_changes[idx] = idx_played;
        continue_playing[idx_played] = true;
        ++it;
        // note that there could be other options, if 'it+1' is also in the right range.
        // but to not complicate the algorithm too much we take the first one.
      }
      break;
    }
  }
}

// we have a limited amount of channels in the synthethizer, so we prioritize
// detected pithces by their respective perceived loudness.
template<typename F>
void
order_pitches_by_perceived_loudness(F perceived_loudness,
                                    // invariant : ordered by pitch
                                    std::vector<PitchVolume> const & new_pitches,
                                    std::vector<float> & autotuned_pitches_perceived_loudness,
                                    std::vector<int> & autotuned_pitches_idx_sorted_by_perceived_loudness) {
  autotuned_pitches_perceived_loudness.clear();
  autotuned_pitches_perceived_loudness.reserve(new_pitches.size());

#ifndef NDEBUG
  double pitch = std::numeric_limits<double>::lowest();
#endif

  for (auto const & pv : new_pitches) {
#ifndef NDEBUG
    Assert(pitch < pv.midipitch); // verify invariant
    pitch = pv.midipitch;
#endif
    autotuned_pitches_perceived_loudness.push_back(perceived_loudness(pv));
  }

  autotuned_pitches_idx_sorted_by_perceived_loudness.clear();
  autotuned_pitches_idx_sorted_by_perceived_loudness.resize(new_pitches.size());
  std::iota(autotuned_pitches_idx_sorted_by_perceived_loudness.begin(),
            autotuned_pitches_idx_sorted_by_perceived_loudness.end(),
            0);
  std::sort(autotuned_pitches_idx_sorted_by_perceived_loudness.begin(),
            autotuned_pitches_idx_sorted_by_perceived_loudness.end(),
            [&autotuned_pitches_perceived_loudness] (int a, int b){
    return
    autotuned_pitches_perceived_loudness[a] > // so that loud frequencies come first
    autotuned_pitches_perceived_loudness[b];
  }
            );
}


// This may change the order of elements
void remove_dead_notes(std::vector<bool> const & continue_playing,
                       std::vector<PlayedNote> & played_pitches) {
  auto new_end = std::remove_if(played_pitches.begin(),
                                played_pitches.end(),
                                [first = played_pitches.data(),
                                 &continue_playing]
                                (PlayedNote const & note) {
    std::ptrdiff_t n = &note-first;
    Assert(n >= 0);
    if (n < continue_playing.size()) {
      return !continue_playing[n];
    } else {
      return false;
    }
  });
  played_pitches.erase(new_end,
                       played_pitches.end());
}

void sort_by_current_pitch(std::vector<PlayedNote> & played_pitches) {
  std::sort(played_pitches.begin(),
            played_pitches.end(),
            [](PlayedNote const & n, PlayedNote const & m) {
    return n.midi_pitch < m.midi_pitch;
  });
}


void print(std::vector<PlayedNote> const & played_pitches) {
  int i=0;
  struct PitchCount {
    int pitch_idx;
    int count;
  };
  std::optional<PitchCount> pc;
  for (auto const & played : played_pitches) {
    int const pitch_idx = static_cast<int>(0.5f + played.midi_pitch);
    if (pc) {
      if (pc->pitch_idx == pitch_idx) {
        ++pc->count;
      } else {
        for (; i < pc->pitch_idx; ++i) {
          std::cout << " ";
        }
        std::cout << pc->count;
        ++i;
        pc->count = 1;
        pc->pitch_idx = pitch_idx;
      }
    } else {
      pc = {pitch_idx, 1};
    }
  }
  if (pc) {
    for (; i < pc->pitch_idx; ++i) {
      std::cout << " ";
    }
    std::cout << pc->count;
  }
  std::cout << std::endl;
}


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

template<typename T>
struct ResynthElementInitializer {
  ResynthElementInitializer(int const sample_rate,
                            int const stride)
  : sample_rate(sample_rate)
  , stride(stride)
  {}

  void operator()(ResynthFinalElement<T> & e) const {
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
, ResynthFinalElement<T>
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator
, 127  // lots of voices
, ResynthElementInitializer<T>
>;

} // NS audioelement


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
    
  using Synth = audioelement::synthOf<Ctxt, double>;
  
  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  
  // producer : analysis thread
  // consumer : data thread
  struct NoteOn {
    PlayedNote note;
  };
  struct NoteOnDropped {
    PlayedNote note;
  };
  struct NoteChange {
    PlayedNote note;
  };
  struct NoteOff {
    NoteId note_id;
  };
  struct CurrentNoteState {
    PlayedNote note;
  };
  struct CountDroppedItems {
    int count = 0;
  };
  struct EndOfAnalysisFrame {
    int64_t frame_id;
  };
  // most of the items will be 'Event' (192 bits) and 'PlayedNote' (192 bits) if the consumer is slow),
  // so it's nice that these 2 types have the biggest size in the variant
  using DataQueueItem = std::variant<
  NoteOn,
  NoteOnDropped,
  NoteChange,
  NoteOff,
  CurrentNoteState,
  CountDroppedItems,
  EndOfAnalysisFrame
  >;
  
  using DataQueue = atomic_queue::AtomicQueueB2<
  /* T = */ DataQueueItem,
  /* A = */ std::allocator<DataQueueItem>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  static constexpr std::size_t data_queue_capacity = 1024; // if we receive too many 'CountDroppedItems' that's a sign that this is not enough
  void try_push_data_note_on(PlayedNote const & n) {
    if (unlikely(pending_dropped_data_items.count)) {
      ++pending_dropped_data_items.count;
      return;
    }
    try_push_data_internal(NoteOn{n});
  }
  void try_push_data_note_on_dropped(PlayedNote const & n) {
    if (unlikely(pending_dropped_data_items.count)) {
      ++pending_dropped_data_items.count;
      return;
    }
    try_push_data_internal(NoteOnDropped{n});
  }
  void try_push_data_note_change(PlayedNote const & n) {
    if (unlikely(pending_dropped_data_items.count)) {
      ++pending_dropped_data_items.count;
      return;
    }
    try_push_data_internal(NoteChange{n});
  }
  void try_push_data_note_off(PlayedNote const & n) {
    if (unlikely(pending_dropped_data_items.count)) {
      ++pending_dropped_data_items.count;
      return;
    }
    try_push_data_internal(NoteOff{n.noteid});
  }
  void try_push_end_of_analysis_frame(int64_t id) {
    if (unlikely(pending_dropped_data_items.count)) {
      if (!data_queue.try_push(pending_dropped_data_items)) {
        return;
      }
      pending_dropped_data_items.count = 0;
      // because some data has been dropped, we send the full state.
      for (PlayedNote const & note : played_pitches) {
        try_push_data_internal(CurrentNoteState{note});
        if (pending_dropped_data_items.count) {
          // we failed to send the full state.
          // we will try to re-send the full state at the next frame end
          return;
        }
      }
    }
    Assert(!pending_dropped_data_items.count);
    try_push_data_internal(EndOfAnalysisFrame{id});
  }
  void try_push_data_internal(DataQueueItem const & item) {
    if (!data_queue.try_push(item)) {
      ++pending_dropped_data_items.count;
    }
  }
  
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
  
  /*
   The UI needs access to the "playing pitches" info.
   
   But we don't want to wait on a mutex in the analysis thread.
   
   Alternative 1:
   -------------
   We have a mutex which we try_lock in the analysis thread:
   if we succeed, we copy the infos to another place, protected by the mutex.
   If we fail, we just increment an atomic to say that an analysis frame was dropped.
   
   The ui takes the lock to copy the infos to yet another place, and then processes it
   
   Alternative 2:
   -------------
   The analysis thread fills a queue with items.
   A reader thread, will:
   - on NoteOn, NoteOff, NoteChange:
   if unlocked, lock the mutex
   change vector_playednotes accordingly
   - on CountDroppedItems :
   if unlocked, lock the mutex
   clear vector_playednotes // the producer will send use the whole state via 'PlayedNote'(s)
   - on CurrentNoteState:
   Assert(mutex is locked) // just before pushing the first 'CurrentNoteState', the producer makes sure that 'DroppedItems' was pushed to the queue
   if unlocked, lock the mutex
   append to vector_playednotes
   - on EndOfAnalysisFrame id :
   if unlocked, lock the mutex
   set frame id
   release the mutex
   
   The ui reads the infos using this same mutex.
   (the ui should allocate its vectors (for copy) outside the mutex scope)
   
   Conclusions:
   -----------
   With Alternative 2, the ui is guaranteed to read the most up-to-date frame,
   while with alternative 1 this is not the case (maybe the most recent frame has been dropped)
   so we chose alternative 2.
   */
  class NonRealtimeAnalysisFrame {
    mutable std::mutex mutex;
    bool owns_mutex = false;
    std::vector<PlayedNote> vec;
    std::vector<PlayedNote> vec_dropped;
    std::optional<int64_t> frame_id;
    
    void initialize_frame_if_needed() {
      if (!owns_mutex) {
        mutex.lock();
        owns_mutex = true;
        vec_dropped.clear();
      }
    }
    void onItem(NoteOn const & n) {
      initialize_frame_if_needed();
      vec.push_back(n.note);
    }
    void onItem(NoteOnDropped const & n) {
      initialize_frame_if_needed();
      vec_dropped.push_back(n.note);
    }
    void onItem(CurrentNoteState const & n) {
      initialize_frame_if_needed();
      vec.push_back(n.note);
    }
    void onItem(NoteOff const & n) {
      initialize_frame_if_needed();
      auto it = std::find_if(vec.begin(),
                             vec.end(),
                             [noteid = n.note_id](PlayedNote const & n) { return n.noteid == noteid; });
      if (it == vec.end()) {
        throw std::logic_error("orphan note off");
      }
      vec.erase(it);
    }
    void onItem(NoteChange const & n) {
      initialize_frame_if_needed();
      auto it = std::find_if(vec.begin(),
                             vec.end(),
                             [noteid = n.note.noteid](PlayedNote const & n) { return n.noteid == noteid; });
      if (it == vec.end()) {
        throw std::logic_error("orphan note change");
      }
      *it = n.note;
    }
    
    void onItem(CountDroppedItems const &) {
      initialize_frame_if_needed();
      vec.clear();
    }
    
    void onItem(EndOfAnalysisFrame const & end_of_frame) {
      initialize_frame_if_needed();
      frame_id = end_of_frame.frame_id;
      mutex.unlock();
      owns_mutex = false;
    }
    
  public:
    NonRealtimeAnalysisFrame() {
      vec.reserve(300);
      vec_dropped.reserve(300);
    }
    
    ~NonRealtimeAnalysisFrame() {
      if (owns_mutex) {
        mutex.unlock();
        owns_mutex = false;
      }
    }
    
    std::mutex & getMutex() const { return mutex; }
    
    // you must lock the mutex for this call
    std::optional<int64_t> getFrameId() const { return frame_id; }
    // you must lock the mutex while you read the vector
    std::vector<PlayedNote> const & getPlayingNotes() const { return vec; }
    // you must lock the mutex while you read the vector
    std::vector<PlayedNote> const & getDroppedNotes() const { return vec_dropped; }
    
    // there is no need to lock the mutex when calling this
    void onItem(DataQueueItem const & i) {
      // we need to be fast on reading 'NoteOn', 'NoteOff', 'NoteChange', then 'CurrentNoteState', then 'EndOfAnalysisFrame', then 'CountDroppedItems'
      if (std::holds_alternative<NoteOn>(i)) {
        onItem(std::get<NoteOn>(i));
      } else if (std::holds_alternative<NoteOff>(i)) {
        onItem(std::get<NoteOff>(i));
      } else if (std::holds_alternative<NoteChange>(i)) {
        onItem(std::get<NoteChange>(i));
      } else if (std::holds_alternative<CurrentNoteState>(i)) {
        onItem(std::get<CurrentNoteState>(i));
      } else if (std::holds_alternative<EndOfAnalysisFrame>(i)) {
        onItem(std::get<EndOfAnalysisFrame>(i));
      } else if (std::holds_alternative<NoteOnDropped>(i)) {
        onItem(std::get<NoteOnDropped>(i));
      } else if (std::holds_alternative<CountDroppedItems>(i)) {
        onItem(std::get<CountDroppedItems>(i));
      } else {
        throw std::logic_error("unhandled alternative");
      }
    }
  };
  
  RtResynth()
  : synth(buffers)
  , channel_handler(ctxt.getChannelHandler())
  , data_queue(data_queue_capacity) {
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
    
    {
      thread_analysis_data_active = true;
      thread_analysis_data = std::make_unique<std::thread>([this](){
        while(thread_analysis_data_active) {
          DataQueueItem item;
          while(data_queue.try_pop(item)) {
            analysis_data.onItem(item);
          }
          std::this_thread::yield();
        }
      });
    }
  }
  
  ~RtResynth() {
    teardown();
    
    thread_analysis_data_active = false;
    thread_analysis_data->join();
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
        
        autotune_pitches([](double pitch){ return pitch; },
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

          audioelement::ResynthElementInitializer<double> initializer{sample_rate, future_stride};
          
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
              try_push_data_note_off(played_pitches[idx]);
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
            
            try_push_data_note_change(played);
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
              try_push_data_note_on(played_pitches.back());
            } else {
              ++dropped_note_on;
              try_push_data_note_on_dropped(new_note);
            }
          }
        }
        
        remove_dead_notes(continue_playing, played_pitches);
        
        sort_by_current_pitch(played_pitches);
        
        try_push_end_of_analysis_frame(n);
        
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
  
  DataQueue data_queue;
  CountDroppedItems pending_dropped_data_items;
  NonRealtimeAnalysisFrame analysis_data;
  std::atomic_bool thread_analysis_data_active{false};
  std::unique_ptr<std::thread> thread_analysis_data;
  
  cyclic<float> delayed_input;
  std::atomic<float> input_delay_seconds = 0.f;
  std::atomic<float> window_size_seconds = 0.1814f;
  std::atomic<float> window_center_stride_seconds = 0.09f;
  std::atomic<float> min_volume = 0.0001;
  std::atomic<float> nearby_distance_tones = 0.4;
  std::atomic<float> max_track_pitches = 1.;
  static_assert(std::atomic<float>::is_always_lock_free);
  
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

} // NS
