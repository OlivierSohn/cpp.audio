
// TODO add a curve 0..1 -> 0..1 to change volumes

namespace imajuscule::audio {

struct PitchVolume {
  double midipitch;
  double volume;
};

void frequencies_to_pitches(Midi const & midi,
                            std::vector<FreqMag<double>> const & fs,
                            std::vector<PitchVolume> & res) {
  res.clear();
  for (auto const & f : fs) {
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
//   ..A..  depending on where we start analyzing, so we should have a notion of pitch interval, and analyze pitches in a monotonic order
//          once the last interval would be bigger than nearby_distance, we create a new interval.
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
  double pitch = std::numeric_limits<double>::min();
#endif
  for (auto const & pv : pitch_volumes) {
#ifndef NDEBUG
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
  double pitch = std::numeric_limits<double>::min();
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
  // the identifier of the played note
  NoteId noteid;

  // the current midi_pitch
  double midi_pitch;

  // the current frequency
  float cur_freq;

  // the current volume (not used but could be used during matching : if the volume is currently low and the now volume is much higher, we could trigger a noteon)
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
  double pitch2 = std::numeric_limits<double>::min();
  for (auto const & p : played_pitches) {
    Assert(pitch2 <= p.midi_pitch);
    pitch2 = p.midi_pitch;
  }
  double pitch = std::numeric_limits<double>::min();
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

template <typename Ctxt, typename T>
using synthOf = sine::Synth < // the name of the namespace is misleading : it can handle all kinds of oscillators
Ctxt::policy
, Ctxt::nAudioOut
, XfadePolicy::SkipXfade
, FinalAudioElement<
    VolumeAdjusted<
      Enveloped<
        FreqCtrl_<
          OscillatorAlgo<T, eNormalizePolicy::FAST>,
          InterpolatedFreq<T>
        >,
        AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
      >
    >
  >
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator
, 128  // lots of voices
>;

} // NS audioelement

void rtResynth(int const sample_rate) {
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

  Ctxt ctxt(sample_rate);
  if (!ctxt.Init(0.006)) {
    throw std::runtime_error("ctxt init failed");
  }
  auto & channel_handler = ctxt.getChannelHandler();

  using Synth = audioelement::synthOf<Ctxt, double>;

  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  std::array<mnc_buffer,n_mnc> buffers;

  auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                             std::min(n_mnc,
                                                                                                      static_cast<int>(std::numeric_limits<uint8_t>::max())));
  NoXFadeChans & channels = channels_;

  Synth synth(buffers);

  synth.initialize(channels);

  bool constexpr logs = true;

  int constexpr window_size = 800;
  int constexpr window_center_stride = 400;
  int constexpr windowoverlapp = std::max(0, window_size - window_center_stride);

  constexpr double min_volume = 0.0001;
  double constexpr nearby_distance_tones = 0.4;
  PitchReductionMethod constexpr pitch_method = PitchReductionMethod::PonderateByVolume;
  VolumeReductionMethod constexpr volume_method = VolumeReductionMethod::SumVolumes;
  double constexpr max_track_pitches = 1.;

  /*
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

   So to cope with this, we would need to introduce some delay and use midi timestamps to accurately
   trigger the relevant actions at the right time in the audio out thread.
   */

  synth.forEachElems([](auto & e) {
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

    e.algo.setMaxFilterIncrement(2. / static_cast<double>(window_center_stride));

    // frequency control

    e.algo.getOsc().getAlgo().getCtrl().setup(window_center_stride,
                                              itp::LINEAR);
  });


  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ SAMPLE,
  /* A = */ std::allocator<SAMPLE>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue input_queue(sample_rate); // one second of input can fit in the queue

  auto onInput = [&input_queue](const SAMPLE * buf, int nFrames){
    for (int i=0; i<nFrames; ++i) {
      input_queue.push(buf[i]);
    }
  };

  AudioInput<AudioPlatform::PortAudio> input;
  if (!input.Init(onInput, sample_rate)) {
    throw std::runtime_error("input init failed");
  }

  std::atomic_bool thread_resynth_active(true);
  std::thread thread_resynth([&input_queue,
                              &thread_resynth_active,
                              sample_rate,
                              &channel_handler,
                              &channels,
                              &synth](){
    std::vector<double> half_window = half_gaussian_window<double>(4, window_size/2);
    normalize_window(half_window);

    auto process = [&half_window](int const sample_rate,
                                  auto from,
                                  auto to,
                                  FrequenciesSqMag<double> & frequencies_sqmag,
                                  std::vector<FreqMag<double>> & freqmags) {
      int constexpr zero_padding_factor = 1;
      int constexpr windowed_signal_stride = 1;

      findFrequenciesSqMag(from,
                           to,
                           windowed_signal_stride,
                           half_window,
                           zero_padding_factor,
                           frequencies_sqmag);

      extractLocalMaxFreqsMags(sample_rate / windowed_signal_stride,
                               frequencies_sqmag,
                               SqMagToDb<double>(),
                               freqmags);
    };

    std::vector<double> samples;
    samples.resize(window_size, {});


    int end = 0;

    FrequenciesSqMag<double> frequencies_sqmag;
    frequencies_sqmag.frequencies_sqmag.reserve(200);

    std::vector<FreqMag<double>> freqmags;
    freqmags.reserve(200);

    std::vector<PitchVolume> freqmags_data;
    freqmags_data.reserve(200);

    std::vector<PitchInterval> pitch_intervals;
    pitch_intervals.reserve(200);

    std::vector<PitchVolume> reduced_pitches, autotuned_pitches;
    reduced_pitches.reserve(200);
    autotuned_pitches.reserve(200);

    std::vector<PlayedNote> played_pitches;
    played_pitches.reserve(200);

    std::vector<std::optional<int>> pitch_changes;
    pitch_changes.reserve(200);

    std::vector<bool> continue_playing;
    continue_playing.reserve(200);

    std::vector<PlayedNote> played_notes;
    played_notes.resize(200);

    Midi midi;

    int n = 0;

    auto step = [sample_rate,
                 &n,
                 &midi,
                 &synth,
                 &channel_handler,
                 &channels,
                 &played_notes,
                 &freqmags_data,
                 &pitch_intervals,
                 &reduced_pitches,
                 &autotuned_pitches,
                 &pitch_changes,
                 &continue_playing,
                 &played_pitches
                 ](std::vector<FreqMag<double>> const & fs) {
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

      bool changed = false;

      // issue "note off" events
      {
        int idx = -1;
        for (auto play : continue_playing) {
          ++idx;
          if (!play) {
            auto res = synth.onEvent2(sample_rate,
                                      mkNoteOff(played_pitches[idx].noteid),
                                      channel_handler,
                                      channels,
                                      {});
            if (logs) std::cout << n << ": XXX pitch " << played_pitches[idx].midi_pitch << " " << res << std::endl;
            if (res != onEventResult::OK) {
              throw std::logic_error("dropped note off");
            }
            changed = true;
          }
        }
      }

      // issue "note change" and "note on" events
      {
        int idx = -1;
        for (auto pitch_change : pitch_changes) {
          ++idx;

          changed = true;

          double const new_pitch = autotuned_pitches[idx].midipitch;
          float const new_freq = midi.midi_pitch_to_freq(new_pitch);

          float const volume = autotuned_pitches[idx].volume;

          if (pitch_change) {
            PlayedNote & played = played_pitches[*pitch_change];

            auto res = synth.onEvent2(sample_rate,
                                      mkNoteChange(played.noteid,
                                                   volume,
                                                   new_freq),
                                      channel_handler,
                                      channels,
                                      {});
            if (res != onEventResult::OK) {
              throw std::logic_error("dropped note change");
            }
            if (logs) std::cout << n << ": pitch " << played.midi_pitch << " newpitch " << new_pitch << " Vol " << volume << " " << res << std::endl;

            played.cur_freq = new_freq;
            played.midi_pitch = new_pitch;
          } else {
            static int noteid = 0;
            ++noteid;
            NoteId const note_id{noteid};
            auto const res = synth.onEvent2(sample_rate,
                                            mkNoteOn(note_id,
                                                     new_freq,
                                                     volume),
                                            channel_handler,
                                            channels,
                                            {});
            if (logs) std::cout << n << ": pitch " << new_pitch << " vol " << volume << " " << res << std::endl;
            if (res == onEventResult::OK) {
              played_pitches.push_back({
                note_id,
                new_pitch,
                new_freq,
                volume
              });
            } else {
              // dropped note:
              //
            }
          }
        }
      }

      remove_dead_notes(continue_playing, played_pitches);

      sort_by_current_pitch(played_pitches);

      if (changed) {
        print(played_pitches);
      }
    };

    while (thread_resynth_active) {
      while (input_queue.try_pop(samples[end])) {
        ++ end;
        if (end == window_size) {
          process(sample_rate,
                  samples.begin(),
                  samples.begin() + end,
                  frequencies_sqmag,
                  freqmags);
          step(freqmags);
          int const offset = window_size-windowoverlapp;
          for (end=0; end<windowoverlapp; ++end) {
            samples[end] = samples[end + offset];
          }
        }
      }
    }
  });

  std::string end;
  std::cin >> end;

  thread_resynth_active = false;
  thread_resynth.join();

  if (!input.Teardown()) {
    throw std::runtime_error("input teardown failed");
  }
  ctxt.onApplicationShouldClose();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ctxt.TearDown();
}

} // NS

int main() {
  imajuscule::audio::rtResynth(44100);
  return 0;
}
