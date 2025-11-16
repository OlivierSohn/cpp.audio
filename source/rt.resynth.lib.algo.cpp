namespace imajuscule::audio::rtresynth {

constexpr float pitch_epsilon = 0.0001f;

struct PitchVolume {
  MidiPitch midipitch;
  double volume;
};

inline void frequencies_to_pitches(Midi const & midi,
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
  MidiPitch min_pitch, max_pitch;
  MidiPitch maxVolumePitch;
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
  
  MidiPitch minPitch() const {
    return min_pitch;
  }
  MidiPitch maxPitch() const {
    return max_pitch;
  }
  
  void extend(PitchVolume const & pv) {
    min_pitch = std::min(min_pitch,
                         pv.midipitch);
    max_pitch = std::max(max_pitch,
                         pv.midipitch);
    aggregate(pv);
  }
  
  MidiPitch getPitch(PitchReductionMethod m) const {
    switch(m) {
      case PitchReductionMethod::IntervalCenter:
        return MidiPitch{0.5 * (min_pitch.get() + max_pitch.get())};
      case PitchReductionMethod::MaxVolume:
        return maxVolumePitch;
      case PitchReductionMethod::PonderateByVolume:
        Assert(sumVolumes);
        return MidiPitch{sumProductsPitchVolume / sumVolumes};
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
    sumProductsPitchVolume += pv.midipitch.get() * pv.volume;
    
    if (maxVolume < pv.volume) {
      maxVolume = pv.volume;
      maxVolumePitch = pv.midipitch;
    }
  }
};


template<typename T>
auto diameter(T a, T b, T c) {
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
  reserve_no_shrink(pitch_intervals,
                    pitch_volumes.size());
  
  std::optional<PitchInterval> cur;
#ifndef NDEBUG
  MidiPitch pitch{std::numeric_limits<float>::lowest()};
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
  reserve_no_shrink(reduced_pitches,
                    pitch_intervals.size());
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
// the amplitudes of frequencies that fall in the same closest frequency will be added.
// Pitches above 'max_pitch' will be played but not autotuned.
// Pitches with a distance from closest allowed pitch bigger than 'pitch_tolerance' will not be played.
template<typename Autotune>
void autotune_pitches(MidiPitch const max_pitch,
                      float const pitch_tolerance,
                      Autotune pitch_transform,
                      // invariant : ordered by pitch
                      std::vector<PitchVolume> const & input,
                      // invariant : ordered by pitch
                      std::vector<PitchVolume> & output) {
  output.clear();
  reserve_no_shrink(output,
                    input.size());
#ifndef NDEBUG
  MidiPitch pitch{std::numeric_limits<float>::lowest()};
#endif
  for (auto const & pv : input) {
#ifndef NDEBUG
    Assert(pitch < pv.midipitch); // verify invariant
    pitch = pv.midipitch;
#endif
    std::optional<MidiPitch> transformedPitch;
    if (pv.midipitch <= max_pitch) {
      transformedPitch = pitch_transform(pv.midipitch);
      if(transformedPitch.has_value() && *transformedPitch > max_pitch)
        // This is to keep output sorted
        transformedPitch = pv.midipitch;
    } else {
      transformedPitch = pv.midipitch;
    }
    if (transformedPitch) {
      if (std::abs(*transformedPitch - pv.midipitch) <= pitch_tolerance) {
        if (!output.empty() && (std::abs(output.back().midipitch - *transformedPitch) < pitch_epsilon)) {
          output.back().volume += pv.volume;
        } else {
          output.push_back({*transformedPitch, pv.volume});
        }
      }
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
  MidiPitch midi_pitch;
  
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
  MidiPitch pitch2{std::numeric_limits<float>::lowest()};
  for (auto const & p : played_pitches) {
    Assert(pitch2 <= p.midi_pitch);
    pitch2 = p.midi_pitch;
  }
  MidiPitch pitch{std::numeric_limits<float>::lowest()};
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

inline void
shift_pitches(float const shift_amount,
              std::vector<PitchVolume> & pitches) {
  if (!shift_amount) {
    return;
  }
  for (auto & p : pitches) {
    p.midipitch += shift_amount;
  }
}

inline void
harmonize_pitches(float const harmonize_amount,
                  std::vector<PitchVolume> & work_vector,
                  std::vector<PitchVolume> & pitches) {
#ifndef NDEBUG
  // verify invariant
  {
    MidiPitch pitch{std::numeric_limits<float>::lowest()};
    for (auto const & pv : pitches) {
      Assert(pitch < pv.midipitch);
      pitch = pv.midipitch;
    }
  }
#endif
  if (!harmonize_amount) {
    return;
  }
  int const sz = static_cast<int>(pitches.size());
  for (int i=0; i<sz; ++i) {
    PitchVolume const & p = pitches[i];
    MidiPitch const harmonized_pitch = p.midipitch + harmonize_amount;
    PitchVolume * closest = find_closest_pitch(harmonized_pitch,
                                               pitches,
                                               [](PitchVolume const & pv){ return pv.midipitch; });
    Assert(closest);
    float const dist = std::abs(closest->midipitch - harmonized_pitch);
    if (dist <= pitch_epsilon) {
      closest->volume += p.volume;
    } else {
      pitches.push_back({harmonized_pitch, p.volume});
    }
  }
  // pitches is sorted from 0 to sz-1, and from sz to new_sz-1
  if (work_vector.capacity() < pitches.size()) {
    work_vector.reserve(2*pitches.size());
  }
  merge_sort_step(pitches.begin(),
                  pitches.begin() + sz,
                  pitches.begin() + sz,
                  pitches.end(),
                  [](PitchVolume const & pv){ return pv.midipitch; },
                  work_vector);
  work_vector.swap(pitches);
#ifndef NDEBUG
  // verify invariant
  {
    MidiPitch pitch{std::numeric_limits<float>::lowest()};
    for (auto const & pv : pitches) {
      Assert(pitch < pv.midipitch);
      pitch = pv.midipitch;
    }
  }
#endif
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
  reserve_no_shrink(autotuned_pitches_perceived_loudness,
                    new_pitches.size());
  
#ifndef NDEBUG
  MidiPitch pitch{std::numeric_limits<float>::lowest()};
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
    int const pitch_idx = static_cast<int>(0.5f + played.midi_pitch.get());
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

} // NS
