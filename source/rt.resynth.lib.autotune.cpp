namespace imajuscule::audio::rtresynth {

enum class AutotuneType {
  MusicalScale,
  Chord,
  FixedSizeIntervals,
  // when adding values, please update 'CountEnumValues<AutotuneType>'
};
enum class MusicalScaleMode {
  Major,
  MinorNatural,
  MinorHarmonic
  // when adding values, please update 'CountEnumValues<MusicalScaleMode>'
};

enum class AutotuneChordFrequencies {
  SingleFreq,
  Harmonics,
  OctavePeriodic
};

} // NS

namespace imajuscule {
template<>
struct CountEnumValues<audio::rtresynth::AutotuneType> {
  static int constexpr count = 3;
};
template<>
struct CountEnumValues<audio::rtresynth::MusicalScaleMode> {
  static int constexpr count = 3;
};
template<>
struct CountEnumValues<audio::rtresynth::AutotuneChordFrequencies> {
  static int constexpr count = 3;
};
} // NS

namespace imajuscule::audio::rtresynth {
inline std::ostream & operator << (std::ostream & os, AutotuneType t) {
  switch(t) {
    case AutotuneType::MusicalScale:
      os << "Scale"; break;
    case AutotuneType::Chord:
      os << "Chord"; break;
    case AutotuneType::FixedSizeIntervals:
      os << "Intervals"; break;
  }
  return os;
}

inline std::ostream & operator << (std::ostream & os, MusicalScaleMode t) {
  switch(t) {
    case MusicalScaleMode::Major:
      os << "Major"; break;
    case MusicalScaleMode::MinorNatural:
      os << "Minor natural"; break;
    case MusicalScaleMode::MinorHarmonic:
      os << "Minor harmonic"; break;
  }
  return os;
}

inline std::ostream & operator << (std::ostream & os, AutotuneChordFrequencies t) {
  switch(t) {
    case AutotuneChordFrequencies::SingleFreq:
      os << "Single"; break;
    case AutotuneChordFrequencies::OctavePeriodic:
      os << "Replicate on every octave"; break;
    case AutotuneChordFrequencies::Harmonics:
      os << "Use harmonics"; break;
  }
  return os;
}

template<typename T>
std::vector<T>
make_equidistant_pitches(std::vector<T> const & pitches) {
  std::vector<T> res;
  int const sz = static_cast<int>(pitches.size());
  Assert(sz > 1);
  res.reserve(sz - 1);
  for (int i = 0; i < (sz-1); ++i) {
    res.push_back(0.5 * (pitches[i] + pitches[i+1]));
  }
  return res;
}

struct MusicalScalePitches {
  MusicalScalePitches(std::vector<double> const & all_pitches)
  : pitches(all_pitches)
  , equidistant_pitches(make_equidistant_pitches(all_pitches)) {}
  
  double distance_to(double const relative_translated_pitch) const {
    Assert(relative_translated_pitch >= 0.);
    Assert(relative_translated_pitch < num_halftones_per_octave);
    // scale_pitches starts at 0 and goes upward until (included) Midi::num_halftones_per_octave
    Assert(equidistant_pitches.size() + 1 ==
           pitches.size());
    int i = 0;
    for (int sz = static_cast<int>(equidistant_pitches.size());
         i < sz;
         ++i) {
      if (relative_translated_pitch < equidistant_pitches[i]) {
        break;
      }
    }
    return relative_translated_pitch - pitches[i];
  }

  template<typename T>
  T closest_pitch (T const root_pitch,
                   T const pitch) const
  {
    // translate pitch to the right octave
    T const half_tones_dist = pitch - root_pitch;
    T const octave_dist = half_tones_dist / num_halftones_per_octave;
    int octaves_translation;
    // static_cast from floating point to integral rounds towards zero
    if (octave_dist >= 0.) {
      octaves_translation = static_cast<int>(octave_dist);
    } else {
      octaves_translation = static_cast<int>(octave_dist) - 1;
    }
    
    T const translated_pitch = pitch - octaves_translation * num_halftones_per_octave;
    
    Assert(translated_pitch >= root_pitch);
    Assert(translated_pitch < root_pitch + num_halftones_per_octave);
    
    T const relative_translated_pitch = translated_pitch - root_pitch;
    
    Assert(relative_translated_pitch >= 0.);
    Assert(relative_translated_pitch < num_halftones_per_octave);
    
    T const offset = distance_to(relative_translated_pitch);
    return pitch - offset;
  }

private:
  std::vector<double> pitches; // the first element is the root (0.) and the last is the root at the next octave (12.)
  std::vector<double> equidistant_pitches;
};

const MusicalScalePitches major_scale{{
  0.,
  2.,
  4.,
  5.,
  7.,
  9.,
  11.,
  12.
}};

const MusicalScalePitches minor_natural_scale{{
  0.,
  2.,
  3.,
  5.,
  7.,
  8.,
  10.,
  12.
}};

const MusicalScalePitches minor_harmonic_scale{{
  0.,
  2.,
  3.,
  5.,
  7.,
  8.,
  11.,
  12.
}};

inline MusicalScalePitches const & getMusicalScale(MusicalScaleMode const t) {
  switch(t) {
    case MusicalScaleMode::Major:
      return major_scale;
    case MusicalScaleMode::MinorNatural:
      return minor_natural_scale;
    case MusicalScaleMode::MinorHarmonic:
      return minor_harmonic_scale;
  }
}

// precondition : pitches are ordered wrt 'get_pitch'
template<typename T, typename F>
T* find_closest_pitch(float const pitch,
                      std::vector<T> & pitches,
                      F && get_pitch) {
  auto const begin = pitches.begin();
  auto const end = pitches.end();
  if (begin == end) {
    return nullptr;
  }
  auto lb = std::lower_bound(begin,
                             end,
                             pitch,
                             [get_pitch](T const & a, float comp_pitch){
    return get_pitch(a) < comp_pitch;
  });
  if (lb == end) {
    return &(*(lb-1));
  } else if (lb == begin) {
    return &(*lb);
  } else {
    float const dist_lb = std::abs(pitch - get_pitch(*lb));
    float const dist_prev_lb = std::abs(pitch - get_pitch(*(lb-1)));
    
    if (dist_lb < dist_prev_lb) {
      return &(*lb);
    } else {
      return &(*(lb-1));
    }
  }
}

} // NS
