namespace imajuscule::audio::rtresynth {

enum class AutotuneType {
  None,
  FixedSizeIntervals,
  MusicalScale,
  // when adding values, please update 'CountEnumValues<AutotuneType>'
};
enum class MusicalScaleType {
  Major,
  MinorNatural,
  MinorHarmonic
  // when adding values, please update 'CountEnumValues<MusicalScaleType>'
};

} // NS

namespace imajuscule {
template<>
struct CountEnumValues<audio::rtresynth::AutotuneType> {
  static int constexpr count = 3;
};
template<>
struct CountEnumValues<audio::rtresynth::MusicalScaleType> {
  static int constexpr count = 3;
};
} // NS

namespace imajuscule::audio::rtresynth {
std::ostream & operator << (std::ostream & os, AutotuneType t) {
  switch(t) {
    case AutotuneType::None:
      os << "None"; break;
    case AutotuneType::FixedSizeIntervals:
      os << "Intervals"; break;
    case AutotuneType::MusicalScale:
      os << "Scale"; break;
  }
  return os;
}

std::ostream & operator << (std::ostream & os, MusicalScaleType t) {
  switch(t) {
    case MusicalScaleType::Major:
      os << "Major"; break;
    case MusicalScaleType::MinorNatural:
      os << "MinorNatural"; break;
    case MusicalScaleType::MinorHarmonic:
      os << "MinorHarmonic"; break;
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
    Assert(relative_translated_pitch < Midi::num_halftones_per_octave);
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

MusicalScalePitches const & getMusicalScale(MusicalScaleType const t) {
  switch(t) {
    case MusicalScaleType::Major:
      return major_scale;
    case MusicalScaleType::MinorNatural:
      return minor_natural_scale;
    case MusicalScaleType::MinorHarmonic:
      return minor_harmonic_scale;
  }
}

} // NS
