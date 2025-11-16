
namespace imajuscule::audio {
namespace well_tempered
{
constexpr auto c_minorScaleAsc = std::array<double, 7>{
  0.,
  2.,
  3.,
  5.,
  7.,
  8.,
  10.
};
constexpr auto c_majorScaleAsc = std::array<double, 7>{
  0.,
  2.,
  4.,
  5.,
  7.,
  9.,
  11.
};
}


template<Constexpr C, size_t N>
inline std::array<double, N> mkScaleFromFreqRatios(const std::array<double, N>& freqRatios)
{
  std::array<double, N> pitches{};
  
  for(size_t i=0; i<N; ++i)
    pitches[i] = frequency_to_midi_pitch<C>(1., freqRatios[i])->get();
  
  const auto offset = pitches[0];
  for(size_t i=0; i<N; ++i)
    pitches[i] -= offset;
  
  return pitches;
}


// harmonic overtone series:
//
// 1 = base
// 2 = octave
// 3 = octave + fifth
// 4 = 2 octaves
// 5 = 2 octaves + Major 3rd
// 6 = 2 octaves + fifth
// etc...

namespace just
{
template<Constexpr C>
inline std::array<double, 7> mkMajorScaleAsc()
{
  const std::array<double, 7> freqRatios{
    1.,
    9./8.,
    5./4.,
    4./3.,
    3./2.,
    5./3.,
    15./8.
  };
  return mkScaleFromFreqRatios<C>(freqRatios);
}
}

namespace pythagorean
{
template<Constexpr C>
inline std::array<double, 7> mkMajorScaleAsc()
{
  const std::array<double, 7> freqRatios{
    1.,
    9./8.,
    81./64.,
    4./3.,
    3./2.,
    27./16.,
    243./128.
  };
  return mkScaleFromFreqRatios<C>(freqRatios);
}
}

template<size_t N>
std::array<MidiPitch, N> toMidiPitches(MidiPitch rootPitch, std::array<double, N> const & scaleOffsets)
{
  std::array<MidiPitch, N> res;
  for(size_t i=0; i<N; ++i)
  {
    res[i] = rootPitch + scaleOffsets[i];
  }
  return res;
}

} // NS
