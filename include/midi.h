namespace imajuscule {

template<typename T>
struct CountEnumValues;

} // NS

namespace imajuscule::audio {

enum class Note {
  Do,
  Dod,
  Re,
  Red,
  Mi,
  Fa,
  Fad,
  Sol,
  Sold,
  La,
  Lad,
  Si,
};
}

namespace imajuscule {
template<>
struct CountEnumValues<audio::Note> {
  static int constexpr count = 12;
};
}

namespace imajuscule::audio {
inline std::ostream &
operator << (std::ostream & os, Note const & n) {
  switch(n) {
    case Note::Do:
      os << "C";
      break;
    case Note::Dod:
      os << "C#";
      break;
    case Note::Re:
      os << "D";
      break;
    case Note::Red:
      os << "D#";
      break;
    case Note::Mi:
      os << "E";
      break;
    case Note::Fa:
      os << "F";
      break;
    case Note::Fad:
      os << "F#";
      break;
    case Note::Sol:
      os << "G";
      break;
    case Note::Sold:
      os << "G#";
      break;
    case Note::La:
      os << "A";
      break;
    case Note::Lad:
      os << "A#";
      break;
    case Note::Si:
      os << "B";
      break;
  }
  return os;
}

inline constexpr
int half_tones_distance(Note const & a,
                               Note const & b) {
  return
  static_cast<int>(to_underlying(b)) -
  static_cast<int>(to_underlying(a));
}

constexpr int num_halftones_per_octave = 12;
// Midi A(69) according to http://subsynth.sourceforge.net/midinote2freq.html
constexpr float freq_A = 440.f;
constexpr int A_pitch = 69;
constexpr int ref_A_octave = 4;
constexpr int max_audible_midi_pitch = 151; // 50 kHz

struct NoteOctave {
  Note note;
  int octave;
  
  NoteOctave add_halftones(long n) const {
    NoteOctave res(*this);
    
    while(n >= num_halftones_per_octave) {
      n -= num_halftones_per_octave;
      ++res.octave;
    }
    while(n < 0) {
      n += num_halftones_per_octave;
      --res.octave;
    }
    Assert(n >= 0);
    Assert(n < num_halftones_per_octave);
    for (; n > 0; --n) {
      if (res.note == Note::Si) {
        res.note = Note::Do;
        ++res.octave;
      } else {
        res.note = static_cast<Note>(to_underlying(res.note)+1);
      }
    }
    return res;
  }
  
  int dist_halftones(NoteOctave const & other) const {
    int const res = half_tones_distance(note, other.note);
    return res + num_halftones_per_octave * (other.octave - octave);
  }
  
  bool operator == (const NoteOctave & other) const {
    return
    std::make_tuple(note, octave) ==
    std::make_tuple(other.note, other.octave);
  }
};


inline std::pair<NoteOctave, double>
midi_pitch_to_note_deviation(double const pitch) {
  double const pitch_from_ref_A = pitch - A_pitch;
  long const half_tones_from_ref_A = std::lround(pitch_from_ref_A);
  double const deviation_halftones = pitch_from_ref_A - half_tones_from_ref_A;
  NoteOctave const no = NoteOctave{Note::La, ref_A_octave}.add_halftones(half_tones_from_ref_A);
  return {no, deviation_halftones};
}

enum class Constexpr {
  Yes, // means we can use constexpr math functions that might be slower than their non-constexpr counterpart because the function call will happen at compile time
  No // means we want the fastest math functions because the function call will happen at run time
};

template<Constexpr is_constexpr, typename T>
constexpr auto log2(T v) -> T {
  if constexpr (is_constexpr == Constexpr::Yes) {
    return sprout::log2(v); // this is slow, but we are at compile time so it's ok
  } else {
    return std::log2(v);
  }
}

template<Constexpr is_constexpr, typename T>
constexpr auto pow(T a, T b) -> T {
  if constexpr (is_constexpr == Constexpr::Yes) {
    return sprout::pow(a, b); // this is slow, but we are at compile time so it's ok
  } else {
    return std::pow(a, b);
  }
}

template<Constexpr is_constexpr, typename T>
constexpr
std::optional<T> frequency_to_midi_pitch(T const tuning_stretch,
                                         T const freq) {
  if (freq <= 0) {
    return {};
  }
  return A_pitch + (num_halftones_per_octave / tuning_stretch) * log2<is_constexpr>(freq/freq_A);
}

template<Constexpr is_constexpr>
struct Midi_ {
  static constexpr double unity_tuning_stretch = 1.;
  
  // TODO constexpr that, using  https://github.com/elbeno/constexpr/blob/master/src/include/cx_math.h
  constexpr Midi_(double const tuning_stretch = unity_tuning_stretch)
  : half_tone_ratio_(pow<is_constexpr>(2., tuning_stretch/num_halftones_per_octave))
  , tuning_stretch_(tuning_stretch)
  {}
  
  constexpr double getHalfToneRatio() const {
    return half_tone_ratio_;
  }
  
  double get_pitch(NoteOctave const & note) const {
    int const num_halftones = NoteOctave{Note::La, ref_A_octave}.dist_halftones(note);
    return A_pitch + tuning_stretch_ * num_halftones;
  }

  constexpr std::optional<double> frequency_to_midi_pitch(double const freq) const {
    return imajuscule::audio::frequency_to_midi_pitch<is_constexpr>(tuning_stretch_,
                                                                    freq);
  }
  
  constexpr double Ainterval_to_freq(double const interval_from_A) const {
    return freq_A * pow<is_constexpr>(half_tone_ratio_, interval_from_A);
  }
  
  constexpr double midi_pitch_to_freq(double pitch) const {
    return Ainterval_to_freq(pitch - A_pitch);
  }

  constexpr double transpose_frequency(double const freq, int n) const {
    return freq * pow<is_constexpr>(half_tone_ratio_,
                                    static_cast<double>(n));
  }
  constexpr double midi_pitch_offset_nth_harmonic(int n) const {
    return (num_halftones_per_octave / tuning_stretch_) * log2<is_constexpr>(static_cast<float>(n));
  }
private:
  double const half_tone_ratio_;
  double const tuning_stretch_;
};

template<int N, Constexpr is_constexpr>
constexpr std::array<double, N>
compute_harmonic_pitch_adds(Midi_<is_constexpr> const & midi) {
  std::array<double, N> res{};
  for (int i=0; i<N; ++i) {
    res[i] = static_cast<int>(0.5 + midi.midi_pitch_offset_nth_harmonic(i+1));
  }
  return res;
}

using Midi = Midi_<Constexpr::No>;
using ConstexprMidi = Midi_<Constexpr::Yes>;

} // NS
