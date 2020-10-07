
namespace imajuscule::audio {

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

template<Constexpr is_constexpr>
struct Midi_ {
  static constexpr double unity_tuning_stretch = 1.;
  
  // Midi A(69) according to http://subsynth.sourceforge.net/midinote2freq.html
  static constexpr float freq_A = 440.f;
  static constexpr double A_pitch = 69.;
  
  // TODO constexpr that, using  https://github.com/elbeno/constexpr/blob/master/src/include/cx_math.h
  constexpr Midi_(double const tuning_stretch = unity_tuning_stretch)
  : half_tone_ratio_(pow<is_constexpr>(2., tuning_stretch/12.))
  , tuning_stretch_(tuning_stretch)
  {}
  
  constexpr double getHalfToneRatio() const {
    return half_tone_ratio_;
  }
  
  constexpr std::optional<double> frequency_to_midi_pitch(double const freq) const {
    if (freq <= 0) {
      return {};
    }
    return 69. + (12. / tuning_stretch_) * log2<is_constexpr>(freq/440.);
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
  
private:
  double const half_tone_ratio_;
  double const tuning_stretch_;
};

using Midi = Midi_<Constexpr::No>;
using ConstexprMidi = Midi_<Constexpr::Yes>;
} // NS
