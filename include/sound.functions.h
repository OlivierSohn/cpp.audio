
namespace imajuscule::audio {

  template<typename T>
  constexpr T sample_rate_milliseconds(int sample_rate) {
    return static_cast<T>(sample_rate) / static_cast<T>(1000);
  };
  template<typename T>
  constexpr T sample_rate_nanoseconds(int sample_rate) {
    return static_cast<T>(sample_rate) / static_cast<T>(1e9);
  };
  template<typename T>
  constexpr T nanos_per_frame(int sample_rate) {
    return static_cast<T>(1e9) / sample_rate;
  };
  template<typename T>
  constexpr T millis_per_frame(int sample_rate) {
    return static_cast<T>(1e3) / sample_rate;
  };

  constexpr int32_t nanoseconds_to_frames(uint64_t ns, int sample_rate) {
    float v = 0.5f + sample_rate_nanoseconds<float>(sample_rate) * static_cast<float>(ns);
    Assert(v >= 0.f);
    Assert(v < static_cast<float>(std::numeric_limits<int32_t>::max()));
    return static_cast<int32_t>(v);
  }
    constexpr int ms_to_frames(float duration_ms, int sample_rate) {
        Assert(duration_ms >= 0.f);
        auto fval = sample_rate_milliseconds<float>(sample_rate) * duration_ms;
        Assert(fval >= 0.f);
        Assert(fval < static_cast<float>(std::numeric_limits<int>::max()));
        return static_cast<int>( 0.5f + fval );
    }

    constexpr float frames_to_ms(int n, int sample_rate) {
        Assert(n >= 0);
        return millis_per_frame<float>(sample_rate) * static_cast<float>(n);
    }

    template<typename T>
    constexpr T period_in_samples_to_freq(T sample_rate, int period) {
        return sample_rate / static_cast<T>(period);
    }

    constexpr int freq_to_period_in_samples( float freq_hz, float sample_rate ) {
        if(freq_hz <= 0.f) {
            return 1;
        }
        return static_cast<int>(sample_rate / freq_hz);
    }

    template<typename T>
    constexpr T get_nyquist_frequency(T sample_rate) {
        return period_in_samples_to_freq(sample_rate, static_cast<T>(2));
    }

    // angle increment unit is "rad / pi"
template<typename T>
constexpr T freq_to_angle_increment(T freq, int sample_rate) {
  static_assert(std::is_floating_point_v<T>);
  return 2 * freq / sample_rate;
}

    template<typename T>
    constexpr T angle_increment_to_freq(T i, T sample_rate) {
        static_assert(std::is_floating_point_v<T>);
        return i * 0.5 * sample_rate;
    }

template<typename T>
T angle_increment_to_period_in_continuous_samples(T i) {
  if(i == 0.f) {
    return static_cast<T>(1);
  }
  return 2. / std::abs(i);
}

    template<typename T>
    constexpr T freq_to_period_in_seconds(T freq) {
      if(freq <= 0) {
          return 1;
      }
      return static_cast<T>(1) / freq;
    }

  template<typename T>
  constexpr T square(T ang) {
    static_assert(std::is_floating_point_v<T>);
    using Tr = NumTraits<T>;
    if( Tr::half() < ang && ang < Tr::one_and_half() ) {
      return -Tr::one();
    } else {
      return Tr::one();
    }
  }

  template<typename T>
  constexpr T pulse(T ang, T pulse_width) {
    static_assert(std::is_floating_point_v<T>);
    using Tr = NumTraits<T>;
    Assert(pulse_width >= 0);
    Assert(ang >= 0);
    Assert(ang <= 2);
    if( ang < pulse_width ) {
      return Tr::one();
    } else {
      return Tr::zero();
    }
  }

  template<typename T>
  constexpr T triangle(T ang) {
    static_assert(std::is_floating_point_v<T>);
    if( ang < static_cast<T>(0.5) ) {        // 0 .. 0.5   ->  0 .. 1
      return static_cast<T>(2) * ang;
    } else if( ang < static_cast<T>(1.5) ) { // 0.5 .. 1.5 ->  1 .. -1
      return static_cast<T>(2) + static_cast<T>(-2) * ang;
    } else {                                 // 1.5 .. 2   ->  -1 .. 0
      Assert( ang <= static_cast<T>(2) );
      return static_cast<T>(-4) + static_cast<T>(2) * ang;
    }
  }

  template<typename T>
  constexpr T saw(T ang) {
    if( ang <= static_cast<T>(1) ) {        // 0 .. 1   ->  0 .. 1
      return ang;
    } else {                                // 1 .. 2   ->  -1 .. 0
      if(ang > static_cast<T>(2)) {
        LG(ERR, "%f", ang);
      }
      Assert( ang <= static_cast<T>(2) );
      return static_cast<T>(-2) + ang;
    }
  }

}
