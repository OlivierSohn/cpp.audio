
namespace imajuscule {

  template<typename T>
  constexpr T sample_rate() { return static_cast<T>(SAMPLE_RATE); };
  template<typename T>
  constexpr T half_sample_rate() { return static_cast<T>(SAMPLE_RATE) / static_cast<T>(2); };
  template<typename T>
  constexpr T sample_rate_milliseconds() { return static_cast<T>(SAMPLE_RATE) / static_cast<T>(1000); };
  template<typename T>
  constexpr T sample_rate_nanoseconds() { return static_cast<T>(SAMPLE_RATE) / static_cast<T>(1e9); };
  template<typename T>
  constexpr T inverse_sample_rate() { return static_cast<T>(1) / static_cast<T>(SAMPLE_RATE); };
  template<typename T>
  constexpr T nanos_per_frame() {
    static_assert(std::is_floating_point_v<T>);
    return inverse_sample_rate<T>() * static_cast<T>(1e9);
  };

  constexpr int32_t nanoseconds_to_frames(uint64_t ns) {
    float v = 0.5f + sample_rate_nanoseconds<float>() * static_cast<float>(ns);
    Assert(v >= 0.f);
    Assert(v < static_cast<float>(std::numeric_limits<int32_t>::max()));
    return static_cast<int32_t>(v);
  }
    constexpr int ms_to_frames(float duration_ms) {
        Assert(duration_ms >= 0.f);
        auto fval = sample_rate_milliseconds<float>() * duration_ms;
        Assert(fval >= 0.f);
        Assert(fval < static_cast<float>(std::numeric_limits<int>::max()));
        return static_cast<int>( 0.5f + fval );
    }

    template<typename T>
    constexpr T period_in_samples_to_freq(int period) {
        return sample_rate<T>() / static_cast<T>(period);
    }

    constexpr int freq_to_period_in_samples( float freq_hz ) {
        if(freq_hz <= 0.f) {
            return 1;
        }
        return static_cast<int>(SAMPLE_RATE / freq_hz);
    }

    template<typename T>
    constexpr T freq_to_period_in_continuous_samples( T freq_hz ) {
        if(freq_hz <= 0.f) {
            return static_cast<T>(1);
        }
        return sample_rate<T>() / freq_hz;
    }


    template<typename T>
    constexpr T get_nyquist_frequency() {
        return period_in_samples_to_freq<T>(static_cast<T>(2));
    }

    // angle increment unit is "rad / pi"
    template<typename T>
    constexpr T freq_to_angle_increment(T freq) {
        static_assert(std::is_floating_point_v<T>);
        return 2 * freq / SAMPLE_RATE;
    }

    template<typename T>
    constexpr T angle_increment_to_freq(T i) {
        static_assert(std::is_floating_point_v<T>);
        return i * half_sample_rate<T>();
    }

    template<typename T>
    constexpr T freq_to_period_in_seconds(T freq) {
      if(freq <= 0) {
          return 1;
      }
      return static_cast<T>(1) / freq;
    }

    template<typename T>
    constexpr T seconds_to_samples(T seconds) {
      return seconds * SAMPLE_RATE;
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
