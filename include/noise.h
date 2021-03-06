
namespace imajuscule::audio {
static inline float white_gaussian_noise(float f = 0.f) {
  return std::normal_distribution<float>{0.f, 1.f}(mersenne<SEEDED::No>());
}

static inline float white_atom_noise(float) {
  return (0==std::uniform_int_distribution<>{0,1}(mersenne<SEEDED::No>())) ? 1.f : -1.f;
}

struct InterpolatedSignal {
#ifndef NDEBUG
  friend class GaussianPinkNoiseAlgo;
#endif
  
  void step(int c, float new_value) {
#ifndef NDEBUG
    stepped_once = true;
    Assert(c == last + 1);
#endif
    update_counter = c;
    prev_value = value;
    value = new_value;
  }
  
  float get(int c, int level)
#ifdef NDEBUG
  const
#endif
  {
#ifndef NDEBUG
    Assert(c == last + 1);
    last = c;
#endif
    Assert(c >= update_counter);
    
    auto period = pow2(level);
    
    float interp = (1 + c - update_counter) / (float) period;
    Assert(interp >= 0.f);
    Assert(interp <= 1.f);
    return interp * value + (1.f-interp) * prev_value;
  }
  
#ifndef NDEBUG
  bool was_stepped_once() const { return stepped_once; }
#endif
private:
  float value = 0.f;
  float prev_value = 0.f;
  int update_counter = 0;
#ifndef NDEBUG
  unsigned int last = -1;
  bool stepped_once = false;
#endif
};

namespace pinkNoise {
constexpr auto lowest_pink_frequency = 10.f; // Hz
constexpr auto n_changes_min_per_sec = lowest_pink_frequency * 2.f;

/*
 level0 : n_changes_per_sec[0] = sampling rate
 level1 : n_changes_per_sec[1] = sampling rate/2
 [...]
 leveln : n_changes_per_sec[n] = sampling rate/2^n
 
 n must be chosen such that :
 n_changes_per_sec[n] <= n_changes_min_per_sec
 => sampling rate/2^n <= n_changes_min_per_sec
 => sampling rate/n_changes_min_per_sec <= 2^n
 => log2(sampling rate/n_changes_min_per_sec) <= n
 */

constexpr auto n_levels(int const sample_rate) {
  return relevantBits(static_cast<unsigned int>(sample_rate / n_changes_min_per_sec));
}
}

// http://www.firstpr.com.au/dsp/pink-noise/

struct GaussianPinkNoiseAlgo {
  GaussianPinkNoiseAlgo(int const sample_rate)
  : levels(pinkNoise::n_levels(sample_rate)) {
    // we initialize the levels, and counter
    
    struct L {
      L(int sample_rate) {
        LG(INFO, "generating pink noise using %d levels of gaussian white noise...", pinkNoise::n_levels(sample_rate));
      }
      ~L() {
        LG(INFO, "... done");
      }
    } l(sample_rate);
    
    for(int i=levels.size()-1; i >= 1; --i) {
      counter = (1 << (levels.size()-2)) - (1 << (i-1));
#ifndef NDEBUG
      levels[i].last = counter-1; // to satisfy assert in levels[i].step
#endif
      do_step();
    }
    
#ifndef NDEBUG
    for(int i=0; i<levels.size(); ++i) {
      levels[i].last = counter; // to satisfy assert in levels[i].step
    }
#endif
  }
  
  void step() {
    ++counter;
    do_step(0);
    do_step();
  }
  
  float get()
#ifdef NDEBUG
  const
#endif
  {
    return compute();
  }
  
private:
  unsigned int counter;
  
  std::vector<InterpolatedSignal> levels;
  
  void do_step() {
    unsigned int index;
    if (likely(counter)) {
      index = 1 + count_trailing_zeroes(counter);
    } else {
      index = 1 + sizeof(decltype(counter)) * 8;
    }
    if(index > levels.size()-1) {
      index = levels.size()-1;
    }
    do_step(index);
  }
  
  void do_step(int index) {
    levels[index].step(counter, white_gaussian_noise());
  }
  
  float compute()
#ifdef NDEBUG
  const
#endif
  {
    auto sum = 0.f;
    for(auto i=0; i<levels.size(); ++i) {
      auto & s = levels[i];
      sum += s.get(counter, i);
    }
    return sum/levels.size();
  }
};


enum class NoiseType {
  Pink,
  White
};

template<typename T>
struct GaussianGreyNoiseAlgo {
  using FPT = T;
  
  GaussianGreyNoiseAlgo(int sample_rate,
                        NoiseType source,
                        unsigned int fft_length,
                        unsigned int NumTaps)
  : source(source)
  , sample_rate_(sample_rate)
  , loudness_compensation_filter(sample_rate, fft_length, NumTaps) {
    ScopedLog l("Pre-fill", "loudness compensation filter");
    int n = loudness_compensation_filter.size() +
    loudness_compensation_filter.getLatency().toInteger();
    for(int i=0; i<n; ++i) {
      // fill filter
      step();
    }
  }
  
  T step() {
    auto const & noise = [this]() -> soundBuffer<double> const & {
      switch(source) {
        case NoiseType::White:
          return getWhiteNoise(sample_rate_);
        case NoiseType::Pink:
          return getPinkNoise(sample_rate_);
      }
    }();
    assert(counter < noise.size());
    
    auto r = loudness_compensation_filter.step(noise[counter]);
    ++counter;
    if(counter == noise.size()) {
      counter = 0;
    }
    return r;
  }
  
private:
  unsigned int counter = 0;
  int sample_rate_;
  audio::audioelement::LoudnessCompensationFilterWithLatency<T> loudness_compensation_filter;
  NoiseType source;
};


template<typename T>
auto make_loudness_adapted_noise(int sample_rate,
                                 NoiseType t,
                                 unsigned int fft_length,
                                 unsigned int NumTaps) -> GaussianGreyNoiseAlgo<T>
{
  return { sample_rate, t, fft_length, NumTaps };
}
}
