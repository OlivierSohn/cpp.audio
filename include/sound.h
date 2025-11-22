
namespace imajuscule::audio {

struct Sound {
  enum Type : unsigned char {
    NOISE, // white, gaussian
    ATOM_NOISE, // white, -1 or 1
    PINK_NOISE, // pink, gaussian
    GREY_NOISE, // grey, gaussian
    
    END_NOISE,
    
    SINE = END_NOISE,
    TRIANGLE,
    SAW,
    SQUARE,
    SILENCE,
    ONE
  } type : 4;
  
  static constexpr auto ConstantSoundDummyFrequency = 1.f;
  
  constexpr bool zeroOnPeriodBoundaries() const { return type == SINE || type == TRIANGLE; }
  constexpr bool operator == (const Sound & other) const { return type == other.type; }
  constexpr bool operator < (const Sound & other) const { return type < other.type; }
  Sound(Type t) : type(t) {}
  Sound() = default;
  
  constexpr int minimalPeriod() const {
    switch(type) {
      case SINE:
        return 3;
      case SQUARE:
        return 3;
      case TRIANGLE:
        return 2;
      case SAW:
        return 3; // 2 would give the same result as a triangle
      case NOISE:
      case ATOM_NOISE:
      case PINK_NOISE:
      case GREY_NOISE:
        return 1;
      case SILENCE:
        return 0;
      case ONE:
        return 0;
    }
    return 1;
  }
};

struct soundId {
  soundId() = default;
  
  soundId(int sample_rate, Sound sound, float freq_hz = 1.f )
  : sound(sound)
  , period_length( (sound == Sound::SILENCE) ? 1 : freq_to_period_in_samples( freq_hz, sample_rate ) )
  , sample_rate_(sample_rate)
  {}
  
  Sound sound;
  int32_t period_length;
  int sample_rate_;
  bool operator < (const soundId & o) const {
    return std::make_tuple(sound, period_length, sample_rate_) < std::make_tuple(o.sound, o.period_length, o.sample_rate_);
  }
};

static constexpr double soundBaseVolume(Sound::Type t) {
  switch(t) {
    case Sound::SILENCE:
      return 1.;
    case Sound::ONE:
      return 1.;
    case Sound::NOISE:
      return 0.5;
    case Sound::ATOM_NOISE:
      return 0.5;
    case Sound::PINK_NOISE:
      return 0.6;
    case Sound::GREY_NOISE:
      return 0.5;
    case Sound::SINE:
      return 1.; // TODO adjust?
    case Sound::TRIANGLE:
      return 1.;
    case Sound::SAW:
      return 0.3;
    case Sound::SQUARE:
      return 0.2;
  }
}

template<typename V>
void normalize_audio(V & values)
{
  using T = typename V::value_type;
  static_assert(std::is_floating_point_v<T>);
  if(values.empty()) {
    return;
  }
  range<float> r;
  for(auto v: values) {
    r.extend(v);
  }
  Assert(!r.empty());
  if(r.delta() < 0.f) {
    return;
  }
  
  auto M = std::max(-r.getMin(), r.getMax());
  float just_below_one = 1.f - FLOAT_EPSILON;
  M = just_below_one / M; // just_below_one is to be sure the signal doesn't go over 1 with numerical errors
  for(auto & v: values) {
    v *= M;
  }  
}

template<typename T>
struct soundBuffer {
  using value_type = T;
  using FPT = value_type;
  
  using buffer = a64::vector<value_type>;
  static constexpr bool computable = false;
  
  
  // no copy
  soundBuffer(const soundBuffer &) = delete;
  soundBuffer & operator=(const soundBuffer&) = delete;
  
  soundBuffer(soundBuffer &&) = default;
  soundBuffer& operator = (soundBuffer &&) = default;
  
  bool empty() const { return values.empty(); }
  auto size() const { return values.size(); }
  
  auto begin() const { return values.begin(); }
  auto end() const { return values.end(); }
  
  auto cbegin() const { return values.cbegin(); }
  auto cend() const { return values.cend(); }
  
  auto operator [] (int i) const { return values[i]; }
  
  soundBuffer(size_t n, float value) : values(n, value) {}
  
  soundBuffer( soundId const & );
  
  auto & getBuffer() { return values; }
  
  void logSummary(int nsamples_per_extremity = 3) const;
private:
  template < typename F >
  void generate( int period, F );
  
  template < typename F >
  void generate_with_smooth_transition( int period, F );
  
  auto begin() { return values.begin(); }
  auto end() { return values.end(); }
  
  auto & operator [] (int i) { return values[i]; }
  
  buffer values;
  
  void normalize();
};

template<typename T, size_t N>
T * editInactiveAudioElement(std::array<T, N> & aes) {
  auto it = std::find_if(aes.begin(), aes.end(), [](T const & elt){ return elt.isInactive(); });
  return (it == aes.end()) ? nullptr : &*it;
}

template<Sound::Type SOUND>
struct FGetBuffer;

template<Sound::Type SOUND>
struct BufferIter {
  using F_GET_BUFFER = FGetBuffer<SOUND>;
  using FPT = float;
  
  void set_sample_rate(int sample_rate) {
    sample_rate_ = sample_rate;
    soundBuffer<double> const & buf = F_GET_BUFFER()(sample_rate_);
    end = buf.cend();
    initializeForRun();
  }
  
  void initializeForRun() {
    soundBuffer<double> const & buf = F_GET_BUFFER()(sample_rate_);
    it = buf.cbegin();
    Assert(end == buf.cend());
    // randomize start position
    auto add = static_cast<int>(std::uniform_real_distribution<>{
      0.f,
      static_cast<float>(buf.size()-1)
    }(mersenne<SEEDED::No>()));
    Assert(add < buf.size());
    it += add;
    if (it >= end) {
      std::cout << std::distance(end, it) << std::endl;
    }
    Assert(it < end);
  }
  
  void log() const {
    LG(INFO, "pink noise iterator @[%d]", getPosition());
  }
  
  void operator ++() {
    ++it;
    if(it == end) {
      it = F_GET_BUFFER()(sample_rate_).cbegin();
    }
  }
  
  float operator *() const {
    auto v = *it;
    Assert(v <=  1.f);
    Assert(v >= -1.f);
    return v;
  }
  
  int getPosition() const { return static_cast<int>(std::distance(F_GET_BUFFER()(sample_rate_).cbegin(), it)); }
  
  float getAbsMean() const { return F_GET_BUFFER().getAbsMean(sample_rate_); }
private:
  int sample_rate_;
  decltype(F_GET_BUFFER()(0).cbegin()) it, end;
};

/////////////////
// instantiations
/////////////////

soundBuffer<double> const & getWhiteNoise(int sample_rate);
float getWhiteNoiseAbsMean(int sample_rate);

soundBuffer<double> const & getPinkNoise(int sample_rate);
float getPinkNoiseAbsMean(int sample_rate);

soundBuffer<double> const & getGreyNoise(int sample_rate);
float getGreyNoiseAbsMean(int sample_rate);

template<>
struct FGetBuffer<Sound::NOISE> {
  soundBuffer<double> const & operator()(int sample_rate) const {
    return getWhiteNoise(sample_rate);
  }
  float getAbsMean(int sample_rate) const {
    return getWhiteNoiseAbsMean(sample_rate);
  }
  float getAngleIncrements() const {
    return 0.5f; // this is used to evaluate the change rate of a signal, so for a sinus, pi/2 leads to the most changes
  }
};
template<>
struct FGetBuffer<Sound::PINK_NOISE> {
  soundBuffer<double> const & operator()(int sample_rate) const {
    return getPinkNoise(sample_rate);
  }
  float getAbsMean(int sample_rate) const {
    return getPinkNoiseAbsMean(sample_rate);
  }
  float getAngleIncrements() const {
    return 0.5f; // this is used to evaluate the change rate of a signal, so for a sinus, pi/2 leads to the most changes
  }
};
template<>
struct FGetBuffer<Sound::GREY_NOISE> {
  soundBuffer<double> const & operator()(int sample_rate) const {
    return getGreyNoise(sample_rate);
  }
  float getAbsMean(int sample_rate) const {
    return getGreyNoiseAbsMean(sample_rate);
  }
  float getAngleIncrements() const {
    return 0.5f; // this is used to evaluate the change rate of a signal, so for a sinus, pi/2 leads to the most changes
  }
};

using PinkNoiseIter = BufferIter<Sound::PINK_NOISE>;
using GreyNoiseIter = BufferIter<Sound::GREY_NOISE>;
}
