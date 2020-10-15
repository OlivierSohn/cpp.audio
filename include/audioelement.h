namespace imajuscule::audio::audioelement {

constexpr auto n_frames_per_buffer = 16;

namespace buffer {

// AudioComponent<float> has a buffer of size 1 cache line
// AudioComponent<double> has a buffer of size 2 cache lines
// each of them have 16 frames worth of data in their buffer
static constexpr auto alignment = cache_line_n_bytes; // 64 or 32

static constexpr auto index_state = 0;

// state values must be distinct from every possible valid value
template<typename T>
constexpr T queued_state() { return -std::numeric_limits<T>::infinity(); } // in *** at most *** one queue
template<typename T>
constexpr T inactive_state() { return std::numeric_limits<T>::infinity(); }// not active in any queue

template<typename T>
auto & state(T * buffer) { return buffer[index_state]; }

} // NS


// lifecycle :
// upon creation, state is inactive()
// when in a queue state is queued()
// when processed state is a float or double
// when done being played state is inactive()
template<typename T>
struct AEBuffer {
  AEBuffer() {
    // assert deactivated as it fails on iphone / iphone simulator. I think I need to implement
    // a freelist of blocks of cache line size to get around this issue related to overaligned types.
    //Assert(0 == reinterpret_cast<unsigned long>(buffer) % buffer_alignment);
    buffer::state(buffer) = buffer::inactive_state<T>();
  }

  // no copy or move because the lambda returned by fCompute() captures 'this'
  AEBuffer(const AEBuffer &) = delete;
  AEBuffer & operator=(const AEBuffer&) = delete;
  AEBuffer(AEBuffer &&) = delete;
  AEBuffer& operator = (AEBuffer &&) = delete;

  ////// [AEBuffer] beginning of the 1st cache line

  alignas(buffer::alignment)
  T buffer[n_frames_per_buffer];

  ////// [AEBuffer<float>] beginning of the 2nd cache line
  ////// [AEBuffer<double>] beginning of the 3rd cache line

  constexpr bool isInactive() const { return getState() == buffer::inactive_state<T>(); }
  auto getState() const { return buffer::state(buffer); }
};

// !! usage of FinalAudioElement is deprecated
template<typename ALGO>
struct FinalAudioElement {
  static constexpr auto hasEnvelope = ALGO::hasEnvelope;
  static constexpr bool computable = true;
  static constexpr auto baseVolume = ALGO::baseVolume;
  using FPT = typename ALGO::FPT;
  static_assert(std::is_floating_point<FPT>::value);
  using buffer_t = AEBuffer<FPT>;

  // no copy or move because the lambda returned by fCompute() captures 'this'
  FinalAudioElement(const FinalAudioElement &) = delete;
  FinalAudioElement & operator=(const FinalAudioElement&) = delete;
  FinalAudioElement(FinalAudioElement &&) = delete;
  FinalAudioElement& operator = (FinalAudioElement &&) = delete;

  template <class... Args>
  FinalAudioElement(buffer_t & b, Args&&... args) : buffer(&b), algo(std::forward<Args>(args)...) {}

  void forgetPastSignals() {
    algo.forgetPastSignals();
  }
  template <class U=ALGO,typename=std::enable_if_t<U::hasEnvelope>>
  auto & getEnvelope() const {
    return algo.getEnvelope();
  }
  template <class U=ALGO,typename=std::enable_if_t<U::hasEnvelope>>
  auto & editEnvelope() {
    return algo.editEnvelope();
  }

  constexpr bool isInactive() const { return buffer->isInactive(); }
  auto getState() const { return buffer->getState(); }

  AEBuffer<FPT> * buffer;
  ALGO algo;

  bool compute(int const nFrames, uint64_t const tNanos) {

    FPT * buf = buffer->buffer;
    auto st = buffer::state(buf);

    if(st == buffer::inactive_state<FPT>()) {
      // Issue : if the buffer just got marked inactive,
      // but no new AudioElementCompute happens
      // and from the main thread someone acquires this and queues it,
      // it will have 2 lambdas because the first lambda will never have seen the inactive state.
      // However the issue is not major, as the 2 lambdas have a chance to be removed
      // the next time
      if constexpr (hasEnvelope) {
        // Temporary, to fix soundengine, where the channel xfade is used instead of the enveloppe xfade:
        // this lets the audioelement be reused in subsequent markov moves
        // and resets the enveloppe state to Done2.
        forgetPastSignals();
      }
      return false;
    }

    Assert(nFrames > 0);
    Assert(nFrames <= n_frames_per_buffer);
    for(int i=0; i != nFrames; ++i) {
      algo.step();
      buf[i] = algo.imag();
    }
    Assert(buffer::state(buf) != buffer::queued_state<FPT>());
    Assert(buffer::state(buf) != buffer::inactive_state<FPT>());
    if constexpr (hasEnvelope) {
      // it is important that isEnvelopeFinished() returns true only one buffer cycle after
      // the real enveloppe end, to avoid race conditions.
      if(getEnvelope().isEnvelopeFinished()) {
        return false;
      };
    }
    return true;
  }

  folly::Function<bool(const int  // the number of frames to compute
                       , const uint64_t // the time of the first frame
                       )> fCompute() {
    return [this](int const nFrames, uint64_t const tNanos) {
      return compute(nFrames, tNanos);
    };
  }
};


/*
 The envelope is used by a compute lambda in the audio thread when it is in states:

 KeyPressed
 KeyReleased
 EnvelopeDone1

 Once the envelope is in state 'EnvelopeDone2', it is not used anymore by the audio thread.
 The first thread doing a "compare and swap" from 'EnvelopeDone2' to 'SoonKeyPressed'
 acquires ownership of the envelope.
 */
enum class EnvelopeState : unsigned char {
  SoonKeyPressed
  , KeyPressed // the envelope is scheduled to be raising, raising or sustained
  , KeyReleased // the envelope is closing
  , EnvelopeDone1 // the envelope has closed, but the last buffer contains samples where the enveloppe was still opened.
  , EnvelopeDone2 // the envelope has closed, the last buffer contains only samples where the enveloppe was closed.
};

inline const char * toString(EnvelopeState s) {
  switch(s) {
    case EnvelopeState::SoonKeyPressed:
      return "SoonKeyPressed";
    case EnvelopeState::KeyPressed:
      return "KeyPressed";
    case EnvelopeState::KeyReleased:
      return "KeyReleased";
    case EnvelopeState::EnvelopeDone1:
      return "EnvelopeDone1";
    case EnvelopeState::EnvelopeDone2:
      return "EnvelopeDone2";
  }
  return "what?";
}

template <typename ALGO, typename Envelope>
struct Enveloped {
  using MeT = Enveloped<ALGO, Envelope>;
  static constexpr auto hasEnvelope = true;
  static constexpr auto baseVolume = ALGO::baseVolume;
  static constexpr auto isMonoHarmonic = ALGO::isMonoHarmonic;
  static constexpr int count_channels = ALGO::count_channels;

  using FPT = typename ALGO::FPT;
  static_assert(std::is_same<typename ALGO::FPT, typename Envelope::FPT>::value);

  void forgetPastSignals() {
    env.forgetPastSignals();
    algo.forgetPastSignals();
  }

  void step() {
    env.step();
    algo.step();
  }

  void setAngle(FPT a) {
    algo.setAngle(a);
  }
  FPT angle() const { return algo.angle(); }

  FPT angleIncrements() const { return algo.angleIncrements(); }

  void synchronizeAngles(MeT const & other) {
    algo.synchronizeAngles(other.algo);
  }

  // [Based on observations]
  // The attack and release lengths need to be longer for lower frequency notes,
  // else we begin to hear cracks.
  static constexpr FPT characTimeMultiplier = static_cast<FPT>(2.5);

  void setAngleIncrements(FPT a)
  {
    {
      FPT const signalPeriodSamples = angle_increment_to_period_in_continuous_samples(a);
      env.setMinChangeDurationSamples(static_cast<int>(0.5f + characTimeMultiplier * signalPeriodSamples));
    }
    algo.setAngleIncrements(a);
  }

  FPT real() const { return algo.real() * env.value(); }
  FPT imag() const { return algo.imag() * env.value(); }

  auto const & getOsc() const { return algo.getOsc(); }
  auto       & getOsc()       { return algo.getOsc(); }
  auto & getAlgo() { return algo; }
  auto const & getEnvelope() const { return env; }
  auto & editEnvelope() { return env; }


  bool tryAcquire() {
    return env.tryAcquire();
  }

  bool acquireStates() const {
    return env.acquireStates();
  }

  void onKeyPressed(int32_t delay) {
    env.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    env.onKeyReleased(delay);
  }
  bool canHandleExplicitKeyReleaseNow(int32_t delay) const {
    return env.canHandleExplicitKeyReleaseNow(delay);
  }
  bool isEnvelopeFinished() const {
    return env.isEnvelopeFinished();
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    algo.setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
  }

  void set_sample_rate(int s) {
    env.set_sample_rate(s);
    algo.set_sample_rate(s);
  }

#ifndef NDEBUG
  void logDiagnostic() {
    LG(INFO, "envelope:");
    env.logDiagnostic();
  }
#endif

private:
  Envelope env;
  ALGO algo;
};


/* The AHDSR envelope is like an ADSR envelope, except we allow to hold the value after the attack:

 | a |h| d |           |r|
 ---                                      < 1
 .   .
 .      .-------------                      < s
 .                     .
 ---                       -------------------  < 0
 ^                     ^ ^             ^
 |                     | envelopeDone1 envelopeDone2    <- state changes
 keyPressed            keyReleased                      <- state changes
 attacking                                              <- inner pressed state changes
 holding                                            <- inner pressed state changes
 decaying                                         <- inner pressed state changes
 sustaining                                    <- inner pressed state changes

 Attack, decay and release interpolations are configurable.

 */
struct AHDSR {
  int32_t attack;
  itp::interpolation attackItp;
  int32_t hold;
  int32_t decay;
  itp::interpolation decayItp;
  int32_t release;
  itp::interpolation releaseItp;
  float sustain;

  std::size_t combine_hash(std::size_t h) const {
    hash_combine(h, attack);
    hash_combine(h, attackItp);
    hash_combine(h, hold);
    hash_combine(h, decay);
    hash_combine(h, decayItp);
    hash_combine(h, release);
    hash_combine(h, releaseItp);
    hash_combine(h, sustain);
    return h;
  }
};
inline bool operator < (AHDSR const& l, AHDSR const& r)
{
  return
  std::make_tuple(l.attack,l.attackItp,l.hold,l.decay,l.decayItp,l.release,l.releaseItp,l.sustain) <
  std::make_tuple(r.attack,r.attackItp,r.hold,r.decay,r.decayItp,r.release,r.releaseItp,r.sustain);
}
inline bool operator == (AHDSR const& l, AHDSR const& r)
{
  return
  std::make_tuple(l.attack,l.attackItp,l.hold,l.decay,l.decayItp,l.release,l.releaseItp,l.sustain) ==
  std::make_tuple(r.attack,r.attackItp,r.hold,r.decay,r.decayItp,r.release,r.releaseItp,r.sustain);
}
inline bool operator != (AHDSR const& l, AHDSR const& r)
{
  return !(l == r);
}

template <Atomicity A>
struct EnvelopeStateAcquisition {
  using stateTraits = maybeAtomic<A,EnvelopeState>;
  using stateType = typename stateTraits::type;

  EnvelopeStateAcquisition() {
    stateTraits::write(state, EnvelopeState::EnvelopeDone2, std::memory_order_relaxed);
  }

  // called from nrt threads and rt thread
  bool tryAcquire() {
    auto cur = EnvelopeState::EnvelopeDone2;
    return stateTraits::compareExchangeStrong(state,
                                              cur,
                                              EnvelopeState::SoonKeyPressed,
                                              std::memory_order_acq_rel);
  }

  /* Eventhough this method is 'const', it ensures that after it returned,
   * the thread owns the state.
   */
  bool acquireStates() const {
    return EnvelopeState::SoonKeyPressed == stateTraits::read(state, std::memory_order_acquire);
  }

  auto getRelaxedState() const { return stateTraits::read(state, std::memory_order_relaxed); }

  void relaxedWrite(EnvelopeState s) {
    stateTraits::write(state, s, std::memory_order_relaxed);
  }

  bool isEnvelopeSoonKeyPressed() const {
    return getRelaxedState() == EnvelopeState::SoonKeyPressed;
  }
  bool isEnvelopeFinished() const {
    return getRelaxedState() == EnvelopeState::EnvelopeDone2;
  }
  bool isEnvelopeRTActive() const {
    auto const s = getRelaxedState();
    return
    s != EnvelopeState::SoonKeyPressed &&
    s != EnvelopeState::EnvelopeDone2;
  }

private:
  stateType state;
};

inline bool isAudible(harmonicProperties_t const & h) {
  return std::abs(h.volume) > 0.000001f;
}

/* returns -1 when no harmonic is audible. */
template <typename Arr>
int indexOfLastAudibleHarmonic(Arr const & props) {
  auto sz = props.size();
  for(int i = sz-1; i >= 0; --i) {
    if (isAudible(props[i])) {
      return i;
    }
  }
  return -1;
}

// Returns a phase in [0,2] interval
template<typename T>
T phaseToNormalForm(T phase) {
  constexpr T modPhase = 2;
  T nfPhase = std::fmod(phase,modPhase);
  if(nfPhase < 0) {
    nfPhase += modPhase;
  }
  Assert(nfPhase >= 0);
  Assert(nfPhase <= modPhase);
  return nfPhase;
}

template<typename Arr>
std::size_t hashHarmonics(Arr const & harmonics) {
  std::size_t res = 0;
  auto add_to_hash = [&res](auto v) {
    hash_combine(res, v);
  };

  int n = 1 + indexOfLastAudibleHarmonic(harmonics);

  // TODO should we allow small differences by reducing the float precision?
  for(auto const & har : take(n, harmonics)) {
    float volume(0.f), nfPhase(0.f);
    // Even if an harmonic is not audible, we must add information in the hash,
    // else [1,0,1] would have the same hash as [1,1].
    if(isAudible(har)) {
      volume = har.volume;
      nfPhase = phaseToNormalForm(har.phase);
    }
    add_to_hash(volume);
    add_to_hash(nfPhase);
  }


  return res;
}

template<typename FPT>
constexpr FPT harmonic_angle(int i, FPT a) {
  return static_cast<FPT>(i) * a;
}

/*
 * Returns the multiplicator that should be applied to a signal of
 * the given frequency. The returned value ensures that we have a smooth
 * transition, when we approach the frequency aliasing limit.
 */
template <typename T>
constexpr T freqAliasingMultiplicator(T angle_increment) {
  if(angle_increment == static_cast<T>(0)) {
    return static_cast<T>(1);
  }
  auto halfNSamplesPerPeriod = static_cast<T>(1) / angle_increment;
  constexpr auto minHalfSPP = static_cast<T>(1);
  constexpr auto maxHalfSPP = static_cast<T>(4);
  constexpr auto invDist = static_cast<T>(1) / (maxHalfSPP - minHalfSPP);
  if(halfNSamplesPerPeriod < minHalfSPP) {
    return static_cast<T>(0);
  }
  if(halfNSamplesPerPeriod > maxHalfSPP) {
    return static_cast<T>(1);
  }
  // halfNSamplesPerPeriod is now in [minHalfSPP, maxHalfSPP]
  return (halfNSamplesPerPeriod - minHalfSPP) * invDist;
}

template <typename ALGO, typename Envelope>
struct MultiEnveloped {
  using MeT = MultiEnveloped<ALGO, Envelope>;

  static constexpr bool hasEnvelope = true;
  static constexpr auto baseVolume = ALGO::baseVolume;
  static constexpr auto isMonoHarmonic = ALGO::isMonoHarmonic;
  static constexpr int count_channels = ALGO::count_channels;

  static constexpr auto atomicity = Envelope::atomicity;

  using NonAtomicEnvelope = typename Envelope::NonAtomic;
  // all inner-envelopes state reads / writes are done in the realtime
  // thread, so we don't need atomicity:
  using EA = Enveloped<ALGO, NonAtomicEnvelope>;

  using FPT = typename ALGO::FPT;

  void set_sample_rate(int) {
    // see setHarmonics
  }

  template <typename Arr>
  void setHarmonics(Arr const & props, int const sample_rate) {
    harmonics.clear();

    // discard the last consecutive harmonics of zero volume.
    int sz = 1 + indexOfLastAudibleHarmonic(props);
    Assert(sz >= 0);
    Assert(sz <= props.size());

    harmonics.reserve(sz);
    for(int i=0; i<sz; ++i) {
      harmonics.emplace_back(EA{},props[i]);
      harmonics.back().first.set_sample_rate(sample_rate);
    }
  }

  void forgetPastSignals() {
    // do not touch stateAcquisition

    forEachHarmonic([](auto & algo) { algo.forgetPastSignals(); } );
  }

  void step() {
    imagValue = {};
    bool goOn = false;
    for(auto & [algo,property] : harmonics) {
      // we could skip zero-volume harmonics.
      algo.step();
      if(!goOn) {
        if(!algo.getEnvelope().isEnvelopeFinished()) {
          goOn = true;
        }
      }
      imagValue += algo.imag() * property.volume;
    }
    if(!goOn) {
      stateAcquisition.relaxedWrite(EnvelopeState::EnvelopeDone2);
    }
  }

  auto &       editEnvelope()      { return *this; }
  auto const & getEnvelope() const { return *this; }

  void setAHDSR(AHDSR const & s, int const sample_rate) {
    forEachHarmonic([&s, sample_rate](auto & algo) { algo.editEnvelope().setAHDSR(s, sample_rate); } );
  }

  bool tryAcquire() {
    return stateAcquisition.tryAcquire();
  }

  bool acquireStates() const {
    return stateAcquisition.acquireStates();
  }

  void onKeyPressed(int32_t delay) {
    forEachHarmonic([delay](auto & algo) { algo.editEnvelope().onKeyPressed(delay); } );
    stateAcquisition.relaxedWrite(EnvelopeState::KeyPressed);
  }
  void onKeyReleased(int32_t delay) {
    forEachHarmonic([delay](auto & algo) { algo.editEnvelope().onKeyReleased(delay); } );
    // we do not write "key released" in stateAcquisition here, because maybe the envelope has been finished already in step()
  }
  bool canHandleExplicitKeyReleaseNow(int32_t delay) const {
    // using any_of here to support (in the future) having
    //   ** some ** of the envelopes autorelease.
    return std::any_of(harmonics.begin(),
                       harmonics.end(),
                       [delay](auto const & a){ return a.first.getEnvelope().canHandleExplicitKeyReleaseNow(delay); });
  }
  bool isEnvelopeFinished() const {
    return stateAcquisition.isEnvelopeFinished();
  }
  bool isEnvelopeSoonKeyPressed() const {
    return stateAcquisition.isEnvelopeSoonKeyPressed();
  }
  bool isEnvelopeRTActive() const {
    return stateAcquisition.isEnvelopeRTActive();
  }

  auto       & getOsc()       { return *this; }
  auto const & getOsc() const { return *this; }

  void setAngle(FPT a) {
    forEachIndexedHarmonic([a](int i, auto & algo, auto const & property) {
      algo.setAngle(property.phase + harmonic_angle(i,a));
    });
  }

  void synchronizeAngles(MeT const & other) {
    Assert(harmonics.size() == other.harmonics.size());
    for(int i=0, sz = std::min(harmonics.size(), other.harmonics.size());
        i != sz;
        ++i) {
      harmonics[i].first.getOsc().synchronizeAngles(other.harmonics[i].first.getOsc());
    }
  }

  /* Returns the angle of the first harmonic. */
  FPT angle() const {
    if(unlikely(harmonics.empty())) {
      return static_cast<FPT>(0);
    }
    return harmonics[0].first.angle() - harmonics[0].second.phase;
  }

  /* Where the i-th harmonic has a frequency i times the frequency of the first harmonic */
  void setAngleIncrements(FPT a)
  {
    forEachIndexedHarmonic([a](int i, auto & algo, auto const & property [[maybe_unused]]) {
      algo.setAngleIncrements(harmonic_angle(i,a));
    });
  }

  FPT angleIncrements() const {
    return harmonics[0].first.angleIncrements();
  }

  FPT imag() const { return imagValue; }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    forEachHarmonic([sample_rate, low_index, log_ratio, loudness_level](auto & algo) {
      algo.setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
    });
  }

private:
  // TODO we could omit harmonics that have a 0 volume, and store the harmonic index in the vector,
  //      synchronizeAngles must be rewritten carefully.
  std::vector<std::pair<EA, harmonicProperties_t>> harmonics;
  EnvelopeStateAcquisition<atomicity> stateAcquisition;
  FPT imagValue;

public:
  template<typename F>
  void forEachHarmonic(F f) {
    for(auto & [a,_] : harmonics) {
      f(a);
    }
  }

  template<typename F>
  void forEachIndexedHarmonic(F f) {
    int i=0;
    for(auto & [algo,property] : harmonics) {
      ++i;
      f(i,algo,property);
    }
  }

};

enum class EnvelopeRelease {
  WaitForKeyRelease,
  ReleaseAfterDecay
};
std::ostream & operator << (std::ostream &, EnvelopeRelease);

template <Atomicity A, typename Base>
struct EnvelopeCRT : public Base {
  static constexpr auto atomicity = A;
  using FPT = typename Base::FPT;
  using T = FPT;
  static constexpr EnvelopeRelease Release = Base::Release;

  using NonAtomic = EnvelopeCRT<Atomicity::No, Base>;

  using Base::startPressed;
  using Base::stepPressed;
  using Base::getReleaseItp;
  using Base::getReleaseTime;
  using Base::isAfterAttackBeforeSustain;
  using Base::updateMinChangeDuration;

  /* on success, the thread calling this method has acquired ownership of the
   envelope. */
  bool tryAcquire() {
    return stateAcquisition.tryAcquire();
    // we don't 'onKeyPressed' yet : maybe the audio element using
    // this envelope needs to be initialized first.
  }

  /* Makes sure this thread owns the state. */
  bool acquireStates() const {
    return stateAcquisition.acquireStates();
  }

  void forgetPastSignals() {
    // do not touch stateAcquisition

    counter = 0;
    updateMinChangeDuration();
    // we don't set '_value', step() sets it.
  }

  void step() {
    if(currentReleaseDelay >= 0) {
      if(currentReleaseDelay == 0) {
        startReleased();
      }
      --currentReleaseDelay;
    }
    ++counter;
    switch(getRelaxedState()) {
      case EnvelopeState::KeyPressed:
      {
        constexpr auto firstCounter = 1;
        if(counter < firstCounter) {
          // we are not done with the delay yet.
          return;
        }
        if(unlikely(counter == firstCounter)) {
          startPressed();
        }
        auto maybeV = stepPressed(counter);
        if(likely(maybeV)) {
          _value = *maybeV;
        }
        else {
          onKeyReleased(0);
          step(); // recursion, and note that we won't infinite loop because
          // onKeyReleased() changes the state.
        }
      }
        break;
      case EnvelopeState::KeyReleased:
        _value = itp::interpolate(getReleaseItp()
                                  , static_cast<T>(counter)
                                  , _topValue
                                  , - _topValue
                                  , static_cast<T>(getReleaseTime()));
        if(counter >= getReleaseTime()) {
          stateAcquisition.relaxedWrite(EnvelopeState::EnvelopeDone1);
          counter = 0;
          _value = static_cast<T>(0);
        }
        break;
      case EnvelopeState::EnvelopeDone1:
        if(counter > n_frames_per_buffer) { // to be sure that all non-zero computed signals
          // were used (this is ony true when using FinalAudioElement buffering.)
          stateAcquisition.relaxedWrite(EnvelopeState::EnvelopeDone2);
          counter = 0;
        }
        [[fallthrough]];
      case EnvelopeState::EnvelopeDone2:
      case EnvelopeState::SoonKeyPressed:
        _value = static_cast<T>(0);
        break;
    }
  }

  T value () const {
    return _value;
  }

  void onKeyPressed(int32_t delay) {
    stateAcquisition.relaxedWrite(EnvelopeState::KeyPressed);
    counter = -delay;
    currentReleaseDelay = -1;
    _value = static_cast<T>(0);
  }

  void onKeyReleased(int32_t delay) {
    Assert(delay >= 0);
    if(getRelaxedState() != EnvelopeState::KeyPressed) {
      return;
    }
    if(counter + delay <= 0) {
      // the key was pressed, but immediately released, so we skip the note.
      stateAcquisition.relaxedWrite(EnvelopeState::EnvelopeDone1);
    }
    else {
      currentReleaseDelay = delay;
    }
  }

  int countStepsToRelease() const {
    return currentReleaseDelay;
  }

  bool isEnvelopeRTActive() const {
    return stateAcquisition.isEnvelopeRTActive();
  }
  bool isEnvelopeFinished() const {
    return stateAcquisition.isEnvelopeFinished();
  }
  bool isEnvelopeSoonKeyPressed() const {
    return stateAcquisition.isEnvelopeSoonKeyPressed();
  }


  bool afterAttackBeforeSustain() const {
    return isAfterAttackBeforeSustain(counter);
  }

  bool canHandleExplicitKeyReleaseNow(int32_t delay) const {
    if constexpr (Release == EnvelopeRelease::ReleaseAfterDecay) {
      return false;
    }
    else {
      return getRelaxedState() == EnvelopeState::KeyPressed &&
      (currentReleaseDelay == -1) && // else a release is scheduled
      (-counter <= delay); // else the release would happen before the keyPress.
    }
  }

#ifndef NDEBUG
  void logDiagnostic() {
    std::cout << "state:     " << toString(stateAcquisition.getRelaxedState()) << std::endl;
    std::cout << "counter:   " << counter << std::endl;
    std::cout << "_value:    " << _value << std::endl;
    std::cout << "_topValue: " << _topValue << std::endl;
  }
#endif

  auto getRelaxedState() const { return stateAcquisition.getRelaxedState(); }

private:
  // between 0 and 1.
  FPT _value = static_cast<T>(0);
  FPT _topValue = static_cast<T>(0); // the value when the key was released
  EnvelopeStateAcquisition<A> stateAcquisition;
  int32_t counter = 0;
  // counter stores the neagted last 'onKeyPressed' delay.
  // It is incremented at each step. The actual release happens when it is 0.
  int32_t currentReleaseDelay = -1;
  // 'currentDelay' stores the last 'onKeyReleased' delay.
  // It is decremented at each step. The actual release happens when it is 0.

  void startReleased() {
    updateMinChangeDuration();
    stateAcquisition.relaxedWrite(EnvelopeState::KeyReleased);
    _topValue = _value;
    counter = 0;
  }
};

struct WithMinChangeDuration {
  void setMinChangeDurationSamples(int nSamples) {
    // we don't change 'minChangeDuration' now as it could break
    // the envelope continuity.
    nextMinChangeDuration = nSamples;
  }

  // fast moog attacks are 1ms according to
  // cf. https://www.muffwiggler.com/forum/viewtopic.php?t=65964&sid=0f628fc3793b76de64c7bceabfbd80ff
  // so we set the max normalized enveloppe velocity to 1ms (i.e the time to go from 0 to 1)
  static constexpr auto normalizedMinDt(int const sample_rate) { return sample_rate/1000; };

protected:
  int32_t minChangeDuration = 0; // allowed to change when the counter is 0, to avoid discontinuities
  int32_t nextMinChangeDuration = 0; // copied to 'minChangeDuration' when 'ahdCounter' == 0

  // TODO do the same mechanism for all envelope params (AHDSR)
  // to allow realtime envelop changes without audio cracks

  void updateMinChangeDuration() {
    minChangeDuration = nextMinChangeDuration;
  }
  int32_t getMinChangeDuration() const {
    return minChangeDuration;
  }
};

/* This inner state describes states when the outer state is 'KeyPressed',
 and the outer state counter is >= 1. */
enum class AHD : unsigned char {
  Attacking,
  Holding,
  Decaying
};

inline Optional<AHD> rotateAHD(AHD s) {
  switch(s) {
    case AHD::Attacking:
      return AHD::Holding;
    case AHD::Holding:
      return AHD::Decaying;
    case AHD::Decaying:
      return {};
    default:
      Assert(0);
      return {};
  }
}

template <typename T, EnvelopeRelease Rel>
struct AHDSREnvelopeBase : public WithMinChangeDuration {
  using FPT = T;
  using Param = AHDSR;
  static constexpr auto Release = Rel;

  void set_sample_rate(int s) {
  }

  void setAHDSR(AHDSR const & s, int const sample_rate) {
    int32_t const min_dt = normalizedMinDt(sample_rate);

    Assert(min_dt > 0);

    bool const hasDecay = s.sustain < 0.999999;

    SMinusOne =
    hasDecay ?
    clamp_ret(s.sustain,
              0.f,
              1.f) - 1.f:
    0.f;

    A = std::max(s.attack,
                 min_dt );
    H = std::max(s.hold,
                 0);
    D =
    hasDecay ?
    std::max(s.decay,
             min_dt) :
    0;

    R = std::max(s.release,
                 min_dt);
    attackItp = s.attackItp;
    decayItp = s.decayItp;
    releaseItp = s.releaseItp;
  }

protected:
  void startPressed() {
    ahdState = AHD::Attacking;
    onAHDStateChange();
  }

  Optional<T> stepPressed(int) {
    if(ahdState) {
      stepAHD();
      ++ahdCounter;
    }
    if(!ahdState) {
      if constexpr (Rel == EnvelopeRelease::WaitForKeyRelease) {
        return static_cast<T>(1) + static_cast<T>(SMinusOne);
      }
      else {
        return {};
      }
    }
    else {
      T from, toMinusFrom;
      switch(get_value(ahdState)) {
        case AHD::Attacking:
          from = static_cast<T>(0);
          toMinusFrom = static_cast<T>(1);
          break;
        case AHD::Holding:
          from = static_cast<T>(1);
          toMinusFrom = static_cast<T>(0);
          break;
        case AHD::Decaying:
          from = static_cast<T>(1);
          toMinusFrom = static_cast<T>(SMinusOne);
          break;
        default:
          toMinusFrom = from = static_cast<T>(0);
          Assert(0);
          break;
      }

      return itp::interpolate(getInterpolation()
                              , static_cast<T>(ahdCounter)
                              , from
                              , toMinusFrom
                              , static_cast<T>(getMaxCounterForAHD()));
    }
  }

  int32_t getReleaseTime() const {
    return std::max(getMinChangeDuration(), R); // safe release
  }

  itp::interpolation getReleaseItp() const { return releaseItp; }

  bool isAfterAttackBeforeSustain(int) const {
    return static_cast<bool>(ahdState);
  }

private:

  // with an alignment of 4 (largest member)
  // [0 bytes]

  int32_t ahdCounter = 0;
  int32_t A = 100000;
  int32_t H = 0;
  int32_t D = 100000;
  int32_t R = 100000;
  float SMinusOne = -0.5f;

  // [28 bytes]

  // this inner state is taken into account only while the outer state is KeyPressed
  // and the outer state counter is >= 1:
  Optional<AHD> ahdState;

  itp::interpolation attackItp;
  itp::interpolation decayItp;

  // [32 bytes]

  itp::interpolation releaseItp;

  // [36 bytes] (with 3 padding bytes)


  void onAHDStateChange() {
    ahdCounter = 0;

    // We are at a "safe point" where we can change 'minChangeDuration'
    // whilst preserving the envelope continuity:
    updateMinChangeDuration();

    if(!ahdState) {
      return;
    }
    if(0==getMaxCounterForAHD()) {
      stepAHD();
    }
  }

  void stepAHD() {
    Assert(ahdState);
    auto maxCounter = getMaxCounterForAHD();
    Assert(ahdCounter >= 0);
    if (ahdCounter < maxCounter) {
      return;
    }
    ahdState = rotateAHD(get_value(ahdState));
    onAHDStateChange();
  }

  int32_t getMaxCounterForAHD() const {
    switch(get_value(ahdState)) {
      case AHD::Attacking:
        return std::max(minChangeDuration, A); // safe attack
      case AHD::Holding:
        return H;
      case AHD::Decaying:
        return
        (D == 0) ?
        0 : // skip decay
        std::max(minChangeDuration, D); // safe decay
      default:
        Assert(0);
        return 0;
    }
  }

  itp::interpolation getInterpolation() const {
    switch(get_value(ahdState)) {
      case AHD::Attacking:
        return attackItp;
      case AHD::Holding:
        return itp::LINEAR;
      case AHD::Decaying:
        return decayItp;
      default:
        Assert(0);
        return itp::LINEAR;
    }
  }
};

template <Atomicity A, typename T, EnvelopeRelease Rel>
using AHDSREnvelope = EnvelopeCRT < A, AHDSREnvelopeBase <T, Rel> >;

/** Adjusts the volume of a mono-frequency algo.
 *
 * To avoid audible cracks when the volume has a non differentiable shape,
 * the volume adjustment is low-pass filtered with a time constant proportional to
 * the current oscillator frequency.
 */
template<typename ALGO>
struct BaseVolumeAdjusted {
  using MeT = BaseVolumeAdjusted<ALGO>;

  static constexpr auto hasEnvelope = ALGO::hasEnvelope;
  static constexpr auto isMonoHarmonic = ALGO::isMonoHarmonic;
  static constexpr int count_channels = ALGO::count_channels;

  using T = typename ALGO::FPT;
  using FPT = T;
  static_assert(std::is_floating_point<FPT>::value);

  T real() const { Assert(volume); return *volume * osc.real(); }

  template<int C = count_channels>
  std::enable_if_t<C == 1, T>
  imag() const {
    Assert(volume);
    return *volume * osc.imag();
  }

  template<int C = count_channels>
  std::enable_if_t<(C > 1), T>
  imag(int i) const {
    Assert(volume);
    return *volume * osc.imag(i);
  }

  T angleIncrements() const { return osc.angleIncrements(); }
  T angle() const { return osc.angle(); }

  auto       & getOsc()       { return osc; }
  auto const & getOsc() const { return osc; }
  
  template <class U=ALGO,typename=std::enable_if_t<U::hasEnvelope>>
  auto & getEnvelope() const {
    return osc.getEnvelope();
  }
  template <class U=ALGO,typename=std::enable_if_t<U::hasEnvelope>>
  auto & editEnvelope() {
    return osc.editEnvelope();
  }

  BaseVolumeAdjusted() = default;

  void set_sample_rate(int s) {
    osc.set_sample_rate(s);
  }

  // Used to limit the speed of variations.
  //
  // Was first introduced in a use case where
  // the volume is re-evaluated every X samples
  // and we want to avoid a "stair" effect.
  void setMaxFilterIncrement(T ai) {
    max_filter_increment = ai;
  }

  void forgetPastSignals() {
    osc.forgetPastSignals();
    volume.reset();
    volume_target.reset();
  }
  void onKeyPressed(int32_t delay) {
    osc.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    osc.onKeyReleased(delay);
  }

  void setFiltersOrder(int order) {
    osc.setFiltersOrder(order);
  }

  void setAngle(T ai) {
    osc.setAngle(ai);
  }

  void synchronizeAngles(MeT const & other) {
    osc.synchronizeAngles(other.osc);
  }

  void step() {
    Assert(volume_target);  // The user needs to set the volume

    osc.step();

    // low-pass the volume using a time characteristic equal to the period implied by angle increments
    if (unlikely(!volume)) {
      filter_init_with_inc = std::min(max_filter_increment,
                                      std::abs(osc.angleIncrements()));
      volume_filter.initWithAngleIncrement(filter_init_with_inc);
      volume_filter.setInitialValue(*volume_target);
    } else if (*volume != *volume_target) {
      auto const inc = std::min(max_filter_increment,
                                std::abs(osc.angleIncrements()));
      if (filter_init_with_inc != inc) {
        filter_init_with_inc = inc;
        volume_filter.initWithAngleIncrement(filter_init_with_inc);
      }
    }
    volume_filter.feed(&*volume_target);
    volume = *volume_filter.filtered();
  }

  bool isEnvelopeFinished() const {
    return osc.isEnvelopeFinished();
  }

private:
  std::optional<T> volume, volume_target;
  T filter_init_with_inc;
  T max_filter_increment = std::numeric_limits<T>::max();
  ALGO osc;
  Filter<T, 1, FilterType::LOW_PASS, 1> volume_filter;

protected:
  void setVolumeTargetInternal(T vol) {
    volume_target = vol;
  }
  void setAngleIncrementsInternal(T ai) {
    osc.setAngleIncrements(ai);
  }
};


/**
 The user of the class can adjust the volume
 */
template<typename ALGO>
struct VolumeAdjusted : BaseVolumeAdjusted<ALGO> {
  using T = typename ALGO::FPT;

  static constexpr auto baseVolume = ALGO::baseVolume;

  auto & getVolumeAdjustment() { return *this; }
  auto const & getVolumeAdjustment() const { return *this; }

  void setVolumeTarget(T vol) {
    this->setVolumeTargetInternal(vol);
  }

  void setAngleIncrements(T ai) {
    this->setAngleIncrementsInternal(ai);
  }
  
  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    this->getOsc().setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
  }
};

// This value makes non-volume adjusted aoscillators be usable
// with adjusted oscillators, at approximately the same volumes
// for medium frequencies.
constexpr float reduceUnadjustedVolumes = 0.1f;

/** Adjusts the volume of a mono-frequency algorithm to reach equal-loudness across all frequencies.
 */
template<typename ALGO>
struct LoudnessVolumeAdjusted : BaseVolumeAdjusted<ALGO> {
  using T = typename ALGO::FPT;

  static constexpr auto baseVolume = ALGO::baseVolume / reduceUnadjustedVolumes;

  // if the underlying algo has more than one frequency
  // then we can't use this volume adjustment algorithm
  // because it would make sense only for the fundamental frequency.
  static_assert(ALGO::isMonoHarmonic);

  LoudnessVolumeAdjusted()
  : log_ratio_(1.f)
  , low_index_(0)
  , BaseVolumeAdjusted<ALGO>()
  {}

  void setAngleIncrements(T ai) {
    this->setVolumeTargetInternal(loudness::equal_loudness_volume_from_freq(angle_increment_to_freq<T>(ai,
                                                                                                       sample_rate_),
                                                                            low_index_,
                                                                            log_ratio_,
                                                                            loudness_level));
    this->setAngleIncrementsInternal(ai);
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    Assert(low_index >= 0);
    Assert(low_index < 16);
    low_index_ = low_index;
    Assert(log_ratio >= 0.f);
    Assert(log_ratio <= 1.f);
    log_ratio_ = log_ratio;
    this->loudness_level = loudness_level;
    sample_rate_ = sample_rate;
  }

private:
  uint32_t low_index_ : 4;
  float loudness_level;
  float log_ratio_;
  int sample_rate_;
};


template<typename ALGO>
struct StereoPanned {
  
  static constexpr auto hasEnvelope = ALGO::hasEnvelope;
  static constexpr auto baseVolume = ALGO::baseVolume;
  static constexpr auto isMonoHarmonic = ALGO::isMonoHarmonic;
  static_assert(ALGO::count_channels == 1);
  static constexpr int count_channels = 2;
  
  using FPT = typename ALGO::FPT;
  
  auto & editEnvelope() { return algo.editEnvelope(); }
  auto const & getEnvelope() const { return algo.getEnvelope(); }
  
  auto       & getOsc()       { return algo; }
  auto const & getOsc() const { return algo; }
  
  auto & getVolumeAdjustment() { return algo.getVolumeAdjustment(); }
  auto const & getVolumeAdjustment() const { return algo.getVolumeAdjustment(); }
  
  
  void set_sample_rate(int s) {
    algo.set_sample_rate(s);
  }
  
  void setup(StereoGain const & g) {
    gain_ = g;
  }
  
  void forgetPastSignals() {
    algo.forgetPastSignals();
  }
  
  void setAngleIncrements(FPT ai) {
    algo.setAngleIncrements(ai);
  }
  
  FPT angleIncrements() const {
    return algo.angleIncrements();
  }
  
  void setAngle(FPT a) {
    algo.setAngle(a);
  }
  
  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    algo.setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
  }
  
  void step() {
    algo.step();
  }
  
  FPT imag(int i) const {
    return gain_.gains[i] * algo.imag();
  }
  
private:
  ALGO algo;
  StereoGain gain_;
};



// the unit of angle and angle increments is "radian / pi"

template<typename T>
struct Phased {
  using FPT = T;
  static_assert(std::is_floating_point<FPT>::value);
  using Tr = NumTraits<T>;

  Phased() = default;
  Phased(T f) {
    setAngleIncrements(f);
  }

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }

  void forgetPastSignals() {
  }

  void synchronizeAngles(Phased<T> const & other) {
    setAngle(other.angle_);
  }

  void setAngle( T angle ) {
    angle_ = phaseToNormalForm(angle);
  }
  T angle() const { return angle_; }

  void setAngleIncrements(T v) {
    angle_increments = phaseToNormalForm(v);
    aliasingMult = freqAliasingMultiplicator(v); // it's important to pass v instead of angle_increments here.
  }
  T angleIncrements() const { return angle_increments; }

  void step() {
    angle_ += angle_increments;
    if(angle_ > Tr::two()) {
      angle_ -= Tr::two();
      Assert(angle_ <= Tr::two());
    }
    else if(angle_ < Tr::zero()) {
      angle_ += Tr::two();
      Assert(angle_ >= Tr::zero());
    }
  }

protected:
  T angle_ = {};
  T angle_increments = {};
  T aliasingMult = static_cast<T>(1);
};

/*
 * Phase controlled oscillator
 *
 * Mostly for test purposes, as it uses 'sin' and 'cos' functions
 * which are less performant than 'OscillatorAlgo'.
 */
template<typename T>
struct PCOscillatorAlgo : public Phased<T> {
  static constexpr auto hasEnvelope = false;
  static constexpr auto isMonoHarmonic = true;
  static constexpr auto baseVolume = reduceUnadjustedVolumes * soundBaseVolume(Sound::SINE);
  static constexpr int count_channels = 1;

  using Phased<T>::angle_;
  using Phased<T>::aliasingMult;

  PCOscillatorAlgo() = default;
  PCOscillatorAlgo(T angle_increments) : Phased<T>(angle_increments) {}

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }

  T real() const { return aliasingMult * std::cos(static_cast<T>(M_PI) * angle_); }
  T imag() const { return aliasingMult * std::sin(static_cast<T>(M_PI) * angle_); }
};

template<typename T>
using PCOscillator = FinalAudioElement<PCOscillatorAlgo<T>>;

template<Sound::Type SOUND>
struct soundBufferWrapperAlgo {
  using MeT = soundBufferWrapperAlgo<SOUND>;

  static constexpr auto hasEnvelope = false;
  static constexpr auto baseVolume = reduceUnadjustedVolumes * soundBaseVolume(SOUND);
  static constexpr auto isMonoHarmonic = false;
  static constexpr int count_channels = 1;

  using F_GET_BUFFER = FGetBuffer<SOUND>;
  using T = double;
  using FPT = T;
  static_assert(std::is_floating_point<FPT>::value);

  soundBufferWrapperAlgo() {
  }

  void set_sample_rate(int s) {
    sb = &F_GET_BUFFER()(s);
    F_GET_BUFFER().getAbsMean(s); // just to initialize the static in it
  }

  auto       & getOsc()       {return *this; }
  auto const & getOsc() const {return *this; }

  void synchronizeAngles(MeT const & other) {
    Assert(other.sb);
    Assert(sb);
    Assert(other.sb->size() == sb->size());
    index = other.index;
  }

  void forgetPastSignals() {
  }
  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {}
  void setAngleIncrements(T ai) {}
  T angleIncrements() const { return 0; }
  void setAngle(T a) {
    Assert(sb);
    // a is between -1 and 1
    Assert(a <= 1.);
    Assert(a >= -1.);

    // -1. -> 0
    //  1. -> 0
    index = static_cast<int>(((a + 1.) * sb->size() * 0.5) + 0.5);
    if (index < 0) {
      index = 0;
    }
    --index; // because index will be incremented in step before being used
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }

  T imag() const {
    Assert(sb);
    return (*sb)[index];
  }

  void step() {
    ++index;
    Assert(sb);
    if(index >= sb->size()) {
      index = 0;
    }
  }

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }

  int index = -1;
  soundBuffer<double> const * sb;
};

template<typename T>
using WhiteNoise = FinalAudioElement< soundBufferWrapperAlgo<Sound::NOISE> >;

template<typename T>
struct ConstOne {
  static constexpr auto hasEnvelope = false;
  static constexpr auto baseVolume = 1.f;
  static constexpr auto isMonoHarmonic = true;
  static constexpr int count_channels = 1;

  using FPT = T;

  T imag() const { return static_cast<T>(1); }

  void forgetPastSignals() const {}
  void step() const {}

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }
};

enum class FOscillator {
  SAW,
  SQUARE,
  TRIANGLE
};

constexpr bool monoHarmonic(FOscillator o) {
  switch(o) {
    case FOscillator::SAW:
    case FOscillator::TRIANGLE:
    case FOscillator::SQUARE:
      return false;
  }
}


/*
 Volume factor to apply to have the same perceived loudness.
 */
template<FOscillator O>
constexpr double refVolume() {
  if constexpr (O == FOscillator::TRIANGLE) {
    return soundBaseVolume(Sound::TRIANGLE);
  }
  else if constexpr (O == FOscillator::SAW) {
    return soundBaseVolume(Sound::SAW);
  }
  else if constexpr (O == FOscillator::SQUARE) {
    return soundBaseVolume(Sound::SQUARE);
  }
  else {
    return 1.;
  }
}

template<typename T, FOscillator O>
struct FOscillatorAlgo : public Phased<T> {
  static constexpr auto hasEnvelope = false;
  static constexpr auto isMonoHarmonic = monoHarmonic(O);
  static constexpr auto baseVolume = reduceUnadjustedVolumes * refVolume<O>();
  static constexpr int count_channels = 1;

  using Phased<T>::angle_;
  using Phased<T>::aliasingMult;

  FOscillatorAlgo() = default;
  FOscillatorAlgo(T angle_increments) : Phased<T>(angle_increments) {}

  auto       & getOsc()       {return *this; }
  auto const & getOsc() const {return *this; }

  void set_sample_rate(int) {}

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {}

  T imag() const {
    if constexpr (O == FOscillator::SAW) {
      return aliasingMult * saw(angle_);
    }
    else if constexpr(O == FOscillator::SQUARE) {
      return aliasingMult * square(angle_);
    }
    else if constexpr(O == FOscillator::TRIANGLE) {
      return aliasingMult * triangle(angle_);
    }
    else {
      Assert(0);
      return 0;
    }
  }
};

template<typename Envel>
using Square = FinalAudioElement<Enveloped<FOscillatorAlgo<typename Envel :: FPT, FOscillator::SQUARE>,Envel>>;

/*
 * first pulse happens at angle = 0
 */
template<typename T>
struct PulseTrainAlgo : public Phased<T> {
  static constexpr auto hasEnvelope = false;
  static constexpr auto baseVolume = reduceUnadjustedVolumes * soundBaseVolume(Sound::SQUARE);
  static constexpr auto isMonoHarmonic = true;
  static constexpr int count_channels = 1;

  using Tr = NumTraits<T>;
  using Phased<T>::angle_;
  using Phased<T>::aliasingMult;

  PulseTrainAlgo() = default;
  PulseTrainAlgo(T angle_increments, T pulse_width) :
  Phased<T>(angle_increments),
  pulse_width(pulse_width) {
    Assert(pulse_width >= angle_increments); // else it's always 0
  }

  void set(T angle_increments, T pulse_width_) {
    Assert(pulse_width_ >= angle_increments); // else it's always 0
    this->setAngleIncrements(angle_increments);
    pulse_width = pulse_width_;
  }

  T imag() const { return aliasingMult * pulse(angle_, pulse_width); }

private:
  T pulse_width{};
};

template<typename T>
using PulseTrain = FinalAudioElement<PulseTrainAlgo<T>>;


template<typename T>
struct BaseVolume {
  static constexpr float value() {
    return T::baseVolume;
  }
};

template<class...AEs>
constexpr float minBaseVolume() {
  return minValue<BaseVolume, AEs...>();
}

template<typename AE, class...AEs>
bool constexpr AllMonoHarmonic() {
  if (!AE::isMonoHarmonic) {
    return false;
  }
  if constexpr (sizeof...(AEs) > 0) {
    return AllMonoHarmonic<AEs...>();
  }
  return true;
}

template<typename AE, class...AEs>
bool constexpr AllHaveEnvelope() {
  if (!AE::hasEnvelope) {
    return false;
  }
  if constexpr (sizeof...(AEs) > 0) {
    return AllHaveEnvelope<AEs...>();
  }
  return true;
}

template<class...AEs>
struct Mix {
  static constexpr auto hasEnvelope = AllHaveEnvelope<AEs...>();
  static constexpr auto baseVolume = minBaseVolume<AEs...>(); // be conservative.
  static constexpr bool isMonoHarmonic = AllMonoHarmonic<AEs...>();
  static constexpr int count_channels = 1; // we could do better

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }

  using T = typename NthTypeOf<0, AEs...>::FPT;
  using FPT = T;
  static_assert(std::is_floating_point<FPT>::value);

  static constexpr auto n_aes = sizeof...(AEs);

private:
  std::tuple<AEs...> aes;
  std::array<float, n_aes> gains;

public:
  Mix() { gains.fill( 1.f ); }

  auto & get() {
    return aes;
  }

  void set_sample_rate(int s) {
    for_each(aes, [s](auto & ae) {
      ae.set_sample_rate(s);
    });
  }

  void setGains(decltype(gains) g) { gains = g; }

  void setFiltersOrder(int order) {
    for_each(aes, [order](auto & ae) {
      ae.setFiltersOrder(order);
    });
  }

  void forgetPastSignals() {
    for_each(aes, [](auto & ae) {
      ae.forgetPastSignals();
    });
  }

  void step() {
    for_each(aes, [](auto & ae) {
      ae.step();
    });
  }

  void setAngleIncrements(T v) {
    for_each(aes, [v](auto & ae) {
      ae.setAngleIncrements(v);
    });
  }

  void setAngle(T v) {
    for_each(aes, [v](auto & ae) {
      ae.setAngle(v);
    });
  }

  // we return the minimum angle, which is the most constraining wrt time constants for volume variations
  T angleIncrements() const {
    T minAngle = 0;
    for_each_i(aes, [&minAngle, this](int i, auto & ae) {
      if (gains[i]) {
        T const ai = ae.angleIncrements();
        minAngle = std::min(minAngle, ai);
      }
    });
    return minAngle;
  }

  T imag() const {
    T sum = 0.f;
    for_each_i(aes, [&sum, this] (int i, auto const & ae) {
      sum += gains[i] * ae.imag();
    });
    return sum;
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    for_each(aes, [=](auto & ae) {
      ae.setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
    });
  }
};

template<class...AEs>
struct Chain {
  using T = typename NthTypeOf<0, AEs...>::FPT;
  using FPT = T;
  static_assert(std::is_floating_point<FPT>::value);

private:
  std::tuple<AEs...> aes;
};

template<int ORDER, typename T>
struct InternalFilterFPTFromOrder {
  using type = double; // for order >= 2 and 0(adjustable), we use double
};

template<typename T>
struct InternalFilterFPTFromOrder<1,T> {
  using type = T;
};

template<typename AEAlgo, FilterType KIND, int ORDER>
struct FilterAlgo {
  static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
  static constexpr auto baseVolume = AEAlgo::baseVolume;
  static constexpr auto isMonoHarmonic = AEAlgo::isMonoHarmonic;
  static constexpr int count_channels = 1;

  using T = typename AEAlgo::FPT;
  using FPT = T;
  using FilterFPT = typename InternalFilterFPTFromOrder<ORDER, FPT>::type;
  static_assert(std::is_floating_point<FPT>::value);

  void set_sample_rate(int s) {
    audio_element.set_sample_rate(s);
  }

  bool isEnvelopeFinished() const {
    return audio_element.isEnvelopeFinished();
  }
  void onKeyPressed(int32_t delay) {
    audio_element.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    audio_element.onKeyReleased(delay);
  }

  bool canHandleExplicitKeyReleaseNow(int32_t delay) const {
    if constexpr (hasEnvelope) {
      return audio_element.canHandleExplicitKeyReleaseNow(delay);
    }
    else {
      Assert(0);
      return false;
    }
  }

private:
  AEAlgo audio_element;
  Filter<FilterFPT, 1, KIND, ORDER> filter_;
public:
  void forgetPastSignals() {
    filter_.forgetPastSignals();
    audio_element.forgetPastSignals();
  }

  void setFiltersOrder(int order) {
    filter_.setOrder(order);
  }

  void step() {
    audio_element.step();
    FilterFPT val = static_cast<FilterFPT>(audio_element.imag());
    filter_.feed(&val);
  }

  // Warning: setAngleIncrements and angleIncrements are not related,
  //one opertaes on the filter, the other on the underlying oscillator

  // sets the filter frequency
  void setAngleIncrements(T v) {
    filter_.initWithAngleIncrement(v);
  }
  //returns the oscillator frequency
  T angleIncrements() const {
    return audio_element.angleIncrements();
  }
  void setAngle(T a) {
    audio_element.setAngle(a);
  }

  T imag() const {
    return *filter_.filtered();
  }

  auto & get_element() { return audio_element; }
  auto & get_element() const { return audio_element; }
  auto & filter() { return filter_; }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {}
};

template<typename T, int ORDER>
using LowPassAlgo = FilterAlgo<T, FilterType::LOW_PASS, ORDER>;

template<typename T, int ORDER>
using HighPassAlgo = FilterAlgo<T, FilterType::HIGH_PASS, ORDER>;

template<typename AEAlgo, int ORDER>
struct BandPassAlgo_ {
  using Algo = AEAlgo;
  static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
  using FPT = typename AEAlgo::FPT;
  using T = FPT;
  static constexpr auto low_index = 0;
  static constexpr auto high_index = 1;

protected:
  void do_set_sample_rate(int s) {
    cascade.set_sample_rate(s);
  }

public:
  void setCompensation(T sq_inv_width_factor) {
    // gain compensation to have an equal power of the central frequency for all widths
    compensation = expt<ORDER>(1 + sq_inv_width_factor);
#ifndef NDEBUG
    // verify accuracy of above simplification

    // inc / low == width_factor
    // inc / high == 1 / width_factor
    auto inv_sq_mag_hp = get_inv_square_filter_magnitude<FilterType::HIGH_PASS>(1/sq_inv_width_factor);
    auto inv_sq_mag_lp = get_inv_square_filter_magnitude<FilterType::LOW_PASS >(sq_inv_width_factor);
    auto inv_mag = sqrt(inv_sq_mag_hp * inv_sq_mag_lp);
    auto original_compensation = inv_mag;
    original_compensation = expt<ORDER>(original_compensation);
    Assert(std::abs(original_compensation - compensation) / (original_compensation + compensation) < FLOAT_EPSILON);
#endif
  }

  T imag() const {
    return compensation * cascade.imag();
  }

  T angleIncrements() const {
    return cascade.angleIncrements();
  }
  void setAngle(T a) {
    cascade.setAngle(a);
  }

  bool isEnvelopeFinished() const {
    return cascade.isEnvelopeFinished();
  }
  void onKeyPressed(int32_t delay) {
    cascade.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    cascade.onKeyReleased(delay);
  }

protected:
  T compensation;
private:
  HighPassAlgo<LowPassAlgo<AEAlgo, ORDER>, ORDER> cascade;
protected:

  auto & getHP() { return cascade; }
  auto & getLP() { return cascade.get_element(); }

  void onWidthFactor(float inv_width_factor) {
    setCompensation(inv_width_factor * inv_width_factor);
  }

  void doStep() {
    cascade.step();
  }
};

template<typename AEAlgo, int ORDER>
struct BandRejectAlgo_ {
  using Algo = AEAlgo;
  static constexpr auto hasEnvelope = AEAlgo::hasEnvelope;
  using FPT = typename AEAlgo::FPT;
  using T = FPT;

protected:
  void do_set_sample_rate(int s) {
    lp.set_sample_rate(s);
    hp.set_sample_rate(s);
  }

public:
  T imag() const { return lp.imag() + hp.imag(); }

  bool isEnvelopeFinished() const {
    return lp.isEnvelopeFinished() && hp.isEnvelopeFinished();
  }
  void onKeyPressed(int32_t delay) {
    lp.onKeyPressed(delay);
    hp.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    lp.onKeyReleased(delay);
    hp.onKeyReleased(delay);
  }
  T angleIncrements() const {
    return std::min(lp.angleIncrements(),
                    hp.angleIncrements());
  }
  void setAngle(T a) {
    lp.setAngle(a);
    hp.setAngle(a);
  }

private:
  LowPassAlgo<AEAlgo, ORDER> lp;
  HighPassAlgo<AEAlgo, ORDER> hp;
protected:
  using Tr = NumTraits<T>;

  static constexpr auto compensation = Tr::one();

  static constexpr auto low_index = 1;
  static constexpr auto high_index = 0;

  auto & getHP() { return hp; }
  auto & getLP() { return lp; }

  void onWidthFactor(float) const {}

  void doStep() {
    lp.step();
    hp.step();
  }
};

template<typename AEAlgoWidth, typename Base>
struct BandAlgo_ : public Base {
  using FPT = typename Base::FPT;
  using Algo = typename Base::Algo;
  using T = FPT;
  using Tr = NumTraits<T>;
  using AEWidth = AEAlgoWidth;
  using Base::do_set_sample_rate;
  using Base::compensation;
  using Base::low_index;
  using Base::high_index;
  using Base::onWidthFactor;
  using Base::getHP;
  using Base::getLP;
  using Base::doStep;

  static constexpr auto baseVolume = Algo::baseVolume;
  static constexpr auto isMonoHarmonic = Algo::isMonoHarmonic;
  static constexpr int count_channels = 1;

  void set_sample_rate(int s) {
    width.set_sample_rate(s);
    do_set_sample_rate(s);
  }

  void forgetPastSignals() {
    getHP().forgetPastSignals();
    getLP().forgetPastSignals();
    width.forgetPastSignals();
  }

  void setFiltersOrder(int order) {
    getHP().setFiltersOrder(order);
    getLP().setFiltersOrder(order);
  }

  void setWidthRange(range<float> const & r) {
    width_range = r;
  }

  void setAngleIncrements(T inc) {
    Assert(inc >= 0.f);
    increment = inc;
  }

  void step() {
    width.step();
    T width_factor = std::pow(Tr::two(),
                              width_range.getAt(std::abs(width.imag())));

    Assert(width_factor);
    auto inv_width_factor = Tr::one() / width_factor;
    auto low  = increment * inv_width_factor;
    auto high = increment * width_factor;

    onWidthFactor(inv_width_factor);
    doSetAngleIncrements({{low, high}});

    doStep();
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) const {}

  AEWidth & getWidth() { return width; }

protected:
  T increment;
  AEAlgoWidth width;
  range<float> width_range;


  void doSetAngleIncrements(std::array<T, 2> incs) {
    getHP().setAngleIncrements(incs[low_index]);
    getLP().setAngleIncrements(incs[high_index]);
  }
};

template<typename AEAlgo, typename AEAlgoWidth, int ORDER>
using BandPassAlgo = BandAlgo_<AEAlgoWidth, BandPassAlgo_<AEAlgo, ORDER>>;

template<typename AEAlgo, typename AEAlgoWidth, int ORDER>
using BandRejectAlgo = BandAlgo_<AEAlgoWidth, BandRejectAlgo_<AEAlgo, ORDER>>;

template<typename T>
struct LoudnessCompensationFilterWithLatency {
  LoudnessCompensationFilterWithLatency(int sample_rate, unsigned int const fft_length, unsigned int const NumTaps) :
  sz(NumTaps)
  {
    filter.setupAndSetCoefficients(FFTConvolutionCRTPSetupParam{static_cast<int>(fft_length)},
                                   1, // we sample by sample so max vector size is 1
                                   loudness::getLoudnessCompensationFIRCoefficients<T>(sample_rate, fft_length, NumTaps));
  }

  Latency getLatency() const { return filter.getLatency(); }

  T step(T val) {
    return filter.step(val);
  }

  auto size() const { return sz; }
private:
  // to have a lowest cpu usage on average, we use a single fft
  using Algo = AlgoFFTConvolutionIntermediate<AlgoFFTConvolutionCRTP<T, a64::Alloc, fft::Fastest>>;
  using F = SelfContainedXYConvolution<Algo>;
  F filter;
  unsigned int sz;
};

template<typename T, int ORDER>
using LPWhiteNoiseAlgo = LowPassAlgo<soundBufferWrapperAlgo<Sound::NOISE>, ORDER>;

template<typename T, int ORDER>
using LPWhiteNoise = FinalAudioElement<LPWhiteNoiseAlgo<T, ORDER>>;

template<typename AEAlgo, int ORDER>
using LowPass = FinalAudioElement<LowPassAlgo<AEAlgo, ORDER>>;

enum class eNormalizePolicy : unsigned char {
  FAST,
  ACCURATE
};

// TODO rename : SineOscillatorAlgo
template<typename T, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
struct OscillatorAlgo {
  using MeT = OscillatorAlgo<T,NormPolicy>;
  static constexpr auto hasEnvelope = false;
  static constexpr auto isMonoHarmonic = true;
  static constexpr auto baseVolume = reduceUnadjustedVolumes * soundBaseVolume(Sound::SINE);
  static constexpr int count_channels = 1;

  using Tr = NumTraits<T>;
  using FPT = T;

  constexpr OscillatorAlgo(T angle_increments) { setAngleIncrements(angle_increments); }
  constexpr OscillatorAlgo() : OscillatorAlgo(0) {}

  auto const & getOsc() const { return *this; }
  auto       & getOsc()       { return *this; }

  void set_sample_rate(int s) {
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
  }
  void forgetPastSignals() {
  }
  void onKeyPressed(int32_t) {
    Assert(0);
  }
  void onKeyReleased(int32_t) {
    Assert(0);
  }

  void setFiltersOrder(int order) const {
  }

  void synchronizeAngles(MeT const & other) {
    cur = other.cur;
  }

  void setAngle(T f) {
    cur = polar(static_cast<T>(M_PI)*f);
  }
  void setAngleIncrements(T f) {
    if (angle_increments && *angle_increments == f) {
      return;
    }
    angle_increments = f;
    mult = polar(static_cast<T>(M_PI)*f);
    aliasingMult = freqAliasingMultiplicator(f);
  }

  void step() {
    cur *= mult;
    if(NormPolicy == eNormalizePolicy::FAST) {
      approx_normalize(); // to fix iterative error accumulation... if it is costly it could be done less frequently
    }
    else {
      normalize();
    }
  }

  T real() const { return aliasingMult * cur.real(); }
  T imag() const { return aliasingMult * cur.imag(); }

  T angle() const { return arg(cur)/M_PI; }
  T angleIncrements() const {
    Assert(angle_increments);
    return *angle_increments;
  }

  bool isEnvelopeFinished() const {
    Assert(0);
    return false;
  }

private:
  complex<T> cur = {Tr::one(), Tr::zero()};
  complex<T> mult = {Tr::one(), Tr::zero()};
  T aliasingMult = static_cast<T>(1);
  std::optional<T> angle_increments;

  void approx_normalize() {
    // http://dsp.stackexchange.com/questions/971/how-to-create-a-sine-wave-generator-that-can-smoothly-transition-between-frequen

    cur *= Tr::half() * (Tr::three() - norm(cur));
  }

  void normalize() {
    cur *= 1/abs(cur);
  }
};

template<typename Envel, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
using Oscillator = FinalAudioElement<Enveloped<OscillatorAlgo<typename Envel::FPT, NormPolicy>, Envel>>;

template<typename Envel, eNormalizePolicy NormPolicy = eNormalizePolicy::FAST>
using MultiOscillator = FinalAudioElement<MultiEnveloped<OscillatorAlgo<typename Envel::FPT, NormPolicy>, Envel>>;

/**
 Periodic ramp
 */
template<typename T>
struct LogRamp {
  static_assert(std::is_floating_point_v<T>, "non-floating point interpolation is not supported");

  using Tr = NumTraits<T>;
  using FPT = T;

  // offsets because we use the value at the beginning of the timestep
  static constexpr auto increasing_integration_offset = 0;
  static constexpr auto decreasing_integration_offset = 1;

  LogRamp() : cur_sample(Tr::zero()), from{}, to{}, duration_in_samples{}
  {}

  void set_sample_rate(int s) {
  }

  void forgetPastSignals() {}

  void setAngleIncrementsRange(range<float> const &) const { Assert(0); } // use set instead
  void set_interpolation(itp::interpolation) const {Assert(0);} // use set instead
  void set_n_slow_steps(unsigned int) const { Assert(0); }

  auto & getUnderlyingIter() { Assert(0); return *this; }

  void setAngleIncrements(T increments) {}

  void setup(T from_increments,
             T to_increments,
             T duration_in_samples_,
             T start_sample,
             itp::interpolation i) {
    if(start_sample >= Tr::zero()) {
      cur_sample = start_sample;
    }
    else {
      // we adapt cur_sample to be at the same ratio in the new range
      // and we adapt bounds order to the existing bounds
      Assert(duration_in_samples);
      cur_sample *= duration_in_samples_ / duration_in_samples;
      if(from_increments > to_increments) {
        if(from < to) {
          std::swap(from_increments, to_increments);
        }
      }
      else if(from > to) {
        std::swap(from_increments, to_increments);
      }
    }
    from = from_increments;
    to = to_increments;
    duration_in_samples = duration_in_samples_;

    C = get_linear_proportionality_constant();

    Assert(duration_in_samples > 0);
    interp.setInterpolation(i);
  }

  T do_step(T proportionality) {
    if(cur_sample + .5f > duration_in_samples) {
      cur_sample = Tr::zero();
      std::swap(from, to);
    }

    // we call get_unfiltered_value instead of get_value because we ensure:
    Assert(cur_sample <= duration_in_samples);
    auto f_result = interp.get_unfiltered_value(cur_sample, duration_in_samples, from, to);
    // Taking the value at cur_sample means taking the value at the beginning of the step.
    // The width of the step depends on that value so if we had taken the value in the middle or at the end of the step,
    // not only would the value be different, but also the step width!

    // we could take the value in the middle and adjust "value + step width" accordingly

    // linear interpolation for parameter
    auto f = from + (to-from) * (cur_sample + .5f) / duration_in_samples;
    cur_sample += proportionality * f;

    return f_result;
  }

  T step() {
    return do_step(C);
  }

  T getFrom() const { return from; }
  T getTo() const { return to; }

  T get_duration_in_samples() const { return duration_in_samples; }

private:
  NormalizedInterpolation<T> interp;
  T from, to, cur_sample;
  T duration_in_samples;
  T C;

  // do not make it public : if bounds are swapped afterwards, using this value can lead to bugs
  T get_linear_proportionality_constant() const {
    // we want to achieve the same effect as PROPORTIONAL_VALUE_DERIVATIVE
    // but without paying the cost of one 'expf' call per audio frame :
    // to achieve the same effect we add to cur_sample a value proportionnal to
    // the current frequency. the factor of proportionnality is adjusted to match
    // the wanted duration_in_samples
    Assert(from > 0);
    Assert(to > 0);
    // else computation cannot be done

    return (to==from) ? 1.f : -std::log(from/to) / (to-from);
  }
};


enum class Extremity {
  Start,
  End
};

/**
 Ramp
 */
template<typename T>
struct LogSingleRamp {
  static_assert(std::is_floating_point_v<T>, "non-floating point interpolation is not supported");

  using Tr = NumTraits<T>;
  using FPT = T;

  // offsets because we use the value at the beginning of the timestep
  static constexpr auto increasing_integration_offset = 0;
  static constexpr auto decreasing_integration_offset = 1;

  LogSingleRamp()
  : cur_sample(Tr::zero())
  , from{}
  , to{}
  , duration_in_samples{}
  {}

  void set_sample_rate(int) {}

  void forgetPastSignals() {}

  void setAngleIncrementsRange(range<float> const &) const { Assert(0); } // use setup / setAngleIncrements instead
  void set_interpolation(itp::interpolation) const {Assert(0);} // use setup instead
  void set_n_slow_steps(unsigned int) const { Assert(0); }

  auto & getUnderlyingIter() { Assert(0); return *this; }

  // Once this has been called, 'setAngleIncrements' _must_ be called
  //   (to specify the start frequency) before calling 'step'.
  // Else, the behaviour is undefined.
  void setup(Extremity e,
             T increments,
             T duration_in_samples_,
             itp::interpolation i) {
    Assert(duration_in_samples_ > 0);

    setup_extremity = e;
    switch(e) {
      case Extremity::Start:
        from = increments;
        break;
      case Extremity::End:
        to = increments;
        break;
    }
    duration_in_samples = duration_in_samples_;
    interp.setInterpolation(i);
  }

  void setAngleIncrements(T increments) {
    // verify that setup has been called
    Assert(duration_in_samples > 0);

    cur_sample = 0;
    switch(setup_extremity) {
      case Extremity::Start:
        to = increments;
        break;
      case Extremity::End:
        from = increments;
        break;
    }
    C = get_linear_proportionality_constant();
  }

  T step() {
    if(cur_sample + .5f > duration_in_samples) {
      cur_sample = duration_in_samples;
    }

    // we call get_unfiltered_value instead of get_value because we ensure:
    Assert(cur_sample <= duration_in_samples);
    auto f_result = interp.get_unfiltered_value(cur_sample, duration_in_samples, from, to);
    // Taking the value at cur_sample means taking the value at the beginning of the step.
    // The width of the step depends on that value so if we had taken the value in the middle or at the end of the step,
    // not only would the value be different, but also the step width!

    // we could take the value in the middle and adjust "value + step width" accordingly

    if (cur_sample < duration_in_samples) {
      // linear interpolation for parameter
      auto f = from + (to-from) * (cur_sample + .5f) / duration_in_samples;
      cur_sample += C * f;
    }

    return f_result;
  }

  T getFrom() const { return from; }
  T getTo() const { return to; }

  T get_duration_in_samples() const { return duration_in_samples; }

private:
  // This interpolation is "composed" with an implicit PROPORTIONAL_VALUE_DERIVATIVE interpolation
  NormalizedInterpolation<T> interp;

  Extremity setup_extremity = Extremity::Start;
  T from, to, cur_sample;
  T duration_in_samples;
  T C;

  T get_linear_proportionality_constant() const {
    // We want to achieve the same effect as PROPORTIONAL_VALUE_DERIVATIVE
    // without paying the cost of one 'expf' call per audio frame :
    // to achieve the same effect, at each frame we add to cur_sample a value proportionnal to
    // the current frequency. The factor of proportionnality is adjusted to match
    // the wanted duration_in_samples

    // Assert that computation can be done
    Assert(from > 0);
    Assert(to > 0);

    return (to==from) ? 1.f : -std::log(from/to) / (to-from);
  }
};


/**
 When the frequency is interpolated, and the target updated at regular time intervals.
 */
template<typename T>
struct InterpolatedFreq {

  using Tr = NumTraits<T>;
  using FPT = T;

  // offsets because we use the value at the beginning of the timestep
  static constexpr auto increasing_integration_offset = 0;
  static constexpr auto decreasing_integration_offset = 1;

  InterpolatedFreq()
  : cur_sample(Tr::zero())
  , from{}
  , to{}
  , duration_in_samples{}
  {}

  void set_sample_rate(int s) {
  }

  void forgetPastSignals() {
    f_result.reset();
  }

  void setAngleIncrementsRange(range<float> const &) const { Assert(0); } // use setup / setAngleIncrements instead
  void set_interpolation(itp::interpolation) const {Assert(0);} // use setup instead
  void set_n_slow_steps(unsigned int) const { Assert(0); }

  auto & getUnderlyingIter() { Assert(0); return *this; }

  // Once this has been called, 'setAngleIncrements' _must_ be called
  //   (to specify the start frequency) before calling 'step', unless 'setAngleIncrements'
  //   has been called at least once after the last 'forgetPastSignals'
  // Else, the behaviour is undefined.
  void setup(T duration_in_samples_,
             itp::interpolation i) {
    Assert(duration_in_samples_ > 0);
    duration_in_samples = duration_in_samples_;
    interp.setInterpolation(i);

    cur_sample = 0;
    if (f_result) {
      from = *f_result;
      C = get_linear_proportionality_constant();
    }
  }

  void setAngleIncrements(T increments) {
    // verify that setup has been called
    Assert(duration_in_samples > 0);

    cur_sample = 0;
    to = increments;

    from = f_result ? (*f_result) : increments;

    C = get_linear_proportionality_constant();
  }

  T step() {
    if (f_result && *f_result == to) { // optimization for the "steady state" case
      return to;
    }
    if(cur_sample + .5f > duration_in_samples) {
      cur_sample = duration_in_samples;
    }

    // we call get_unfiltered_value instead of get_value because we ensure:
    Assert(cur_sample <= duration_in_samples);
    f_result = interp.get_unfiltered_value(cur_sample, duration_in_samples, from, to);
    // Taking the value at cur_sample means taking the value at the beginning of the step.
    // The width of the step depends on that value so if we had taken the value in the middle or at the end of the step,
    // not only would the value be different, but also the step width!

    // we could take the value in the middle and adjust "value + step width" accordingly

    if (cur_sample < duration_in_samples) {
      // linear interpolation for parameter
      auto f = from + (to-from) * (cur_sample + .5f) / duration_in_samples;
      cur_sample += C * f;
    }

    return *f_result;
  }

  T getFrom() const { return from; }
  T getTo() const { return to; }

  T get_duration_in_samples() const { return duration_in_samples; }

private:
  // This interpolation is "composed" with an implicit PROPORTIONAL_VALUE_DERIVATIVE interpolation
  NormalizedInterpolation<T> interp;

  T from, to, cur_sample;
  std::optional<T> f_result;
  T duration_in_samples;
  T C;

  T get_linear_proportionality_constant() const {
    // We want to achieve the same effect as PROPORTIONAL_VALUE_DERIVATIVE
    // without paying the cost of one 'expf' call per audio frame :
    // to achieve the same effect, at each frame we add to cur_sample a value proportionnal to
    // the current frequency. The factor of proportionnality is adjusted to match
    // the wanted duration_in_samples

    // Assert that computation can be done
    Assert(from > 0);
    Assert(to > 0);

    return (to==from) ? 1.f : -std::log(from/to) / (to-from);
  }
};

/*
 * std::abs(<value>)
 */
template<typename Iterator>
struct AbsIter {
  using FPT = typename Iterator::FPT;

  void set_sample_rate(int s) {
    it.set_sample_rate(s);
  }

  void initializeForRun() {
    it.initializeForRun();
  }

  void operator ++() {
    ++it;
  }

  auto operator *() const {
    return std::abs(*it);
  }

  float getAbsMean() const { return it.getAbsMean(); }
private:
  Iterator it;
};

/*
 * Slow down iteration by an integer factor and chose interpolation.
 */
template<typename Iterator>
struct SlowIter {
  using FPT = typename Iterator::FPT;

  using uint_steps = int32_t;

  SlowIter(itp::interpolation interp = itp::LINEAR)
  : interp(interp) {
  }

  void set_sample_rate(int s) {
    it.set_sample_rate(s);
  }

  void set_interpolation(itp::interpolation i) { interp.setInterpolation(i); }
  void set_n_slow_steps(uint_steps n) {
    if(n == n_steps) {
      return;
    }
    if(slow_it) {
      // adapt the iterator
      Assert(n_steps);
      auto ratio = (slow_it + .5f) / static_cast<float>(n_steps);
      Assert(ratio < 1.f);
      Assert(ratio > 0.f);
      slow_it = ratio * static_cast<float>(n);
      if(slow_it == n) {
        onMajorStep();
      }
    }
    n_steps = n;
    Assert(n_steps > 0);
  }

  void initializeForRun() {
    it.initializeForRun();
    onMajorStep();
  }

  bool increment() {
    ++slow_it;
    if(slow_it < n_steps) {
      return false;
    }
    onMajorStep();
    return true;
  }

  void operator ++() {
    increment();
  }

  float operator *() const {
    // todo verify that this is inlined (probably not...)
    // for performance we may need to have a NormalizedInterpolation class templated for interp
    Assert(n_steps >= 1);
    return interp.get_unfiltered_value(slow_it, n_steps, prev, *it);
  }

  bool isDiminishing() const {
    return prev > *it;
  }

  bool isAugmenting() const {
    return !isDiminishing();
  }

  auto const & getUnderlyingIterator() const { return it; }

  float getAbsMean() const { return it.getAbsMean(); }

private:
  uint_steps n_steps = -1;
  uint_steps slow_it = 0;
  NormalizedInterpolation<> interp;
  float prev;
  Iterator it;

  void onMajorStep() {
    slow_it = 0;
    prev = *it;
    ++it;
  }

};

/*
 * Makes ascending iteration faster than descending.
 */
template<typename UnderlyingIt, int SCALE_UP = 3>
struct WindFreqIter {
  template <class... Args>
  WindFreqIter(Args&&... args) : it(std::forward<Args>(args)...) {}

  void set_sample_rate(int s) {
    it.set_sample_rate(s);
  }

  void initializeForRun() {
    it.initializeForRun();
  }

  void operator ++() {
    auto const n = [this]() {
      if(it.isDiminishing()) {
        return 1;
      }
      return SCALE_UP;
    }();

    for(int i=0; i<n; ++i) {
      if(it.increment()) {
        return;
      }
    }
  }

  auto operator *() const {
    return *it;
  }

  float getAbsMean() const { return it.getAbsMean(); }

  auto & getUnderlyingIter() {
    return it;
  }

private:
  UnderlyingIt it;
};

template<typename ITER>
struct Ctrl {
  using FPT = typename ITER::FPT;
  using T = FPT;
  using Tr = NumTraits<T>;

  void set_sample_rate(int s) {
    it.set_sample_rate(s);
  }

  void forgetPastSignals() {
    it.initializeForRun();
  }
  void setFiltersOrder(int ) const {}
  void setAngleIncrements(T ) const {}

  void step() { ++it; }
  T imag() const { return *it; }

  auto & getUnderlyingIter() {
    return it.getUnderlyingIter();
  }

  float getAbsMean() const { return it.getAbsMean(); }
private:
  WindFreqIter<ITER> it = { itp::LINEAR };
};

enum class LoudnessVolumeAdjust : unsigned char{
  Yes,
  No
};

template<LoudnessVolumeAdjust V, typename T>
struct OscillatorAlgo_;

template<typename T>
struct OscillatorAlgo_<LoudnessVolumeAdjust::No, T> {
  using type = OscillatorAlgo<T>;
};

template<typename T>
struct OscillatorAlgo_<LoudnessVolumeAdjust::Yes, T> {
  using type = LoudnessVolumeAdjusted<OscillatorAlgo<T>>;
};

template<LoudnessVolumeAdjust V, typename T>
using LoudnessAdjustableVolumeOscillatorAlgo = typename OscillatorAlgo_<V,T>::type;


template<typename ALGO, typename CTRLS, int N>
struct SetFreqs {
  void operator()(CTRLS & ctrls, ALGO & osc) {
    constexpr auto sz = std::tuple_size<CTRLS>::value;
    std::array<typename ALGO::FPT, sz> increments;
    for_each_i(ctrls, [&increments] (int i, auto & c) {
      increments[i] = c.step();
    });
    osc.setAngleIncrements(std::move(increments));
  }
};

template<typename ALGO, typename CTRLS>
struct SetFreqs<ALGO, CTRLS, 1> {
  void operator()(CTRLS & ctrls, ALGO & osc) {
    osc.setAngleIncrements(std::get<0>(ctrls).step());
  }
};

template<typename ALGO, typename CTRLS>
void setFreqs(CTRLS & ctrls, ALGO & osc) {
  SetFreqs<ALGO,CTRLS, std::tuple_size<CTRLS>::value>{}(ctrls, osc);
}

template<typename ALGO, typename ...CTRLS>
struct FreqCtrl_ {
  using MeT = FreqCtrl_<ALGO, CTRLS...>;

  static constexpr auto hasEnvelope = ALGO::hasEnvelope;
  static constexpr auto baseVolume = ALGO::baseVolume;
  static constexpr auto isMonoHarmonic = ALGO::isMonoHarmonic;
  static constexpr int count_channels = 1;

  using Ctrl = std::tuple<CTRLS...>;
  using T = typename ALGO::FPT;
  using FPT = T;

  using Tr = NumTraits<T>;

  void set_sample_rate(int s) {
    for_each(ctrls, [s](auto & c) {
      c.set_sample_rate(s);
    });
    osc.set_sample_rate(s);
  }

  T angle() const { return osc.angle(); }
  void setAngle(T a) {
    osc.setAngle(a);
  }
  void synchronizeAngles(MeT const & other) {
    osc.synchronizeAngles(other.osc);
  }

  void setLoudnessParams(int sample_rate, int low_index, float log_ratio, float loudness_level) {
    osc.setLoudnessParams(sample_rate, low_index, log_ratio, loudness_level);
  }

  void setFiltersOrder(int order) {
    for_each(ctrls, [order](auto & c) {
      c.setFiltersOrder(order);
    });
    osc.setFiltersOrder(order);
  }

  void forgetPastSignals() {
    for_each(ctrls, [](auto & c) {
      c.forgetPastSignals();
    });
    osc.forgetPastSignals();
  }

  void setAngleIncrements(T v) {
    for_each(ctrls, [v](auto & c) {
      c.setAngleIncrements(v);
    });
    // not for osc : it will be done in step()
  }

  // It is debatable, whether this should return the oscillator's increment, or the control's.
  //
  // The reason why it returns the oscillator frequency is because
  // 'BaseVolumeAdjusted' uses this to adjust the time constant of the filter that controls the amplitude,
  // so it is safer imho to return the current oscillator frequency, instead of a future frequency.
  // (Note that in that use case, it would be even safer to return the minimum frequency between the current
  // and the future frequencies).
  FPT angleIncrements() const { return osc.angleIncrements(); }

  void step() {
    setFreqs(ctrls, osc);
    osc.step();
  }

  T real() const { return osc.real(); }
  T imag() const { return osc.imag(); }

  auto const & getOsc() const { return osc; }
  auto       & getOsc()       { return osc; }

  auto & getCtrl() {
    assert(std::tuple_size<Ctrl>::value == 1);
    return std::get<0>(ctrls);
  }

  bool isEnvelopeFinished() const {
    return osc.isEnvelopeFinished();
  }
  void onKeyPressed(int32_t delay) {
    osc.onKeyPressed(delay);
  }
  void onKeyReleased(int32_t delay) {
    osc.onKeyReleased(delay);
  }

private:
  Ctrl ctrls;
  ALGO osc;
};

template<typename T, LoudnessVolumeAdjust V>
using FreqRampOscillatorAlgo_ = FreqCtrl_<LoudnessAdjustableVolumeOscillatorAlgo<V,T>, LogRamp<T>>;

template<typename T, LoudnessVolumeAdjust V>
using FreqSingleRampOscillatorAlgo_ = FreqCtrl_<LoudnessAdjustableVolumeOscillatorAlgo<V,T>, LogSingleRamp<T>>;

template<typename T>
using FreqRampAlgo = FreqRampOscillatorAlgo_<T, LoudnessVolumeAdjust::Yes>;

// This uses a low-pass filter on the volume so that when using varying frequencies,
// like in a sweep, there is no audible audio crack.
// (TODO) Instead of using low-passed volume adjustment, we could also directly filter the signal
template<typename T>
using FreqSingleRampAlgo = FreqSingleRampOscillatorAlgo_<T, LoudnessVolumeAdjust::Yes>;

template<typename Envel>
using FreqRamp = FinalAudioElement<Enveloped<FreqRampAlgo<typename Envel::FPT>, Envel>>;

template<typename T, int ORDER>
using FreqRampLPWhiteNoiseAlgo_ = FreqCtrl_<LPWhiteNoiseAlgo<T, ORDER>, LogRamp<T>>;

template<typename T, int ORDER>
using FreqRampLPWhiteNoise = FinalAudioElement<FreqRampLPWhiteNoiseAlgo_<T, ORDER>>;


template<typename A1, typename A2>
struct RingModulationAlgo {
  static constexpr auto hasEnvelope = A1::hasEnvelope || A2::hasEnvelope;
  static constexpr auto baseVolume = minBaseVolume<A1, A2>(); // TODO adjust with real use case
  static constexpr auto isMonoHarmonic = false;
  static constexpr int count_channels = 1;

  using T = typename A1::FPT;
  using FPT = T;

  static_assert(std::is_same<typename A1::FPT, typename A2::FPT>::value); // else choice for T is arbitrary

  using Tr = NumTraits<T>;

  auto const & getOsc() const { return *this; }
  auto       & getOsc()       { return *this; }

  void set(T angle_increments1, T angle_increments2, bool reset = true) {
    osc1.setAngleIncrements(angle_increments1);
    if(reset) {
      osc1.setAngle(0.f);
    }
    osc2.setAngleIncrements(angle_increments2);
    if(reset) {
      osc1.setAngle(0.f);
    }
  }

  void forgetPastSignals() {
    osc1.forgetPastSignals();
    osc2.forgetPastSignals();
  }

  void step() {
    osc1.step();
    osc2.step();
  }

  T real() const { return osc1.real() * osc2.real(); }
  T imag() const { return osc1.imag() * osc2.imag(); }

  auto & get_element_1() { return osc1; }
  auto & get_element_2() { return osc2; }

  bool isEnvelopeFinished() const {
    if(A1::hasEnvelope && osc1.isEnvelopeFinished()) {
      return true;
    }
    if(A2::hasEnvelope && osc2.isEnvelopeFinished()) {
      return true;
    }
    if constexpr (!A1::hasEnvelope && !A2::hasEnvelope) {
      Assert(0);
    }
    return false;
  }
  void onKeyPressed(int32_t delay) {
    if constexpr (A1::hasEnvelope) {
      osc1.onKeyPressed(delay);
    }
    if constexpr (A2::hasEnvelope) {
      osc2.onKeyPressed(delay);
    }
    if constexpr (!A1::hasEnvelope && !A2::hasEnvelope) {
      Assert(0);
    }
  }
  void onKeyReleased(int32_t delay) {
    if constexpr (A1::hasEnvelope) {
      osc1.onKeyReleased(delay);
    }
    if constexpr (A2::hasEnvelope) {
      osc2.onKeyReleased(delay);
    }
    if constexpr (!A1::hasEnvelope && !A2::hasEnvelope) {
      Assert(0);
    }
  }

  bool canHandleExplicitKeyReleaseNow(int32_t delay) const {
    bool res = false;
    if constexpr (A1::hasEnvelope) {
      res = osc1.canHandleExplicitKeyReleaseNow(delay);
    }
    if constexpr (A2::hasEnvelope) {
      res = res || osc2.canHandleExplicitKeyReleaseNow(delay);
    }
    if constexpr (!A1::hasEnvelope && !A2::hasEnvelope) {
      Assert(0);
    }
    return res;
  }

private:
  A1 osc1;
  A2 osc2;
};

template<typename A1, typename A2>
using RingModulation = FinalAudioElement<RingModulationAlgo<A1,A2>>;

} // NS imajuscule::audio::audioelement
