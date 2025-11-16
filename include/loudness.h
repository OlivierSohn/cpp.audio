namespace imajuscule::audio {
namespace loudness {
// TODO use doubles?

// based on http://mariobon.com/Glossario/Loudness_ISO_226_2003b.pdf

constexpr auto n_freq = 29;

constexpr std::array<float,n_freq> freqs {{
  20.f,
  25.f,
  31.5f,
  40.f,
  50.f,
  63.f,
  80.f,
  100.f,
  125.f,
  160.f,
  200.f,
  250.f,
  315.f,
  400.f,
  500.f,
  630.f,
  800.f,
  1000.f,
  1250.f,
  1600.f,
  2000.f,
  2500.f,
  3150.f,
  4000.f,
  5000.f,
  6300.f,
  8000.f,
  10000.f,
  12500.f,
}};

// we interpolate in pitch space, instead of frequency space

constexpr std::array<MidiPitch, n_freq> create_pitches() {
  constexpr ConstexprMidi midi;
  std::array<MidiPitch, n_freq> res;
  int idx = 0;
  for(auto const & f : freqs) {
    res[idx] = *midi.frequency_to_midi_pitch(f);
    ++idx;
  }
  return res;
}

constexpr std::array<MidiPitch, n_freq> pitches = create_pitches();

constexpr std::array<float,n_freq> alpha_f_ {{
  .532f,
  .506f,
  .480f,
  .455f,
  .432f,
  .409f,
  .387f,
  .367f,
  .349f,
  .330f,
  .315f,
  .301f,
  .288f,
  .276f,
  .267f,
  .259f,
  .253f,
  .250f,
  .246f,
  .244f,
  .243f,
  .243f,
  .243f,
  .242f,
  .242f,
  .245f,
  .254f,
  .271f,
  .301f,
}};

constexpr std::array<float,n_freq> Lu_ {{
  -31.6f,
  -27.2f,
  -23.0f,
  -19.1f,
  -15.9f,
  -13.0f,
  -10.3f,
  -8.1f,
  -6.2f,
  -4.5f,
  -3.1f,
  -2.0f,
  -1.1f,
  -0.4f,
  0.f,
  .3f,
  .5f,
  0.f,
  -2.7f,
  -4.1f,
  -1.0f,
  1.7f,
  2.5f,
  1.2f,
  -2.1f,
  -7.1f,
  -11.2f,
  -10.7f,
  -3.1f
}};

constexpr std::array<float,n_freq> Tf_ {{
  78.5f,
  68.7f,
  59.5f,
  51.1f,
  44.0f,
  37.5f,
  31.5f,
  26.5f,
  22.1f,
  17.9f,
  14.4f,
  11.4f,
  8.6f,
  6.2f,
  4.4f,
  3.0f,
  2.2f,
  2.4f,
  3.5f,
  1.7f,
  -1.3f,
  -4.2f,
  -6.0f,
  -5.4f,
  -1.5f,
  6.0f,
  12.6f,
  13.9f,
  12.3f
}};

// returns pair<index, ratio>, where:
// . ratio is the proportion of "index",
// . 1-ratio is the proportion of "index-1"
template<typename T>
static inline std::pair<int, float>
closest(std::array<T,n_freq> const & arr,
        T const value) {
  auto min_ = 0;
  auto max_ = n_freq-1;
  do {
    auto center = (min_ + max_) / 2;
    if(value < arr[center]) {
      Assert(max_ != center);
      max_ = center;
    }
    else {
      Assert(min_ != center);
      min_ = center;
    }
  }
  while(max_ - min_ >= 2);
  
  Assert(max_ == min_ + 1);
  Assert(value >= arr[min_] || min_ == 0);
  Assert(value <= arr[max_] || max_ == n_freq - 1);
  if(value <= arr[min_]) {
    Assert(min_==0 || value == arr[min_]);
    return {min_, 1.f};
  }
  if(value >= arr[max_]) {
    Assert(max_ == n_freq-1);
    return {max_, 1.f};
  }
  return {max_, (value - arr[min_]) / (arr[max_] - arr[min_])};
}

constexpr float compute_equal_loudness_volume(int const i, float const LN) {
  auto alpha_f = alpha_f_[i];
  auto Lu = Lu_[i];
  auto Tf = Tf_[i];
  
  auto Af = 4.47e-3f * (sprout::pow(10.f, .025f * LN) - 1.14f) + sprout::pow(.4f * sprout::pow(10.f, ((Tf + Lu) * .1f) - 9.f), alpha_f);
  
  auto Lp = 94.f - Lu + (10.f/alpha_f)* sprout::log(Af) / sprout::log(10.f);
  return Lp;
}

constexpr std::array<float, n_freq> compute_elv(float level) {
  std::array<float, n_freq> v{};
  for (int i=0; i < static_cast<int>(v.size()); ++i) {
    v[i] = compute_equal_loudness_volume(i, level);
  }
  return v;
}


constexpr std::array<std::array<float, n_freq>, 9> compute_elvs() {
  std::array<std::array<float, n_freq>, 9> elvs2{{{},{},{},{},{},{},{},{},{}}};
  for (int i=0; i < static_cast<int>(elvs2.size()); ++i) {
    elvs2[i] = compute_elv((i+2)*10.f);
  }
  return elvs2;
}

static constexpr std::array<std::array<float, n_freq>, 9> elvs{compute_elvs()};

constexpr auto LN_default = 40.f; // unit : phons

template<typename T>
constexpr int phons_to_index(T level) {
  static_assert(std::is_floating_point_v<T>);
  
  // 20 .. 100 -> 0 .. 8;
  int i = static_cast<int>(level * .1f) - 2;
  i = std::max(0, i);
  i = std::min(static_cast<int>(elvs.size()) - 1, i);
  return i;
}

template<typename T>
static inline float equal_loudness_volume_db(std::array<T,n_freq> const & arr,
                                             T value,
                                             int level) {
  auto const [i, ratio] = closest(arr, value);

  auto & elv = elvs[level];
  if(ratio == 1.f) {
    return elv[i];
  }
  Assert(ratio < 1.f);
  Assert(ratio >= 0.f);
  return ratio * elv[i] + (1.f-ratio) * elv[i-1];
}

template<typename T>
T db_to_amplitude(T const db, T const max_db, T const log_ratio) {
  if(db > max_db) {
    return 1.f;
    // equivalent to:
    // db = max_db;
  }
  return std::pow(static_cast<T>(10),
                  log_ratio * (db-max_db)/20.f);
}

template<typename T>
static inline float equal_loudness_volume_generic(std::array<T, n_freq> const & arr,
                                                  T value,
                                                  int index_freq_ref = 0,
                                                  float log_ratio = 1.f,
                                                  float level = LN_default) {
  int i = phons_to_index(level);
  
  auto max_db = elvs[i][index_freq_ref];
  auto db = equal_loudness_volume_db(arr, value, i);
  
  return db_to_amplitude(db,
                         max_db,
                         log_ratio);
}

static inline float equal_loudness_volume_from_freq(float freq, int index_freq_ref = 0, float log_ratio = 1.f, float level = LN_default) {
  return equal_loudness_volume_generic(freqs, freq, index_freq_ref, log_ratio, level);
}
static inline float equal_loudness_volume_from_pitch(MidiPitch pitch, int index_freq_ref = 0, float log_ratio = 1.f, float level = LN_default) {
  return equal_loudness_volume_generic(pitches, pitch, index_freq_ref, log_ratio, level);
}
}
} // NS imajuscule
