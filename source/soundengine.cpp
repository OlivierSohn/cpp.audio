
namespace imajuscule::audio::audioelement {

template<SoundEngineMode M, int nOuts, Atomicity fa, typename Logger, typename Mix>
enumTraversal SoundEngine<M, nOuts, fa, Logger, Mix>::ModeTraversal(static_cast<unsigned int>(SoundEngineMode::BEGIN),
                                                                    static_cast<unsigned int>(SoundEngineMode::END),
                                                                    [](int val)->const char* {
  auto v = static_cast<SoundEngineMode>(val);
  switch(v) {
    case SoundEngineMode::ROBOTS:
      return "Robots";
    case SoundEngineMode::SWEEP:
      return "Sweep";
    case SoundEngineMode::BIRDS:
      return "Birds";
    case SoundEngineMode::WIND:
      return "Wind";
    default:
      return "?";
  }
});

} // namespace

namespace imajuscule::audio {

enumTraversal const & xfade_freq_traversal() {
  static enumTraversal et(static_cast<unsigned int>(FreqXfade::BEGIN),
                          static_cast<unsigned int>(FreqXfade::END),
                          [](int val)->const char* {
    auto v = static_cast<FreqXfade>(val);
    switch(v) {
      case FreqXfade::No:
        return "None";
      case FreqXfade::NonTrivial:
        return "Non Trivial";
      case FreqXfade::All:
        return "All";
      default:
        return "?";
    }
  });
  return et;
}

} // namespace
