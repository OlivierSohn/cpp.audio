
namespace imajuscule {
    namespace audio {
        template<SoundEngineMode M, typename Logger, typename Mix>
        enumTraversal SoundEngine<M, Logger, Mix>::ModeTraversal(
                                                            static_cast<unsigned int>(SoundEngineMode::BEGIN),
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

        enumTraversal const & xfade_freq_traversal() {
            static enumTraversal et(
                                    static_cast<unsigned int>(FreqXfade::BEGIN),
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

        soundBuffer & getSilence() {
            static soundBuffer silence{1, 0.f};
            return silence;
        }
    }
} // namespaces
