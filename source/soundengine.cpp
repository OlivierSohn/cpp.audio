
namespace imajuscule {
    namespace audio {
        template<SoundEngineMode M, typename Logger>
        enumTraversal SoundEngine<M, Logger>::ModeTraversal(
                                                            static_cast<unsigned int>(SoundEngineMode::BEGIN),
                                                            static_cast<unsigned int>(SoundEngineMode::END),
                                                            [](int val)->const char* {
                                                                auto v = static_cast<SoundEngineMode>(val);
                                                                switch(v) {
                                                                    case SoundEngineMode::ROBOTS:
                                                                        return "Robots";
                                                                        break;
                                                                    case SoundEngineMode::SWEEP:
                                                                        return "Sweep";
                                                                        break;
                                                                    case SoundEngineMode::BIRDS:
                                                                        return "Markov Birds";
                                                                        break;
                                                                }
                                                                return "?";
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
                                                break;
                                            case FreqXfade::NonTrivial:
                                                return "Non Trivial";
                                                break;
                                            case FreqXfade::All:
                                                return "All";
                                                break;
                                        }
                                        return "?";
                                    });
            return et;
        }
        
        soundBuffer & getSilence() {
            static soundBuffer silence{1, 0.f};
            return silence;
        }
    }
} // namespaces
