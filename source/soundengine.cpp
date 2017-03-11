
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
                                                                    case SoundEngineMode::MARKOV:
                                                                        return "Markov Birds";
                                                                        break;
                                                                }
                                                                return "?";
                                                            });
    }
} // namespaces
