
namespace imajuscule {
    namespace audio {
        template<typename Logger, UpdateMode u>
        enumTraversal SoundEngine<Logger, u>::ModeTraversal(
                                                            static_cast<unsigned int>(Mode::BEGIN),
                                                            static_cast<unsigned int>(Mode::END),
                                                            [](int val)->const char* {
                                                                auto v = static_cast<Mode>(val);
                                                                switch(v) {
                                                                    case Mode::ROBOTS:
                                                                        return "Robots";
                                                                        break;
                                                                    case Mode::BIRDS:
                                                                        return "Birds";
                                                                        break;
                                                                    case Mode::MARKOV:
                                                                        return "Markov Birds";
                                                                        break;
                                                                }
                                                                return "?";
                                                            });
    }
} // namespaces
