
namespace imajuscule {
    namespace audio {
        namespace voice {
            const float Limits<FREQ_SCATTER>::m = 0.f;
            const float Limits<FREQ_SCATTER>::M = 1.f;
            
            const float Limits<PHASE_RATIO1>::m = 0.f;
            const float Limits<PHASE_RATIO1>::M = 1.f;

            const float Limits<PHASE_RATIO2>::m = 0.f;
            const float Limits<PHASE_RATIO2>::M = 1.f;
            
            const float Limits<HARMONIC_ATTENUATION>::m = 0.f;
            const float Limits<HARMONIC_ATTENUATION>::M = 0.98f;
            
            const float Limits<LENGTH>::m = 10.f;
            const float Limits<LENGTH>::M = 500.f;
            
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
    }
} // namespaces
