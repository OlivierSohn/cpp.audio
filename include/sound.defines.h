// this hugly hack should be removed once we platform-templatize audioIn, like we did for audioOut
typedef void PaStream; //from "portaudio.h"

namespace imajuscule {
    using SAMPLE = float;
#ifndef CUSTOM_SAMPLE_RATE
    constexpr int SAMPLE_RATE = 44100;
# define MAYBE_CONSTEXPR_SAMPLE_RATE constexpr
#else
  extern int SAMPLE_RATE;
# define MAYBE_CONSTEXPR_SAMPLE_RATE
#endif
}
