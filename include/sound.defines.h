// this hugly hack should be removed once we platform-templatize audioIn, like we did for audioOut
typedef void PaStream; //from "portaudio.h"

namespace imajuscule {
    using SAMPLE = float;
    constexpr int SAMPLE_RATE = 44100;
}
