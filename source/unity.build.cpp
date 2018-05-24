#include "private.h"

#include "read.wav.cpp"
#include "write.wav.cpp"
#include "samples.cpp"
#include "sound.cpp"
#include "parse.music.cpp"
#include "gen.voice.cpp"
#include "gen.cutramp.cpp"
#include "loudness_filter.cpp"
#include "soundengine.cpp"
#include "normalization.cpp"

namespace imajuscule {
    namespace audio {
        AudioLockPolicyImpl<AudioOutPolicy::Slave> & fakeAudioLock() {
            static AudioLockPolicyImpl<AudioOutPolicy::Slave> l;
            return l;
        }
    }
}
