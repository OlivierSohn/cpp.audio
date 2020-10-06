#include "private.h"

#include "events.cpp"
#include "sound.cpp"
#include "parse.music.cpp"
#include "gen.voice.cpp"
#include "loudness_filter.cpp"
#include "soundengine.cpp"
#include "normalization.cpp"
#include "audio_platforms.cpp"
#if TARGET_OS_IOS
# include "audio_platform_au.cpp"
#else
# include "audio_platform_pa.cpp"
#endif
#include "audio_context.cpp"

namespace imajuscule::audio {
  /*
      TODO to hear exactly what other players are hearing,
      each client should use the same delays for the same sources,
      plus a per-client global offset.
  */
  std::unordered_map<uint64_t, std::optional<uint64_t>> & midiDelays() {
    static std::unordered_map<uint64_t, std::optional<uint64_t>> m(100);
    return m;
  }

  /*
  * Can be deduced from the source characteristics:
  *  - poll period, on the source client
  *  - network characteristics
  *  - etc...
  */
  uint64_t & maxMIDIJitter() {
    static uint64_t v = 0;
    return v;
  }

  namespace audioelement {
    std::ostream & operator << (std::ostream & os, EnvelopeRelease e) {
        switch(e) {
            case EnvelopeRelease::WaitForKeyRelease:
              os << "WaitForKeyRelease"; break;
            case EnvelopeRelease::ReleaseAfterDecay:
              os << "ReleaseAfterDecay"; break;
        }
        return os;
    }
  }
}
