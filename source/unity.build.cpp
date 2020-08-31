#include "private.h"

#include "samples.cpp"
#include "sound.cpp"
#include "parse.music.cpp"
#include "gen.voice.cpp"
#include "gen.cutramp.cpp"
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
      The first call is expensive, as the array is allocated.

      TODO to hear exactly what other players are hearing,
      each client should use the same delays for the same sources,
      plus a per-client global offset.
  */
  std::array<TimeDelay, MIDITimestampAndSource::nSources> & midiDelays() {
    // 'TimeDelay's will be default initialized by the array.
    static std::array<TimeDelay, MIDITimestampAndSource::nSources> arr;
    return arr;
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
