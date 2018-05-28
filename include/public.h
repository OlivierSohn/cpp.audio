#pragma once

#include <algorithm>
#include <atomic>
#include <complex>
#include <map>
#include <queue>
#include <set>
#include <thread>
#include <type_traits>
#include <vector>

#include "../../cpp.algorithms/include/public.h"
#include "../../cpp.os.logs/include/public.h"
#include "../../cpp.os.storage/include/public.h"

extern "C" {
#include "public_c.h"
}
inline bool operator < (AHDSR_t const& l, AHDSR_t const& r)
{
    return
      std::make_tuple(l.attack,l.hold,l.decay,l.sustain,l.release) <
      std::make_tuple(r.attack,r.hold,r.decay,r.sustain,r.release);
}

#include "audio_platforms.h"

#include "sound.defines.h"

#if TARGET_OS_IOS
# import <AudioToolbox/AudioToolbox.h>
# include "audio_platform_au.h"
#else
# include "portaudio.h"
# include "audio_platform_pa.h"
#endif

#include "samples.h"
#include "loudness_filter_coefficients.h"
#include "loudness.h"
#include "sound.functions.h"
#include "stereo.h"
#include "sound.h"
#include "loudness_filter.h"
#include "audioelement.h"
#include "noise.h"
#include "sounds.h"
#include "request.h"
#include "channel.h"
#include "read.wav.h"
#include "out.h"
#include "audio_context.h"
#include "channels.h"
#include "channels_aggregate.h"
#include "note.h"
#include "soundengine.h"
#include "parse.music.h"
#include "events.h"
#include "mononotechannel.h"
#include "paramspec.h"
#include "smoothparam.h"
#include "program.h"
#include "midi.h"
#include "normalization.h"
#include "gen.crtp.h"
#include "gen.cutramp.h"
#include "gen.sine.h"
#include "gen.vasine.h"
#include "gen.voice.h"
#include "events.impl.h"
