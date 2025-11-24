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

#include "sound.defines.h"
#include "audio_platforms.h"

#include "midi.h"
#include "scales.h"
#include "pitch_generators.h"

#include "sound.functions.h"
#include "stereo.h"
#include "sound.h"

#include "loudness_filter_coefficients.h"
#include "loudness.h"
#include "loudness_filter.h"

#include "audioelement.h"

#include "noise.h"
#include "sounds.h"

#include "request.h"
#include "note.h"


#if NO_OUTPUT
#else
# if TARGET_OS_IOS
#  import <AudioToolbox/AudioToolbox.h>
#  include "audio_platform_au.h"
# else
#  ifndef __EMSCRIPTEN__
#   include "portaudio.h"
#   include "audio_platform_pa.h"
#  endif  // __EMSCRIPTEN__
# endif
#endif

#include "channel.h"
#include "out.h"
#include "simple_audio_context.h"
#include "audio_context.h"
#include "channels.h"
#include "channels_aggregate.h"
#include "soundengine.h"
#include "parse.music.h"
#include "events.h"
#include "eventstream.h"
#include "paramspec.h"
#include "smoothparam.h"
#include "program.h"
#include "normalization.h"
#include "gen.crtp.h"
#include "gen.sine.h"
#include "gen.voice.h"
#include "events.impl.h"
#include "resynth.hpp"
