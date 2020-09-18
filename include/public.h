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

#include "audio_platforms.h"

#include "sound.defines.h"

#include "loudness_filter_coefficients.h"
#include "loudness.h"
#include "sound.functions.h"
#include "stereo.h"
#include "sound.h"
#include "loudness_filter.h"
#include "audioelement.h"

#ifndef NO_OUTPUT
# if TARGET_OS_IOS
#  import <AudioToolbox/AudioToolbox.h>
#  include "audio_platform_au.h"
# else
#  include "portaudio.h"
#  include "audio_platform_pa.h"
# endif
#endif

#include "midi.h"
#include "noise.h"
#include "sounds.h"
#include "request.h"
#include "channel.h"
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
#include "normalization.h"
#include "gen.crtp.h"
#include "gen.cutramp.h"
#include "gen.sine.h"
#include "gen.voice.h"
#include "events.impl.h"
#include "resynth.hpp"
