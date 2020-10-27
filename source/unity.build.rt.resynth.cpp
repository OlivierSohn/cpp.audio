#define IMJ_DEBUG_AUDIO_OUT 1
#define IMJ_DEBUG_AUDIO_IN 1
//#define IMJ_LOG_MIDI 1
//#define IMJ_DEBUG_VOCODER 1

#include <mutex>
#include <iostream>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
# include <wx/wx.h>
#endif
#include <wx/dcbuffer.h>

#include "public.h"

#include "rt.resynth.lib.midi.cpp"
#include "rt.resynth.lib.metaqueue.cpp"
#include "rt.resynth.lib.input.cpp"
#include "rt.resynth.lib.periodicfft.cpp"
#include "rt.resynth.lib.vocoder.cpp"
#include "rt.resynth.lib.autotune.cpp"
#include "rt.resynth.lib.algo.cpp"
#include "rt.resynth.lib.locked.data.cpp"
#include "rt.resynth.lib.test.cpp"
#include "rt.resynth.lib.cpp"

#include "rt.resynth.ui.params.cpp"
#include "rt.resynth.ui.analysis.cpp"
#include "rt.resynth.ui.vocoder.cpp"
#include "rt.resynth.ui.autotune.cpp"
#include "rt.resynth.ui.cpp"
