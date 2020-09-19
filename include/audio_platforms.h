

namespace imajuscule::audio {

enum class AudioPlatform {
  PortAudio,
  AudioUnits
};

enum class Features {
  JustOut,
  InAndOut
};

// for output:
template<AudioPlatform A, Features F, typename Chans>
struct Context;

// for input:
template<AudioPlatform A>
struct AudioInput;
using RecordF = std::function<void(const SAMPLE*, int)>;



constexpr auto initial_n_audio_cb_frames = -1;
// 'n_audio_cb_frames' is initially 'initial_n_audio_cb_frames', until the audio callback sets the value.
extern std::atomic<int32_t> n_audio_cb_frames;
static_assert(std::atomic<int32_t>::is_always_lock_free);

extern int wait_for_first_n_audio_cb_frames();

} // NS
