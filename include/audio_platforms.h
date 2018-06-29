

namespace imajuscule {
    namespace audio {

        enum class AudioPlatform {
            PortAudio,
            AudioUnits
        };

        enum class Features {
              JustOut
            , InAndOut
        };

        template<AudioPlatform A, Features F, typename Chans>
        struct Context;

      /*
      * Denormals can appear in reverb algorithm, when signal becomes close to 0.
      */
      void disableDenormals();

      constexpr auto initial_n_audio_cb_frames = -1;
      // 'n_audio_cb_frames' is initially 'initial_n_audio_cb_frames', until the audio callback sets the value.
      extern std::atomic<int32_t> n_audio_cb_frames;
      static_assert(std::atomic<int32_t>::is_always_lock_free);

      extern int wait_for_first_n_audio_cb_frames();

    }
}
