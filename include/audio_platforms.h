

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

        extern int32_t n_audio_cb_frames;


      /*
      * Denormals can appear in reverb algorithm, when signal becomes close to 0.
      */
      void disableDenormals();

    }
}
