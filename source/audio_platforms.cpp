
namespace imajuscule {
    namespace audio {

        /*
        * Denormals can appear in reverb algorithm, when signal becomes close to 0.
        */
        void disableDenormals() {
#if __has_include(<xmmintrin.h>)
          _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
          _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#elif __has_include(<fenv.h>)
          fesetenv(FE_DFL_DISABLE_SSE_DENORMS_ENV);
#  if TARGET_OS_IOS
          fesetenv(FE_DFL_DISABLE_DENORMS_ENV);
#  endif
#endif
        }

        // no need to synchronize access to this : it's 4 bytes-aligned,
        // and only one thread writes it (the audio thread), except for initialization time.
        int32_t n_audio_cb_frames = initial_n_audio_cb_frames;

        int wait_for_first_n_audio_cb_frames() {
            // Note this could deadlock if there is an audio bug at os level
            while(n_audio_cb_frames == initial_n_audio_cb_frames) {
                std::this_thread::yield();
            }
            return n_audio_cb_frames;
        }

    }
}
