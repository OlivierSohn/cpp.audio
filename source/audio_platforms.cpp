
namespace imajuscule {
    namespace audio {

        /*
        * Denormals can appear in reverb algorithm, when signal becomes close to 0.
        */
        void disableDenormals() {
          #if __APPLE__
            fesetenv(FE_DFL_DISABLE_SSE_DENORMS_ENV);
            fesetenv(FE_DFL_DISABLE_DENORMS_ENV);
          #else
          #define CSR_FLUSH_TO_ZERO         (1 << 15)
              unsigned csr = __builtin_ia32_stmxcsr();
              csr |= CSR_FLUSH_TO_ZERO;
              __builtin_ia32_ldmxcsr(csr);
          #endif
        }

        AudioLockPolicyImpl<AudioOutPolicy::Slave> & fakeAudioLock() {
            static AudioLockPolicyImpl<AudioOutPolicy::Slave> l;
            return l;
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
