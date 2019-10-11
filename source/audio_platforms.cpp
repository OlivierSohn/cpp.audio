
namespace imajuscule {
    namespace audio {

        // no need to synchronize access to this : it's 4 bytes-aligned,
        // and only one thread writes it (the audio thread), except for initialization time.
        std::atomic<int32_t> n_audio_cb_frames = initial_n_audio_cb_frames;

        int wait_for_first_n_audio_cb_frames() {
            // Note this could deadlock if there is an audio bug at os level
            while(n_audio_cb_frames == initial_n_audio_cb_frames) {
                std::this_thread::yield();
            }
            return n_audio_cb_frames;
        }

    }
}
