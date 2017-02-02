
namespace imajuscule {
    namespace audio {
        
        struct MonoNoteChannel {
            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            float tuning;
            uint8_t channel = AUDIO_CHANNEL_NONE;
            audioelement::FreqRamp<float> osc;
        };
        
    }
} // namespace
