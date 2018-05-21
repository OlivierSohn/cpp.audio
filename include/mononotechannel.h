
namespace imajuscule {
    namespace audio {

        template<typename AudioElem>
        struct MonoNoteChannel {
            static_assert(AudioElem::hasEnveloppe,"");

            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            float tuning;
            uint8_t channel = AUDIO_CHANNEL_NONE;
            AudioElem elem;

            template<WithLock lock_policy, typename OutputData>
            bool open(OutputData & out, float inital_volume) {
                channel = out.template openChannel<lock_policy>({inital_volume}, ChannelClosingPolicy::ExplicitClose, 0);
                return channel != AUDIO_CHANNEL_NONE;
            }

            template<typename OutputData>
            void setVolume(OutputData & out, float volume) {
                out.template setVolume(channel, volume);
            }
        };

    }
} // namespace
