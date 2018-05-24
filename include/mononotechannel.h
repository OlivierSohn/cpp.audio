
namespace imajuscule {
    namespace audio {

        template<typename AudioElem>
        struct MonoNoteChannel {
            static_assert(AudioElem::hasEnvelope,"");

            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            float tuning;
            uint8_t channel = AUDIO_CHANNEL_NONE; // TODO use a pointer to the channel instead.
            AudioElem elem;

            template<WithLock lock_policy, typename ChannelsT>
            bool open(ChannelsT & out, float inital_volume) {
                channel = out.template openChannel<lock_policy>({inital_volume}, ChannelClosingPolicy::ExplicitClose, 401);
                return channel != AUDIO_CHANNEL_NONE;
            }

            template<typename ChannelsT>
            void reset(ChannelsT & out, int xfade_len) {
              if(channel != AUDIO_CHANNEL_NONE) {
                out.template editChannel(channel).reset(xfade_len);
              }
            }

            template<typename ChannelsT>
            void show(ChannelsT & out) {
              if(channel != AUDIO_CHANNEL_NONE) {
                out.template editChannel(channel).show();
              }
              else {
                std::cout << "none channel";
              }
            }

            template<typename ChannelsT>
            void setVolume(ChannelsT & out, float volume) {
                out.template setVolume(channel, volume);
            }
        };

    }
} // namespace
