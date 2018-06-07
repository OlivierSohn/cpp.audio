
namespace imajuscule {
    namespace audio {

        template<typename AudioElem>
        struct MonoNoteChannel {
            static_assert(AudioElem::hasEnvelope,"");
            using buffer_t = typename AudioElem::buffer_t;
          
            MonoNoteChannel(buffer_t & b) : elem(b) {}

            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            uint8_t channel = AUDIO_CHANNEL_NONE; // TODO use a pointer to the channel instead.
            float tuning;
            AudioElem elem;

            template<WithLock lock_policy, typename ChannelsT>
            bool open(ChannelsT & out, float inital_volume) {
                constexpr auto xfadeLen = (ChannelsT::XFPolicy==XfadePolicy::UseXfade)?401:0;
                channel = out.template openChannel<lock_policy>({inital_volume}, ChannelClosingPolicy::ExplicitClose
                                                                , xfadeLen);
                return channel != AUDIO_CHANNEL_NONE;
            }

            template<typename ChannelsT>
            void reset(ChannelsT & out) {
                Assert(channel != AUDIO_CHANNEL_NONE);
                out.template editChannel(channel).reset();
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
                Assert(channel != AUDIO_CHANNEL_NONE);
                out.template setVolume(channel, volume);
            }

            template<typename ChannelsT>
            void setXFade(ChannelsT & out, int xf) {
                Assert(channel != AUDIO_CHANNEL_NONE);
                out.template setXFade(channel, xf);
            }
        };

    }
} // namespace
