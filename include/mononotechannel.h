
namespace imajuscule {
    namespace audio {

        template<typename AudioElem, typename Chan>
        struct MonoNoteChannel {
            static_assert(AudioElem::hasEnvelope,"");
            using buffer_t = typename AudioElem::buffer_t;

            MonoNoteChannel(buffer_t & b) : elem(b) {}

            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            float tuning;
            Chan * channel = nullptr;
            AudioElem elem;

            template<WithLock lock_policy, typename ChannelsT>
            bool open(ChannelsT & out, float inital_volume) {
                static_assert(ChannelsT::XFPolicy==Chan::XFPolicy);
                constexpr auto xfadeLen = (ChannelsT::XFPolicy==XfadePolicy::UseXfade)?401:0;
                auto cid = out.template openChannel<lock_policy>({inital_volume}, ChannelClosingPolicy::ExplicitClose
                                                                , xfadeLen);
                if(cid == AUDIO_CHANNEL_NONE) {
                  channel = nullptr;
                }
                else {
                  channel = &out.editChannel(cid);
                }
                return channel != nullptr;
            }

            void reset() {
                Assert(channel);
                channel->reset();
            }

            void show() {
              if(channel) {
                channel->show();
              }
              else {
                std::cout << "none channel";
              }
            }

            void setVolume(float volume) {
                Assert(channel);
                channel->setVolume(volume);
            }

        };

    }
} // namespace
