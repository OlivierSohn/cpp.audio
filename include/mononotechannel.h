
namespace imajuscule::audio {

      struct TunedPitch {
        // instead of noteId, on note off we receive the pitch, so pitch + tuning is the key
        TunedPitch(int pitch, float tuning):
        note (midi::tuned_note(pitch, tuning))
        {}

        TunedPitch() :
        note(sentinel)
        {}

        void clear() {
          note = sentinel;
        }

        bool hasValue() const {
          return note != sentinel;
        }

        float getValue() const {
          Assert(hasValue());
          return note;
        }

        bool operator != (const TunedPitch & other) const {
          return !(*this==other);
        }
        bool operator == (const TunedPitch & other) const {
          return std::abs(note-other.note) < 1e-6;
        }

      private:
        float note;

        static constexpr auto sentinel = std::numeric_limits<decltype(TunedPitch::note)>::min();
      };

        template<typename AudioElem, typename Chan>
        struct MonoNoteChannel {
            static_assert(AudioElem::hasEnvelope);
            using buffer_t = typename AudioElem::buffer_t;

            MonoNoteChannel(buffer_t & b) : elem(b) {}

            Chan * channel = nullptr;
            AudioElem elem;
#ifndef CUSTOM_SAMPLE_RATE
            Optional<MIDITimestampAndSource> midiDelay;
#endif
          
            template<WithLock lock_policy, typename ChannelsT>
            bool open(ChannelsT & out, float inital_volume) {
                constexpr auto xfadeLen = (Chan::XFPolicy==XfadePolicy::UseXfade)?401:0;
                auto cid = out.template openChannel<lock_policy>({inital_volume}
                                                                , ChannelClosingPolicy::ExplicitClose
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
} // NS imajuscule::audio
