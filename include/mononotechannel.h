
namespace imajuscule {
    namespace audio {
        
        template<int N, typename AudioElem>
        struct MonoNoteChannel {
            static constexpr auto n_channels_per_note = N;
            
            MonoNoteChannel() {
                channels.fill(AUDIO_CHANNEL_NONE);
            }
            
            uint8_t pitch; // instead of noteId, on note off we receive the pitch, so pitch is the key
            float tuning;
            std::array<uint8_t, N> channels;
            AudioElem elem;
            
            bool opened() const {
                for(auto & c : channels) {
                    if(c == AUDIO_CHANNEL_NONE) {
                        return false;
                    }
                }
                return true;
            }
            
            bool closed() const { return !opened(); }
            
            template<WithLock lock_policy, typename OutputData>
            bool open(OutputData & out, float inital_volume, int xfade_len) {
                
                // "all or nothing" strategy
                
                for(auto & c : channels) {
                    c = out.template openChannel<lock_policy>({inital_volume},  ChannelClosingPolicy::ExplicitClose, xfade_len);
                    if(c == AUDIO_CHANNEL_NONE) {
                        for(auto & c : channels) {
                            if(c == AUDIO_CHANNEL_NONE) {
                                continue;
                            }
                            out.template closeChannel<lock_policy>(c, CloseMode::NOW, 0);
                            c = AUDIO_CHANNEL_NONE;
                        }
                        return false;
                    }
                }
                return true;
            }
            
            template<WithLock lock_policy, typename OutputData>
            bool close(OutputData & out, CloseMode m, int xfade_len = 0) {

                // "all or nothing" strategy
                
                for(auto & c : channels) {
                    if(c == AUDIO_CHANNEL_NONE) {
                        return false;
                    }
                }
                for(auto & c : channels) {
                    out.template closeChannel<lock_policy>(c, m, xfade_len);
                    c = AUDIO_CHANNEL_NONE;
                }
                return true;
            }
        };
        
    }
} // namespace
