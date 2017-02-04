
namespace imajuscule {
    namespace audio {
        
        template<int N, typename AudioElem>
        struct MonoNoteChannel {
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
            
            template<typename OutputData>
            bool open(OutputData & out, float inital_volume, int xfade_len) {
                
                // "all or nothing" strategy
                
                for(auto & c : channels) {
                    c = out.openChannel({inital_volume},  ChannelClosingPolicy::ExplicitClose, xfade_len);
                    if(c == AUDIO_CHANNEL_NONE) {
                        for(auto & c : channels) {
                            if(c == AUDIO_CHANNEL_NONE) {
                                continue;
                            }
                            out.closeChannel(c, CloseMode::NOW, 0);
                            c = AUDIO_CHANNEL_NONE;
                        }
                        return false;
                    }
                }
                return true;
            }
            
            template<typename OutputData>
            bool close(OutputData & out, CloseMode m, int xfade_len = 0) {

                // "all or nothing" strategy
                
                for(auto & c : channels) {
                    if(c == AUDIO_CHANNEL_NONE) {
                        return false;;
                    }
                }
                for(auto & c : channels) {
                    out.closeChannel(c, m, xfade_len);
                    c = AUDIO_CHANNEL_NONE;
                }
                return true;
            }
        };
        
    }
} // namespace
