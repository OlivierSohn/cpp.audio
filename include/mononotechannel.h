
namespace imajuscule::audio {

template<typename AudioElem, typename Chan>
struct MonoNoteChannel {
  static_assert(AudioElem::hasEnvelope);
  using buffer_t = typename AudioElem::buffer_t;
  
  MonoNoteChannel(buffer_t & b) : elem(b) {}
  
  Chan * channel = nullptr;
  AudioElem elem;
  Optional<MIDITimestampAndSource> midiDelay;
  
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
  
  template<WithLock lock_policy, typename ChannelsT>
  void close(ChannelsT & out) {
    Assert(channel);
    out.template closeChannel<lock_policy>(out.getChannelId(*channel),
                                           CloseMode::NOW);
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
