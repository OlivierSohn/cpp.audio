
namespace imajuscule {
  namespace audio {

    template<int nOuts, AudioOutPolicy P>
    struct ChannelsVecAggregate {
      static constexpr auto nAudioOut = nOuts;
      static constexpr auto policy = P;
      static constexpr auto atomicity = getAtomicity<policy>();

      using XFadeChans         = Channels<nAudioOut, XfadePolicy::UseXfade,  MaxQueueSize::One, policy>;
      using NoXFadeChans       = Channels<nAudioOut, XfadePolicy::SkipXfade, MaxQueueSize::One, policy>;
      using XFadeInfiniteChans = Channels<nAudioOut, XfadePolicy::UseXfade,  MaxQueueSize::Infinite, policy>;

      using Request = Request<atomicity, nAudioOut>;
      using Volumes = Volumes<nAudioOut>;

      auto & getChannelsXFade() { return cX; }
      auto & getChannelsNoXFade() { return cNoX; }
      auto & getChannelsXFadeInfinite() { return cXInf; }

      template <typename F>
      void forEach(F f) {
        cX.forEach(   [&f] (auto & c) { c.forEach(f); });
        cNoX.forEach( [&f] (auto & c) { c.forEach(f); });
        cXInf.forEach([&f] (auto & c) { c.forEach(f); });
      }

      void run_computes(int nFrames, uint64_t tNanos) {
        cX.forEach(   [nFrames, tNanos] (auto & c) { c.run_computes(nFrames, tNanos); });
        cNoX.forEach( [nFrames, tNanos] (auto & c) { c.run_computes(nFrames, tNanos); });
        cXInf.forEach([nFrames, tNanos] (auto & c) { c.run_computes(nFrames, tNanos); });
      }

      void closeAllChannels(int xfade) {
        cX.forEach(   [xfade] (auto & c) { c.closeAllChannels(xfade); });
        cNoX.forEach( [xfade] (auto & c) { c.closeAllChannels(xfade); });
        cXInf.forEach([xfade] (auto & c) { c.closeAllChannels(xfade); });
      }

    private:
      template<typename T>
      using container = imajuscule::lockfree::scmp::forward_list<T>; // TODO specialize the container for single thread
      container<XFadeChans> cX;
      container<NoXFadeChans> cNoX;
      container<XFadeInfiniteChans> cXInf;
    };

  }
}
