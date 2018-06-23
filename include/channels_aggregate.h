
namespace imajuscule {
  namespace audio {

    template<int nOuts, AudioOutPolicy P>
    struct ChannelsVecAggregate {
      static constexpr auto nAudioOut = nOuts;
      static constexpr auto Policy = P;
      static constexpr auto atomicity = getAtomicity<Policy>();

      using XFadeChans         = Channels<nAudioOut, XfadePolicy::UseXfade,  MaxQueueSize::One, Policy>;
      using NoXFadeChans       = Channels<nAudioOut, XfadePolicy::SkipXfade, MaxQueueSize::One, Policy>;
      using XFadeInfiniteChans = Channels<nAudioOut, XfadePolicy::UseXfade,  MaxQueueSize::Infinite, Policy>;

      using Request = Request<atomicity, nAudioOut>;
      using Volumes = Volumes<nAudioOut>;

      auto & getChannelsXFade() { return cX; }
      auto & getChannelsNoXFade() { return cNoX; }
      auto & getChannelsXFadeInfinite() { return cXInf; }

      template <typename F>
      void forEach(F f) {
        for(auto & c : cX) {
          c->forEach(f);
        }
        for(auto & c : cNoX) {
          c->forEach(f);
        }
        for(auto & c : cXInf) {
          c->forEach(f);
        }
      }

      void run_computes(bool tictac, int nFrames) {
        for(auto & c : cX) {
          c->run_computes(tictac, nFrames);
        }
        for(auto & c : cNoX) {
          c->run_computes(tictac, nFrames);
        }
        for(auto & c : cXInf) {
          c->run_computes(tictac, nFrames);
        }
      }

      void closeAllChannels(int xfade) {
        for(auto & c : cX) {
          c->closeAllChannels(xfade);
        }
        for(auto & c : cNoX) {
          c->closeAllChannels(xfade);
        }
        for(auto & c : cXInf) {
          c->closeAllChannels(xfade);
        }
      }

    private:
      std::vector<std::unique_ptr<XFadeChans>> cX;
      std::vector<std::unique_ptr<NoXFadeChans>> cNoX;
      std::vector<std::unique_ptr<XFadeInfiniteChans>> cXInf;
    };

  }
}
