
namespace imajuscule {
  namespace audio {

    template<int nOuts, AudioOutPolicy P>
    struct ChannelsVecAggregate {
      static constexpr auto nAudioOut = nOuts;
      static constexpr auto Policy = P;

      using XFadeChans = Channels<nAudioOut, XfadePolicy::UseXfade, Policy>;
      using NoXFadeChans = Channels<nAudioOut, XfadePolicy::SkipXfade, Policy>;

      using Request = Request<nAudioOut>;
      using Volumes = Volumes<nAudioOut>;

      auto & getChannelsXFade() { return cX; }
      auto & getChannelsNoXFade() { return cNoX; }

      template <typename F>
      void forEach(F f) {
        for(auto & c : cX) {
          c->forEach(f);
        }
        for(auto & c : cNoX) {
          c->forEach(f);
        }
      }

      void run_computes(bool tictac) {
        for(auto & c : cX) {
          c->run_computes(tictac);
        }
        for(auto & c : cNoX) {
          c->run_computes(tictac);
        }
      }

      void closeAllChannels(int xfade) {
        for(auto & c : cX) {
          c->closeAllChannels(xfade);
        }
        for(auto & c : cNoX) {
          c->closeAllChannels(xfade);
        }
      }

    private:
      std::vector<std::unique_ptr<XFadeChans>> cX;
      std::vector<std::unique_ptr<NoXFadeChans>> cNoX;
    };

  }
}
