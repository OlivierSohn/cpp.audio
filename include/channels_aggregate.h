
namespace imajuscule {
  namespace audio {
    
    template<typename Chans1, typename Chans2>
    struct ChannelsAggregate {
      static constexpr auto nAudioOut = Chans1::nAudioOut;
      static_assert(Chans1::nAudioOut == Chans2::nAudioOut);

      static constexpr auto Policy = Chans1::Policy;
      static_assert( Chans1::Policy == Chans2::Policy);

      using Request = Request<nAudioOut>;
      using Volumes = Volumes<nAudioOut>;

      template<typename ...Args>
      ChannelsAggregate(AudioLockPolicyImpl<Policy>&l, Args ... args):
        c1(l, args...)
      , c2(l, args...)
      {}

      auto & getChannels1() { return c1; }
      auto & getChannels2() { return c2; }

      template <typename F>
      void forEach(F f) {
        c1.forEach(f);
        c2.forEach(f);
      }

      void run_computes(bool tictac) {
        c1.run_computes(tictac);
        c2.run_computes(tictac);
      }

      void closeAllChannels(int xfade) {
        c1.closeAllChannels(xfade);
        c2.closeAllChannels(xfade);
      }

    private:
      Chans1 c1;
      Chans2 c2;
    };

  }
}
