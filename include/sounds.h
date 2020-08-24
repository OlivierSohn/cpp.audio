
namespace imajuscule::audio {
  namespace detail {
    extern std::map< soundId, soundBuffer > & getSounds();
  }
    template<Atomicity A>
    class Sounds {

      // this is not ideal, as buffers are not necessarily in the same page.
      std::vector<std::unique_ptr<audioelement::AEBuffer<float>>> buffers;
      auto & takeBuffer() {
        buffers.emplace_back(std::make_unique<audioelement::AEBuffer<float>>());
        return *(buffers.back());
      }

      std::array<audioelement::Square<audioelement::SimpleLinearEnvelope<A, float>>, 8> squares;
      std::array<audioelement::Oscillator<audioelement::SimpleLinearEnvelope<A, float>>, 8> oscillators;
      std::array<audioelement::FreqRamp<audioelement::SimpleLinearEnvelope<A, float>>, 6> ramps;

      std::array<
        audioelement::FinalAudioElement<
          audioelement::SimplyEnveloped< A, 
            audioelement::RingModulationAlgo<
              audioelement::LowPassAlgo<audioelement::PulseTrainAlgo<float>, 1>,
              audioelement::OscillatorAlgo<float>
            >>>
      , 6> ringmods;

        std::array<audioelement::LowPass<audioelement::PulseTrainAlgo<float>, 1>, 6> lptrains;
    public:

      Sounds() :
      squares{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , oscillators{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , ramps{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , ringmods{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , lptrains{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      {}

      soundBuffer & get( soundId id )
      {
        auto & sounds = detail::getSounds();
        {
          auto it = sounds.find(id);
          if( it != sounds.end() ) {
            return it->second;
          }
        }
        auto it = sounds.emplace(id, id);
        Assert(it.second);
        return it.first->second;
      }

        auto * getInactiveOscillator() {
            return editInactiveAudioElement(oscillators);
        }

        auto * getInactiveSquare() {
            return editInactiveAudioElement(squares);
        }

        auto * getInactiveFreqRamp() {
            return editInactiveAudioElement(ramps);
        }

        auto * getInactiveRingMod() {
            return editInactiveAudioElement(ringmods);
        }

        auto * getInactiveLPTrain() {
            return editInactiveAudioElement(lptrains);
        }
    };

}
