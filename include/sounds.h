
namespace imajuscule {

    class Sounds {
        std::map< soundId, soundBuffer > sounds;

      // this is not ideal, as buffers are not necessarily in the same page.
      std::vector<std::unique_ptr<audioelement::AEBuffer<float>>> buffers;
      auto & takeBuffer() {
        buffers.emplace_back();
        return *(buffers.back());
      }

      std::array<audioelement::Square<audioelement::SimpleLinearEnvelope<float>>, 8> squares;
        std::array<audioelement::Oscillator<audioelement::SimpleLinearEnvelope<float>>, 8> oscillators;
        std::array<audioelement::FreqRamp<audioelement::SimpleLinearEnvelope<float>>, 6> ramps;

        std::array<audioelement::RingModulation<
        audioelement::LowPassAlgo<audioelement::PulseTrainAlgo<float>, 1>,
        audioelement::Envelopped<audioelement::OscillatorAlgo<float>,audioelement::SimpleLinearEnvelope<float>>
        >, 6> ringmods;

        std::array<audioelement::LowPass<audioelement::PulseTrainAlgo<float>, 1>, 6> lptrains;
    public:
      
      Sounds() :
      squares{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , oscillators{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , ramps{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , ringmods{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      , lptrains{takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer(), takeBuffer()}
      {}

      soundBuffer & get( soundId );

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
