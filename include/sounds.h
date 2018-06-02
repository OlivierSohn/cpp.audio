
namespace imajuscule {

    class Sounds {
        std::map< soundId, soundBuffer > sounds;
        std::array<audioelement::Square<audioelement::SimpleLinearEnvelope<float>>, 8> squares;
        std::array<audioelement::Oscillator<audioelement::SimpleLinearEnvelope<float>>, 8> oscillators;
        std::array<audioelement::FreqRamp<audioelement::SimpleLinearEnvelope<float>>, 6> ramps;

        std::array<audioelement::RingModulation<
        audioelement::LowPassAlgo<audioelement::PulseTrainAlgo<float>, 1>,
        audioelement::Envelopped<audioelement::OscillatorAlgo<float>,audioelement::SimpleLinearEnvelope<float>>
        >, 6> ringmods;

        std::array<audioelement::LowPass<audioelement::PulseTrainAlgo<float>, 1>, 6> lptrains;
    public:
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
