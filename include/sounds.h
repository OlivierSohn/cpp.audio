
namespace imajuscule {

    class Sounds {
        std::map< soundId, soundBuffer > sounds;
        std::array<audioelement::Square<float>, 8> squares;
        std::array<audioelement::Oscillator<float>, 8> oscillators;
        std::array<audioelement::FreqRamp<float>, 6> ramps;
 
        std::array<audioelement::RingModulation<
        audioelement::LowPassAlgo<audioelement::PulseTrainAlgo<float>>,
        audioelement::OscillatorAlgo<float>
        >, 6> ringmods;
        
        std::array<audioelement::LowPass<audioelement::PulseTrainAlgo<float>>, 6> lptrains;
    public:
        soundBuffer & get( soundId );
        
        audioelement::Oscillator<float> * getInactiveOscillator() {
            return editInactiveAudioElement(oscillators);
        }
        
        audioelement::Square<float> * getInactiveSquare() {
            return editInactiveAudioElement(squares);
        }
        
        audioelement::FreqRamp<float> * getInactiveFreqRamp() {
            return editInactiveAudioElement(ramps);
        }
        
        auto * getInactiveRingMod() {
            return editInactiveAudioElement(ringmods);
        }
        
        audioelement::LowPass<audioelement::PulseTrainAlgo<float>> * getInactiveLPTrain() {
            return editInactiveAudioElement(lptrains);
        }
    };
    
}
