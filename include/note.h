
namespace imajuscule::audio {

struct Midi {
  static constexpr double unity_tuning_stretch = 1.;
  
  // Midi A(69) according to http://subsynth.sourceforge.net/midinote2freq.html
  static constexpr float freq_A = 440.f;
  static constexpr uint8_t A_pitch = 69;
  
  // TODO constexpr that, using  https://github.com/elbeno/constexpr/blob/master/src/include/cx_math.h
  Midi(double const tuning_stretch = unity_tuning_stretch)
  : half_tone_ratio_(std::pow(2., tuning_stretch/12.))
  , tuning_stretch_(tuning_stretch)
  {}
  
  double getHalfToneRatio() const {
    return half_tone_ratio_;
  }
  
  std::optional<double> frequency_to_midi_pitch(double const freq) const {
    if (freq <= 0) {
      return {};
    }
    return 69. + (12. / tuning_stretch_) * std::log2(freq/440.);
  }
  
  double Ainterval_to_freq(double const interval_from_A) const {
    return freq_A * std::pow(half_tone_ratio_, interval_from_A);
  }

  double midi_pitch_to_freq(int pitch) const {
    return Ainterval_to_freq(pitch - A_pitch);
  }
  
  double transpose_frequency(double const freq, int n) const {
    return freq * std::pow(half_tone_ratio_, n);
  }

private:
  double const half_tone_ratio_;
  double const tuning_stretch_;
};


    enum Note : unsigned char{
        NOTE_ERROR,
        Silence,
        Do,
        Dod,
        Re,
        Red,
        Mi,
        Fa,
        Fad,
        Sol,
        Sold,
        La,
        Lad,
        Si,
    };
    
    struct NoteSpec {
        Note note : 4;
        bool loud : 1;
        unsigned int duration : 7; // max. 128
    };
        
    template<int nAudioOut, Atomicity A, typename Request = Request<A, nAudioOut>>
    Request to_request(NoteSpec s,
                       float time_unit,
                       float harmonic_factor,
                       Midi const & midi,
                       Sounds<A> & sounds,
                       Volumes<nAudioOut> volumes) {
        if(s.note == Silence) {
            return {
                sounds,
                Sound::SILENCE,
                Sound::ConstantSoundDummyFrequency,
                0.f,
                time_unit * (float)s.duration
            };
        }
        else {
            return {
                sounds,
                Sound::SINE,
                harmonic_factor * static_cast<float>(midi.Ainterval_to_freq(static_cast<int>(s.note) -
                                                                            static_cast<int>(La))),
                volumes * (s.loud ? 2.f : 1.f),
                time_unit * (float)s.duration
            };
        }
    }
}
