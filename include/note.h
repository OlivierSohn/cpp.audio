
namespace imajuscule::audio {

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
Request to_request(int sample_rate,
                   NoteSpec s,
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
      time_unit * (float)s.duration,
      sample_rate
    };
  }
  else {
    return {
      sounds,
      Sound::SINE,
      harmonic_factor * static_cast<float>(midi.Ainterval_to_freq(static_cast<int>(s.note) -
                                                                  static_cast<int>(La))),
      volumes * (s.loud ? 2.f : 1.f),
      time_unit * (float)s.duration,
      sample_rate
    };
  }
}
}
