namespace imajuscule {

template<typename T>
struct CountEnumValues;

} // NS

namespace imajuscule::audio {

enum class Note {
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
}

namespace imajuscule {
template<>
struct CountEnumValues<audio::Note> {
  static int constexpr count = 12;
};
}

namespace imajuscule::audio {
inline std::ostream &
operator << (std::ostream & os, Note const n) {
  switch(n) {
    case Note::Do:
      os << "C";
      break;
    case Note::Dod:
      os << "C#";
      break;
    case Note::Re:
      os << "D";
      break;
    case Note::Red:
      os << "D#";
      break;
    case Note::Mi:
      os << "E";
      break;
    case Note::Fa:
      os << "F";
      break;
    case Note::Fad:
      os << "F#";
      break;
    case Note::Sol:
      os << "G";
      break;
    case Note::Sold:
      os << "G#";
      break;
    case Note::La:
      os << "A";
      break;
    case Note::Lad:
      os << "A#";
      break;
    case Note::Si:
      os << "B";
      break;
  }
  return os;
}

inline int half_tones_distance(Note const & a,
                               Note const & b) {
  return
  static_cast<int>(to_underlying(b)) -
  static_cast<int>(to_underlying(a));
}

struct NoteSpec {
  std::optional<Note> note; // no value means silence
  bool loud;
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
  if(!s.note) {
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
      harmonic_factor * static_cast<float>(midi.Ainterval_to_freq(static_cast<int>(to_underlying(*s.note)) -
                                                                  static_cast<int>(to_underlying(Note::La)))),
      volumes * (s.loud ? 2.f : 1.f),
      time_unit * (float)s.duration,
      sample_rate
    };
  }
}
}
