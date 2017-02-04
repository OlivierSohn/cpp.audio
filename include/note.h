
namespace imajuscule {
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
   
    static inline float compute_half_tone(float stretch) {
        return powf(2.f, stretch/12.f);
    }
    
    constexpr float freq_do = 261.64f; // Midi C6(60) according to http://subsynth.sourceforge.net/midinote2freq.html
    constexpr uint8_t Do_midi = 60;

    static inline float to_freq(float interval_from_C6, float half_tone) {
        return freq_do * powf(half_tone, interval_from_C6);
    }
    
    static inline float to_freq(Note n, float half_tone) {
        auto diff = static_cast<int>(n) - static_cast<int>(Do);
        return to_freq(static_cast<float>(diff), half_tone);
    }
    
    
    static inline float transpose_frequency(float freq, float half_tone, int n) {
        return freq * powf(half_tone, n);
    }
    
    template<int nAudioOut, typename Request = Request<nAudioOut>>
    Request to_request(NoteSpec s, float time_unit, float harmonic_factor, float half_tone, Sounds & sounds, Volumes<nAudioOut> volumes) {
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
                harmonic_factor * to_freq(s.note, half_tone),
                volumes * (s.loud ? 2.f : 1.f),
                time_unit * (float)s.duration
            };
        }
    }
}
