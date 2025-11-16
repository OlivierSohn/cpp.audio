namespace imajuscule::audio::rtresynth {
inline void testAutotune() {

  {
    NoteOctave const note{Note::Fad, 4};
    Assert((note.add_halftones(1) == NoteOctave{Note::Sol, 4}));
    Assert((note.add_halftones(13) == NoteOctave{Note::Sol, 5}));
    Assert((note.add_halftones(-11) == NoteOctave{Note::Sol, 3}));
    
    Assert((note.dist_halftones(NoteOctave{Note::Sol, 4}) == 1));
    Assert((note.dist_halftones(NoteOctave{Note::Sol, 5}) == 13));
    Assert((note.dist_halftones(NoteOctave{Note::Sol, 3}) == -11));
  }

  {
    MusicalScalePitches const & msp = getMusicalScale(MusicalScaleMode::Major);
    Midi const midi;

    NoteOctave const note{Note::Fad, 4};
    MidiPitch const pitch = midi.get_pitch(note);
    
    MidiPitch const expected_autotune_pitch = midi.get_pitch({Note::Sol, 4});

    MidiPitch const autotuned_pitch =
    msp.closest_pitch(midi.get_pitch({Note::Do, 4}),
                      pitch);
    Assert(std::abs(expected_autotune_pitch - autotuned_pitch) < 0.0001);
  }
}
}
