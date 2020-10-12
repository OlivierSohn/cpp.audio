namespace imajuscule::audio::rtresynth {

struct Autotune {
  EnumeratedParamProxy<AutotuneType> type;
  EnumeratedParamProxy<AutotuneChordFrequencies> chord_frequencies;
  EnumeratedCombinationParamProxy<Note> chord;
  EnumeratedParamProxy<MusicalScaleMode> scale_type;
  EnumeratedParamProxy<Note> root_note;
  ParamProxy<int> root_note_transpose;
  ParamProxy<int> intervals_size;
};

wxSizer * mkAutotuneSizer(wxWindow * parent,
                          Autotune const & a) {
  wxStaticBoxSizer * global_sizer = new wxStaticBoxSizer(wxHORIZONTAL,
                                                         parent,
                                                         "");
  global_sizer->GetStaticBox()->SetBackgroundColour(autotune_bg_color1);

  {
    wxStaticBoxSizer * sizer_vert = new wxStaticBoxSizer(wxVERTICAL,
                                                         parent,
                                                         "");
    wxBoxSizer * sizer_horiz = new wxBoxSizer(wxHORIZONTAL);
    sizer_vert->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_vert->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    
    wxSizer * scale_type = createChoice(parent,
                                        a.scale_type,
                                        color_slider_label_2,
                                        ChoiceType::RadioBoxH);
    wxSizer * chord_freqs = createChoice(parent,
                                         a.chord_frequencies,
                                         color_slider_label_2,
                                         ChoiceType::RadioBoxH);
    
    wxSizer * chord = createCombination(parent,
                                        a.chord,
                                        color_slider_label_2,
                                        CombinationType::CheckBoxH);
    
    wxSizer * intervals = createIntSlider(parent,
                                          a.intervals_size,
                                          color_slider_label_2);
    
    wxSizer * root_note = createChoice(parent,
                                       a.root_note,
                                       color_slider_label_2,
                                       ChoiceType::RadioBoxH);
    wxSizer * root_note_transpose = createIntSlider(parent,
                                                    a.root_note_transpose,
                                                    color_slider_label_2);
    wxSizer * type = createChoice(parent,
                                  a.type,
                                  color_slider_label_2,
                                  ChoiceType::RadioBoxV,
                                  [intervals, scale_type, chord_freqs, chord, root_note, root_note_transpose](AutotuneType const t){
      forEachWindow(intervals,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::FixedSizeIntervals); });
      forEachWindow(scale_type,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::MusicalScale); });
      forEachWindow(chord_freqs,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::Chord); });
      forEachWindow(chord,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::Chord); });
      forEachWindow(root_note,
                    [t](wxWindow & w) { w.Enable(t != AutotuneType::None); });
      forEachWindow(root_note_transpose,
                    [t](wxWindow & w) { w.Enable(t != AutotuneType::None); });
    });

    Add(scale_type,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(intervals,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(chord,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(chord_freqs,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    
    Add(root_note,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);
    Add(root_note_transpose,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);
    Add(sizer_horiz,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);

    Add(type,
        global_sizer,
        0,
        wxALL);
    Add(sizer_vert,
        global_sizer,
        1,
        wxALL | wxALIGN_CENTER);
  }
  return global_sizer;
}

} // NS
