namespace imajuscule::audio::rtresynth {

struct Autotune {
  EnumeratedParamProxy<AutotuneType> type;
  ParamProxy<int> max_pitch;
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
    wxBoxSizer * sizer_horiz2 = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBoxSizer * sizer_root = new wxStaticBoxSizer(wxVERTICAL,
                                                         parent,
                                                         "Root note");
    wxStaticBoxSizer * sizer_chord = new wxStaticBoxSizer(wxVERTICAL,
                                                          parent,
                                                          "Chord");
    sizer_vert->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_vert->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    sizer_chord->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_chord->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    sizer_root->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_root->GetStaticBox()->SetForegroundColour(color_slider_label_2);

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

    wxSizer * max_pitch = createIntSlider(parent,
                                          a.max_pitch,
                                          color_slider_label_2);
    
    Add(chord,
        sizer_chord,
        0,
        wxALL | wxALIGN_CENTER);
    Add(chord_freqs,
        sizer_chord,
        0,
        wxALL | wxALIGN_LEFT);

    Add(scale_type,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(intervals,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(sizer_chord,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);

    Add(root_note,
        sizer_root,
        0,
        wxALL | wxALIGN_CENTER);
    Add(root_note_transpose,
        sizer_root,
        0,
        wxALL | wxALIGN_CENTER);

    Add(sizer_root,
        sizer_horiz2,
        0,
        wxALL | wxALIGN_CENTER);
    Add(max_pitch,
        sizer_horiz2,
        0,
        wxALL | wxALIGN_CENTER);

    Add(sizer_horiz2,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);
    Add(sizer_horiz,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);

    wxSizer * type = createChoice(parent,
                                  a.type,
                                  color_slider_label_2,
                                  ChoiceType::RadioBoxV,
                                  [intervals, scale_type, sizer_chord, sizer_horiz2](AutotuneType const t){
      forEachWindow(intervals,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::FixedSizeIntervals); });
      forEachWindow(scale_type,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::MusicalScale); });
      forEachWindow(sizer_chord,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::Chord); });
      forEachWindow(sizer_horiz2,
                    [t](wxWindow & w) { w.Enable(t != AutotuneType::None); });
    });
    
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
