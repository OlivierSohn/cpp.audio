namespace imajuscule::audio::rtresynth {

struct Autotune {
  ParamProxy<bool> use;
  EnumeratedParamProxy<AutotuneType> type;
  ParamProxy<int> max_pitch;
  ParamProxy<float> pitch_tolerance;
  EnumeratedParamProxy<AutotuneChordFrequencies> chord_frequencies;
  EnumeratedCombinationParamProxy<Note> chord;
  EnumeratedParamProxy<MusicalScaleMode> scale_type;
  EnumeratedParamProxy<Note> root_note;
  ParamProxy<int> root_note_transpose;
  ParamProxy<int> intervals_size;
};

wxSizer * mkAutotuneSizer(wxWindow * parent,
                          Autotune const & a,
                          std::vector<UpdateFunc> & update_param) {
  wxStaticBoxSizer * global_sizer = new wxStaticBoxSizer(wxVERTICAL,
                                                         parent,
                                                         "");
  global_sizer->GetStaticBox()->SetBackgroundColour(autotune_bg_color1);

  {
    wxStaticBoxSizer * sizer_vert = new wxStaticBoxSizer(wxVERTICAL,
                                                         parent,
                                                         "");
    
    wxBoxSizer * sizer_config = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer * sizer_horiz = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer * sizer_horiz2 = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBoxSizer * sizer_root = new wxStaticBoxSizer(wxVERTICAL,
                                                         parent,
                                                         "Root note");
    wxStaticBoxSizer * sizer_chord = new wxStaticBoxSizer(wxVERTICAL,
                                                          parent,
                                                          "Chord");
    auto filtering_sizer = new wxStaticBoxSizer(wxVERTICAL,
                                                parent,
                                                "Filtering");
    sizer_vert->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_vert->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    sizer_chord->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_chord->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    
    sizer_root->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_root->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    filtering_sizer->GetStaticBox()->SetBackgroundColour(autotune_bg_color3);
    filtering_sizer->GetStaticBox()->SetForegroundColour(color_slider_label_2);

        
    wxSizer * scale_type = createChoice(parent,
                                        a.scale_type,
                                        color_slider_label_2,
                                        ChoiceType::RadioBoxV,
                                        &update_param);
    wxSizer * chord_freqs = createChoice(parent,
                                         a.chord_frequencies,
                                         color_slider_label_2,
                                         ChoiceType::RadioBoxH,
                                         &update_param);
    
    wxSizer * chord = createCombination(parent,
                                        a.chord,
                                        color_slider_label_2,
                                        CombinationType::CheckBoxH,
                                        update_param);
    
    wxSizer * intervals = createIntSlider(parent,
                                          a.intervals_size,
                                          color_slider_label_2,
                                          update_param);
    
    wxSizer * root_note = createChoice(parent,
                                       a.root_note,
                                       color_slider_label_2,
                                       ChoiceType::RadioBoxH,
                                       &update_param);
    wxSizer * root_note_transpose = createIntSlider(parent,
                                                    a.root_note_transpose,
                                                    color_slider_label_2,
                                                    update_param);

    wxSizer * max_pitch = createIntSlider(parent,
                                          a.max_pitch,
                                          color_slider_label_2,
                                          update_param);
    wxSizer * pitch_tolerance = createFloatSlider(parent,
                                                  a.pitch_tolerance,
                                                  color_slider_label_2,
                                                  update_param);

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
    Add(sizer_chord,
        sizer_horiz,
        0,
        wxALL | wxALIGN_CENTER);
    Add(intervals,
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

    Add(max_pitch,
        filtering_sizer,
        0,
        wxALL | wxALIGN_CENTER);
    Add(pitch_tolerance,
        filtering_sizer,
        0,
        wxALL | wxALIGN_CENTER);
    
    Add(sizer_root,
        sizer_horiz2,
        0,
        wxALL | wxALIGN_CENTER);
    Add(filtering_sizer,
        sizer_horiz2,
        0,
        wxALL | wxALIGN_CENTER);

    Add(sizer_horiz,
        sizer_vert,
        0,
        wxALL | wxALIGN_CENTER);

    auto update =
    [&a, sizer_horiz2, intervals, scale_type, sizer_chord]
    (wxSizer * type = nullptr) {
      bool const used = a.use.get();
      forEachWindow(sizer_horiz2,
                    [used](wxWindow & w) { w.Enable(used); });
      
      AutotuneType const t = a.type.get();
      forEachWindow(intervals,
                    [used, t](wxWindow & w) { w.Enable(used && t == AutotuneType::FixedSizeIntervals); });
      forEachWindow(scale_type,
                    [used, t](wxWindow & w) { w.Enable(used && t == AutotuneType::MusicalScale); });
      forEachWindow(sizer_chord,
                    [used, t](wxWindow & w) { w.Enable(used && t == AutotuneType::Chord); });
      if (type) {
        forEachWindow(type,
                      [used](wxWindow & w) { w.Enable(used); });
      }
    };
    
    wxSizer * type = createChoice(parent,
                                  a.type,
                                  color_slider_label_2,
                                  ChoiceType::RadioBoxH,
                                  &update_param,
                                  update);

    wxCheckBox * use = createCheckBox(parent,
                                      a.use,
                                      color_slider_label_2,
                                      update_param,
                                      [update, type](){ update(type); });

    update(type);
    
    Add(use,
        sizer_config,
        0,
        wxALL | wxALIGN_CENTER);
    Add(type,
        sizer_config,
        0,
        wxALL | wxALIGN_CENTER);
    
    Add(sizer_config,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);
    Add(sizer_vert,
        global_sizer,
        1,
        wxALL | wxALIGN_CENTER);
    Add(sizer_horiz2,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);
  }
  return global_sizer;
}

} // NS
