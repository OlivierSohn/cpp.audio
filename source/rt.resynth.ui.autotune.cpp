namespace imajuscule::audio::rtresynth {

struct Autotune {
  EnumeratedParamProxy<AutotuneType> type;
  EnumeratedParamProxy<MusicalScaleMode> scale_type;
  EnumeratedParamProxy<Note> scale_root;
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
    
    wxSizer * intervals = createIntSlider(parent,
                                          a.intervals_size,
                                          color_slider_label_2);
    
    wxSizer * scale_root = createChoice(parent,
                                        a.scale_root,
                                        color_slider_label_2,
                                        ChoiceType::RadioBoxH);
    wxSizer * type = createChoice(parent,
                                  a.type,
                                  color_slider_label_2,
                                  ChoiceType::RadioBoxV,
                                  [intervals, scale_type, scale_root](AutotuneType const t){
      forEachWindow(intervals,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::FixedSizeIntervals); });
      forEachWindow(scale_type,
                    [t](wxWindow & w) { w.Enable(t == AutotuneType::MusicalScale); });
      forEachWindow(scale_root,
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
    
    Add(scale_root,
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
