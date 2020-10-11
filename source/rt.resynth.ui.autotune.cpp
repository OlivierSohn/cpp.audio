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
    wxStaticBoxSizer * sizer = new wxStaticBoxSizer(wxHORIZONTAL,
                                                    parent,
                                                    "");
    wxStaticBoxSizer * sizer_scale = new wxStaticBoxSizer(wxVERTICAL,
                                                          parent,
                                                          "Scale");
    wxStaticBoxSizer * sizer_interval = new wxStaticBoxSizer(wxVERTICAL,
                                                             parent,
                                                             "Intervals");
    sizer->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_scale->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer_interval->GetStaticBox()->SetBackgroundColour(autotune_bg_color2);
    sizer->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    sizer_scale->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    sizer_interval->GetStaticBox()->SetForegroundColour(color_slider_label_2);
    {
      {
        
        wxSizer * scale_type = createChoice(parent,
                                            a.scale_type,
                                            color_slider_label_2,
                                            ChoiceType::RadioBoxH);
        wxSizer * scale_root = createChoice(parent,
                                            a.scale_root,
                                            color_slider_label_2,
                                            ChoiceType::RadioBoxH);
        Add(scale_type,
            sizer_scale,
            0,
            wxALL | wxALIGN_LEFT);
        Add(scale_root,
            sizer_scale,
            0,
            wxALL | wxALIGN_LEFT);
      }
      {
        
        wxSizer * intervals = createIntSlider(parent,
                                              a.intervals_size,
                                              color_slider_label_2);
        Add(intervals,
            sizer_interval,
            0,
            0);
      }
      
      Add(sizer_interval,
          sizer,
          0,
          wxALL | wxALIGN_TOP);
      Add(sizer_scale,
          sizer,
          0,
          wxALL | wxALIGN_TOP);
    }
    wxSizer * type = createChoice(parent,
                                  a.type,
                                  color_slider_label_2,
                                  ChoiceType::RadioBoxV,
                                  [sizer_interval, sizer_scale](AutotuneType const t){
      forEachWindow(sizer_interval,
                    [enabled = (t == AutotuneType::FixedSizeIntervals)](wxWindow & w) { w.Enable(enabled); });
      forEachWindow(sizer_scale,
                    [enabled = (t == AutotuneType::MusicalScale)](wxWindow & w) { w.Enable(enabled); });
    });
    
    Add(type,
        global_sizer,
        0,
        wxALL);
    Add(sizer,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);
  }
  return global_sizer;
}

} // NS
