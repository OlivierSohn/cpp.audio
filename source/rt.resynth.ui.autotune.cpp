namespace imajuscule::audio::rtresynth {

struct Autotune {
  EnumeratedParamProxy<AutotuneType> type;
  EnumeratedParamProxy<MusicalScaleType> scale_type;
  EnumeratedParamProxy<Note> scale_root;
  ParamProxy<int> intervals_size;
};

wxSizer * mkAutotuneSizer(wxWindow * parent,
                          Autotune const & a) {
  wxSizer * sizer = new wxBoxSizer(wxHORIZONTAL);
  
  wxSizer * intervals = createIntSlider(parent,
                                        a.intervals_size,
                                        color_slider_label_2);

  wxSizer * scale_type = createChoice(parent,
                                      a.scale_type,
                                      color_slider_label_2);
  wxSizer * scale_root = createChoice(parent,
                                      a.scale_root,
                                      color_slider_label_2);

  wxSizer * type = createChoice(parent,
                                a.type,
                                color_slider_label_2,
                                [intervals, scale_type, scale_root](AutotuneType const t){
    forEachWindow(intervals,
                  [enabled = (t == AutotuneType::FixedSizeIntervals)](wxWindow & w) { w.Enable(enabled); });
    forEachWindow(scale_type,
                  [enabled = (t == AutotuneType::MusicalScale)](wxWindow & w) { w.Enable(enabled); });
    forEachWindow(scale_root,
                  [enabled = (t == AutotuneType::MusicalScale)](wxWindow & w) { w.Enable(enabled); });
  });
  
  Add(type,
      sizer);
  Add(intervals,
      sizer);
  Add(scale_type,
      sizer);
  Add(scale_root,
      sizer);
  return sizer;
}

} // NS
