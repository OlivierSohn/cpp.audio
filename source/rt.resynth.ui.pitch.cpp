
namespace imajuscule::audio::rtresynth {

constexpr double func_inv_log_1() {
  constexpr double r = sprout::log(1.);
  return 1. / r;
}

struct PitchWindow
: public wxWindow {
  using PitchFunction = std::function<void(std::vector<PlayedNote> &,
                                           std::vector<PlayedNote> &,
                                           std::optional<int64_t> &)>;

  PitchWindow(wxWindow * parent,
              PitchFunction const & f)
  : wxWindow(parent,
            wxID_ANY,
            wxDefaultPosition,
            wxSize(200,600))
  , pitch_func(f)
  {
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    // The line above is to fix this assert:
    /*
    wxAutoBufferedPaintDC(wxWindow* win)
    : wxAutoBufferedPaintDCBase(win)
    {
      wxASSERT_MSG( win->GetBackgroundStyle() == wxBG_STYLE_PAINT,
                   "You need to call SetBackgroundStyle(wxBG_STYLE_PAINT) in ctor, "
                   "and also, if needed, paint the background in wxEVT_PAINT handler."
                   );
    }
    */
    
    this->Connect( wxEVT_PAINT, wxPaintEventHandler( PitchWindow::OnPaint ), NULL, this );
    played_notes.reserve(200);
    dropped_notes.reserve(400);
  }
  
  // returns true if a new frame was fetched
  bool TryFetchNewFrame() {
    std::optional<int64_t> const prev_frame_id = frame_id;
    pitch_func(played_notes, dropped_notes, frame_id);
    if (prev_frame_id && frame_id) {
      if ((*frame_id - *prev_frame_id) > 1) {
        /*LG(INFO,
           "skipped display of %d analysis frames @ %d",
           (*frame_id - *prev_frame_id) - 1,
           *prev_frame_id);
        */
      }
    }
    return frame_id != prev_frame_id;
  }
private:
  PitchFunction pitch_func;
  std::vector<PlayedNote> played_notes, dropped_notes;
  std::optional<int64_t> frame_id;
  
  const wxColor color_pitch_note_change{51, 204, 204};
  const wxColor color_pitch_note_on{255, 255, 255};
  const wxColor color_pitch_note_on_dropped{255, 0, 0};
  
  static constexpr float max_pitch = 163; // 100000 Hz
  static constexpr float min_pitch = 0;   //      8 Hz
  static constexpr double vol_min = 0.00001;
  static constexpr double log_vol_min = sprout::log(vol_min);
  static constexpr double inv_log_vol_min = 1. / log_vol_min;

  static constexpr int margin = 1;

  static constexpr int x_start_pitches = margin;
  
  void OnPaint(wxPaintEvent& event) {
    wxAutoBufferedPaintDC dc(this);

    // 0,0 is top left corner.
    
    wxSize const sz = GetClientSize();
    dc.SetBrush(*wxBLACK_BRUSH);
    dc.DrawRectangle(sz);

    auto pitch_to_height = [height_pitches = sz.y]
    (float pitch) -> float {
      float ratio = (pitch - min_pitch) / (max_pitch - min_pitch);
      return (1.f - ratio) * height_pitches;
    };
    
    auto draw_volume = [&dc,
                        width_pitches = sz.x - x_start_pitches]
    (float const velocity, int const y) {
      double logVelocity = (log_vol_min - std::log(velocity)) * inv_log_vol_min;
      wxCoord const w = static_cast<int>(std::max(1., logVelocity * width_pitches));
      dc.DrawLine(x_start_pitches,
                  y,
                  x_start_pitches + w,
                  y);
    };
    
    
    dc.SetPen(color_pitch_note_on_dropped);
    
    for (auto const & n : dropped_notes) {
      int const y = pitch_to_height(n.midi_pitch);
      draw_volume(n.cur_velocity, y);
    }
    
    dc.SetPen(color_pitch_note_change);
    
    for (auto const & n : played_notes) {

      Assert(frame_id);
      if (n.note_on_frame_id == *frame_id) {
        dc.SetPen(color_pitch_note_on);
      }
      int const y = pitch_to_height(n.midi_pitch);
      draw_volume(n.cur_velocity, y);
      
      if (n.note_on_frame_id == *frame_id) {
        dc.SetPen(color_pitch_note_change);
      }
    }

  }
};

} // NS
