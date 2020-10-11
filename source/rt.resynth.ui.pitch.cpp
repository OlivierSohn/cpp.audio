
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
    if (frame_id == prev_frame_id) {
      return false;
    }
    prev_frame_id_ = prev_frame_id;
    return true;
  }

private:
  PitchFunction pitch_func;
  std::vector<PlayedNote> played_notes, dropped_notes;
  std::optional<int64_t> frame_id, prev_frame_id_;

  std::optional<int64_t> countDroppedFrames() const {
    if (frame_id && prev_frame_id_) {
      int const dropped = (*frame_id - *prev_frame_id_) - 1;
      if (dropped >= 1) {
        return {dropped};
      }
    }
    return {};
  }
  
  const wxColor color_pitch_note_change{51, 204, 204};
  const wxColor color_pitch_note_on{255, 255, 255};
  const wxColor color_pitch_note_on_dropped{255, 0, 0};
  
  static constexpr float max_pitch = 163; // 100000 Hz
  static constexpr float min_pitch = 0;   //      8 Hz
  static constexpr double vol_min = 0.00001;
  static constexpr double log_vol_min = sprout::log(vol_min);
  static constexpr double inv_log_vol_min = 1. / log_vol_min;

  static constexpr int margin = 1;
  static constexpr int x_start_note_names = margin;
  static constexpr int sz_note_names = 20;
  static constexpr int x_start_pitches = x_start_note_names + sz_note_names + margin;
  
  void OnPaint(wxPaintEvent& event) {
    wxAutoBufferedPaintDC dc(this);

    //wxFont font(75, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false);
    //dc.SetFont(font);
    dc.SetTextForeground(*wxWHITE);

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
    
    auto f = [](PlayedNote const & n) {
      if (!n.midi_pitch) {
        return 0.;
      }
      return n.cur_velocity / (n.midi_pitch * n.midi_pitch);
    };
    
    std::optional<PlayedNote> max_volume;
    for (auto const & n : played_notes) {
      if (!max_volume || f(*max_volume) < f(n)) {
        max_volume = n;
      }

      Assert(frame_id);
      if (n.note_on_frame_id == *frame_id) {
        dc.SetPen(color_pitch_note_on);
      }
      int const y = pitch_to_height(n.midi_pitch);
      draw_volume(n.cur_velocity, y);
      auto [note, deviation] = midi_pitch_to_note_deviation(n.midi_pitch);
      if (std::abs(deviation) < 0.00001) {
        std::ostringstream s;
        s << note.note;
        s << note.octave;
        dc.DrawText(s.str(),
                    x_start_note_names,
                    y);
      }
      
      if (n.note_on_frame_id == *frame_id) {
        dc.SetPen(color_pitch_note_change);
      }
    }
    
    auto const dropped = countDroppedFrames();
    
    auto str = "dropped display frames : " + std::to_string(dropped ? *dropped : 0);
    dc.DrawText(str, 20., 20.);

    //auto str2 = "max volume at pitch : " + (max_volume ? std::to_string(max_volume->midi_pitch) : "?");
    //dc.DrawText(str2, 20., 50.);

  }
};

} // NS
