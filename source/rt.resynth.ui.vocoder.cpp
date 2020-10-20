
namespace imajuscule::audio::rtresynth {

template<typename DC>
void drawRectangle(DC & dc,
                   wxCoord x1,
                   wxCoord y1,
                   wxCoord x2,
                   wxCoord y2) {
  dc.DrawRectangle(x1,
                   y1,
                   x2-x1, // TODO is it ok to have negative height / width ?
                   y2-y1);
}

struct VocoderWindow
: public wxWindow {
  
  static wxSize getSize(int x, int y, Orientation o) {
    switch(o) {
      case Orientation::Vertical:
        return {x, y};
      case Orientation::Horizontal:
        return {y, x};
    }
  }

  using VocoderFunc = std::function<void(std::vector<double> &, std::vector<double> &)>;
  VocoderWindow(wxWindow * parent,
                Orientation o,
                VocoderFunc const & f)
  : wxWindow(parent,
             wxID_ANY,
             wxDefaultPosition,
             getSize(200, 200, o))
  , vocoder_func(f)
  , orientation(o)
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
    
    this->Connect( wxEVT_PAINT, wxPaintEventHandler( VocoderWindow::OnPaint ), NULL, this );
    envelopes.reserve(200);
  }
  
  bool TryFetchNewFrame() {
    vocoder_func(new_envelopes,
                 new_frequencies);
    bool changed = false;
    if (new_envelopes != envelopes) {
      envelopes.swap(new_envelopes);
      changed = true;
    }
    if (new_frequencies != frequencies) {
      frequencies.swap(new_frequencies);
      changed = true;
    }
    return changed;
  }

private:
  VocoderFunc vocoder_func;
  Orientation orientation;
  std::vector<double> envelopes;
  std::vector<double> frequencies;
  std::vector<double> new_envelopes;
  std::vector<double> new_frequencies;

  const wxColor color_envelope{255, 255, 255};
  const wxColor color_log_envelope{255, 0, 0};
  const wxColor color_freq{100, 100, 100};

  static constexpr double vol_min = 0.00001;
  static constexpr double log_vol_min = sprout::log(vol_min);
  static constexpr double inv_log_vol_min = 1. / log_vol_min;

  static constexpr int margin = 1;
  static constexpr int x_start_frequencies = margin;
  static constexpr int sz_frequencies = 15;
  static constexpr int x_start_envelopes = x_start_frequencies + sz_frequencies + margin;

  static constexpr int margin_bands = 20;

  void OnPaint(wxPaintEvent& event) {
    wxAutoBufferedPaintDC dc(this);

    //wxFont font(75, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false);
    //dc.SetFont(font);
    dc.SetTextForeground(*wxWHITE);

    // 0,0 is top left corner.
    
    wxSize const sz = GetClientSize();
    dc.SetBrush(*wxBLACK_BRUSH);
    dc.DrawRectangle(sz);

    int const count_bands = static_cast<int>(envelopes.size());
    int const height_bands = (orientation == Orientation::Vertical) ? sz.y : sz.x;
    // height_bands is:
    //
    // .........///////////////////////////////////////////////..........
    // margin   actual bands                                   margin
    int const height_actual_bands = height_bands - 2 * margin_bands;

    auto envelope_idx_to_heights = [this,
                                    count_bands,
                                    height_actual_bands]
    (int idx) -> std::pair<int, int> {
      
      float const ratio_min = static_cast<float>(idx) / count_bands;
      float const ratio_max = static_cast<float>(idx+1) / count_bands;
      if(orientation == Orientation::Vertical) {
        return {
          margin_bands + static_cast<int>(0.5f + (1.f - ratio_min) * height_actual_bands),
          margin_bands + static_cast<int>(0.5f + (1.f - ratio_max) * height_actual_bands)
        };
      } else {
        return {
          margin_bands + static_cast<int>(0.5f + ratio_min * height_actual_bands),
          margin_bands + static_cast<int>(0.5f + ratio_max * height_actual_bands)
        };
      }
    };
    
    int const width_envelopes = ((orientation == Orientation::Vertical) ? sz.x : sz.y) - x_start_envelopes;
    auto draw_volume = [&dc,
                        this,
                        width_envelopes]
    (float const velocity, std::pair<int, int> const height_bounds) {
      wxCoord const w = static_cast<int>(std::max(1.f,
                                                  velocity * (width_envelopes-1)));
      if(orientation == Orientation::Vertical) {
        drawRectangle(dc,
                      x_start_envelopes,
                      height_bounds.first,
                      x_start_envelopes + w,
                      height_bounds.second);
      } else {
        drawRectangle(dc,
                      height_bounds.first,
                      width_envelopes,
                      height_bounds.second,
                      width_envelopes - w);
      }
    };
    
    Assert(frequencies.size() == (envelopes.size() + 1));

    auto write_freq = [&dc,
                       this,
                       width_envelopes]
    (float freq,
     int const y) {
      std::ostringstream s;
      s << static_cast<int>(freq + 0.5);
      wxString const text = s.str();
      wxCoord text_w, text_h;
      dc.GetTextExtent(text, &text_w, &text_h);
      if(orientation == Orientation::Vertical) {
        dc.DrawText(text,
                    x_start_frequencies,
                    y - text_h/2);
      } else {
        dc.DrawText(text,
                    y - text_w/2,
                    width_envelopes - x_start_frequencies);
      }
    };
    
    dc.SetBrush(*wxTRANSPARENT_BRUSH); // so that envelope rectangles do not hide each other

    double max_velocity = 0.;
    for (auto const & velocity : envelopes) {
      max_velocity = std::max(max_velocity, velocity);
    }

    int idx = -1;
    for (auto const & velocity : envelopes) {
      ++idx;
      std::pair<int, int> const height_bounds = envelope_idx_to_heights(idx);
      double const logVelocity = (log_vol_min - std::log(velocity)) * inv_log_vol_min;

      dc.SetPen(color_log_envelope);

      draw_volume(logVelocity, height_bounds);

      dc.SetPen(color_envelope);

      draw_volume(max_velocity ? (velocity / max_velocity) : velocity, height_bounds);

      dc.SetPen(color_freq);

      write_freq(frequencies[idx],
                 height_bounds.first);
      if (idx+1 == static_cast<int>(envelopes.size())) {
        write_freq(frequencies[idx+1],
                   height_bounds.second);
      }
    }
  }
};

} // NS
