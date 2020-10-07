
#include <string>

namespace imajuscule::audio::rtresynth {

inline std::string float_to_string(float v) {
  std::string s = std::to_string(v);
  if (s.find('.') != std::string::npos) {
    while(s.back() == '0') {
      s.pop_back();
    }
    if (s.back() == '.') {
      s.push_back('0');
    }
  }
  return s;
}

struct Param {
  std::string name, unit;
  std::function<float()> get;
  std::function<void(float)> set;
  float min, max;
};

struct PollParam {
  using OptionalVariant = std::optional<std::variant<int, float>>;
  std::string name;
  std::function<OptionalVariant()> get;
};

template<typename T, typename U>
void Add(T * widget,
         U * sizer,
         int proportion = 1,
         int flag = wxALL | wxEXPAND, // cannot mix wxEXPAND with wxALIGN***
         int border = 5) {
  sizer->Add(widget,
             proportion,
             flag,
             border);
}

enum class TimerId {
  RefreshUI
};

class MyFrame : public wxFrame
{
public:
  MyFrame(std::vector<Param> const & non_realtime_params,
          std::vector<Param> const & realtime_params,
          std::vector<std::vector<PollParam>> const & poll_params)
  : wxFrame(NULL,
            wxID_ANY,
            "Resynth",
            wxDefaultPosition,
            wxDefaultSize,
            wxSTAY_ON_TOP |
            wxDEFAULT_FRAME_STYLE |
            wxFULL_REPAINT_ON_RESIZE)
  , uiTimer(this,
            static_cast<int>(TimerId::RefreshUI))
  {
    this->Connect( wxEVT_TIMER, wxTimerEventHandler( MyFrame::OnUITimer ), NULL, this );
    
    auto menuFile = new wxMenu();

    auto const helloId = wxWindow::NewControlId();
    menuFile->Append(helloId, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);

    auto menuHelp = new wxMenu();

    menuHelp->Append(wxID_ABOUT);

    auto menuBar = new wxMenuBar();

    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    SetMenuBar(menuBar);


    Bind(wxEVT_MENU, &MyFrame::OnHello, this, helloId);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
    
    auto global_sliders_sizer = new wxBoxSizer(wxVERTICAL);
    {
      auto sliders_sizer = new wxBoxSizer(wxHORIZONTAL);
      wxColor label_color(100,
                          150,
                          200);

      for (auto const & param : non_realtime_params) {
        Add(createFloatSlider(param,
                              label_color),
            sliders_sizer);
      }
      Add(sliders_sizer,
          global_sliders_sizer);
    }
    {
      auto sliders_sizer = new wxBoxSizer(wxHORIZONTAL);
      wxColor label_color(150,
                          100,
                          200);
      for (auto const & param : realtime_params) {
        Add(createFloatSlider(param,
                              label_color),
            sliders_sizer);
      }
      Add(sliders_sizer,
          global_sliders_sizer);
    }
    {
      auto rt_sizer = new wxBoxSizer(wxVERTICAL);
      wxColor label_color(100,
                          100,
                          250);
      
      for (auto const & params: poll_params) {
        auto rt2_sizer = new wxBoxSizer(wxHORIZONTAL);
        for (auto const & param:params) {
          auto [sizer, value] = createPollParamUI(param,
                                                  label_color);
          poll_params_ui.emplace_back(param, value);
          Add(sizer,
              rt2_sizer,
              0,
              wxALL | wxALIGN_CENTER);
        }
        Add(rt2_sizer,
            rt_sizer,
            0,
            wxALL);
      }
      Add(rt_sizer,
          global_sliders_sizer);
    }
    SetSizerAndFit(global_sliders_sizer);

    uiTimer.Start(100);
  }
  
  ~MyFrame() {
    if (uiTimer.IsRunning()) {
      uiTimer.Stop();
    }
  }

private:
  wxTimer uiTimer;
  std::vector<std::pair<PollParam, wxStaticText*>> poll_params_ui;
  
  void OnExit(wxCommandEvent& event)
  {
    Close(true);
  }
  void OnAbout(wxCommandEvent& event)
  {
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
  }
  void OnHello(wxCommandEvent& event)
  {
    wxLogMessage("Hello world from wxWidgets!");
  }
  void OnUITimer(wxTimerEvent &) {
    for (auto & [param, ui] : poll_params_ui) {
      std::optional<std::variant<int, float>> const value = param.get();
      std::string label;
      if (value) {
        label = std::visit([&label](auto && v) {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_floating_point_v<T>) {
            return float_to_string(v);
          } else {
            return std::to_string(v);
          }
        }, *value);
      }
      ui->SetLabel(label);
    }
  }
  
  const wxColor dark_grey{
    100,
    100,
    100
  };
  const wxColor value_unchanged_color{
    220,
    220,
    220
  };
  const wxColor value_incorrect_color{
    *wxRED
  };

  wxBoxSizer * createFloatSlider(Param const & param,
                                 wxColor const & label_color) {
    std::string const & name = param.name;
    std::string const & unit = param.unit;
    float const min_float_value = param.min;
    float const max_float_value = param.max;
    
    auto const sliderId = wxWindow::NewControlId();

    float const current_float_value = param.get();
    Assert(current_float_value <= max_float_value);
    Assert(current_float_value >= min_float_value);
    
    int constexpr min_int_value = 0;
    int constexpr max_int_value = 1000;
    int constexpr n_int_values = 1 + max_int_value - min_int_value;

    auto float_value_to_int_value = [min_float_value, max_float_value](float float_value){
      float float_value_ratio = (float_value - min_float_value) / (max_float_value - min_float_value);
      
      return static_cast<int>(float_value_ratio * n_int_values);
    };
    
    int const current_int_value = std::min(max_int_value,
                                           float_value_to_int_value(current_float_value));
    auto slider = new wxSlider(this,
                               sliderId,
                               current_int_value,
                               min_int_value,
                               max_int_value,
                               wxDefaultPosition,
                               wxDefaultSize,
                               wxSL_HORIZONTAL,
                               wxDefaultValidator);

    auto position_to_float = [min_float_value,
                              max_float_value
                              ](int position){
      return min_float_value + (max_float_value - min_float_value) * (position - min_int_value) / static_cast<float>(n_int_values-1);
    };

    auto global_sizer = new wxBoxSizer(wxVERTICAL);

    {
      auto title_sizer = new wxBoxSizer(wxHORIZONTAL);
      {
        auto label = new wxStaticText(this,
                                      wxWindow::NewControlId(),
                                      name+":",
                                      wxDefaultPosition,
                                      wxDefaultSize);
        label->SetForegroundColour(label_color);
        
        wxSize valueSz(70, -1);
        
        auto value_label = new wxTextCtrl(this,
                                          wxWindow::NewControlId(),
                                          float_to_string(current_float_value),
                                          wxDefaultPosition,
                                          valueSz,
                                          wxTE_PROCESS_ENTER);
        value_label->SetForegroundColour(value_unchanged_color);
        
        auto unit_label = new wxStaticText(this,
                                           wxWindow::NewControlId(),
                                           unit,
                                           wxDefaultPosition,
                                           wxDefaultSize);
        unit_label->SetForegroundColour(dark_grey);
        
        Add(label,
            title_sizer,
            1,
            wxALL | wxALIGN_CENTER);
        Add(value_label,
            title_sizer,
            0,
            wxALL | wxALIGN_CENTER);
        Add(unit_label,
            title_sizer,
            1,
            wxALL | wxALIGN_CENTER);
                
        value_label->Bind(wxEVT_TEXT_ENTER,
                     [this,
                      slider,
                      &param,
                      float_value_to_int_value,
                      value_label
                      ](wxCommandEvent & event){
          try {
            std::string const str = event.GetString().ToStdString();
            float const candidate = std::stof(str);
            int const i = float_value_to_int_value(candidate);
            slider->SetValue(i);
            if (candidate != param.get()) {
              value_label->SetForegroundColour(dark_grey);
              // I would like the window to be redrawn, taking this color into account
              // but unfortunately, nothing hereunder triggers a redraw, so I comment it out.
              /*
              value_label->Refresh();
              value_label->Update();
              Refresh();
              Update();
              value_label->GetParent()->Layout();
              Refresh();
              Update();
              */
              // The UI may becomes unresponsive if this is long:
              param.set(candidate);
            }
            value_label->SetForegroundColour(value_unchanged_color);
          } catch (std::invalid_argument const & e) {
            value_label->SetForegroundColour(value_incorrect_color);
            std::cout << e.what() << std::endl;
          }
        });
        
        slider->Bind(wxEVT_SCROLL_THUMBTRACK,
                     [this,
                      &param,
                      value_label,
                      position_to_float
                      ](wxScrollEvent & event){
          float const candidate = position_to_float(event.GetPosition());
          value_label->SetLabel(float_to_string(candidate));
          value_label->SetForegroundColour((candidate == param.get()) ? value_unchanged_color : dark_grey);
          value_label->GetParent()->Layout();
        });
        
        // wxEVT_SCROLL_CHANGED is not sent on osx, so we use wxEVT_SCROLL_THUMBRELEASE instead
        slider->Bind(wxEVT_SCROLL_THUMBRELEASE,
                     [this,
                      &param,
                      value_label,
                      position_to_float
                      ](wxScrollEvent & event){
          float const candidate = position_to_float(event.GetPosition());
          if (candidate != param.get()) {
            // The UI may becomes unresponsive if this is long:
            param.set(candidate);
          }
          value_label->SetForegroundColour(value_unchanged_color);
        });

      }
      Add(title_sizer,
          global_sizer,
          1,
          wxALL); // do not expand
    }
    {
      auto slider_sizer = new wxBoxSizer(wxHORIZONTAL);
      {
        wxSize boundsSz(50, -1);
        
        auto minText = new wxStaticText(this,
                                        wxWindow::NewControlId(),
                                        float_to_string(min_float_value),
                                        wxDefaultPosition,
                                        boundsSz,
                                        wxST_NO_AUTORESIZE);
        minText->SetForegroundColour(dark_grey);
        
        auto maxText = new wxStaticText(this,
                                        wxWindow::NewControlId(),
                                        float_to_string(max_float_value),
                                        wxDefaultPosition,
                                        boundsSz,
                                        wxST_NO_AUTORESIZE);
        maxText->SetForegroundColour(dark_grey);
        
        Add(minText,
            slider_sizer,
            0,
            wxALL | wxALIGN_CENTER);
        Add(slider,
            slider_sizer,
            1);
        Add(maxText,
            slider_sizer,
            0,
            wxALL | wxALIGN_CENTER);
      }
      Add(slider_sizer,
          global_sizer);
    }

    return global_sizer;
  }

  std::pair<wxBoxSizer *, wxStaticText*>
  createPollParamUI(PollParam const & param,
                    wxColor const & label_color)
  {
    auto sizer = new wxBoxSizer(wxHORIZONTAL);
    
    auto label = new wxStaticText(this,
                                  wxWindow::NewControlId(),
                                  param.name + ":",
                                  wxDefaultPosition,
                                  wxDefaultSize);
    label->SetForegroundColour(label_color);
    
    wxSize valueSz(80, -1);
    
    auto value = new wxStaticText(this,
                                  wxWindow::NewControlId(),
                                  "",
                                  wxDefaultPosition,
                                  valueSz);
    value->SetForegroundColour(dark_grey);
    Add(label,
        sizer,
        0,
        wxALL | wxALIGN_CENTER);
    Add(value,
        sizer,
        0,
        wxALL | wxALIGN_CENTER);
    return {sizer, value};
  }

};

struct ReinitializingParameters {
  // modifiying sample_rate requires to reinitialize the resynth
  int sample_rate = 88200;
  
  void init(RtResynth & r) {
    r.init(sample_rate);
  }
};

struct MyApp : public wxApp {
  MyApp()
  : reinitializing_params{
    // no need to control sample rate for now
    {
      "Input delay",
      "seconds",
      [this](){ return resynth.getInputDelaySeconds(); },
      [this](float v){ resynth.setInputDelaySeconds(v); },
      0.f,
      1.f
    },
    {
      "Analysis period",
      "seconds",
      [this](){ return resynth.getWindowCenterStrideSeconds(); },
      [this](float v){ resynth.setWindowCenterStrideSeconds(v); },
      0.f,
      1.f
    },
    {
      "Analysis window size",
      "seconds",
      [this](){ return resynth.getWindowSizeSeconds(); },
      [this](float v){ resynth.setWindowSizeSeconds(v); },
      0.f,
      1.f
    },
  }
  , realtime_params{
    {
      "Min volume",
      "",
      [this](){ return resynth.getMinVolume(); },
      [this](float v){ resynth.setMinVolume(v); },
      0.f,
      0.001f
    },
    {
      "Pitch interval diameter",
      "pitches",
      [this](){ return resynth.getNearbyDistanceTones(); },
      [this](float v){ resynth.setNearbyDistanceTones(v); },
      0.f,
      10.f
    },
    {
      "Pitch tracking distance",
      "pitches",
      [this](){ return resynth.getMaxTrackPitches(); },
      [this](float v){ resynth.setMaxTrackPitches(v); },
      0.f,
      10.f
    }
  },
  poll_params{
    {
      {
        "Total Load",
        [this]() -> PollParam::OptionalVariant {
          std::optional<float> a = resynth.getDurationProcess();
          std::optional<float> b = resynth.getDurationStep();
          std::optional<float> c = resynth.getDurationCopy();
          if (a && b && c) {
            float const secs = *a + *b + *c;
            float analysis_stride_secs = resynth.getEffectiveWindowCenterStrideSeconds(reinit_params.sample_rate);
            if (analysis_stride_secs) {
              return secs / analysis_stride_secs;
            }
          }
          return {};
        }
      },
      {
        "Process Load",
        [this]() -> PollParam::OptionalVariant {
          if (std::optional<float> secs = resynth.getDurationProcess()) {
            float analysis_stride_secs = resynth.getEffectiveWindowCenterStrideSeconds(reinit_params.sample_rate);
            if (analysis_stride_secs) {
              return *secs / analysis_stride_secs;
            }
          }
          return {};
        }
      },
      {
        "Step Load",
        [this]() -> PollParam::OptionalVariant {
          if (std::optional<float> secs = resynth.getDurationStep()) {
            float analysis_stride_secs = resynth.getEffectiveWindowCenterStrideSeconds(reinit_params.sample_rate);
            if (analysis_stride_secs) {
              return *secs / analysis_stride_secs;
            }
          }
          return {};
        }
      },
      {
        "Copy Load",
        [this]() -> PollParam::OptionalVariant {
          if (std::optional<float> secs = resynth.getDurationCopy()) {
            float analysis_stride_secs = resynth.getEffectiveWindowCenterStrideSeconds(reinit_params.sample_rate);
            if (analysis_stride_secs) {
              return *secs / analysis_stride_secs;
            }
          }
          return {};
        }
      }
    },
    {
      {
        "Audio input queue fill ratio",
        [this](){ return resynth.getAudioInputQueueFillRatio(); }
      },
      {
        "Audio input frames dropped",
        [this](){ return resynth.countDroppedInputFrames(); }
      }
    },
    {
      {
        "Notes dropped\n"
        "(increase 'Pitch interval diameter'\n"
        "to avoid them)",
        [this](){ return resynth.countDroppedNoteOns(); }
      },
      {
        "Failed compute insertions",
        [this](){ return resynth.countFailedComputeInsertions(); }
      },
      {
        "Retried oneshot insertions",
        [this](){ return resynth.countRetriedOneshotInsertions(); }
      },
    }
    
  }
  {}

  bool OnInit() override {
    reinit_params.init(resynth);
    
    auto frame = new MyFrame(reinitializing_params,
                             realtime_params,
                             poll_params);
    frame->Show(true);
    return true;
  }
private:
  RtResynth resynth;
  
  ReinitializingParameters reinit_params;
  std::vector<Param> reinitializing_params, realtime_params;
  std::vector<std::vector<PollParam>> poll_params;
};


}

wxIMPLEMENT_APP(imajuscule::audio::rtresynth::MyApp);
