
namespace imajuscule::audio::rtresynth {

enum class TimerId {
  RefreshUI
};

class MyFrame : public wxFrame
{
public:

  MyFrame(std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> const & params,
          std::vector<std::vector<ParamPollProxy>> const & poll_params,
          Autotune const & autotune,
          PitchWindow::PitchFunction const & pitch_func)
  : wxFrame(NULL,
            wxID_ANY,
            "Resynth",
            wxDefaultPosition,
            wxDefaultSize,
            wxDEFAULT_FRAME_STYLE |
            wxFULL_REPAINT_ON_RESIZE)
  , uiTimer(this,
            static_cast<int>(TimerId::RefreshUI))
  {
    pitch_ui = new PitchWindow(this, pitch_func);

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

    auto sizer = new wxBoxSizer(wxHORIZONTAL);
    {
      auto sliders_sizer= new wxBoxSizer(wxVERTICAL);
      {
        auto params_sliders_sizer = new wxBoxSizer(wxVERTICAL);
        for (auto const & [params2, label_color] : params) {
          auto sliders_sizer2 = new wxBoxSizer(wxHORIZONTAL);
          
          for (auto const & param : params2) {
            Add(createFloatSlider(this,
                                  param,
                                  label_color),
                sliders_sizer2);
          }
          Add(sliders_sizer2,
              params_sliders_sizer);
        }
        Add(params_sliders_sizer,
            sliders_sizer);
      }
      {
        auto autotune_sizer = mkAutotuneSizer(this,
                                              autotune);
        
        Add(autotune_sizer,
            sliders_sizer);
      }
      {
        auto poll_params_sizer = new wxBoxSizer(wxVERTICAL);
        {
          auto rt_sizer = new wxBoxSizer(wxVERTICAL);
          wxColor label_color(100,
                              100,
                              250);
          
          for (auto const & params: poll_params) {
            auto rt2_sizer = new wxBoxSizer(wxHORIZONTAL);
            for (auto const & param:params) {
              auto [sizer, value] = createPollParamUI(this,
                                                      param,
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
              poll_params_sizer);
        }
        Add(poll_params_sizer,
            sliders_sizer);
      }
      Add(sliders_sizer,
          sizer);
    }
    Add(pitch_ui,
        sizer,
        0,
        wxALL);

    SetSizerAndFit(sizer);

    uiTimer.Start(100);
  }
  
  ~MyFrame() {
    if (uiTimer.IsRunning()) {
      uiTimer.Stop();
    }
  }

private:
  wxTimer uiTimer;
  std::vector<std::pair<ParamPollProxy, wxStaticText*>> poll_params_ui;
  PitchWindow * pitch_ui;
  
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
    if (pitch_ui->TryFetchNewFrame()) {
      pitch_ui->Refresh();
    }
    
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
  : params
  {
    {
      {
        // no need to control sample rate for now
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
        {
          "Min volume",
          "",
          [this](){ return resynth.getMinVolume(); },
          [this](float v){ resynth.setMinVolume(v); },
          0.f,
          0.001f
        },
      },
      color_slider_label_1
    },
    {
      {
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
        },
      },
      color_slider_label_2
    }
  }
  , poll_params
  {
    {
      {
        "Total Load",
        [this]() -> ParamPollProxy::OptionalVariant {
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
        [this]() -> ParamPollProxy::OptionalVariant {
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
        [this]() -> ParamPollProxy::OptionalVariant {
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
        [this]() -> ParamPollProxy::OptionalVariant {
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
  },
  autotune{
    {
      "Autotune",
      [this](){ return resynth.getAutotuneType(); },
      [this](AutotuneType v){ resynth.setAutotuneType(v); },
    },
    {
      "Mode",
      [this](){ return resynth.getAutotuneMusicalScaleMode(); },
      [this](MusicalScaleMode v){ resynth.setAutotuneMusicalScaleMode(v); },
    },
    {
      "Root",
      [this](){ return resynth.getAutotuneMusicalScaleRoot(); },
      [this](Note v){ resynth.setAutotuneMusicalScaleRoot(v); },
    },
    {
      "Intervals",
      "semitones",
      [this](){ return resynth.getAutotuneFactor(); },
      [this](int v){ resynth.setAutotuneFactor(v); },
      0,
      40
    },
  }
  {}

  bool OnInit() override {
    reinit_params.init(resynth);

    auto frame = new MyFrame(params,
                             poll_params,
                             autotune,
                             [this](std::vector<PlayedNote> & result, std::vector<PlayedNote> & result_dropped, std::optional<int64_t> & frame_id){
      result.clear();
      result_dropped.clear();

      NonRealtimeAnalysisFrame const & analysis_data = resynth.getAnalysisData();

      auto & vec = analysis_data.getPlayingNotes();
      auto & vec_dropped = analysis_data.getDroppedNotes();

      while(true)
      {
        std::size_t sz, sz_dropped;
        {
          std::lock_guard g(analysis_data.getMutex());
          sz = vec.size();
          sz_dropped = vec_dropped.size();
          if ((result.capacity() >= sz) &&
              (result_dropped.capacity() >= sz_dropped)) {
            std::copy(vec.begin(),
                      vec.end(),
                      std::back_inserter(result));
            std::copy(vec_dropped.begin(),
                      vec_dropped.end(),
                      std::back_inserter(result_dropped));
            frame_id.reset();
            if (auto status = analysis_data.getFrameStatus()) {
              frame_id = status->frame_id;
            }
            return;
          }
        }
        // allocate when we don't own the mutex
        result.reserve(3*sz);
        result_dropped.reserve(3*sz_dropped);
      }
    });
    frame->Show(true);
    return true;
  }
private:
  RtResynth resynth;
  
  ReinitializingParameters reinit_params;
  std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> params;
  std::vector<std::vector<ParamPollProxy>> poll_params;
  Autotune autotune;
};


}

wxIMPLEMENT_APP(imajuscule::audio::rtresynth::MyApp);
