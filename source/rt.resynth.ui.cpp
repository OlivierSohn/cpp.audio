
namespace imajuscule::audio::rtresynth {

enum class TimerId {
  RefreshUI
};

class MyFrame : public wxFrame
{
public:

  MyFrame(std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> const & params_before_autotune,
          Autotune const & autotune,
          std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> const & params_after_autotune,
          std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> const & params_envelope,
          std::vector<std::vector<ParamPollProxy>> const & poll_params,
          PitchWindow::PitchFunction const & pitch_func)
  : wxFrame(NULL,
            wxID_ANY,
            "Resynth",
            wxPoint(100,30),
            wxDefaultSize,
            wxDEFAULT_FRAME_STYLE |
            wxFULL_REPAINT_ON_RESIZE)
  , uiTimer(this,
            static_cast<int>(TimerId::RefreshUI))
  {
    pitch_ui = new PitchWindow(this, Orientation::Horizontal, pitch_func);

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
    
    auto make_params_sizer = [this](std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> const & params, wxOrientation orient, wxBoxSizer * sizer) {
      for (auto const & [params2, label_color] : params) {
        auto sliders_sizer2 = new wxBoxSizer(orient);
        
        for (auto const & param : params2) {
          Add(createFloatSlider(this,
                                param,
                                label_color),
              sliders_sizer2,
              0,
              wxALL | wxALIGN_CENTER);
        }
        Add(sliders_sizer2,
            sizer,
            0,
            wxALL | wxALIGN_CENTER);
      }
    };

    auto global_sizer = new wxBoxSizer(wxHORIZONTAL);
    auto sliders_sizer = new wxBoxSizer(wxVERTICAL);
    auto post_sizer = new wxBoxSizer(wxVERTICAL);

    make_params_sizer(params_envelope,
                      wxVERTICAL,
                      post_sizer);

    make_params_sizer(params_before_autotune,
                      wxHORIZONTAL,
                      sliders_sizer);
    Add(mkAutotuneSizer(this,
                        autotune),
        sliders_sizer);
    make_params_sizer(params_after_autotune,
                      wxHORIZONTAL,
                      sliders_sizer);
    Add(pitch_ui,
        sliders_sizer,
        0,
        wxALL | wxEXPAND);

    auto poll_params_sizer = new wxBoxSizer(wxVERTICAL);
    {
      for (auto const & params: poll_params) {
        for (auto const & param:params) {
          auto [sizer, value] = createPollParamUI(this,
                                                  param,
                                                  poll_label_color);
          poll_params_ui.emplace_back(param, value);
          Add(sizer,
              poll_params_sizer,
              0,
              wxALL | wxALIGN_CENTER);
        }
      }
    }
    
    Add(sliders_sizer,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);
    Add(post_sizer,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);
    Add(poll_params_sizer,
        global_sizer,
        0,
        wxALL | wxALIGN_CENTER);

    SetSizerAndFit(global_sizer);

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
  : params_before_autotune
  {
    {
      {
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
          "Pitch interval diameter",
          "semitones",
          [this](){ return resynth.getNearbyDistanceTones(); },
          [this](float v){ resynth.setNearbyDistanceTones(v); },
          0.f,
          10.f
        },
      },
      color_slider_label_1
    },
    {
      {
        {
          "Min partial volume",
          "",
          [this](){ return resynth.getMinVolume(); },
          [this](float v){ resynth.setMinVolume(v); },
          0.f,
          0.001f
        },
        {
          "Pitch shift before autotune",
          "semitones",
          [this](){ return resynth.getPitchShiftPreAutotune(); },
          [this](float v){ resynth.setPitchShiftPreAutotune(v); },
          -12.f,
          12.f
        },
        {
          "Harmonize before autotune",
          "semitones",
          [this](){ return resynth.getHarmonizePreAutotune(); },
          [this](float v){ resynth.setHarmonizePreAutotune(v); },
          -12.f,
          12.f
        }
      },
      color_slider_label_2
    }
  },
  params_after_autotune
  {
    {
      {
        {
          "Pitch shift after autotune",
          "semitones",
          [this](){ return resynth.getPitchShiftPostAutotune(); },
          [this](float v){ resynth.setPitchShiftPostAutotune(v); },
          -12.f,
          12.f
        },
        {
          "Harmonize after autotune",
          "semitones",
          [this](){ return resynth.getHarmonizePostAutotune(); },
          [this](float v){ resynth.setHarmonizePostAutotune(v); },
          -12.f,
          12.f
        },
        {
          "Pitch tracking distance",
          "semitones",
          [this](){ return resynth.getMaxTrackPitches(); },
          [this](float v){ resynth.setMaxTrackPitches(v); },
          0.f,
          10.f
        },
      },
      color_slider_label_3
    },
    {
      {
        {
          "Stereo spread",
          "",
          [this](){ return resynth.getStereoSpread(); },
          [this](float v){ resynth.setStereoSpread(v); },
          0.f,
          1.f
        },
        {
          "Volume",
          "",
          [this](){ return resynth.getAnalysisVolume(); },
          [this](float v){ resynth.setAnalysisVolume(v); },
          0.f,
          1.f
        },
      },
      color_slider_label_5
    }
  }
  , params_envelope
  {
    {
      {
        {
          "Attack",
          "seconds",
          [this](){ return resynth.getEnvAttackSeconds(); },
          [this](float v){ resynth.setEnvAttackSeconds(v); },
          0.f,
          1.f
        },
        {
          "Hold",
          "seconds",
          [this](){ return resynth.getEnvHoldSeconds(); },
          [this](float v){ resynth.setEnvHoldSeconds(v); },
          0.f,
          1.f
        },
        {
          "Decay",
          "seconds",
          [this](){ return resynth.getEnvDecaySeconds(); },
          [this](float v){ resynth.setEnvDecaySeconds(v); },
          0.f,
          1.f
        },
        {
          "Sustain",
          "",
          [this](){ return resynth.getEnvSustainLevel(); },
          [this](float v){ resynth.setEnvSustainLevel(v); },
          0.f,
          1.f
        },
        {
          "Release",
          "seconds",
          [this](){ return resynth.getEnvReleaseSeconds(); },
          [this](float v){ resynth.setEnvReleaseSeconds(v); },
          0.f,
          1.f
        },
      },
      color_slider_label_4
    },
    {
      {
        {
          "Vocoder env follower cutoff ratio",
          "",
          [this](){ return resynth.getVocoderEnvFollowerCutoffRatio(); },
          [this](float v){ resynth.setVocoderEnvFollowerCutoffRatio(v); },
          0.f,
          2.f
        },
        {
          "Vocoder volume",
          "",
          [this](){ return resynth.getVocoderVolume(); },
          [this](float v){ resynth.setVocoderVolume(v); },
          0.f,
          1.f
        },
        {
          "Dry voice volume",
          "",
          [this](){ return resynth.getDirectVoiceVolume(); },
          [this](float v){ resynth.setDirectVoiceVolume(v); },
          0.f,
          1.f
        },
      },
      color_slider_label_5
    }
  }
  , poll_params
  {
    {
      {
        "Input stream load",
        [this]() -> ParamPollProxy::OptionalVariant {
          return resynth.getInputStreamCpuLoad();
        }
      },
      {
        "Output stream load",
        [this]() -> ParamPollProxy::OptionalVariant {
          return resynth.getOutputStreamCpuLoad();
        }
      },
      {
        "Output limitting factor",
        [this]() -> ParamPollProxy::OptionalVariant {
          return resynth.getCompressionFactor();
        }
      },
      {
        "Analysis Load",
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
        "Analysis/process Load",
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
        "Analysis/step Load",
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
        "Analysis/copy Load",
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
  autotune
  {
    {
      "Autotune",
      "",
      [this](){ return resynth.getUseAutotune(); },
      [this](bool v){ resynth.setUseAutotune(v); },
      0,
      1
    },
    {
      "",
      [this](){ return resynth.getAutotuneType(); },
      [this](AutotuneType v){ resynth.setAutotuneType(v); },
    },
    {
      "Ignore pitches above",
      "",
      [this](){ return resynth.getAutotuneMaxPitch(); },
      [this](int pitch){ resynth.setAutotuneMaxPitch(pitch); },
      0,
      150,
      [](int pitch) {
        auto [noteoctave, deviation] = midi_pitch_to_note_deviation(pitch);
        Assert(deviation == 0.);
        std::ostringstream os;
        os << noteoctave.note << noteoctave.octave;
        return os.str();        
      }
    },
    {
      "Off-key pitches drop distance",
      "semitones",
      [this](){ return resynth.getAutotunePitchTolerance(); },
      [this](float v){ resynth.setAutotunePitchTolerance(v); },
      0.f,
      150.f
    },
    {
      "",
      [this](){ return resynth.getAutotuneChordFrequencies(); },
      [this](AutotuneChordFrequencies v){ resynth.setAutotuneChordFrequencies(v); },
    },
    {
      "",
      [this](Note const & n){ return resynth.autotuneBitChordHasNote(n); },
      [this](Note const & n, bool enabled){ resynth.autotuneBitChordSetNote(n, enabled); }
    },
    {
      "Scale mode",
      [this](){ return resynth.getAutotuneMusicalScaleMode(); },
      [this](MusicalScaleMode v){ resynth.setAutotuneMusicalScaleMode(v); },
    },
    {
      "",
      [this](){ return resynth.getAutotuneMusicalScaleRoot(); },
      [this](Note v){ resynth.setAutotuneMusicalScaleRoot(v); },
    },
    {
      "Transpose",
      "semitones",
      [this](){ return resynth.getAutotuneRootTranspose(); },
      [this](int v){ resynth.setAutotuneRootTranspose(v); },
      -50,
      50
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

    auto frame = new MyFrame(params_before_autotune,
                             autotune,
                             params_after_autotune,
                             params_envelope,
                             poll_params,
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
  std::vector<std::pair<std::vector<ParamProxy<float>>, wxColor>> params_before_autotune, params_after_autotune, params_envelope;
  std::vector<std::vector<ParamPollProxy>> poll_params;
  Autotune autotune;
};


}

wxIMPLEMENT_APP(imajuscule::audio::rtresynth::MyApp);
