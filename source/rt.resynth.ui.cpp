

namespace imajuscule::audio::rtresynth {

// modifiying one of these params requires to reinitialize the resynth
struct ReinitializingParameters {
  int sample_rate = 88200;
  float input_delay_seconds = 1.f;
  float window_size_seconds = 0.1814f;
  float window_stride_ratio = 0.5f;
  
  void init(RtResynth & r) {
    r.init(sample_rate,
           input_delay_seconds,
           window_size_seconds,
           window_stride_ratio);
  }
};

template<typename T, typename U>
void Add(T * widget, U * sizer) {
  sizer->Add(widget);
}

class MyFrame : public wxFrame
{
public:
  template<typename F>
  MyFrame(ReinitializingParameters & reinit_params, F afterChange)
  : wxFrame(NULL,
            wxID_ANY,
            "Resynth",
            wxDefaultPosition,
            wxDefaultSize,
            wxSTAY_ON_TOP | wxDEFAULT_FRAME_STYLE)
  {
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

    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    Bind(wxEVT_MENU, &MyFrame::OnHello, this, helloId);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
    
    auto sliders_sizer = new wxBoxSizer(wxVERTICAL);
    
    // no need to control sample rate for now
    Add(createFloatSlider(&reinit_params.input_delay_seconds,
                          "Input delay (seconds)",
                          0.f,
                          1.f,
                          afterChange),
        sliders_sizer);
    Add(createFloatSlider(&reinit_params.window_size_seconds,
                          "FFT length (seconds)",
                          0.f,
                          0.5f,
                          afterChange),
        sliders_sizer);
    Add(createFloatSlider(&reinit_params.window_stride_ratio,
                          "Temporal stride / FFT length",
                          0.f,
                          10.f,
                          afterChange),
        sliders_sizer);
    
    SetSizerAndFit(sliders_sizer);
  }

private:
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
  
  template<typename F>
  wxSlider * createFloatSlider(float * valuePtr,
                         std::string const & name,
                         float min_float_value,
                         float max_float_value,
                         F afterChange) {
    auto const sliderId = wxWindow::NewControlId();

    float const current_float_value = *valuePtr;
    Assert(current_float_value <= max_float_value);
    Assert(current_float_value >= min_float_value);
    
    int constexpr min_int_value = 0;
    int constexpr max_int_value = 1000;
    int constexpr n_int_values = 1 + max_int_value - min_int_value;

    float current_float_value_ratio = (current_float_value - min_float_value) / (max_float_value - min_float_value);
    Assert(current_float_value_ratio >= 0.f);
    Assert(current_float_value_ratio <= 1.f);
    
    int const current_int_value =
    std::min(max_int_value,
             static_cast<int>(current_float_value_ratio * n_int_values));
    
    auto slider = new wxSlider(this,
                               sliderId,
                               current_int_value,
                               min_int_value,
                               max_int_value,
                               wxDefaultPosition,
                               wxDefaultSize,
                               wxSL_HORIZONTAL,
                               wxDefaultValidator,
                               name);
    
    // wxEVT_SCROLL_CHANGED is not sent on osx, so we use wxEVT_SCROLL_THUMBRELEASE instead
    slider->Bind(wxEVT_SCROLL_THUMBRELEASE,
                 [valuePtr,
                  min_float_value,
                  max_float_value,
                  afterChange](wxScrollEvent & event){
      *valuePtr =
      min_float_value + (max_float_value - min_float_value) * (event.GetPosition() - min_int_value) / static_cast<float>(n_int_values-1);
      afterChange();
    });
    
    return slider;
  }
};


struct MyApp : public wxApp
{
  bool OnInit() override {
    
    reinit_params.init(resynth);
    
    auto frame = new MyFrame(reinit_params,
                             [this](){
      reinit_params.init(resynth);
    });
    frame->Show(true);
    return true;
  }
private:
  RtResynth resynth;
  
  ReinitializingParameters reinit_params;
};


}

wxIMPLEMENT_APP(imajuscule::audio::rtresynth::MyApp);
