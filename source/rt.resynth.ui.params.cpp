namespace imajuscule::audio::rtresynth {

using UpdateFunc = std::function<void(void)>;

struct Border{
  explicit Border(int border): border(border) {}
  int border;
};
struct Flag{
  explicit Flag(int flags): flags(flags) {}
  int flags;
};
enum class CanShrink{Yes, No};
template<typename T, typename U>
void Add(T * widget,
         U * sizer,
         Flag flag = Flag{wxALL | wxEXPAND}, // cannot mix wxEXPAND with wxALIGN***
         Border border = Border{1},
         CanShrink canShrink = CanShrink::No
         ) {
  sizer->Add(widget,
             /*proportion, 0 means cannot resize (cannot shrink)*/
             canShrink == CanShrink::No ? 0 : 1,
             flag.flags,
             border.border);
}

template<typename F>
void forEachWindow(wxSizer * sizer,
                   F const & f) {
  for (int i= 0, sz = static_cast<int>(sizer->GetItemCount());
       i < sz;
       ++i) {
    wxSizerItem * item = sizer->GetItem(i);
    if (auto window = item->GetWindow()) {
      f(*window);
    } else if (auto sizer = item->GetSizer()) {
      forEachWindow(sizer, f);
    }
  }
}

template<typename T>
struct EnumeratedParamProxy {
  using value_type = T;
  
  std::string name;
  std::function<T()> get;
  std::function<void(T)> set;
  auto list() const {
    std::vector<T> values;
    values.reserve(CountEnumValues<T>::count);
    for (int i=0; i<CountEnumValues<T>::count; ++i) {
      T const val = static_cast<T>(i);
      values.emplace_back(val);
    }
    return values;
  }
};

template<typename T>
struct DynamicChoiceParamProxy {
  using value_type = T;
  
  std::string name;
  std::function<T()> get;
  std::function<void(T)> set;
  std::function<std::vector<T>()> list;
};

template<typename T>
struct EnumeratedCombinationParamProxy {
  std::string name;
  std::function<bool(T)> enabled;
  std::function<void(T, bool)> enable;
};

template<typename T>
struct ParamProxy {
  std::string name, unit;
  std::function<T()> get;
  std::function<void(T)> set;
  T min, max;
  std::function<std::string(T)> show = {}; // optional
};

struct ParamPollProxy {
  using OptionalVariant = std::optional<std::variant<int, float>>;
  std::string name;
  std::function<OptionalVariant()> get;
};


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

const wxColor color_slider_label_1{
  100,
  150,
  200
};
const wxColor color_slider_label_2{
  150,
  100,
  200
};
const wxColor color_slider_label_3{
  150,
  100,
  150
};
const wxColor color_slider_label_4{
  150,
  150,
  150
};
const wxColor color_slider_label_5{
  100,
  100,
  100
};
const wxColor poll_label_color{
  100,
  100,
  250
};

const wxColor autotune_bg_color1{
  30,
  30,
  30
};
const wxColor autotune_bg_color2{
  40,
  40,
  40
};
const wxColor autotune_bg_color3{
  35,
  35,
  35
};

const wxColor color_midi_label_1{
  200,
  191,
  0
};

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

enum class ChoiceType {
  RadioBoxH,
  RadioBoxV,
  ComboBox
};

enum class CombinationType {
  CheckBoxH,
  CheckBoxV
};


template<typename U, typename F = std::function<void(void)>>
wxBoxSizer * createChoice(wxWindow * parent,
                          const U & param,
                          wxColor const & label_color,
                          ChoiceType const choice_type,
                          std::vector<UpdateFunc> * update_on_new_preset,
                          F const & onNewValue = [](){}) {
  using T = typename U::value_type;
  
  std::vector<T> vals = param.list();
  T const current_value = param.get();

  std::vector<wxString> values;
  values.reserve(vals.size());
  std::optional<int> cur_idx;
  {
    int i=-1;
    for (auto const & v : vals) {
      ++i;
      std::ostringstream os;
      os << v;
      values.emplace_back(os.str());
      
      if (v == current_value) {
        cur_idx = i;
      }
    }
  }
  
  auto on_event = [parent,
                   onNewValue,
                   &param
                   ](int idx){
    auto vals = param.list();
    if(idx >= 0 && idx < vals.size()) {
      T const & candidate = vals[idx];
      if (candidate != param.get()) {
        // The UI may becomes unresponsive if this is long:
        param.set(candidate);
        onNewValue();
      }
    }
  };
    
  auto global_sizer = new wxBoxSizer(wxVERTICAL);
  
  switch(choice_type) {
    case ChoiceType::RadioBoxH:
    case ChoiceType::RadioBoxV:
    {
      // with 'wxRadioBox' we can't customize the label color
      // (there is a display bug where only part of the label is displayed)
      // so we do this:
      auto inner_sizer = new wxBoxSizer((choice_type == ChoiceType::RadioBoxH) ? wxHORIZONTAL : wxVERTICAL);
      
      bool first = true;
      int idx = 0;
      std::vector<wxRadioButton*> radios;
      radios.reserve(values.size());
      for (auto const & label : values) {
        wxRadioButton * radio = new wxRadioButton(parent,
                                                  wxWindow::NewControlId(),
                                                  label,
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  first?wxRB_GROUP:0);
        radios.push_back(radio);
        radio->Bind(wxEVT_RADIOBUTTON,
                    [on_event, idx](wxCommandEvent &){
          on_event(idx);
        });
        if (vals[idx] == current_value) {
          radio->SetValue(true);
        }
        first = false;
        Add(radio,
            inner_sizer,
            Flag{wxALL});
        ++idx;
      }
      wxStaticBoxSizer * stat_box_sizer = new wxStaticBoxSizer((choice_type == ChoiceType::RadioBoxH) ? wxHORIZONTAL: wxVERTICAL,
                                                               parent,
                                                               param.name);
      stat_box_sizer->GetStaticBox()->SetForegroundColour(label_color);
      Add(inner_sizer,
          stat_box_sizer,
          Flag{0},
          Border{0});
      Add(stat_box_sizer,
          global_sizer,
          Flag{0},
          Border{0});
      
      // not all choices need to update when a preset is loaded
      if (update_on_new_preset) {
        update_on_new_preset->emplace_back([&param, radios, onNewValue](){
          auto const vals = param.list();
          T const current_value = param.get();
          int idx = 0;
          for (wxRadioButton * radio : radios) {
            radio->SetValue(idx < vals.size() && vals[idx] == current_value);
            ++idx;
          }
          onNewValue();
        });
      }
      break;
    }
    case ChoiceType::ComboBox:
    {
      wxComboBox * combo_box = new wxComboBox(parent,
                                              wxWindow::NewControlId(),
                                              cur_idx ? values[*cur_idx] : "",
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              static_cast<int>(values.size()),
                                              values.data(),
                                              wxCB_READONLY);
      combo_box->Bind(wxEVT_COMBOBOX,
                      [on_event](wxCommandEvent & e){
        on_event(e.GetInt());
      });
      auto label = new wxStaticText(parent,
                                    wxWindow::NewControlId(),
                                    param.name+":",
                                    wxDefaultPosition,
                                    wxDefaultSize);
      label->SetForegroundColour(label_color);
      
      Add(label,
          global_sizer,
          Flag{wxALL | wxALIGN_CENTER},
          Border{0});
      Add(combo_box,
          global_sizer,
          Flag{wxALL | wxALIGN_CENTER},
          Border{0});
      
      if(update_on_new_preset) {
        update_on_new_preset->emplace_back([&param, combo_box, onNewValue](){
          T const current_value = param.get();
          auto const vals = param.list();
          std::optional<int> cur_idx;
          {
            int i=-1;
            for (auto const & v : vals) {
              ++i;
              if (v == current_value) {
                cur_idx = i;
              }
            }
          }
          
          if (cur_idx) {
            combo_box->SetSelection(*cur_idx);
            onNewValue();
          }
        });
      }
      break;
    }
  }
  
  return global_sizer;
}

template<typename T>
wxBoxSizer * createCombination(wxWindow * parent,
                               const EnumeratedCombinationParamProxy<T> & param,
                               wxColor const & label_color,
                               CombinationType const combination_type,
                               std::vector<UpdateFunc> & updates) {
  std::vector<wxString> values;
  values.reserve(CountEnumValues<T>::count);
  for (int i=0; i<CountEnumValues<T>::count; ++i) {
    T const val = static_cast<T>(i);
    std::ostringstream os;
    os << val;
    values.emplace_back(os.str());
  }
  
  wxStaticBoxSizer * stat_box_sizer = new wxStaticBoxSizer((combination_type == CombinationType::CheckBoxH) ? wxHORIZONTAL: wxVERTICAL,
                                                           parent,
                                                           param.name);

  auto inner_sizer = new wxBoxSizer((combination_type == CombinationType::CheckBoxH) ? wxHORIZONTAL : wxVERTICAL);
  
  std::vector<wxCheckBox*> checkboxes;
  checkboxes.reserve(values.size());
  
  int idx = 0;
  for (auto const & label : values) {
    T const enum_value = static_cast<T>(idx);
    wxCheckBox * checkbox = new wxCheckBox(stat_box_sizer->GetStaticBox(),
                                           wxWindow::NewControlId(),
                                           label);
    checkboxes.push_back(checkbox);
    checkbox->Bind(wxEVT_CHECKBOX,
                   [enum_value, &param](wxCommandEvent & e){
      param.enable(enum_value, static_cast<bool>(e.GetInt()));
    });
    checkbox->SetValue(param.enabled(enum_value));
    Add(checkbox,
        inner_sizer,
        Flag{wxALL});
    ++idx;
  }
  stat_box_sizer->GetStaticBox()->SetForegroundColour(label_color);
  Add(inner_sizer,
      stat_box_sizer,
      Flag{0});
  
  updates.emplace_back([checkboxes, &param](){
    int idx = 0;
    for (auto * checkbox : checkboxes) {
      T const enum_value = static_cast<T>(idx);
      checkbox->SetValue(param.enabled(enum_value));
      ++idx;
    }
  });
  return stat_box_sizer;
}

std::string T_to_string(uint64_t v) {
  return std::to_string(v);
}
std::string T_to_string(int v) {
  return std::to_string(v);
}
std::string T_to_string(float v) {
  return float_to_string(v);
}

void string_to_T(std::string const & str, float & res) {
  res = std::stof(str);
}
void string_to_T(std::string const & str, int & res) {
  res = std::stoi(str);
}
void string_to_T(std::string const & str, uint64_t & res) {
  res = std::stoull(str);
}


template<typename F = std::function<void(bool)>>
wxCheckBox * createCheckBox(wxWindow * parent,
                            const ParamProxy<bool> & param,
                            wxColor const & label_color,
                            std::vector<UpdateFunc> & updates,
                            F const & onNewValue = [](){}) {
  auto checkbox = new wxCheckBox(parent,
                                 wxWindow::NewControlId(),
                                 param.name);
  
  checkbox->Bind(wxEVT_CHECKBOX,
                 [&param, onNewValue](wxCommandEvent & e){
    param.set(static_cast<bool>(e.GetInt()));
    onNewValue();
  });
  checkbox->SetValue(param.get());
  checkbox->SetForegroundColour(label_color);

  updates.emplace_back([&param, checkbox, onNewValue](){
    checkbox->SetValue(param.get());
    onNewValue();
  });
  return checkbox;
}

template<typename T, typename f1, typename f2>
wxBoxSizer * createSlider(wxWindow * parent,
                          const ParamProxy<T> & param,
                          int const min_int_value,
                          int const max_int_value,
                          f1 T_value_to_int_value,
                          f2 position_to_T,
                          wxColor const & label_color,
                          std::vector<UpdateFunc> & updates) {
  T const current_T_value = param.get();
  
  int const current_int_value = std::max(min_int_value,
                                         std::min(max_int_value,
                                                  static_cast<int>(T_value_to_int_value(current_T_value))));
  
  auto slider = new wxSlider(parent,
                             wxWindow::NewControlId(),
                             current_int_value,
                             min_int_value,
                             max_int_value,
                             wxDefaultPosition,
                             wxSize{-1, 3},
                             wxSL_HORIZONTAL,
                             wxDefaultValidator);

  auto global_sizer = new wxBoxSizer(wxVERTICAL);
  
  {
    auto title_sizer = new wxBoxSizer(wxHORIZONTAL);
    {
      auto label = new wxStaticText(parent,
                                    wxWindow::NewControlId(),
                                    param.name+":",
                                    wxDefaultPosition,
                                    wxDefaultSize);
      label->SetForegroundColour(label_color);
      
      wxSize valueSz(70, -1);
      
      wxTextCtrl * value_label = nullptr;
      
      wxStaticText * show_label = nullptr;
      if (param.show) {
        show_label = new wxStaticText(parent,
                                      wxWindow::NewControlId(),
                                      param.show(current_T_value),
                                      wxDefaultPosition,
                                      valueSz);
      } else {
        value_label = new wxTextCtrl(parent,
                                     wxWindow::NewControlId(),
                                     T_to_string(current_T_value),
                                     wxDefaultPosition,
                                     valueSz,
                                     wxTE_PROCESS_ENTER);
        value_label->SetForegroundColour(value_unchanged_color);
      }
      
      auto unit_label = new wxStaticText(parent,
                                         wxWindow::NewControlId(),
                                         param.unit,
                                         wxDefaultPosition,
                                         wxDefaultSize);
      unit_label->SetForegroundColour(dark_grey);
      
      Add(label,
          title_sizer,
          Flag{wxALL | wxALIGN_CENTER});
      if (value_label) {
        Add(value_label,
            title_sizer,
            Flag{wxALL | wxALIGN_CENTER});
      }
      if (show_label) {
        Add(show_label,
            title_sizer,
            Flag{wxALL | wxALIGN_CENTER});
      }
      Add(unit_label,
          title_sizer,
          Flag{wxALL | wxALIGN_CENTER});
      
      if (value_label) {
        value_label->Bind(wxEVT_TEXT_ENTER,
                          [slider,
                           &param,
                           T_value_to_int_value,
                           show_label,
                           value_label
                           ](wxCommandEvent & event){
          try {
            std::string const str = event.GetString().ToStdString();
            T candidate;
            string_to_T(str, candidate);
            int const i = T_value_to_int_value(candidate);
            slider->SetValue(i);
            if (candidate != param.get()) {
              // This should be fast, else the UI will become unresponsive
              param.set(candidate);
            }
            value_label->SetForegroundColour(value_unchanged_color);
            if (show_label) {
              show_label->SetLabel(param.show(candidate));
            }
          } catch (std::invalid_argument const & e) {
            value_label->SetForegroundColour(value_incorrect_color);
            std::cout << e.what() << std::endl;
          }
        });
      }
      
      slider->Bind(wxEVT_SCROLL_THUMBTRACK,
                   [&param,
                    show_label,
                    value_label,
                    position_to_T
                    ](wxScrollEvent & event){
        T const candidate = position_to_T(event.GetPosition());
        if (candidate != param.get()) {
          // This should be fast, else the UI will become unresponsive
          param.set(candidate);
        }
        if (value_label) {
          value_label->SetValue(T_to_string(candidate));
          value_label->GetParent()->Layout();
          value_label->SetForegroundColour(value_unchanged_color);
        }
        if (show_label) {
          show_label->SetLabel(param.show(candidate));
        }
      });
      
      updates.emplace_back([&param,
                            slider,
                            value_label,
                            show_label,
                            min_int_value,
                            max_int_value,
                            T_value_to_int_value]() {
        T const candidate = param.get();
        slider->SetValue(std::max(min_int_value,
                                  std::min(max_int_value,
                                           static_cast<int>(T_value_to_int_value(candidate)))));
        if (value_label) {
          value_label->SetValue(T_to_string(candidate));
          value_label->GetParent()->Layout();
          value_label->SetForegroundColour(value_unchanged_color);
        }
        if (show_label) {
          show_label->SetLabel(param.show(candidate));
        }
      });
    }
    Add(title_sizer,
        global_sizer,
        Flag{wxALL | wxALIGN_CENTER},
        Border{0});
  }
  {
    auto slider_sizer = new wxBoxSizer(wxHORIZONTAL);
    {
      wxSize boundsSz(50, -1);
      
      auto minText = new wxStaticText(parent,
                                      wxWindow::NewControlId(),
                                      param.show ? param.show(param.min) : T_to_string(param.min),
                                      wxDefaultPosition);
      minText->SetForegroundColour(dark_grey);
      
      auto maxText = new wxStaticText(parent,
                                      wxWindow::NewControlId(),
                                      param.show ? param.show(param.max) : T_to_string(param.max),
                                      wxDefaultPosition);
      maxText->SetForegroundColour(dark_grey);
      
      Add(minText,
          slider_sizer,
          Flag{wxALL | wxALIGN_CENTER});
      Add(slider,
          slider_sizer,
          Flag{wxALL | wxALIGN_CENTER});
      Add(maxText,
          slider_sizer,
          Flag{wxALL | wxALIGN_CENTER});
    }
    Add(slider_sizer,
        global_sizer,
        Flag{wxALL | wxALIGN_CENTER},
        Border{0});
  }

  return global_sizer;
}

wxBoxSizer * createFloatSlider(wxWindow * parent,
                               ParamProxy<float> const & param,
                               wxColor const & label_color,
                               std::vector<UpdateFunc> & updates) {
  float const min_float_value = param.min;
  float const max_float_value = param.max;
  
  int constexpr min_int_value = 0;
  int constexpr max_int_value = 1000;
  int constexpr n_int_values = 1 + max_int_value - min_int_value;
  
  auto float_value_to_int_value = [min_float_value, max_float_value](float float_value){
    float float_value_ratio = (float_value - min_float_value) / (max_float_value - min_float_value);
    
    return static_cast<int>(float_value_ratio * n_int_values);
  };
  
  auto position_to_float = [min_float_value,
                            max_float_value
                            ](int position){
    return min_float_value + (max_float_value - min_float_value) * (position - min_int_value) / static_cast<float>(n_int_values-1);
  };
  
  return createSlider(parent,
                      param,
                      min_int_value,
                      max_int_value,
                      float_value_to_int_value,
                      position_to_float,
                      label_color,
                      updates);
}

template<typename T>
wxBoxSizer * createIntSlider(wxWindow * parent,
                             ParamProxy<T> const & param,
                             wxColor const & label_color,
                             std::vector<UpdateFunc> & updates) {
  static_assert(std::is_integral_v<T>);
  return createSlider(parent,
                      param,
                      (param.min < std::numeric_limits<int>::min()) ? std::numeric_limits<int>::min() : static_cast<int>(param.min),
                      (param.max > std::numeric_limits<int>::max()) ? std::numeric_limits<int>::max() : static_cast<int>(param.max),
                      [](T v){ return v; },
                      [](T v){ return v; },
                      label_color,
                      updates);
}


std::pair<wxBoxSizer *, wxStaticText*>
createPollParamUI(wxWindow * parent,
                  ParamPollProxy const & param,
                  wxColor const & label_color)
{
  auto sizer = new wxBoxSizer(wxHORIZONTAL);
  
  auto label = new wxStaticText(parent,
                                wxWindow::NewControlId(),
                                param.name + ":",
                                wxDefaultPosition,
                                wxDefaultSize);
  label->SetForegroundColour(label_color);
  
  wxSize valueSz(80, -1);
  
  auto value = new wxStaticText(parent,
                                wxWindow::NewControlId(),
                                "",
                                wxDefaultPosition,
                                valueSz);
  value->SetForegroundColour(dark_grey);
  Add(label,
      sizer,
      Flag{wxALL | wxALIGN_CENTER});
  Add(value,
      sizer,
      Flag{wxALL | wxALIGN_CENTER});
  return {sizer, value};
}

} // NS
