#include "portmidi.h"

namespace imajuscule::audio::rtresynth {

struct MidiName {
  std::string interf;
  std::string name;
  
  bool operator == (MidiName const & o) const {
    return interf == o.interf && name == o.name;
  }
  bool operator != (MidiName const & o) const {
    return !(*this == o);
  }
};

inline std::ostream & operator << (std::ostream & os, const MidiName & n) {
  os << n.interf << ":" << n.name;
  return os;
}
inline std::ostream & operator << (std::ostream & os, const std::optional<MidiName> & n) {
  if (n) {
    os << *n;
  } else {
    os << "No midi";
  }
  return os;
}
} // NS

namespace imajuscule::audio::rtresynth::midi {
struct AllNotesOff {
};
inline std::ostream & operator << (std::ostream & os, AllNotesOff const & n) {
  os << "AllNotesOff";
  return os;
}
struct NoteOn {
  uint8_t channel;
  uint8_t key;
  uint8_t velocity;
};
inline std::ostream & operator << (std::ostream & os, NoteOn const & n) {
  os << "NoteOn(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.key) << ", " << static_cast<int>(n.velocity) << ")";
  return os;
}
struct NoteOff {
  uint8_t channel;
  uint8_t key;
  uint8_t velocity;
};
inline std::ostream & operator << (std::ostream & os, NoteOff const & n) {
  os << "NoteOff(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.key) << ", " << static_cast<int>(n.velocity) << ")";
  return os;
}
struct KeyPressure {
  uint8_t channel;
  uint8_t key;
  uint8_t pressure;
};
inline std::ostream & operator << (std::ostream & os, KeyPressure const & n) {
  os << "KeyPressure(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.key) << ", " << static_cast<int>(n.pressure) << ")";
  return os;
}
struct ControlChange {
  uint8_t channel;
  uint8_t controller_number;
  uint8_t controller_value;
};
inline std::ostream & operator << (std::ostream & os, ControlChange const & n) {
  os << "ControlChange(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.controller_number) << ", " << static_cast<int>(n.controller_value) << ")";
  return os;
}
struct ProgramChange {
  uint8_t channel;
  uint8_t preset;
};
inline std::ostream & operator << (std::ostream & os, ProgramChange const & n) {
  os << "ProgramChange(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.preset) << ")";
  return os;
}
struct ChannelPressure {
  uint8_t channel;
  uint8_t pressure;
};
inline std::ostream & operator << (std::ostream & os, ChannelPressure const & n) {
  os << "ChannelPressure(" << static_cast<int>(n.channel) << ", " << static_cast<int>(n.pressure) << ")";
  return os;
}
struct PitchWheel {
  uint8_t channel;
  uint16_t pitchweel;
  
  // returns a value in [-1, 1)
  float getCenteredValue() const {
    return -1.f + 2.f * (static_cast<int>(pitchweel) - min) * factor;
  }
private:
  static constexpr int max = 0x7F7F; // Arturia minilab
  static constexpr int min = 0x0000; // Arturia minilab
  static constexpr float factor = 1.f / (max - min);
};
inline std::ostream & operator << (std::ostream & os, PitchWheel const & n) {
  os << "PitchWheel(" << static_cast<int>(n.channel) << ", " << n.pitchweel << ")";
  return os;
}

using Event = std::variant<
NoteOn,
NoteOff,
KeyPressure,
ControlChange,
ProgramChange,
ChannelPressure,
PitchWheel,
AllNotesOff
>;

// see 'msgToMidi' and
// https://hackage.haskell.org/package/Euterpea-2.0.2/src/Euterpea/IO/MIDI/MidiIO.lhs
inline
std::optional<Event>
decode(int32_t const message) {
  uint8_t const m = Pm_MessageStatus(message);
  uint8_t const d1 = Pm_MessageData1(message);
  uint8_t const d2 = Pm_MessageData2(message);
  
  uint8_t const c = m & 0x0F;
  
  switch((m & 0xF0) >> 4) {
    case 0x8:
      return NoteOff{ c, d1, d2 };
    case 0x9:
      if (!d2) { // 0 velocity
        return NoteOff{ c, d1, d2 };
      } else {
        return NoteOn{ c, d1, d2 };
      }
    case 0xA:
      return KeyPressure{ c, d1, d2 };
    case 0xB:
      return ControlChange{ c, d1, d2 };
    case 0xC:
      return ProgramChange{ c, d1 };
    case 0xD:
      return ChannelPressure{ c, d1 };
    case 0xE:
      return PitchWheel{ c, static_cast<uint16_t>(d1 + (static_cast<unsigned>(d2) << 8))};
    case 0xF:
      // SysEx event not handled
    default:
      return {};
  }
}

struct PortMidi {
  using SendFunc = folly::Function<void(Event, std::optional<uint64_t>)>;
  PortMidi(SendFunc);

  ~PortMidi();
  
  std::vector<MidiName> list_input_devices_names() const;

  void open_input_stream(MidiName const&);
  void close_input_stream();
  
  std::optional<MidiName> const & get_input_stream() const { return input_stream_name; }
 
  bool input_stream_has_data();
  
  void read_input_stream();

private:
  PortMidiStream * input_stream = nullptr;
  std::optional<MidiName> input_stream_name;
  
  SendFunc send;
  
  std::vector<PmEvent> buffer;
  
  std::optional<int> find_device_id(MidiName const&) const;
};

PortMidi::PortMidi(SendFunc s)
: send(std::move(s)) {
  buffer.reserve(256);
  
  PmError const err = Pm_Initialize();
  if (err != pmNoError) {
    throw std::runtime_error("failed to initialize portmidi");
  }
  // use the 1st one
  for (auto const & n : list_input_devices_names()) {
    open_input_stream(n);
    return;
  }
}

void PortMidi::open_input_stream(MidiName const & name) {
  close_input_stream();
  
  if (std::optional<int> device_id = find_device_id(name)) {
    if (PmError const err = Pm_OpenInput(&input_stream,
                                         *device_id,
                                         nullptr,
                                         0,
                                         nullptr,
                                         nullptr)) {
      std::cerr << "failed to open input stream : " << Pm_GetErrorText(err) << std::endl;
      return;
    }
    input_stream_name = name;
  } else {
    std::cerr << "failed to open input stream : Not found." << std::endl;
    return;
  }
}

void PortMidi::close_input_stream() {
  if (input_stream) {
    
    send(AllNotesOff{}, {});
    
    PmError const err = Pm_Close(input_stream);
    if (err != pmNoError) {
      std::cerr << "failed to close input stream : " << Pm_GetErrorText(err) << std::endl;
    }
    input_stream = nullptr;
    input_stream_name.reset();
  }
}

PortMidi::~PortMidi() {
  close_input_stream();
  {
    PmError const err = Pm_Terminate();
    if (err != pmNoError) {
      std::cerr << "failed to terminate portmidi : " << Pm_GetErrorText(err) << std::endl;
    }
  }
}

std::vector<MidiName> PortMidi::list_input_devices_names() const {
  std::vector<MidiName> res;
  int n_devices = Pm_CountDevices();
  res.reserve(n_devices);
  for (int i=0; i<n_devices; ++i) {
    const PmDeviceInfo* info = Pm_GetDeviceInfo(i);
    if (info->input) {
      res.push_back({info->interf, info->name});
    }
  }
  return res;
}

std::optional<int> PortMidi::find_device_id(MidiName const&n) const {
  int n_devices = Pm_CountDevices();
  for (int i=0; i<n_devices; ++i) {
    const PmDeviceInfo* info = Pm_GetDeviceInfo(i);
    if (info->input) {
      if (info->interf == n.interf && info->name == n.name) {
        return i;
      }
    }
  }
  return {};
}


bool PortMidi::input_stream_has_data() {
  PmError const info = Pm_Poll(input_stream);
  if (info == pmNoData) {
    return false;
  } else if(info == pmGotData) {
    return true;
  }
  std::cerr << "midi failed to poll : " << Pm_GetErrorText(info) << std::endl;
  return false;
}

void PortMidi::read_input_stream()
{
  buffer.resize(buffer.capacity());
  int const n = Pm_Read(input_stream,
                        buffer.data(),
                        static_cast<int>(buffer.capacity()));
  if (n == pmBufferOverflow) {

    send(AllNotesOff{}, {});

    buffer.clear();
  } else {
    Assert(n <= buffer.capacity());
    buffer.resize(n);
    
    for (auto const & e : buffer) {
      Assert(e.timestamp >= 0);
      if(auto value = decode(e.message)) {
        send(*value, e.timestamp);
      } else {
        // unhandled midi message
      }
      
      /*
       std::cout << "[midi] @" << e.timestamp << " ";
       if (!value) {
       // unhandled midi message
       std::cout << "unhandled midi message";
       } else {
       std::visit([](auto && d){ std::cout << d; },
       *value);
       }
       std::cout << std::endl;
       */
    }
  }
}

template<typename Q>
void listen_to_midi_input(std::atomic_bool & active,
                          Q & queue,
                          PortMidi & pm) {
  auto check_queue = [&pm, &queue] {
    bool worked = false;
    typename Q::value_type f;
    while(queue.try_pop(f)) {
      f(pm);
      worked = true;
    }
    return worked;
  };
  
  while(active) {
    if (!pm.get_input_stream()) {
      if (check_queue()) {
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (!pm.input_stream_has_data()) {
      if (check_queue()) {
        continue;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    pm.read_input_stream();
  }
}

} // NS
