#include "portmidi.h"

namespace imajuscule::audio::rtresynth::midi {

struct PortMidi {
  PortMidi();

  ~PortMidi();
 
  bool input_stream_has_data();
  
  bool read_input_stream(std::vector<PmEvent> & buffer);

private:
  PortMidiStream * input_stream = nullptr;

  int findInputDevice();
  
  void openInputStream(int device_id);
};

PortMidi::PortMidi() {
  PmError const err = Pm_Initialize();
  if (err != pmNoError) {
    throw std::runtime_error("failed to initialize portmidi");
  }
  openInputStream(findInputDevice());
}

PortMidi::~PortMidi() {
  if (input_stream) {
    PmError const err = Pm_Close(input_stream);
    if (err != pmNoError) {
      std::cerr << "failed to close input stream : " << Pm_GetErrorText(err) << std::endl;
    }
    input_stream = nullptr;
  }
  {
    PmError const err = Pm_Terminate();
    if (err != pmNoError) {
      std::cerr << "failed to terminate portmidi : " << Pm_GetErrorText(err) << std::endl;
    }
  }
}

int PortMidi::findInputDevice() {
  int n_devices = Pm_CountDevices();
  std::cout << n_devices << " midi devices:" << std::endl;
  std::vector<int> inputs;
  for (int i=0; i<n_devices; ++i) {
    const PmDeviceInfo* info = Pm_GetDeviceInfo(i);
    std::cout << "input " << info->input << " output " << info->output << " " << info->interf << ":" << info->name << std::endl;
    if (info->input) {
      //if (!isInfixOf("through", info->name))
      inputs.push_back(i);
    }
  }
  
  if (inputs.size() == 1) {
    std::cout << "One midi input available" << std::endl;
    return inputs.front();
  }
  if (inputs.empty()) {
    throw std::runtime_error("No midi input available");
  } else {
    throw std::runtime_error(std::to_string(inputs.size()) + " midi inputs available (don't know which one to use)");
  }
}

void PortMidi::openInputStream(int device_id) {
  PmError const err = Pm_OpenInput(&input_stream,
                                   device_id,
                                   nullptr,
                                   0,
                                   nullptr,
                                   nullptr);
  if (err == pmNoError) {
    return;
  }
  throw std::runtime_error("failed to open input stream : " + std::string(Pm_GetErrorText(err)));
}

bool PortMidi::input_stream_has_data() {
  PmError const info = Pm_Poll(input_stream);
  if (info == pmNoData) {
    return false;
  } else if(info == pmGotData) {
    return true;
  }
  throw std::runtime_error("failed to poll : " + std::string(Pm_GetErrorText(info)));
}

bool PortMidi::read_input_stream(std::vector<PmEvent> & buffer)
{
  buffer.resize(buffer.capacity());
  int const n = Pm_Read(input_stream,
                        buffer.data(),
                        static_cast<int>(buffer.capacity()));
  if (n == pmBufferOverflow) {
    buffer.clear();
    return false;
  } else {
    Assert(n <= buffer.capacity());
    buffer.resize(n);
    return true;
  }
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
PitchWheel
>;

// see 'msgToMidi' and
// https://hackage.haskell.org/package/Euterpea-2.0.2/src/Euterpea/IO/MIDI/MidiIO.lhs
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

template<typename F, typename G>
void listen_to_midi_input(std::atomic_bool & active,
                          F && onReceivedMidiEvent,
                          G && onLostMidiEvents) {
  PortMidi pm;
  
  std::vector<PmEvent> buffer;
  buffer.reserve(256);

  while(active) {
    if (!pm.input_stream_has_data()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    if (!pm.read_input_stream(buffer)) {
      // buffer input overflow
      Assert(buffer.empty());
      onLostMidiEvents();
    } else {
      for (auto const & e : buffer) {
        Assert(e.timestamp >= 0);
        if(auto value = decode(e.message)) {
          onReceivedMidiEvent(*value,
                              e.timestamp);
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
}

} // NS
