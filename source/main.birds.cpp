
namespace imajuscule::audio {

namespace audioelement {

template<typename Ctxt, SoundEngineMode MODE>
using Voice = imajuscule::audio::voice::Impl_<
Ctxt::policy,
Ctxt::nAudioOut,
MODE,
true,
std::vector<float>,
EventIterator,
ProcessData
>;

} // NS audioelement

void birds(int const sample_rate) {
  auto mk_note_id = [] {
    static int64_t i = 0;
    ++i;
    return NoteId{i};
  };

  static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

  using AllChans = ChannelsVecAggregate< 2, audioEnginePolicy >;

  using NoXFadeChans = typename AllChans::NoXFadeChans;
  using XFadeChans = typename AllChans::XFadeChans;

  using ChannelHandler = outputDataBase< AllChans, ReverbType::Realtime_Synchronous >;

  using Ctxt = AudioOutContext<
  ChannelHandler,
  Features::JustOut,
  AudioPlatform::PortAudio
  >;

  Ctxt ctxt;
  if (!ctxt.Init(sample_rate, 0.008)) {
    throw std::runtime_error("ctxt init failed");
  }
  auto & channel_handler = ctxt.getChannelHandler();

  constexpr audioelement::SoundEngineMode mode =
  //audioelement::SoundEngineMode::ROBOTS;
  audioelement::SoundEngineMode::WIND;
  //audioelement::SoundEngineMode::BIRDS;

  using Synth = audioelement::Voice<Ctxt, mode>;

  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  std::array<mnc_buffer,n_mnc> buffers;

  auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                              std::min(n_mnc,
                                                                                                       static_cast<int>(std::numeric_limits<uint8_t>::max())));
  NoXFadeChans & channels = channels_;

  Synth synth(buffers);

  synth.initialize(channels);

  auto & v = synth;
  v.initializeSlow(); // does something only the 1st time

  synth.forEachElems([](auto & e) {
    // no need to setup anything
  });


  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::optional<NoteId> noteid;

  constexpr int program_index = 5;
  // for 7 (Light rain in a car) we use 1 ms per cb (filter order is 89 !!!).
  // for 6 (Light rain) we use 0.2 ms
  v.useProgram(program_index); // keep it first as it reinitializes params
  std::cout << "using program '" << v.getProgram(program_index).name << "'" << std::endl;


  /*
  {
    noteid = mk_note_id();
    float volume = 0.1f;
    float frequency = 200.f;
    auto const res  = synth.onEvent2(sample_rate,
                                     mkNoteOn(*noteid,
                                              frequency,
                                              volume),
                                     channel_handler,
                                     channels,
                                     {});
    std::cout << *noteid << ": pitch " << frequency << " vol " << volume << " " << res << std::endl;
    while(true) {
      std::this_thread::yield();
    }
  }
  //*/

  while(true) {

    if(noteid) {
      std::cout << "enter number to change program, or letter to play note, or 'q' to quit:" << std::endl;
      std::string str;
      std::cin >> str;

      std::cout << "pressed:" << str << std::endl;
      if (str == "q") {
        std::cout << "quitting" << std::endl;
        break;
      }

      auto res = synth.onEvent2(sample_rate,
                                mkNoteOff(*noteid),
                                channel_handler,
                                channels,
                                {});
      std::cout << "XXX " << *noteid << " " << res << std::endl;
      noteid.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
      try {
        int const n = std::stoi(str);
        if (n >= 0 && n < v.countPrograms()) {
          v.useProgram(n); // keep it first as it reinitializes params
          std::cout << "using program '" << v.getProgram(n).name << "'" << std::endl;
        }
      } catch(std::invalid_argument const &) {
        std::cout << "not a number" << std::endl;
      }
    }

    /*v.set_random_pan(false);
     v.set_random(b.random);
     if(!b.random) {
     v.set_seed(b.seed);
     }
     v.set_gain(b.gain);
     v.set_pan(b.pan);
     */
    v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!

    noteid = mk_note_id();
    float volume = 0.1f;
    float frequency = 200.f;

    auto const res  = synth.onEvent2(sample_rate,
                                     mkNoteOn(*noteid,
                                              frequency,
                                              volume),
                                     channel_handler,
                                     channels,
                                     {});
    std::cout << *noteid << ": pitch " << frequency << " vol " << volume << " " << res << std::endl;

    /*switch(mode) {
      case audioelement::SoundEngineMode::WIND:
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        break;
      case audioelement::SoundEngineMode::BIRDS:
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        break;
    }*/
  }

  ctxt.onApplicationShouldClose();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ctxt.TearDown();
}

} // NS

int main() {
  imajuscule::audio::birds(96000);
  return 0;
}
