
namespace imajuscule::audio {

namespace audioelement {

template<int nAudioOut, AudioOutPolicy policy, SoundEngineMode MODE>
using Voice = imajuscule::audio::voice::Impl_<
policy,
nAudioOut,
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

  static constexpr int nAudioOut = 2;

  using Ctxt = Context<
  AudioPlatform::PortAudio,
  Features::JustOut,
  SimpleAudioOutContext<
  nAudioOut,
  audioEnginePolicy
  >
  >;

  constexpr audioelement::SoundEngineMode mode =
  audioelement::SoundEngineMode::ROBOTS;
  //audioelement::SoundEngineMode::WIND;
  //audioelement::SoundEngineMode::BIRDS;

  using Synth = audioelement::Voice<nAudioOut, audioEnginePolicy, mode>;

  Ctxt ctxt(GlobalAudioLock<audioEnginePolicy>::get(),
            Synth::n_channels * 4 /* one shot */,
            1 /* a single compute is needed (global for the synth)*/);
  
  if (!ctxt.doInit(0.008, sample_rate)) {
    throw std::runtime_error("ctxt init failed");
  }

  Synth synth;

  synth.initialize(ctxt.getStepper());

  auto & v = synth;
  v.initializeSlow(); // does something only the 1st time

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::optional<NoteId> noteid;

  const int program_index = std::min(6,
                                     v.countPrograms()-1);
  // for 7 (Light rain in a car) we use 1 ms per cb (filter order is 89 !!!).
  // for 6 (Light rain) we use 0.2 ms
  v.useProgram(program_index); // keep it first as it reinitializes params
  std::cout << "using program '" << v.getProgram(program_index).name << "'" << std::endl;


  /*
  {
    noteid = mk_note_id();
    float volume = 0.1f;
    float frequency = 200.f;
    auto const res  = synth.onEvent(sample_rate,
                                     mkNoteOn(*noteid,
                                              frequency,
                                              volume),
   ctxt.getStepper(),
   ctxt.getStepper(),
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

      auto res = synth.onEvent(sample_rate,
                               mkNoteOff(*noteid),
                               ctxt.getStepper(),
                               ctxt.getStepper(),
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
    float volume = 1.f;
    float frequency = 200.f;

    auto const res  = synth.onEvent(sample_rate,
                                    mkNoteOn(*noteid,
                                             frequency,
                                             volume),
                                    ctxt.getStepper(),
                                    ctxt.getStepper(),
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

  synth.finalize(ctxt.getStepper());

  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // so that the audio produced during finalization has a chance to be played

  ctxt.doTearDown();
}

} // NS

int main() {
  imajuscule::audio::birds(96000);
  return 0;
}
