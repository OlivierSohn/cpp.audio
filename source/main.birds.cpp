
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
  
  Ctxt ctxt(sample_rate);
  if (!ctxt.Init(0.006)) {
    throw std::runtime_error("ctxt init failed");
  }
  auto & channel_handler = ctxt.getChannelHandler();
  
  using Synth = audioelement::Voice<Ctxt, audioelement::SoundEngineMode::BIRDS>;
  
  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  std::array<mnc_buffer,n_mnc> buffers;
  
  auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                              std::min(n_mnc,
                                                                                                       static_cast<int>(std::numeric_limits<uint8_t>::max())));
  NoXFadeChans & channels = channels_;
  
  Synth synth(sample_rate, buffers);
  
  synth.initialize(channels);
  
  auto & v = synth;
  v.initializeSlow(); // does something only the 1st time
  
  synth.forEachElems([](auto & e) {
    // no need to setup anything
  });
  
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /*
  auto res = synth.onEvent2(mkNoteChange(ref,
                                         vol,
                                         fs[*match_idx].freq),
                            channel_handler,
                            channels,
                            {});
  if (res != onEventResult::OK) {
    throw std::logic_error("dropped note change");
  }
  std::cout << n << ": pitch " << ref.getFrequency() << " newpitch " << fs[*match_idx].freq << " Vol " << vol  << " initial_vol " << initial_velocity[i] << " " << res << std::endl;
  */
  
  std::optional<ReferenceFrequencyHerz> ref;

  v.useProgram(8); // keep it first as it reinitializes params
  std::cout << "using program 8" << std::endl;
  
  {
    ref = ReferenceFrequencyHerz(200);
    float volume = 0.1f;
    
    auto const res  = synth.onEvent2(mkNoteOn(*ref,
                                              volume),
                                     channel_handler,
                                     channels,
                                     {});
    std::cout << "pitch " << ref->getFrequency() << " vol " << volume << " " << res << std::endl;
    while(true) {
      std::this_thread::yield();
    }
  }
  
  while(true) {
    if(ref) {
      auto res = synth.onEvent2(mkNoteOff(*ref),
                                channel_handler,
                                channels,
                                {});
      std::cout << "XXX pitch " << ref->getFrequency() << " " << res << std::endl;
      ref.reset();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "enter char or 'q':" << std::endl;
    char ch;
    std::cin >> ch;
    
    std::cout << "pressed:" << ch << std::endl;
    if (ch == 'q') {
      std::cout << "quitting" << ch << std::endl;
      break;
    }

    int const n = ch - '0';
    if (n >= 0 && n < 10 && n < v.countPrograms()) {
      v.useProgram(n); // keep it first as it reinitializes params
      std::cout << "using program " << n << std::endl;
    }
    /*v.set_random_pan(false);
     v.set_random(b.random);
     if(!b.random) {
     v.set_seed(b.seed);
     }
     v.set_gain(b.gain);
     v.set_pan(b.pan);
     v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!
     */

    ref = ReferenceFrequencyHerz(200);
    float volume = 0.1f;
    
    auto const res  = synth.onEvent2(mkNoteOn(*ref,
                                              volume),
                                     channel_handler,
                                     channels,
                                     {});
    std::cout << "pitch " << ref->getFrequency() << " vol " << volume << " " << res << std::endl;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  ctxt.onApplicationShouldClose();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ctxt.TearDown();
}

} // NS

int main() {
  imajuscule::audio::birds(44100);
  return 0;
}
