

namespace imajuscule::audio {

namespace audioelement {

template <typename Ctxt>
using synthOf = sine::Synth < // the name of the namespace is misleading : it can handle all kinds of oscillators
Ctxt::policy
, Ctxt::nAudioOut
, XfadePolicy::SkipXfade
, FinalAudioElement<
    Enveloped<
      OscillatorAlgo< double, eNormalizePolicy::FAST >,
      AHDSREnvelope<Atomicity::Yes, double, EnvelopeRelease::WaitForKeyRelease>
    >
  >
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator<IEventList>
, NoteOnEvent
, NoteOffEvent>;

} // NS audioelement

std::optional<double> frequency_to_midi_pitch(double freq) {
  if (freq <= 0) {
    return {};
  }
  return 69. + 12. * std::log2(freq/440.);
}

void rtResynth() {
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

  Ctxt ctxt(SAMPLE_RATE);
  if (!ctxt.Init(0.006)) {
    throw std::runtime_error("ctxt init failed");
  }
  auto & channel_handler = ctxt.getChannelHandler();
  
  using Synth = audioelement::synthOf<Ctxt>;
  
  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  std::array<mnc_buffer,n_mnc> buffers;
  
  static constexpr auto n_max_orchestrator_per_channel = 0; // we don't use orchestrators
  auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                             std::min(n_mnc,
                                                                                                      static_cast<int>(std::numeric_limits<uint8_t>::max())),
                                                                                             n_max_orchestrator_per_channel);
  NoXFadeChans & channels = channels_;

  audioelement::synthOf<Ctxt> synth(buffers);
  
  synth.initialize(channels);
  
  synth.forEachElems([](auto & e) {
    e.algo.editEnvelope().setAHDSR(audioelement::AHDSR{
      1000, itp::LINEAR,
      0,
      1000, itp::LINEAR,
      10000, itp::LINEAR,
      0.7f
    }, SAMPLE_RATE);
  });

  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ SAMPLE,
  /* A = */ std::allocator<SAMPLE>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue input_queue(44100); // one second of input can fit in the queue

  auto onInput = [&input_queue](const SAMPLE * buf, int nFrames){
    for (int i=0; i<nFrames; ++i) {
      input_queue.push(buf[i]);
    }
  };
  
  AudioInput<AudioPlatform::PortAudio> input;
  if (!input.Init(onInput, SAMPLE_RATE)) {
    throw std::runtime_error("input init failed");
  }
  
  int const sample_rate = input.getSampleRate();
  
  std::atomic_bool thread_resynth_active(true);
  std::thread thread_resynth([&input_queue,
                              &thread_resynth_active,
                              sample_rate,
                              &channel_handler,
                              &channels,
                              &synth](){
    
    int constexpr window_size = 800;
    int constexpr window_center_stride = 400;
    int constexpr windowoverlapp = std::max(0, window_size - window_center_stride);
    
    std::vector<double> half_window = half_gaussian_window<double>(4, window_size/2);
    
    auto process = [&half_window](int const sample_rate,
                                  auto from,
                                  auto to,
                                  FrequenciesSqMag<double> & frequencies_sqmag,
                                  std::vector<FreqMag<double>> & freqmags) {
      int constexpr zero_padding_factor = 1;
      int constexpr windowed_signal_stride = 1;
      
      findFrequenciesSqMag(from,
                           to,
                           windowed_signal_stride,
                           half_window,
                           zero_padding_factor,
                           frequencies_sqmag);

      extractLocalMaxFreqsMags(sample_rate / windowed_signal_stride,
                               frequencies_sqmag,
                               SqMagToDb<double>(),
                               freqmags);
    };
        
    std::vector<double> samples;
    samples.resize(window_size, {});
    int end = 0;
 
    FrequenciesSqMag<double> frequencies_sqmag;
    std::vector<FreqMag<double>> freqmags;
    std::vector<std::optional<double>> freqmags_midipitches;
    
    std::vector<bool> midi_pitches;
    midi_pitches.resize(200, false);

    int n = 0;
    auto step = [&n,
                 &synth,
                 &channel_handler,
                 &channels,
                 &midi_pitches,
                 &freqmags_midipitches](std::vector<FreqMag<double>> const & fs) {
      ++n;
      
      constexpr double min_volume = 0.01;
      
      freqmags_midipitches.clear();
      for (auto const & f : fs) {
        freqmags_midipitches.push_back(frequency_to_midi_pitch(f.freq));
      }
      
      // If a frequency that is playing is not present, stop it
      for (int i=0, sz = midi_pitches.size(); i<sz; ++i) {
        if (!midi_pitches[i]) {
          continue;
        }
        bool found = false;
        int idx = -1;
        for (auto const & pitch : freqmags_midipitches) {
          ++idx;
          if (!pitch) {
            continue;
          }
          int ipitch = static_cast<int>(*pitch + 0.5);
          Assert(ipitch >= 0);
          Assert(ipitch < midi_pitches.size());
          if (ipitch == i) {
            double const volume = 0.1 * DbToSqMag<double>()(fs[idx].mag);
            if (volume < min_volume) {
              continue;
            }
            found = true;
            break;
          }
        }
        if (found) {
          continue;
        }
        auto res = synth.onEvent2(mkNoteOff(i),
                                  channel_handler,
                                  channels,
                                  {});
        std::cout << n << ": XXX pitch " << i << " " << res << std::endl;
        midi_pitches[i] = false;
      }

      // If the frequency is not played yet, start a new note
      int idx=-1;
      for (auto const & pitch : freqmags_midipitches) {
        ++idx;
        if (!pitch) {
          continue;
        }
        int i = static_cast<int>(*pitch + 0.5);
        Assert(i >= 0);
        Assert(i < midi_pitches.size());
        if (midi_pitches[i]) {
          continue;
        }
        double const volume = 0.1 * DbToSqMag<double>()(fs[idx].mag);
        if (volume < min_volume) {
          continue;
        }
        auto const res  = synth.onEvent2(mkNoteOn(i,
                                                  volume),
                                         channel_handler,
                                         channels,
                                         {});
        std::cout << n << ": pitch " << i << " vol " << volume << " " << res << std::endl;
        midi_pitches[i] = true;
      }
            
      // later:

      // If the frequency is already played, adjust the volume

      // If similar amplitude and frequency, use sweep with low passed frequency, using window_center_stride as time constant.
    };

    while (thread_resynth_active) {
      while (input_queue.try_pop(samples[end])) {
        ++ end;
        if (end == window_size) {
          break;
        }
      }
      if (end == window_size) {
        process(sample_rate,
                samples.begin(),
                samples.begin() + end,
                frequencies_sqmag,
                freqmags);
        step(freqmags);
        int const offset = window_size-windowoverlapp;
        for (end=0; end<windowoverlapp; ++end) {
          samples[end] = samples[end + offset];
        }
      }
    }
  });

  std::string end;
  std::cin >> end;
  
  thread_resynth_active = false;
  thread_resynth.join();

  if (!input.Teardown()) {
    throw std::runtime_error("input teardown failed");
  }
  ctxt.onApplicationShouldClose();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ctxt.TearDown();
}

} // NS

int main() {
  imajuscule::audio::rtResynth();
  return 0;
}
