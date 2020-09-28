

namespace imajuscule::audio {

namespace audioelement {

template <typename Ctxt, typename T>
using synthOf = sine::Synth < // the name of the namespace is misleading : it can handle all kinds of oscillators
Ctxt::policy
, Ctxt::nAudioOut
, XfadePolicy::SkipXfade
, FinalAudioElement<
    VolumeAdjusted<
      Enveloped<
        FreqCtrl_<
          OscillatorAlgo<T, eNormalizePolicy::FAST>,
          InterpolatedFreq<T>
        >,
        AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
      >
    >
  >
, SynchronizePhase::Yes
, DefaultStartPhase::Random
, true
, EventIterator<IEventList>
, NoteOnEvent
, NoteOffEvent
, 128  // lots of voices
>;

} // NS audioelement

void rtResynth(int const sample_rate) {
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
  
  using Synth = audioelement::synthOf<Ctxt, double>;
  
  static constexpr auto n_mnc = Synth::n_channels;
  using mnc_buffer = typename Synth::MonoNoteChannel::buffer_t;
  std::array<mnc_buffer,n_mnc> buffers;
  
  static constexpr auto n_max_orchestrator_per_channel = 0; // we don't use orchestrators
  auto [channels_,remover] = channel_handler.getChannels().getChannelsNoXFade().emplace_front(channel_handler.get_lock_policy(),
                                                                                             std::min(n_mnc,
                                                                                                      static_cast<int>(std::numeric_limits<uint8_t>::max())),
                                                                                             n_max_orchestrator_per_channel);
  NoXFadeChans & channels = channels_;

  Synth synth(sample_rate, buffers);
  
  synth.initialize(channels);

  int constexpr window_size = 800;
  int constexpr window_center_stride = 400;
  int constexpr windowoverlapp = std::max(0, window_size - window_center_stride);

  /*
   Below, we will describe what is likely to happen with threads jitter.
   
   To simplify, we will assume that each audio input callback call handles 'widow_center_stride' input samples,
   and that the input signal contains 4 events (0, 1, 2, 3) which are exactly 'window_center_stride' apart, like so:
  
   input signal: 0--1--2--3--

   We have 3 threads running: the audio input thread, the processing thread and the audio output thread.
   The audio input thread writes in a queue read by the process thread, and
   the process thread writes in a queue read by the audio output thread.

   In an ideal world, all threads run at the same regular intervals, with no overlap:

   audio in thread :    0--      1--      2--      3--       // we represent both the input signal and the time it takes for the input thread to run
   process thread  :       ---      ---      ---      ---    // we represent the time it takes for the thread to run
   audio out thread:          0--      1--      2--      3-- // we represent both the output signal and the time it takes for the output thread to run
   
   -> output signal : 0--1--2--3--
   
   In this ideal case, the events maintain their timings. But let's see how we can diverge from the ideal case:
   
   A. We can imagine that the 'process' thread has jitter:
   
   audio in thread :    0--      1--      2--      3--
   process thread  :       ---                ---     ---
   audio out thread:          0--      ---      1-2      3--
   
   -> output signal : 0-----1-23--
   
   B. We can imagine that the 'process' thread runs less often that the audio threads:
   
   audio in thread :    0--      1--      2--      3--
   process thread  :       ---               ---
   audio out thread:          0--      ---      12-      3--
   
   -> output signal : 0-----12-3--
   
   C. we can imagine that the audio in thread has jitter:
   
   audio in thread :    0--      1--            2--3--
   process thread  :       ---      ---      ---      ---
   audio out thread:          0--      1--      ---      23-
   
   -> output signal : 0--1-----23-
   
   D. we can imagine that the audio out thread has jitter:
   
   audio in thread :    0--      1--      2--      3--
   process thread  :       ---      ---      ---      ---
   audio out thread:          0--      1--            2-3---
   
   -> output signal : 0--1--2-3---

   In the real world, it is likely that A., B. C. and D. happen simulataneously.
   
   So to cope with this, we would need to introduce some delay and use midi timestamps to accurately
   trigger the relevant actions at the right time in the audio out thread.
   */

  synth.forEachElems([](auto & e) {
    // envelope (now that we track the volume, we only need a minimal envelope)

    // TODO try the following effect via volume tracking only (not evelope):
    // make the volume decrease slower than in reality (notion of meta envelope)

    e.algo.editEnvelope().setAHDSR(audioelement::AHDSR{
      0, itp::LINEAR,
      0,
      0, itp::LINEAR,
      0, itp::LINEAR,
      1.f
    }, SAMPLE_RATE);

    // volume adjustment
    
    e.algo.setMaxFilterIncrement(2. / static_cast<double>(window_center_stride));
    
    // frequency control

    e.algo.getOsc().getAlgo().getCtrl().setup(window_center_stride,
                                              itp::LINEAR);
  });

  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ SAMPLE,
  /* A = */ std::allocator<SAMPLE>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue input_queue(sample_rate); // one second of input can fit in the queue

  auto onInput = [&input_queue](const SAMPLE * buf, int nFrames){
    for (int i=0; i<nFrames; ++i) {
      input_queue.push(buf[i]);
    }
  };
  
  AudioInput<AudioPlatform::PortAudio> input;
  if (!input.Init(onInput, sample_rate)) {
    throw std::runtime_error("input init failed");
  }
    
  std::atomic_bool thread_resynth_active(true);
  std::thread thread_resynth([&input_queue,
                              &thread_resynth_active,
                              sample_rate,
                              &channel_handler,
                              &channels,
                              &synth](){
    std::vector<double> half_window = half_gaussian_window<double>(4, window_size/2);
    normalize_window(half_window);

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
    struct Data {
      std::optional<double> midipitch;
      double volume;
    };
    std::vector<Data> freqmags_data;
    
    // This is questionable : we have at most one active oscillator per midi pitch,
    // eventhough the frequencies can vary.
    // TODO instead we should use a std::set<ReferenceFrequencyHerz> to remove this limitation
    std::vector<std::optional<ReferenceFrequencyHerz>> midi_pitches;
    std::vector<bool> fs_used;
    midi_pitches.resize(200);
    fs_used.reserve(200);
    std::vector<float> initial_velocity, cur_freq;
    initial_velocity.resize(200);
    cur_freq.resize(200);
    
    struct FreqMatch {
      int index; // in midi_pitches
      float freq_ratio; // >= 1.f
    };
    std::vector<std::optional<FreqMatch>> matches;
    matches.reserve(200);

    Midi midi;
    
    int n = 0;
    auto step = [&n,
                 &synth,
                 &channel_handler,
                 &channels,
                 &midi_pitches,
                 &initial_velocity,
                 &cur_freq,
                 &matches,
                 &fs_used,
                 &freqmags_data,
                 &midi](std::vector<FreqMag<double>> const & fs) {
      ++n;

      constexpr double min_volume = 0.01;
      
      freqmags_data.clear();
      for (auto const & f : fs) {
        freqmags_data.push_back({
          midi.frequency_to_midi_pitch(f.freq),
          DbToMag<double>()(f.mag_db)
        });
      }
      
      matches.clear();
      matches.resize(fs.size());
      fs_used.clear();
      fs_used.resize(fs.size(), false);

      // we need to find the best match between
      // - frequencies currently played (represented by index of 'midi_pitches')
      // - new frequencies (represented by index of 'fs'
      for (int i=0, sz = midi_pitches.size(); i<sz; ++i) {
        if (!midi_pitches[i]) {
          continue;
        }
        double const played_frequency = cur_freq[i];
        std::optional<int> min_freq_dist, best;
        int idx = -1;
        for (auto const & f : fs) {
          ++idx;
          if (f.freq <= 0) {
            continue;
          }
          if (freqmags_data[idx].volume < min_volume) {
            continue;
          }
          double const diff = std::abs(f.freq - played_frequency);
          if (!min_freq_dist || *min_freq_dist > diff) {
            min_freq_dist = diff;
            best = idx;
          }
        }
        if (best) {
          float const ratio = std::max(played_frequency, fs[*best].freq) / std::min(played_frequency, fs[*best].freq);
          Assert(ratio >= 1.f);
          if (ratio < 1.1f) { // TODO adjust magic number
            // frequencies are close enough so that we can consider this to be a note change.
            if (!matches[*best] || matches[*best]->freq_ratio > ratio) {
              matches[*best] = FreqMatch{i, ratio};
            }
          }
        }
      }

      bool changed = false;
      
      for (int i=0, sz = midi_pitches.size(); i<sz; ++i) {
        if (!midi_pitches[i]) {
          continue;
        }
        
        // find the match
        std::optional<int> match_idx;
        {
          int idx = -1;
          for (auto const & match : matches) {
            ++idx;
            if (match && (match->index == i)) {
              match_idx = idx;
              break;
            }
          }
        }
        float vol = 0.;
        if (match_idx) {
          // having a match is not enough, we also need to see if there are frequencies, in the vicinity of the match,
          // that have not been associated yet with another note.
          
          // sum the volumes of the new freqs that correspond to the matched frequency (with half tone resolution)
          int idx = -1;
          for (auto const & [pitch, volume] : freqmags_data) {
            ++idx;
            if (!pitch) {
              continue;
            }
            if (fs_used[idx]) {
              continue;
            }
            float const ratio = std::max(fs[idx].freq, fs[*match_idx].freq) / std::min(fs[idx].freq, fs[*match_idx].freq);
            if (ratio > midi.getHalfToneRatio()) {
              continue;
            }
            fs_used[idx] = true;
            if (vol) {
              std::cout << " sum " << volume << std::endl;
            }
            vol += volume;
          }
        }
        
        ReferenceFrequencyHerz const & ref = *midi_pitches[i];
        
        if (!vol) {
          auto res = synth.onEvent2(mkNoteOff(ref),
                                    channel_handler,
                                    channels,
                                    {});
          std::cout << n << ": XXX pitch " << ref.getFrequency() << " " << res << std::endl;
          if (res != onEventResult::OK) {
            throw std::logic_error("dropped note off");
          }
          midi_pitches[i].reset();
          changed = true;
        } else {
          // we have a match, and a volume for that match
          Assert(match_idx);
          auto res = synth.onEvent2(mkNoteChange(ref,
                                                 vol,
                                                 fs[*match_idx].freq),
                                    channel_handler,
                                    channels,
                                    {});
          if (res != onEventResult::OK) {
            throw std::logic_error("dropped note change");
          }
          cur_freq[i] = fs[*match_idx].freq;
          std::cout << n << ": pitch " << ref.getFrequency() << " newpitch " << fs[*match_idx].freq << " Vol " << vol  << " initial_vol " << initial_velocity[i] << " " << res << std::endl;
        }
      }

      // If the frequency is not played yet, start a new note
      int idx = -1;
      for (auto const & [pitch, volume] : freqmags_data) {
        ++idx;
        if (fs_used[idx]) {
          continue;
        }
        if (!pitch) {
          continue;
        }
        if (volume < min_volume) {
          continue;
        }
        int i = static_cast<int>(*pitch + 0.5);
        Assert(i >= 0);
        Assert(i < midi_pitches.size());
        if (midi_pitches[i]) {
          // drop it, a frequency for this midi pitch is already used (TODO we should sum the volumes instead?)
          std::cout << "drop" << std::endl;
          continue;
        }
        ReferenceFrequencyHerz ref(fs[idx].freq);
        auto const res  = synth.onEvent2(mkNoteOn(ref,
                                                  volume),
                                         channel_handler,
                                         channels,
                                         {});
        std::cout << n << ": pitch " << ref.getFrequency() << " vol " << volume << " " << res << std::endl;
        if (res == onEventResult::OK) {
          midi_pitches[i] = ref;
          initial_velocity[i] = volume;
          cur_freq[i] = fs[idx].freq;
          changed = true;
        } else {
          // dropped note:
          //
        }
      }
      
      if (changed) {
        for (int i=0, sz = midi_pitches.size(); i<sz; ++i) {
          if (!midi_pitches[i]) {
            continue;
          }
          std::cout << " " << i;
        }
        std::cout << std::endl;
        for (int i=0, sz = midi_pitches.size(); i<sz; ++i) {
          if (!midi_pitches[i]) {
            std::cout << " ";
          } else {
            std::cout << "|";
          }
        }
        std::cout << std::endl;
      }

      // later:

      // If similar amplitude and frequency, use sweep with low passed frequency, using window_center_stride as time constant.
    };

    while (thread_resynth_active) {
      while (input_queue.try_pop(samples[end])) {
        ++ end;
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
  imajuscule::audio::rtResynth(44100);
  return 0;
}
