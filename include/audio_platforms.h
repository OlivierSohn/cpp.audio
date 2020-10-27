

namespace imajuscule::audio {

enum class AudioPlatform {
  PortAudio,
  AudioUnits
};

enum class Features {
  JustOut,
  InAndOut
};

// for output:
template<AudioPlatform A, Features F>
struct Context;

// for input:
template<AudioPlatform A>
struct AudioInput;

// for input and output in the same stream
template<AudioPlatform A>
struct FullDuplexContext;

// output callback:
using PlayF = std::function<void(SAMPLE *,
                                 int,
                                 uint64_t const)>;
// input callback:
// Note that we could use PlayF for input too, but there is currently no use case where we need the time of inputs
using RecordF = std::function<void(const SAMPLE*,
                                   int /* num frames */)>;


constexpr auto initial_n_audio_cb_frames = -1;
// 'n_audio_cb_frames' is initially 'initial_n_audio_cb_frames', until the audio callback sets the value.
extern std::atomic<int32_t> n_audio_cb_frames;
static_assert(std::atomic<int32_t>::is_always_lock_free);

extern int wait_for_first_n_audio_cb_frames();

/* Debug utility to write a wav file asynchronously */
struct AsyncWavWriter {
  static constexpr float queueCapacityInSecondsOfAudioSignal = 1.;
  
  AsyncWavWriter(int nAudioChans, int sampleRate, std::string const & prefix)
  : n_audio_chans(nAudioChans)
  , sample_rate(sampleRate)
  , queue(queueCapacityInSecondsOfAudioSignal * (sampleRate * nAudioChans))
  , active(true) {
    thread = std::make_unique<std::thread>([this, prefix]() {
    auto rootDir = "/Users/Olivier/dev/hs.hamazed/";
#if 0
    // verify interpolation curves
    {
      std::vector<double> v;
      v.reserve(10000);
      for(auto i : itp::interpolation_traversal().realValues())
      {
        auto const itp_type = static_cast<itp::interpolation>(i);
        v.clear();
        for (int i = 0; i<=10000; ++i) {
          double const x = static_cast<double>(i) / 10000;
          v.push_back(itp::interpolate(itp_type, 0., x, 1., -0.5, 0.5));
        }
        write_wav(rootDir,
                  std::string(itp::interpolationInfo(itp_type)) + ".wav",
                  std::vector<std::vector<double>>{v},
                  100);
      }
    }
#endif
    auto header = pcm(WaveFormat::IEEE_FLOAT,
                      sample_rate,
                      numberToNChannels(n_audio_chans),
                      AudioSample<SAMPLE>::format);
    
    std::string const filename = prefix + ".wav";
    
    WAVWriter writer(rootDir,
                     filename,
                     header);
    auto res = writer.Initialize();
    
    if(ILE_SUCCESS != res) {
      LG(ERR, "Audio debug : failed to open '%s' in '%s' to write audio", filename.c_str(), rootDir);
      return;
    } else {
      LG(INFO, "Audio debug : opened '%s' in '%s' to write audio", filename.c_str(), rootDir);
    }
    
    while (true) {
      SAMPLE val;
      while (queue.try_pop(val)) {
        writer.writeSample(val);
      }
      if (!active) {
        break;
      }
      // sleep instead of yield to avoid 100% cpu usage.
      // The queue can hold one second of audio, so sleeping 10 milliseconds
      // will not be too much.
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(queueCapacityInSecondsOfAudioSignal * 1000 / 100.)));
    }
    });
  }
  

  template<typename T>
  void sync_feed_frames(T * buf, int numFrames) {
    for (int i=0; i<numFrames; ++i) {
      sync_feed_frame(&buf[n_audio_chans*i]);
    }
  }
  template<typename T>
  void sync_feed_frame(T * buf) {
    for (int j=0; j<n_audio_chans; ++j) {
      if (!queue.try_push(buf[j])) {
        LG(ERR, "Audio debug : dropped sample");
      }
    }
  }
  
  ~AsyncWavWriter() {
    active = false;
    thread->join();
  }

private:
  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ SAMPLE,
  /* A = */ std::allocator<SAMPLE>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue queue;
  std::atomic_bool active;
  std::unique_ptr<std::thread> thread;

  int const n_audio_chans;
  int const sample_rate;
};

/* Debug utility to asynchronously log messages coming from the realtime threads */
template<typename Data>
struct AsyncLogger {
  static constexpr float queueCapacityInSecondsOfAudioSignal = 1.;
  static constexpr auto prefix = "!!! ";
  
  AsyncLogger(int queueCapacity)
  : queue(queueCapacity)
  , active(true) {
    thread = std::make_unique<std::thread>([this]() {
      while (true) {
        if (int const n = count_dropped.exchange(0)) {
          std::cout << prefix << n << " messages have been dropped (queue was full)" << std::endl;
        }
        Data data;
        while (queue.try_pop(data)) {
          std::cout << prefix;
          std::visit([](auto && d){ std::cout << d; }, data);
          std::cout << std::endl;
        }
        if (!active) {
          break;
        }
        // sleep instead of yield to avoid 100% cpu usage.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }
  
  template<typename T>
  void sync_feed(T const& t) {
    if (!queue.try_push(Data{t})) {
      ++count_dropped;
    }
  }
  
  ~AsyncLogger() {
    active = false;
    thread->join();
  }
  
private:
  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ Data,
  /* A = */ std::allocator<Data>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue queue;
  std::atomic_bool active;
  std::unique_ptr<std::thread> thread;
  
  std::atomic_int count_dropped{0};
  static_assert(decltype(count_dropped)::is_always_lock_free);
};

} // NS
