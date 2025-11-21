

namespace imajuscule::audio {

enum class AudioPlatform {
  PortAudio,
  AudioUnits
};

enum class Features {
  JustOut,
  InAndOut
};

enum class TimeSource {
  // Time is defined by MIDI.
  MIDI,

  // Time is incremented each time the context callback is called.
  Monotonic
};

// for output:
template<AudioPlatform A, Features F, TimeSource Time>
struct Context;

// for input:
template<AudioPlatform A>
struct AudioInput;

// for input and output in the same stream
template<AudioPlatform A, TimeSource Time>
struct FullDuplexContext;


struct DurationNanos
{
  constexpr explicit DurationNanos(uint64_t t = 0)
  : nanos(t)
  {}
  constexpr uint64_t get() const {return nanos;}
  
  friend constexpr bool operator <(DurationNanos a, DurationNanos b)
  {
    return a.get() < b.get();
  }
  template<typename Duration>
  constexpr DurationNanos & operator +=(const Duration& d) {
    nanos += std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
    return *this;
  }
private:
  uint64_t nanos;  
};
struct TimeNanos
{
  constexpr explicit TimeNanos(uint64_t t = 0)
  : nanos(t)
  {}
  constexpr uint64_t get() const {return nanos;}
  
  constexpr TimeNanos & operator +=(DurationNanos d) {
    nanos += d.get();
    return *this;
  }
  template<typename Duration>
  constexpr TimeNanos & operator +=(const Duration& d) {
    nanos += std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
    return *this;
  }
  friend constexpr bool operator <(TimeNanos a, TimeNanos b)
  {
    return a.get() < b.get();
  }
  friend constexpr bool operator >=(TimeNanos a, TimeNanos b)
  {
    return a.get() >= b.get();
  }
  friend constexpr bool operator ==(TimeNanos a, TimeNanos b)
  {
    return a.get() == b.get();
  }
private:
  uint64_t nanos;  
};
constexpr TimeNanos operator +(TimeNanos a, DurationNanos b)
{
  return TimeNanos{a.get() + b.get()};
}
constexpr DurationNanos operator -(TimeNanos a, TimeNanos b)
{
  Assert(a >= b);
  return DurationNanos{a.get() - b.get()};
}
constexpr DurationNanos operator +(DurationNanos a, DurationNanos b)
{
  return DurationNanos{a.get() + b.get()};
}


// output callback:
using PlayF = std::function<void(SAMPLE *,
                                 int,
                                 TimeNanos const)>;
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
  
  AsyncWavWriter(int nAudioChans, int sampleRate, std::string const & prefix, std::optional<int> const maxSamples = {})
  : n_audio_chans(nAudioChans)
  , sample_rate(sampleRate)
  , queue(queueCapacityInSecondsOfAudioSignal * (sampleRate * nAudioChans))
  , active(true)
  , m_maxSamples(maxSamples){
    thread = std::make_unique<std::thread>([this, prefix]() {
      const std::filesystem::path rootDir{"/Users/Olivier/dev/hs.hamazed/"};
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
    
    const auto path = rootDir / filename;
    WAVWriter writer(path,
                     header);
    auto res = writer.Initialize();
    
    if(ILE_SUCCESS != res) {
      LG(ERR, "Audio debug : failed to open '%s' to write audio", path.u8string().c_str());
      return;
    } else {
      LG(INFO, "Audio debug : opened '%s' to write audio", path.u8string().c_str());
    }
    
    int countSamples = 0;
    while (true) {
      SAMPLE val;
      while (queue.try_pop(val)) {
        writer.writeSample(val);
        ++countSamples;
        if (m_maxSamples && *m_maxSamples < countSamples)
          return;
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

  std::optional<int> m_maxSamples;
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
