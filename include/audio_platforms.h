

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
template<AudioPlatform A, Features F, typename Chans>
struct Context;

// for input:
template<AudioPlatform A>
struct AudioInput;
using RecordF = std::function<void(const SAMPLE*, int /* num frames */)>;



constexpr auto initial_n_audio_cb_frames = -1;
// 'n_audio_cb_frames' is initially 'initial_n_audio_cb_frames', until the audio callback sets the value.
extern std::atomic<int32_t> n_audio_cb_frames;
static_assert(std::atomic<int32_t>::is_always_lock_free);

extern int wait_for_first_n_audio_cb_frames();

/* Debug utility to write a wav file asynchronously */
struct AsyncWavWriter {
  AsyncWavWriter(int nAudioChans, std::string const & prefix)
  : n_audio_chans(nAudioChans)
  , wav_write_queue(44100 * nAudioChans) // one second of output can fit in the queue
  , wav_write_active(true)
  , thread_write_wav{[this, prefix]() {
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
                      SAMPLE_RATE,
                      numberToNChannels(n_audio_chans),
                      AudioSample<SAMPLE>::format);
    
    std::string const filename = prefix + ".wav";
    
    WAVWriter writer(rootDir,
                     filename,
                     header);
    auto res = writer.Initialize();
    
    if(ILE_SUCCESS != res) {
      LG(ERR, "audio debug : failed to open '%s' in '%s' to write audio", filename.c_str(), rootDir);
      return;
    } else {
      LG(INFO, "audio debug : opened '%s' in '%s' to write audio", filename.c_str(), rootDir);
    }
    
    while (wav_write_active) {
      SAMPLE val;
      while (wav_write_queue.try_pop(val)) {
        writer.writeSample(val);
      }
    }
  }}
  {}
  

  template<typename T>
  void sync_feed(T * buf, int numFrames) {
    for (int i=0; i<numFrames; ++i) {
      for (int j=0; j<n_audio_chans; ++j) {
        if (!wav_write_queue.try_push(buf[n_audio_chans*i + j])) {
          LG(ERR, "audio debug : dropped sample");
        }
      }
    }
  }
  
  ~AsyncWavWriter() {
    wav_write_active = false;
    thread_write_wav.join();
  }

private:
  using Queue = atomic_queue::AtomicQueueB2<
  /* T = */ SAMPLE,
  /* A = */ std::allocator<SAMPLE>,
  /* MAXIMIZE_THROUGHPUT */ true,
  /* TOTAL_ORDER = */ true,
  /* SPSC = */ true
  >;
  Queue wav_write_queue;
  std::atomic_bool wav_write_active;
  std::thread thread_write_wav;
  int const n_audio_chans;
};

} // NS
