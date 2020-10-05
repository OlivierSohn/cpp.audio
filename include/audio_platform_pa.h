// It is not necessary to use async logging for 'IMJ_LOG_MEMORY' because
// no logging should occur from the realtime threads
// (else it's a bug bcause we should not allocate / deallocate from realtime trheads)

#if (!defined(NDEBUG)) || defined(IMJ_LOG_AUDIO_TIME) || defined(IMJ_LOG_AUDIO_OVERFLOW)
# define IMJ_AUDIO_NEEDS_ASYNC_LOGGING 1
#else
# define IMJ_AUDIO_NEEDS_ASYNC_LOGGING 0
#endif

namespace imajuscule::audio {

void printDevices();
double getGoodSuggestedLatency(double seconds, int const sample_rate);

template<typename T>
struct PortAudioSample;

template<>
struct PortAudioSample<float> {
  static constexpr auto format = paFloat32;
};

#if IMJ_AUDIO_NEEDS_ASYNC_LOGGING
struct AudioCbTimeStats {
  AudioCbTimeStats() {
    reset();
  }

  void reset() {
    n = 0;
    dt = 0;
    maxDt = 0;
  }
  
  // number of callback iterations measured
  uint64_t n;
  
  // sum of execution times of every iterations
  uint64_t dt;
  
  // max execution time, , over all iterations
  uint64_t maxDt;
};

inline std::ostream & operator << (std::ostream& os, AudioCbTimeStats const & s) {
  os << "AudioCbTimeStats(n "
     << s.n
     << "|avg(us) "
     << s.dt / static_cast<float>(s.n)
     << "|max(us) "
     << s.maxDt
     << ")";
  return os;
}

struct PartialCallback {};

inline std::ostream & operator << (std::ostream& os, PartialCallback const &) {
  os << "PartialCallback(The audio engine assumes that the playCallback call executes till the end, but the previous call has been killed.)";
  return os;
}


struct AudioOverflow {
  double value;
};

inline std::ostream & operator << (std::ostream& os, AudioOverflow const & a) {
  os << "AudioOverflow("<< a.value << ")";
  return os;
}

struct SignificantTimeDeviation {
  uint64_t diff;
  float nSamples;
};


inline std::ostream & operator << (std::ostream& os, SignificantTimeDeviation const & d) {
  os << "SignificantTimeDeviation("<< d.diff << " nanos (" << d.nSamples << " frames))";
  return os;
}


using PortaudioAsyncLogger = AsyncLogger<std::variant<
AudioCbTimeStats,
PartialCallback,
AudioOverflow,
SignificantTimeDeviation
>>;

#endif

#ifdef IMJ_LOG_AUDIO_TIME
struct TimeStats {

  void tic() {
    using namespace std::chrono;
    start = high_resolution_clock::now();
  }
  void tac(PortaudioAsyncLogger & logger) {
    using namespace std::chrono;
    auto now = high_resolution_clock::now();
    auto thisDt = duration_cast<microseconds>(now-start).count();
    if(thisDt > stats.maxDt) {
      stats.maxDt = thisDt;
    }
    stats.dt += thisDt;
    ++stats.n;
    if(stats.n == 1000) {
      logger.sync_feed(stats);
      stats.reset();
    }
  }
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  AudioCbTimeStats stats;
};

struct LogAudioTime {
  LogAudioTime(PortaudioAsyncLogger & l)
  : logger(l)
  {
    stats().tic();
  }
  ~LogAudioTime() {
    stats().tac(logger);
  }
private:
  PortaudioAsyncLogger & logger;

  TimeStats & stats() {
    static TimeStats s;
    return s;
  };
};
#endif

#ifndef NDEBUG
// 'PartialCallbackCheck' verifies that every audio-callback call completes.
struct PartialCallbackCheck {
  PartialCallbackCheck(PortaudioAsyncLogger & logger)
  {
    if(unlikely(flag() != 0)) {
      logger.sync_feed(PartialCallback());
      Assert(0);
    }
    flag()++;
  }
  ~PartialCallbackCheck() { flag()--; }
private:
  static std::atomic_int& flag();
  static_assert(std::atomic_int::is_always_lock_free);
};

void analyzeTime(uint64_t t,
                 int nFrames,
                 int sample_rate,
                 PortaudioAsyncLogger &);
#endif

#ifdef IMJ_LOG_MEMORY
struct ScopedThreadNature {
  ScopedThreadNature(ThreadNature from, ThreadNature to) : to(to) {
    threadNature() = from;
  }
  ~ScopedThreadNature() { threadNature() = to; }
private:
  ThreadNature to;
};
#endif

constexpr uint64_t secondsToNanos(PaTime t) {
  return static_cast<uint64_t>(0.5 + t * 1e9);
}

constexpr uint64_t noTime = std::numeric_limits<uint64_t>::min();

template <Features F, typename Chans>
struct Context<AudioPlatform::PortAudio, F, Chans> {
  using MeT =
  Context<AudioPlatform::PortAudio, F, Chans>;

  static constexpr auto nAudioOut = Chans::nOuts;

  template<typename Lock, typename ...Args>
  Context(Lock & l, Args... args)
  : chans(l, args ...)
  , bInitialized(false)
  {}

private:
  int sample_rate_ = 0;
  uint64_t nanos_per_audioelement_buffer = 0;
  
protected:
  Chans chans;
  bool bInitialized : 1;

private:
  PaStream *stream = nullptr;

#if IMJ_DEBUG_AUDIO_OUT
  std::unique_ptr<AsyncWavWriter> async_wav_writer;
  int writer_idx = 0;
#endif
#if IMJ_AUDIO_NEEDS_ASYNC_LOGGING
  std::unique_ptr<PortaudioAsyncLogger> async_logger;
  
  // called from realtime trhead
  PortaudioAsyncLogger & asyncLogger() {
    Assert(async_logger);
    return *async_logger;
  }
#endif

  static int playCallback(const void *inputBuffer,
                          void *outputBuffer,
                          unsigned long numFrames,
                          const PaStreamCallbackTimeInfo* timeInfo,
                          PaStreamCallbackFlags f [[maybe_unused]],
                          void *userData)
  {
    auto This = static_cast<MeT*>(userData);

#ifndef NDEBUG
    PartialCallbackCheck partial_cb_check(This->asyncLogger());
#endif

#ifdef IMJ_LOG_AUDIO_TIME
    LogAudioTime log_time(This->asyncLogger());
#endif

#ifdef IMJ_LOG_MEMORY
    ScopedThreadNature stn{
      ThreadNature::RealTime_Program, // inside this function
      ThreadNature::RealTime_OS       // outside this function
    };
#endif

#ifdef IMJ_LOG_AUDIO_OVERFLOW
    if(f) {
      This->asyncLogger().sync_feed(AudioOverflow{static_cast<double>(f)});
    }
#endif

    // This global variable is used to dynamically optimize
    // the reverb algorithms according to the
    // number of frames we need to compute per callback.
    n_audio_cb_frames.store(numFrames, std::memory_order_relaxed);

    Assert(timeInfo);
    uint64_t tNanos = [timeInfo]() -> uint64_t {
      if(likely(timeInfo)) {
        // on osx, it seems the outputBufferDacTime value is incorrect, see traces of analyzeTime.
        // the error becomes large (sometimes 5 samples offset) with the UA Apollo x4 soundcard
        return secondsToNanos(timeInfo->outputBufferDacTime);
      }
      return noTime; // then, MIDI synchronizatin will not work.
    }();
    
#ifndef NDEBUG
# ifndef IMJ_LOG_AUDIO_TIME // when we have a callback that takes too long to exectute, analyzeTime(...) will detect a problem, so to avoid gaving too much of these logs, when IMJ_LOG_AUDIO_TIME is on, we deactivate analyzeTime
    analyzeTime(tNanos,
                numFrames,
                This->sample_rate_,
                This->asyncLogger());
# endif
#endif

    This->chans.step(static_cast<SAMPLE*>(outputBuffer),
                     static_cast<int>(numFrames),
                     tNanos,
                     This->nanos_per_audioelement_buffer);
#if IMJ_DEBUG_AUDIO_OUT
    This->async_wav_writer->sync_feed(static_cast<SAMPLE*>(outputBuffer),
                                      static_cast<int>(numFrames));
#endif

    return paContinue;
  }

protected:
  // minLatency : latency in seconds
  bool doInit(float minLatency, int sample_rate) {
    LG(INFO, "AudioOut::doInit");
    LG(INFO, "AudioOut::doInit : initializing %s", Pa_GetVersionText());
    PaError err = Pa_Initialize();
    if(likely(err == paNoError)) {
      bInitialized = true;

      LG(INFO, "AudioOut::doInit : done initializing %s", Pa_GetVersionText());

      LG(INFO,"AudioOut::doInit : %d host apis", Pa_GetHostApiCount());

      printDevices();
      
      PaStreamParameters p;
      p.device = Pa_GetDefaultOutputDevice();
      if (unlikely(p.device == paNoDevice)) {
        LG(ERR, "AudioOut::doInit : No default output device");
        Assert(0);
        return false;
      }
      LG(INFO, "AudioOut::doInit : audio device : id %d", p.device);

      p.channelCount = nAudioOut;
      p.sampleFormat = PortAudioSample<SAMPLE>::format;

      auto pi = Pa_GetDeviceInfo( p.device );
      LG(INFO, "AudioOut::doInit : audio device : hostApi    %d", pi->hostApi);
      LG(INFO, "AudioOut::doInit : audio device : name       %s", pi->name);
      LG(INFO, "AudioOut::doInit : audio device : maxIC      %d", pi->maxInputChannels);
      LG(INFO, "AudioOut::doInit : audio device : maxOC      %d", pi->maxOutputChannels);
      LG(INFO, "AudioOut::doInit : audio device : def. sr    %f", pi->defaultSampleRate);
      LG(INFO, "AudioOut::doInit : audio device : def. lolat %f", pi->defaultLowOutputLatency);
      LG(INFO, "AudioOut::doInit : audio device : def. holat %f", pi->defaultHighOutputLatency);
      
      LG(INFO, "AudioOut::doInit : audio device : min user lat %f", minLatency);


      p.suggestedLatency = getGoodSuggestedLatency(std::max(static_cast<double>(minLatency),
                                                            pi->defaultLowOutputLatency),
                                                   sample_rate);

      p.hostApiSpecificStreamInfo = nullptr;

#if IMJ_DEBUG_AUDIO_OUT
      if (!async_wav_writer) {
        async_wav_writer = std::make_unique<AsyncWavWriter>(nAudioOut,
                                                            sample_rate,
                                                            "debug_audioout" + std::to_string(writer_idx));
        ++writer_idx;
      }
#endif  // IMJ_DEBUG_AUDIO_OUT
#if IMJ_AUDIO_NEEDS_ASYNC_LOGGING
      if (!async_logger) {
        async_logger = std::make_unique<PortaudioAsyncLogger>(100);
      }
#endif

      nanos_per_audioelement_buffer = static_cast<uint64_t>(0.5f +
                                                            audio::nanos_per_frame<float>(sample_rate) *
                                                            static_cast<float>(audio::audioelement::n_frames_per_buffer));
      sample_rate_ = sample_rate;
      PaError err = Pa_OpenStream(&stream,
                                  nullptr,
                                  &p,
                                  sample_rate,
                                  paFramesPerBufferUnspecified, /* to decrease latency */
                                  paClipOff | paPrimeOutputBuffersUsingStreamCallback, /* we won't output out of range samples so don't bother clipping them */
                                  playCallback,
                                  this);
      if( unlikely(err != paNoError) )
      {
        stream = nullptr;
        LG(ERR, "AudioOut::doInit : Pa_OpenStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return false;
      }

      const PaStreamInfo * si = Pa_GetStreamInfo(stream);

      LG(INFO, "AudioOut::doInit : stream : sample rate %f",
         si->sampleRate);
      LG(INFO, "AudioOut::doInit : stream : output lat  %f (%f frames)",
         si->outputLatency, si->outputLatency * si->sampleRate);
      LG(INFO, "AudioOut::doInit : stream : input lat   %f (%f frames)",
         si->inputLatency,  si->inputLatency  * si->sampleRate);

      err = Pa_StartStream( stream );
      if( unlikely(err != paNoError) )
      {
        LG(ERR, "AudioOut::doInit : Pa_StartStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return false;
      }
    }
    else
    {
      LG(ERR, "AudioOut::doInit : PA_Initialize failed : %s", Pa_GetErrorText(err));
      Assert(0);
      return false;
    }
    LG(INFO, "AudioOut::doInit : success");
    return true;
  }

public:
  void doTearDown() {
    LG(INFO, "AudioOut::doTearDown");
    if(stream)
    {
      PaError err = Pa_CloseStream( stream );
      stream = nullptr;
      if( unlikely(err != paNoError) ) {
        LG(ERR, "AudioOut::doTearDown : Pa_CloseStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return;
      }
    }

    if( bInitialized ) { // don't call Pa_Terminate if Pa_Initialize failed
      bInitialized = false;

      PaError err = Pa_Terminate();
      if(err != paNoError)
      {
        LG(ERR, "AudioOut::doTearDown : PA_Terminate failed : %s", Pa_GetErrorText(err));
        return;
      }
    }
#if IMJ_DEBUG_AUDIO_OUT
    async_wav_writer.reset();
#endif
#if IMJ_AUDIO_NEEDS_ASYNC_LOGGING
    async_logger.reset();
#endif
    LG(INFO, "AudioOut::doTearDown : success");
  }
};

template<>
struct AudioInput<AudioPlatform::PortAudio> {
  AudioInput()
  {}

  bool Init(RecordF f, int const sample_rate, double const minLatency) {
    LG(INFO, "AudioIn::do_wakeup : initializing %s", Pa_GetVersionText());
    auto err = Pa_Initialize();
    if(likely(err == paNoError))
    {
      LG(INFO, "AudioIn::do_wakeup : done initializing %s", Pa_GetVersionText());

      LG(INFO,"AudioIn::do_wakeup : %d host apis", Pa_GetHostApiCount());

      PaStreamParameters inputParameters;
      inputParameters.device = Pa_GetDefaultInputDevice();
      if (unlikely(inputParameters.device == paNoDevice)) {
        LG(ERR, "AudioIn::do_wakeup : No default input device");
        Assert(0);
        return false;
      }
      LG(INFO, "AudioIn::do_wakeup : audio device : id %d", inputParameters.device);

      inputParameters.channelCount = 1;
      inputParameters.sampleFormat = audio::PortAudioSample<SAMPLE>::format;

      auto pi = Pa_GetDeviceInfo( inputParameters.device );
      LG(INFO, "AudioIn::do_wakeup : audio device : hostApi    %d", pi->hostApi);
      LG(INFO, "AudioIn::do_wakeup : audio device : name       %s", pi->name);
      LG(INFO, "AudioIn::do_wakeup : audio device : maxIC      %d", pi->maxInputChannels);
      LG(INFO, "AudioIn::do_wakeup : audio device : maxOC      %d", pi->maxOutputChannels);
      LG(INFO, "AudioIn::do_wakeup : audio device : def. sr    %f", pi->defaultSampleRate);
      LG(INFO, "AudioIn::do_wakeup : audio device : def. lilat %f", pi->defaultLowInputLatency);
      LG(INFO, "AudioIn::do_wakeup : audio device : def. hilat %f", pi->defaultHighInputLatency);

      LG(INFO, "AudioIn::do_wakeup : audio device : min user lat %f", minLatency);

      inputParameters.suggestedLatency = getGoodSuggestedLatency(std::max(static_cast<double>(minLatency),
                                                                          pi->defaultLowInputLatency),
                                                                 sample_rate);

      inputParameters.hostApiSpecificStreamInfo = nullptr;

      recordF = f;

#if IMJ_DEBUG_AUDIO_IN
      if (!async_wav_writer) {
        async_wav_writer = std::make_unique<AsyncWavWriter>(1, // n. audio channels
                                                            sample_rate,
                                                            "debug_audioin" + std::to_string(writer_idx));
        ++writer_idx;
      }
#endif  // IMJ_DEBUG_AUDIO_IN
      
      /* Record some audio. -------------------------------------------- */
      PaError err = Pa_OpenStream(&stream,
                                  &inputParameters,
                                  nullptr,                  /* &outputParameters, */
                                  sample_rate,
                                  paFramesPerBufferUnspecified /* to decrease latency*/,
                                  paClipOff,      /* we won't output out of range samples so don't bother clipping them */
                                  recordCallback,
                                  this);
      if( unlikely(err != paNoError) )
      {
        stream = nullptr;
        LG(ERR, "AudioIn::do_wakeup : Pa_OpenStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return false;
      }
      sample_rate_ = sample_rate;

      const PaStreamInfo * si = Pa_GetStreamInfo(stream);

      LG(INFO, "AudioIn::do_wakeup : stream : output lat  %f", si->outputLatency);
      LG(INFO, "AudioIn::do_wakeup : stream : input lat   %f", si->inputLatency);
      LG(INFO, "AudioIn::do_wakeup : stream : sample rate %f", si->sampleRate);

      err = Pa_StartStream( stream );
      if( unlikely(err != paNoError) )
      {
        LG(ERR, "AudioIn::do_wakeup : Pa_StartStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return false;
      }
    }
    else
    {
      LG(ERR, "AudioIn::do_wakeup : PA_Initialize failed : %s", Pa_GetErrorText(err));
      Assert(0);
      return false;
    }
    return true;
  }

  bool Teardown() {
    if(stream)
    {
      PaError err = Pa_CloseStream( stream );
      stream = nullptr;
      if( unlikely(err != paNoError) ) {
        LG(ERR, "AudioIn::do_sleep : Pa_CloseStream failed : %s", Pa_GetErrorText(err));
        Assert(0);
        return false;
      }
    }

    PaError err = Pa_Terminate();
    if(unlikely(err != paNoError))
    {
      LG(ERR, "AudioIn::do_sleep : PA_Terminate failed : %s", Pa_GetErrorText(err));
      return false;
    }
#if IMJ_DEBUG_AUDIO_IN
    if (async_wav_writer) {
      async_wav_writer.reset();
    }
#endif
    return true;
  }

  int getSampleRate() const {
    return sample_rate_;
  }

private:
  PaStream *stream = nullptr;
  int sample_rate_ = 0;
  RecordF recordF;
#if IMJ_DEBUG_AUDIO_IN
  std::unique_ptr<AsyncWavWriter> async_wav_writer;
  int writer_idx = 0;
#endif

  static int recordCallback( const void *inputBuffer, void *outputBuffer,
                            unsigned long nFrames,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData )
  {
    AudioInput<AudioPlatform::PortAudio> *This = static_cast<AudioInput<AudioPlatform::PortAudio>*>(userData);

    (void) outputBuffer;
    (void) timeInfo;
    (void) statusFlags;
    (void) userData;

    This->recordF(static_cast<const SAMPLE*>(inputBuffer), (int)nFrames);
#if IMJ_DEBUG_AUDIO_IN
    This->async_wav_writer->sync_feed(static_cast<const SAMPLE*>(inputBuffer), (int)nFrames);
#endif

    return paContinue;
  }

};

} // NS imajuscule::audio
