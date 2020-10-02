namespace imajuscule::audio {

template<typename T>
struct PortAudioSample;

template<>
struct PortAudioSample<float> {
  static constexpr auto format = paFloat32;
};

#ifdef IMJ_LOG_AUDIO_TIME
struct TimeStats {

  void tic() {
    using namespace std::chrono;
    start = high_resolution_clock::now();
  }
  void tac() {
    using namespace std::chrono;
    auto now = high_resolution_clock::now();
    auto thisDt = duration_cast<microseconds>(now-start).count();
    if(thisDt>maxDt) {
      maxDt = thisDt;
    }
    dt += thisDt;
    ++n;
    if(n==1000) {
      LG(INFO, "avg: %f (us) max: %d (us)", dt / static_cast<float>(n), maxDt);
      n = 0;
      dt = 0;
      maxDt = 0;
    }
  }
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  uint64_t n = 0;
  uint64_t dt = 0;
  uint64_t maxDt = 0;
};

struct LogAudioTime {
  LogAudioTime() { stats().tic(); }
  ~LogAudioTime() { stats().tac(); }
private:
  TimeStats & stats() {
    static TimeStats s;
    return s;
  };
};
#endif

#ifndef NDEBUG
// 'PartialCallbackCheck' verifies that every audio-callback call completes.
struct PartialCallbackCheck {
  PartialCallbackCheck()
  {
    if(unlikely(flag() != 0)) {
      ScopedNoMemoryLogs s; // because we log an error
      LG(ERR, "The audio engine assumes that the playCallback call executes till the end, but the previous call has been killed.");
      Assert(0);
    }
    flag() = 1;
  }
  ~PartialCallbackCheck() { flag() = 0; }
private:
  static int& flag() {
    static int f = 0;
    return f;
  }
};

void analyzeTime(uint64_t t, int nFrames, int sample_rate);
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

  static int playCallback(const void *inputBuffer,
                          void *outputBuffer,
                          unsigned long numFrames,
                          const PaStreamCallbackTimeInfo* timeInfo,
                          PaStreamCallbackFlags f [[maybe_unused]],
                          void *userData)
  {
#ifndef NDEBUG
    PartialCallbackCheck partial_cb_check;
#endif

#ifdef IMJ_LOG_AUDIO_TIME
    LogAudioTime log_time;
#endif

#ifdef IMJ_LOG_MEMORY
    ScopedThreadNature stn{
      ThreadNature::RealTime_Program, // inside this function
      ThreadNature::RealTime_OS       // outside this function
    };
#endif

#ifdef IMJ_LOG_AUDIO_OVERFLOW
    if(f) {
      ScopedNoMemoryLogs s; // because we log an error
      LG(ERR, "audio overflow: %d", f);
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
    
    auto This = static_cast<MeT*>(userData);
#ifndef NDEBUG
    analyzeTime(tNanos,
                numFrames,
                This->sample_rate_);
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

      // To optimize cache use, the callback should be asked to compute
      // a multiple of n_frames_per_buffer frames at each call.
      // So we round the latency such that this requirement is met.

      auto suggestedLatency =
      // on windows it's important to not set suggestedLatency too low, else samples are lost (for example only 16 are available per timestep)
      std::max(static_cast<double>(minLatency),
               pi->defaultLowOutputLatency);
      int suggestedLatencyInSamples = static_cast<int>(0.5f + suggestedLatency * static_cast<float>(sample_rate));
      LG(INFO, "suggested latency in samples       : %d", suggestedLatencyInSamples);
      using namespace audioelement;

      // we to the next multiple of 'n_frames_per_buffer'
      int ceiledSLIS = n_frames_per_buffer * (1 + (suggestedLatencyInSamples-1) / n_frames_per_buffer);
      LG(INFO, " ceiled to the next multiple of %d : %d", n_frames_per_buffer, ceiledSLIS);

      // we have cracks on osx, rounding to the next power of 2 fixed this.
      ceiledSLIS = ceil_power_of_two(ceiledSLIS);
      LG(INFO, " ceiled to the next power of 2 : %d", ceiledSLIS);

      // The current portaudio doc says, about suggestedLatency:
      //   "implementations should round the suggestedLatency up to the next practical value
      //     - ie to provide an equal or higher latency than suggestedLatency wherever possible"
      //   http://portaudio.com/docs/v19-doxydocs/structPaStreamParameters.html#aa1e80ac0551162fd091db8936ccbe9a0
      //
      // But what I found is that the resulting latency is 'equal or lower' to the suggested one.
      // Hence, I add epsilon here instead of substracting it, to be sure I will get the right buffer sizes.
      p.suggestedLatency = static_cast<float>(ceiledSLIS) / static_cast<float>(sample_rate) + 1e-6;
      LG(INFO, "p.suggestedLatency          : %f", p.suggestedLatency);

      p.hostApiSpecificStreamInfo = nullptr;

#if IMJ_DEBUG_AUDIO_OUT
      if (!async_wav_writer) {
        async_wav_writer = std::make_unique<AsyncWavWriter>(nAudioOut,
                                                            sample_rate,
                                                            "debug_audioout" + std::to_string(writer_idx));
        ++writer_idx;
      }
#endif  // IMJ_DEBUG_AUDIO_OUT

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
    if (async_wav_writer) {
      async_wav_writer.reset();
    }
#endif
    LG(INFO, "AudioOut::doTearDown : success");
  }
};

template<>
struct AudioInput<AudioPlatform::PortAudio> {
  AudioInput()
  {}

  bool Init(RecordF f, int const sample_rate) {
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

      inputParameters.suggestedLatency =
      // on windows it's important to not set suggestedLatency too low, else samples are lost (for example only 16 are available per timestep)
      pi->defaultLowInputLatency;

      inputParameters.hostApiSpecificStreamInfo = nullptr;

      recordF = f;

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
#if IMJ_DEBUG_AUDIO_IN
    if (!async_wav_writer) {
      async_wav_writer = std::make_unique<AsyncWavWriter>(1, // n. audio channels
                                                          sample_rate,
                                                          "debug_audioin" + std::to_string(writer_idx));
      ++writer_idx;
    }
#endif  // IMJ_DEBUG_AUDIO_OUT
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
