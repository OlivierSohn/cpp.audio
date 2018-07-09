
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

  constexpr int64_t secondsToNanos(PaTime t) {
    return static_cast<int64_t>(0.5 + t * 1e9);
  }

  template <Features F, typename Chans>
  struct Context<AudioPlatform::PortAudio, F, Chans> {
    static constexpr auto nAudioOut = Chans::nOuts;

    template<typename Lock, typename ...Args>
    Context(Lock & l, Args... args) : chans(l, args ...)
    , bInitialized(false)
    {}

  protected:
    Chans chans;
    bool bInitialized : 1;
  private:
    PaStream *stream = nullptr;

    static int playCallback( const void *inputBuffer, void *outputBuffer,
                            unsigned long numFrames,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags f [[maybe_unused]],
                            void *userData )
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

      Assert(timeInfo); // the time is not unique, and computes won't work.

      int64_t tNanos = [timeInfo](){
        static int64_t t = 0;
        if(likely(timeInfo)) {
          t = secondsToNanos(timeInfo->outputBufferDacTime);
        }
        else {
          ++t; // it's important that the time is different at each step.
        }
        return t;
      }();

      reinterpret_cast<Chans*>(userData)->step(reinterpret_cast<SAMPLE*>(outputBuffer), static_cast<int>(numFrames), tNanos);

      return paContinue;
    }

  protected:
    // minLatency : latency in seconds
    bool doInit(float minLatency) {
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

        // To optimize cache use, the callback should be asked to compute
        // a multiple of n_frames_per_buffer frames at each call.

        auto suggestedLatency =
          // on windows it's important to not set suggestedLatency too low, else samples are lost (for example only 16 are available per timestep)
          std::max(
            static_cast<double>(minLatency),
            pi->defaultLowOutputLatency);
        int suggestedLatencyInSamples = static_cast<int>(0.5f + suggestedLatency * static_cast<float>(SAMPLE_RATE));
        LG(INFO, "suggested latency in samples       : %d", suggestedLatencyInSamples);
        using namespace audioelement;
        int ceiledSLIS = n_frames_per_buffer * (1 + (suggestedLatencyInSamples-1) / n_frames_per_buffer);
        LG(INFO, " ceiled to the next multiple of %d : %d", n_frames_per_buffer, ceiledSLIS);
        // The current portaudio doc says, about suggestedLatency:
        //   "implementations should round the suggestedLatency up to the next practical value
        //     - ie to provide an equal or higher latency than suggestedLatency wherever possible"
        //   http://portaudio.com/docs/v19-doxydocs/structPaStreamParameters.html#aa1e80ac0551162fd091db8936ccbe9a0
        //
        // But what I found is that the resulting latency is 'equal or lower' to the suggested one.
        // Hence, I add epsilon here instead of substracting it, to be sure I will get the right buffer sizes.
        p.suggestedLatency = static_cast<float>(ceiledSLIS) / static_cast<float>(SAMPLE_RATE) + 1e-6;
        LG(INFO, "p.suggestedLatency          : %f", p.suggestedLatency);

        p.hostApiSpecificStreamInfo = nullptr;

        PaError err = Pa_OpenStream(&stream,
                                    nullptr,
                                    &p,
                                    SAMPLE_RATE,
                                    paFramesPerBufferUnspecified, /* to decrease latency */
                                    paClipOff | paPrimeOutputBuffersUsingStreamCallback, /* we won't output out of range samples so don't bother clipping them */
                                    playCallback,
                                    &chans);
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
          LG(INFO, "AudioOut::doTearDown : success");
      }
  };
} // NS imajuscule::audio
