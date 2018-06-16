
#ifndef NDEBUG
#  ifdef IMJ_LOG_OVERFLOW
#    error redefinition of IMJ_LOG_OVERFLOW
#  endif
#  define IMJ_LOG_OVERFLOW 1
#endif

namespace imajuscule {
    namespace audio {

        template<typename T>
        struct PortAudioSample;

        template<>
        struct PortAudioSample<float> {
            static constexpr auto format = paFloat32;
        };

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
                                    PaStreamCallbackFlags f,
                                    void *userData )
            {
#ifndef NDEBUG // verify that our thread is not killed during the call
                static int flag = 0;
                if(unlikely(flag != 0)) {
                  LG(ERR, "The audio engine assumes that the playCallback call executes till the end, but the previous call has been killed.");
                  Assert(0);
                }
                flag = 1;
#endif

                // This global variable is used to dynamically optimize
                // the reverb algorithms according to the
                // number of frames we need to compute.
                n_audio_cb_frames = numFrames;

                Chans *chans = (Chans*)userData;

                (void) timeInfo;
                (void) f;

#ifdef IMJ_LOG_OVERFLOW
                if(f) {
                  LG(ERR, "overflow flag: %d %d %d %d %d"
                    , f & paInputUnderflow
                    , f & paInputOverflow
                    , f & paOutputUnderflow
                    , f & paOutputOverflow
                    , f & paPrimingOutput
                  );
                  Assert(0);
                }
#endif

                chans->step((SAMPLE*)outputBuffer, static_cast<int>(numFrames));

#ifndef NDEBUG
                flag = 0;
#endif

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

                    p.suggestedLatency =
                      // on windows it's important to not set suggestedLatency too low, else samples are lost (for example only 16 are available per timestep)
                      std::max(
                        static_cast<double>(minLatency),
                        pi->defaultLowOutputLatency);

                    p.hostApiSpecificStreamInfo = nullptr;

                    /* Listen to some audio. -------------------------------------------- */
                    PaError err = Pa_OpenStream(
                                                &stream,
                                                nullptr,
                                                &p,                  /* &outputParameters, */
                                                SAMPLE_RATE,
                                                paFramesPerBufferUnspecified, /*if not 0 an additional buffering may be used for some host apis, increasing latency*/
                                                paClipOff | paPrimeOutputBuffersUsingStreamCallback,      /* we won't output out of range samples so don't bother clipping them */
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

                    LG(INFO, "AudioOut::doInit : stream : sample rate %f", si->sampleRate);
                    LG(INFO, "AudioOut::doInit : stream : output lat  %f (%f frames)", si->outputLatency, si->outputLatency * si->sampleRate);
                    LG(INFO, "AudioOut::doInit : stream : input lat   %f (%f frames)", si->inputLatency, si->inputLatency * si->sampleRate);

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
            void TearDown() {
                LG(INFO, "AudioOut::TearDown");
                if(stream)
                {
                    PaError err = Pa_CloseStream( stream );
                    stream = nullptr;
                    if( unlikely(err != paNoError) ) {
                        LG(ERR, "AudioOut::TearDown : Pa_CloseStream failed : %s", Pa_GetErrorText(err));
                        Assert(0);
                        return;
                    }
                }

                if( bInitialized ) { // don't call Pa_Terminate if Pa_Initialize failed
                    bInitialized = false;

                    PaError err = Pa_Terminate();
                    if(err != paNoError)
                    {
                        LG(ERR, "AudioOut::TearDown : PA_Terminate failed : %s", Pa_GetErrorText(err));
                        return;
                    }
                }
                LG(INFO, "AudioOut::TearDown : success");
            }
        };
    }
}
