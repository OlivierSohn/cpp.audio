

namespace imajuscule::audio {

        int initAudioSession();
        extern OSStatus startAudioUnit(AudioUnit audioUnit);
        extern OSStatus stopProcessingAudio(AudioUnit audioUnit);
        int initAudioStreams(Features f, AudioUnit & audioUnit, void * cb_data,
                             AURenderCallback cb, int nOuts, int sample_rate,
                             AudioStreamBasicDescription & streamDescription);

        template <typename T>
        struct iOSOutputData { // TODO could that be part of Context ?
            using Chans = T;
            // we cannot know for sure how much the os will ask us to compute.
            // on my iPhone 4s I observed 512 was asked.
            static constexpr auto initial_buffer_size = 1024;

            iOSOutputData() {
                // preallocate to avoid dynamic allocation in audio thread
                buf.reserve(initial_buffer_size);
            }

            Chans * chans = nullptr;
            std::vector<typename Chans::T> buf;
        };

        template<Features Feat, typename Chans>
        struct Context <AudioPlatform::AudioUnits, Feat, Chans> {
            static constexpr auto nAudioOut = Chans::nOuts;

            template<typename Lock, typename ...Args>
            Context(Lock & l, Args... args) : chans(l, args ...)
            , bInitialized(false)
            {}

            AudioUnit getAudioUnit() const {
                return audioUnit_out;
            }

            AudioStreamBasicDescription const & getStreamDescription() const {
                return desc;
            }
            int getSampleRate() const {
              return getStreamDescription().mSampleRate;
            }
        protected:
            Chans chans;
            bool bInitialized : 1;
        private:
            AudioStreamBasicDescription desc;
            AudioUnit audioUnit_out = nullptr;

            static OSStatus renderCallback_out(void                 *userData,
                                               AudioUnitRenderActionFlags  *actionFlags,
                                               const AudioTimeStamp        *audioTimeStamp,
                                               UInt32                      busNumber,
                                               UInt32                      numFrames,
                                               AudioBufferList             *buffers) {

                n_audio_cb_frames.store(numFrames, std::memory_order_relaxed);

                auto ios_data = reinterpret_cast<iOSOutputData<Chans>*>(userData);
                auto sizeBuffer = numFrames * nAudioOut;
                for (UInt32 i=0; i<buffers->mNumberBuffers; ++i) {
                    // might be less than previous value (when using parot headphones in bluetooth)
                    auto v = buffers->mBuffers[i].mDataByteSize / sizeof(SInt16);
                    if(v < sizeBuffer) {
                        LG(INFO, "diff %d", sizeBuffer-v);
                        sizeBuffer = v;
                    }
                    break;
                }

                auto & outputBuffer = ios_data->buf;
                outputBuffer.resize(sizeBuffer); // hopefully we already reserved enough

              ios_data->chans->step(outputBuffer.data(),
                                    numFrames,
                                    // TODO is audioTimeStamp->mWordClockTime in nanoseconds?
                                    audioTimeStamp?audioTimeStamp->mWordClockTime:0,
                                    0);

                for (UInt32 i=0; i<buffers->mNumberBuffers; ++i) {
                    Assert(sizeBuffer * sizeof(SInt16) <= buffers->mBuffers[i].mDataByteSize);
                    Assert(nAudioOut == buffers->mBuffers[i].mNumberChannels);
                    auto buffer = (SInt16*)(buffers->mBuffers[i].mData);
                    for( UInt32 j=0; j<sizeBuffer; j++ ) {
                        auto val = (SInt16)(outputBuffer[j] * 32767.f);
                        *buffer = val;
                        ++buffer;
                    }
                }

                return noErr;
            }

        protected:
          // TODO how can we set latency on ios?
            bool doInit(float minLatency, int const sample_rate) {
                LG(INFO, "AudioOut::doInit");
                bInitialized = true;
                if(0==initAudioSession())
                {
                    static imajuscule::audio::iOSOutputData<Chans> ios_odata;
                    Assert(!ios_odata.chans || (ios_odata.chans==&chans));
                    ios_odata.chans = &chans;

                    if(0==initAudioStreams(Feat, audioUnit_out, &ios_odata, renderCallback_out, nAudioOut, sample_rate, desc))
                    {
                        OSStatus res = startAudioUnit(audioUnit_out);
                        if( noErr != res )
                        {
                            LG(ERR, "AudioOut::doInit : startAudioUnit failed : %d", res);
                            Assert(0);
                            return false;
                        }
                    }
                    else
                    {
                        LG(ERR, "AudioOut::doInit : initAudioStreams failed");
                        Assert(0);
                        return false;
                    }
                }
                else
                {
                    LG(ERR, "AudioOut::doInit : initAudioSession failed");
                    Assert(0);
                    return false;
                }
                LG(INFO, "AudioOut::doInit : success");
                return true;
            }

            void doTearDown() {
                if(!bInitialized) {
                  LG(INFO, "AudioOut::doTearDown : audio was not initialized.");
                  return;
                }
                bInitialized = false;
                LG(INFO, "AudioOut::doTearDown : stopProcessingAudio.");
                OSStatus err = stopProcessingAudio(audioUnit_out);
                if( noErr != err ) {
                    LG(ERR, "AudioOut::doTearDown : stopProcessingAudio failed : %d", err);
                    Assert(0);
                    return;
                }
                LG(INFO, "AudioOut::doTearDown : success");
            }
        };


template<>
struct AudioInput<AudioPlatform::AudioUnits> {

  bool Init(RecordF f, int const sample_rate) {
    if(0==audio::initAudioSession())
    {
      recordF = f;
      convertedSampleBuffer.resize(1024);
      if(0==initAudioStreams(audio::Features::InAndOut, audioUnit_in, this, renderCallback_in, 1,sample_rate, desc))
      {
        OSStatus res;
        res = audio::startAudioUnit(audioUnit_in);
        if( noErr != res )
        {
          LG(ERR, "AudioIn::do_wakeup : startAudioUnit failed : %d", res);
          Assert(0);
          return false;
        }
      }
      else
      {
        LG(ERR, "AudioIn::do_wakeup : initAudioStreams failed");
        Assert(0);
        return false;
      }
    }
    else
    {
      LG(ERR, "AudioIn::do_wakeup : initAudioSession failed");
      // fails on simulator
      //Assert(0);
      return false;
    }
    return true;
  }

  bool Teardown() {
    OSStatus err = audio::stopProcessingAudio(audioUnit_in);
    if( noErr != err ) {
      LG(ERR, "AudioIn::do_sleep : stopProcessingAudio failed : %d", err);
      Assert(0);
      return false;
    }
    return true;
  }
  AudioStreamBasicDescription const & getStreamDescription() const {
    return desc;
  }
  int getSampleRate() const {
    return getStreamDescription().mSampleRate;
  }

private:
  AudioUnit audioUnit_in = nullptr;
  AudioStreamBasicDescription desc;
  RecordF recordF;
  std::vector<float> convertedSampleBuffer;

  static OSStatus renderCallback_in(void                        *userData,
                             AudioUnitRenderActionFlags  *actionFlags,
                             const AudioTimeStamp        *audioTimeStamp,
                             UInt32                      busNumber,
                             UInt32                      numFrames,
                             AudioBufferList             *buffers) {
    AudioInput<AudioPlatform::AudioUnits> *This = static_cast<AudioInput<AudioPlatform::AudioUnits>*>(userData);
    
    OSStatus status = AudioUnitRender(This->audioUnit_in,
                                      actionFlags,
                                      audioTimeStamp,
                                      1,
                                      numFrames,
                                      buffers);
    if(status != noErr) {
      if(status == kAudioUnitErr_CannotDoInCurrentContext) {
        LG(ERR, "the app probably went in the background, need to return something else?");
      }
      LG(ERR,"renderCallback (audio) : error %d", status);
      return status;
    }

    float * buf = This->convertedSampleBuffer.data();
    
    int startFrame = 0;
    do {
      SInt16 *inputFrames = startFrame + (SInt16*)(buffers->mBuffers->mData);
      int const end = std::min(numFrames, static_cast<UInt32>(This->convertedSampleBuffer.size()));
      for(auto i = 0; i < end; i++) {
        buf[i] = (float)inputFrames[i] / 32768.f;
      }
      This->recordF((const SAMPLE*)buf, end);
      numFrames -= end;
      startFrame += end;
    } while(numFrames);
    
    // mute audio
    for (UInt32 i=0; i<buffers->mNumberBuffers; ++i) {
      memset(buffers->mBuffers[i].mData, 0, buffers->mBuffers[i].mDataByteSize);
    }
    
    return noErr;
  }
};

} // NS
