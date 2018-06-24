

namespace imajuscule {
    namespace audio {

        int initAudioSession();
        extern OSStatus startAudioUnit(AudioUnit audioUnit);
        extern OSStatus stopProcessingAudio(AudioUnit audioUnit);
        int initAudioStreams(Features f, AudioUnit & audioUnit, void * chans,
                             AURenderCallback cb, int nOuts,
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

                ios_data->chans->step(outputBuffer.data(), numFrames);

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
            bool doInit(float minLatency) {
                LG(INFO, "AudioOut::doInit");
                bInitialized = true;
                if(0==initAudioSession())
                {
                    static imajuscule::audio::iOSOutputData<Chans> ios_odata;
                    Assert(!ios_odata.chans || (ios_odata.chans==&chans));
                    ios_odata.chans = &chans;

                    if(0==initAudioStreams(Feat, audioUnit_out, &ios_odata, renderCallback_out, nAudioOut, desc))
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

        public:
            void TearDown() {
                LG(INFO, "AudioOut::TearDown");
                if( bInitialized ) {
                    bInitialized = false;
                    OSStatus err = stopProcessingAudio(audioUnit_out);
                    if( noErr != err ) {
                        LG(ERR, "AudioOut::TearDown : stopProcessingAudio failed : %d", err);
                        Assert(0);
                        return;
                    }
                }
                LG(INFO, "AudioOut::TearDown : success");
            }
        };

    }
}
