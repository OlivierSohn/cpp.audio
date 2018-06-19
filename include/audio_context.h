

namespace imajuscule {
    namespace audio {

        template <AudioOutPolicy>
        struct GlobalAudioLock;

        template <>
        struct GlobalAudioLock<AudioOutPolicy::Slave> {
            static auto & get() {
                static AudioLockPolicyImpl<AudioOutPolicy::Slave> l;
                return l;
            }
        };

        template <>
        struct GlobalAudioLock<AudioOutPolicy::MasterGlobalLock> {
            static auto & get() {
                static AudioLockPolicyImpl<AudioOutPolicy::MasterGlobalLock> l;
                return l;
            }
        };

        void setPortaudioLatencyMillis(int latency);

        constexpr auto initial_n_audio_cb_frames = -1;

        constexpr auto impulse_responses_root_dir = "audio.ir";

        int wait_for_first_n_audio_cb_frames();


        template<typename OutputData>
        bool useConvolutionReverb(OutputData & chans,
                                  std::string const & dirname, std::string const & filename) {
            WAVReader reader(dirname, filename);

            auto res = reader.Initialize();

            if(ILE_SUCCESS != res) {
                LG(WARN, "Cannot read '%s' as a '.wav' file. If the file exists in '%s', it might be corrupted.", filename.c_str(), dirname.c_str());
                return false;
            }

            auto mod = reader.countChannels() % OutputData::nOut;
            if((reader.countChannels() > OutputData::nOut) && mod) {
                LG(ERR, "cannot use a '%d' channels reverb for '%d' outs", reader.countChannels(), OutputData::nOut);
                return false;
            }

            FFT_T stride = reader.getSampleRate() / static_cast<float>(SAMPLE_RATE);

            std::vector<FFT_T> buf(static_cast<int>(reader.countFrames() / stride) * reader.countChannels());
            MultiChannelReSampling<decltype(reader)> mci(reader);
            mci.Read(buf.begin(), buf.end(), stride);
            buf.resize(std::distance(buf.begin(), buf.end()));

            chans.setConvolutionReverbIR(std::move(buf),
                                        reader.countChannels(),
                                        wait_for_first_n_audio_cb_frames());
            return true;
        }

        template <typename T, Features F, AudioPlatform AUP >
        struct AudioOutContext : public Context<AUP, F, T> {
            static constexpr auto nAudioOut = T::nOuts;
            using Chans=T;
            using LockFromRT = typename Chans::LockFromRT;
            using LockFromNRT = typename Chans::LockFromNRT;
            using Request = typename Chans::Request;
            using Volumes = typename Chans::ChannelsT::Volumes;
            using Base = Context<AUP, F, T>;
            using Base::chans;
            using Base::bInitialized;
            using Base::doInit;
          
            // the min latency used in case the initialization is done lazily
            static constexpr float minLazyLatency = 0.005f;


        private:
            static constexpr auto xfade_on_close = 200;

            bool closing : 1;

        public:
            template<typename Lock, typename ...Args>
            AudioOutContext(Lock & l, Args... args) : Base(l, args ...)
            , closing(false)
            {}

            ~AudioOutContext() {
                finalize();
            }

            bool Initialized() const { return bInitialized; }

            auto & getChannelHandler() { return chans; }

            void onApplicationShouldClose() {
                if(closing) {
                    return;
                }
                closing = true;
                chans.getChannels().closeAllChannels(xfade_on_close);
                LG(INFO, "Fading out Audio before shutdown...");
            }

            void finalize() {
                if(!bInitialized) {
                    return;
                }
                chans.getChannels().closeAllChannels(0);
            }

            void Init(float minLatency) {
                if(bInitialized) {
                    return;
                }
                if(doInit(minLatency)) {
                    initializeConvolutionReverb();
                }
            }

            void initializeConvolutionReverb()
            {
                // for Wind app we want to let the user decide to have reverb
                dontUseConvolutionReverbs(chans);

                // this one needs to be high pass filtered (5hz loud stuff)
                /*    std::string dirname = std::string(impulse_responses_root_dir) + "/nyc.showroom";
                 constexpr auto filename = "BigRoomStereo (16).wav";
                 //std::string dirname = std::string(impulse_responses_root_dir) + "/im.reverbs";
                 //constexpr auto filename = "Conic Long Echo Hall.wav";
                 audio::useConvolutionReverb(chans, dirname, filename);
                 */
            }


            uint8_t openChannel(float volume, ChannelClosingPolicy p, int xfade_length) {
                if(closing) {
                    return AUDIO_CHANNEL_NONE;
                }
                Init(minLazyLatency);
                return getFirstXfadeInfiniteChans().template openChannel<WithLock::Yes>(volume, p, xfade_length);
            }

            void play( uint8_t channel_id, StackVector<Request> && v ) {
                if(closing) {
                    return;
                }
                getFirstXfadeInfiniteChans().play( channel_id, std::move( v ) );
            }

            template<typename U>
            void playGeneric( uint8_t channel_id, U & buf, Request && req ) {
                if(closing) {
                    return;
                }
              getFirstXfadeInfiniteChans().playGeneric( chans, channel_id, buf, std::move(req) );
            }

            void toVolume( uint8_t channel_id, float volume, int nSteps ) {
                if(closing) {
                    return;
                }
                getFirstXfadeInfiniteChans().toVolume( channel_id, volume, nSteps);
            }

            void closeChannel(uint8_t channel_id, CloseMode mode) {
                if(closing) {
                    return;
                }
                getFirstXfadeInfiniteChans().closeChannel( channel_id, mode );
            }

          auto & getFirstXfadeInfiniteChans() {
            auto p = getChannelHandler().getChannels().getChannelsXFadeInfinite()[0].get();
            Assert(p);
            return *p;
          }
          auto & getFirstXfadeChans() {
            auto p = getChannelHandler().getChannels().getChannelsXFade()[0].get();
            Assert(p);
            return *p;
          }
        };

    }
}
