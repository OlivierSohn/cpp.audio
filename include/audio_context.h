

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
        struct GlobalAudioLock<AudioOutPolicy::MasterLockFree> {
            static auto & get() {
                static AudioLockPolicyImpl<AudioOutPolicy::MasterLockFree> l;
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

        bool overridePortaudioMinLatencyMillis(int latency);

        constexpr auto impulse_responses_root_dir = "audio.ir";

        int wait_for_first_n_audio_cb_frames();

        template<typename OutputData>
        bool useConvolutionReverb(OutputData & chans,
                                  std::string const & dirname, std::string const & filename, ResponseTailSubsampling rts) {
            WAVReader reader(dirname, filename);

            auto res = reader.Initialize();

            if(ILE_SUCCESS != res) {
                LG(WARN, "Cannot read '%s' as a '.wav' file. If the file exists in '%s', it might be corrupted.", filename.c_str(), dirname.c_str());
                return false;
            }

            auto mod = reader.countChannels() % OutputData::nOuts;
            if((reader.countChannels() > OutputData::nOuts) && mod) {
                LG(ERR, "cannot use a '%d' channels reverb for '%d' outs", reader.countChannels(), OutputData::nOuts);
                return false;
            }

            double stride = reader.getSampleRate() / static_cast<double>(SAMPLE_RATE);

            std::vector<double> buf(static_cast<int>(reader.countFrames() / stride) * reader.countChannels());
            MultiChannelReSampling<decltype(reader)> mci(reader);
            mci.Read(buf.begin(), buf.end(), stride);
            buf.resize(std::distance(buf.begin(), buf.end()));

            return chans.getPost().setConvolutionReverbIR(std::move(buf),
                                                          reader.countChannels(),
                                                          wait_for_first_n_audio_cb_frames(),
                                                        rts);
        }

        constexpr int xfade_on_close = 5000; // in samples

        template <typename T, Features Feat, AudioPlatform AUP >
        struct AudioOutContext : public Context<AUP, Feat, T> {
            static constexpr auto nAudioOut = T::nOuts;
            static constexpr auto policy = T::policy;
            using Chans=T;
            using LockFromRT = typename Chans::LockFromRT;
            using LockFromNRT = typename Chans::LockFromNRT;
            using Request = typename Chans::Request;
            using Volumes = typename Chans::ChannelsT::Volumes;
            using Base = Context<AUP, Feat, T>;
            using Base::chans;
            using Base::bInitialized;
            using Base::doInit;
            using Base::doTearDown;

            // the min latency used in case the initialization is done lazily
            static constexpr float minLazyLatency = 0.005f;

        private:

            std::atomic_bool closing;

        public:
            template<typename ...Args>
            AudioOutContext(Args... args) : Base(GlobalAudioLock<policy>::get(), args ...)
            {
              closing.store(false, std::memory_order_relaxed);
            }

            ~AudioOutContext() {
                finalize();
            }

            bool Initialized() const { return bInitialized; }

            auto & getChannelHandler() { return chans; }

            void onApplicationShouldClose() {
              auto cur = false;
              if(!closing.compare_exchange_strong(cur, true, std::memory_order_acq_rel)) {
                // already done
                return;
              }
              chans.getChannels().closeAllChannels(xfade_on_close);
              LG(INFO, "Fading out Audio before shutdown...");
            }

            void finalize() {
              if(bInitialized) {
                chans.getChannels().closeAllChannels(0);
              }
            }

            /*
            During this method call, no concurrent call to any other method is allowed.
            */
            void TearDown() {
              // because we want to be able to use *this again after a successfull 'Init' :
              closing.store(false, std::memory_order_release);
              finalize();
              doTearDown();
            }

            [[nodiscard]] bool Init(float minLatency) {
              if(bInitialized) {
                return true;
              }
              if(!doInit(minLatency)) {
                return false;
              }
              initializeConvolutionReverb();
              return true;
            }

            void initializeConvolutionReverb()
            {
              dontUseConvolutionReverbs(chans);

              // this one needs to be high pass filtered (5hz loud stuff)
              /*    std::string dirname = std::string(impulse_responses_root_dir) + "/nyc.showroom";
               constexpr auto filename = "BigRoomStereo (16).wav";
               //std::string dirname = std::string(impulse_responses_root_dir) + "/im.reverbs";
               //constexpr auto filename = "Conic Long Echo Hall.wav";
               audio::useConvolutionReverb(chans, dirname, filename);
               */
            }


          uint8_t openChannel(float volume, ChannelClosingPolicy p, int xfade_length)
          {
            if(closing.load(std::memory_order_acquire)) {
              return AUDIO_CHANNEL_NONE;
            }
            if(!Init(minLazyLatency)) {
              return AUDIO_CHANNEL_NONE;
            }
            if(auto c = getFirstXfadeInfiniteChans()) {
              return c->template openChannel<WithLock::Yes>(volume, p, xfade_length);
            }
            return AUDIO_CHANNEL_NONE;
          }

          [[nodiscard]] bool play( uint8_t channel_id, StackVector<Request> && v ) {
            if(closing.load(std::memory_order_acquire)) {
              return false;
            }
            if(auto c = getFirstXfadeInfiniteChans()) {
              return c->play( channel_id, std::move( v ) );
            }
            return false;
          }

          template<typename Algo>
          [[nodiscard]] bool playComputable( PackedRequestParams<nAudioOut> params, audioelement::FinalAudioElement<Algo> & e) {
            if(closing.load(std::memory_order_acquire)) {
              return false;
            }
            if(auto c = getFirstXfadeInfiniteChans()) {
              return c->playComputable( params, e);
            }
            return false;
          }

          void toVolume( uint8_t channel_id, float volume, int nSteps ) {
            if(closing.load(std::memory_order_acquire)) {
              return;
            }
            if(auto c = getFirstXfadeInfiniteChans()) {
              c->toVolume( channel_id, volume, nSteps);
            }
          }

          void closeChannel(uint8_t channel_id, CloseMode mode) {
            if(closing.load(std::memory_order_acquire)) {
              return;
            }
            if(auto c = getFirstXfadeInfiniteChans()) {
              c->closeChannel( channel_id, mode );
            }
          }

          typename Chans::ChannelsT::XFadeInfiniteChans * getFirstXfadeInfiniteChans() {
            if(auto m = getChannelHandler().getChannels().getChannelsXFadeInfinite().maybe_front()) {
              return &get_value(m).first;
            }
            return {};
          }

          typename Chans::ChannelsT::XFadeChans * getFirstXfadeChans() {
            if(auto m = getChannelHandler().getChannels().getChannelsXFade().maybe_front()) {
              return &get_value(m).first;
            }
            return {};
          }
        };

    }
}
