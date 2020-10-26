

namespace imajuscule::audio {

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

template<typename Post>
bool useConvolutionReverb(int const sample_rate,
                          Post & post,
                          std::string const & dirname, std::string const & filename) {

  try{
    WAVReader reader(dirname, filename);
    reader.Initialize();
    ResampleSincStats stats;
    using T = double;
    InterlacedBuffer ib(reader, sample_rate, stats);
    if (ib.countChannels() <= 0) {
      throw std::runtime_error("negative count channels");
    }
    if (ib.countFrames() <= 0) {
      throw std::runtime_error("negative count frames");
    }
    DeinterlacedBuffers<T> db(ib);
    return post.setConvolutionReverbIR(sample_rate,
                                       db,
                                       wait_for_first_n_audio_cb_frames());
  }
  catch(std::exception const & e) {
    LG(ERR, "useConvolutionReverb error : %s", e.what());
    return false;
  }
  return true;
}

constexpr int xfade_on_close = 5000; // in samples

template <typename Stepper, Features Feat, AudioPlatform AUP >
struct AudioOutContext : public Context<AUP, Feat> {
  static constexpr auto nAudioOut = Stepper::nOuts;
  static constexpr auto policy = Stepper::policy;
  using LockFromRT = typename Stepper::LockFromRT;
  using LockFromNRT = typename Stepper::LockFromNRT;
  using Request = typename Stepper::Request;
  using Volumes = typename Stepper::ChannelsT::Volumes;
  using Base = Context<AUP, Feat>;
  using Base::doInit;
  using Base::doTearDown;

  // the min latency used in case the initialization is done lazily
  static constexpr float minLazyLatency = 0.008f;
  static constexpr int lazySamplingRate = 96000;

private:

  std::atomic_bool closing;
  std::optional<int> sample_rate_;
  Stepper stepper;

public:
  template<typename ...Args>
  AudioOutContext(Args... args)
  : stepper(GlobalAudioLock<policy>::get(), args ...)
  {
    closing.store(false, std::memory_order_relaxed);
  }

  std::optional<int> getSampleRate() const {
    return sample_rate_;
  }

  ~AudioOutContext() {
    finalize();
  }

  auto & getChannelHandler() { return stepper; }

  void onApplicationShouldClose() {
    auto cur = false;
    if(!closing.compare_exchange_strong(cur, true, std::memory_order_acq_rel)) {
      // already done
      return;
    }
    stepper.getChannels().closeAllChannels(xfade_on_close);
    LG(INFO, "Fading out Audio before shutdown...");
  }

  void finalize() {
    // Commented out because 'closeAllChannels' uses a queue to execute its action,
    // but since the audio stream is deactivated right after this, the action stays
    // in the queue until another stream reactivates the queue, and the channel
    // close themselves at that point, which is not what we want. (Note that the channel activation
    // is not done through the queue, but directly).
    //
    // Another good reason to comment this out is that it is recommended to call 'onApplicationShouldClose'
    // and sleep one second before calling finalize, so if you follow this recommendation,
    // the channels should be closed already at this point, hence this action becomes useless.

    /*if(Initialized()) {

      stepper.getChannels().closeAllChannels(0);
    }*/
  }

  /*
   During this method call, no concurrent call to any other method is allowed.
   */
  void TearDown() {
    // because we want to be able to use *this again after a successfull 'Init' :
    closing.store(false, std::memory_order_release);
    finalize();
    doTearDown();
    sample_rate_.reset();
  }

  [[nodiscard]] bool Init(int sample_rate, float minLatency) {
    if(this->Initialized()) {
      return true;
    }
    if(!doInit(minLatency,
               sample_rate,
               nAudioOut,
               [this,
                nanos_per_audioelement_buffer = static_cast<uint64_t>(0.5f +
                                                                      audio::nanos_per_frame<float>(sample_rate) *
                                                                      static_cast<float>(audio::audioelement::n_frames_per_buffer))]
               (SAMPLE *outputBuffer,
                int nFrames,
                uint64_t const tNanos){
      stepper.step(outputBuffer,
                   nFrames,
                   tNanos,
                   nanos_per_audioelement_buffer);
    })) {
      return false;
    }
    sample_rate_ = sample_rate;
    initializeConvolutionReverb(sample_rate);
    return true;
  }

  void initializeConvolutionReverb(int sample_rate)
  {
    dontUseConvolutionReverbs(stepper,
                              sample_rate);

    // this one needs to be high pass filtered (5hz loud stuff)
    /*    std::string dirname = std::string(impulse_responses_root_dir) + "/nyc.showroom";
     constexpr auto filename = "BigRoomStereo (16).wav";
     //std::string dirname = std::string(impulse_responses_root_dir) + "/im.reverbs";
     //constexpr auto filename = "Conic Long Echo Hall.wav";
     audio::useConvolutionReverb(sample_rate_, stepper, dirname, filename);
     */
  }


  uint8_t openChannel(float volume, ChannelClosingPolicy p, int xfade_length)
  {
    if(closing.load(std::memory_order_acquire)) {
      return AUDIO_CHANNEL_NONE;
    }
    if(!Init(lazySamplingRate, minLazyLatency)) {
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
  [[nodiscard]] bool playComputable(PackedRequestParams<nAudioOut> params,
                                    audioelement::FinalAudioElement<Algo> & e) {
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

  typename Stepper::ChannelsT::XFadeInfiniteChans * getFirstXfadeInfiniteChans() {
    if(auto m = getChannelHandler().getChannels().getChannelsXFadeInfinite().maybe_front()) {
      return &get_value(m).first;
    }
    return {};
  }

  typename Stepper::ChannelsT::NoXFadeChans * getFirstNoXfadeChans() {
    if(auto m = getChannelHandler().getChannels().getChannelsNoXFade().maybe_front()) {
      return &get_value(m).first;
    }
    return {};
  }
};

} // NS
