#if 0

Analyse d'un enregistrement pour la justesse, pour voir les tendances que j'ai.

Analyse en temps reel de la justesse?



#endif

namespace imajuscule::audio {

namespace audioelement {

template <typename T>
using TuneElement =
//StereoPanned<
VolumeAdjusted<
//Enveloped
MultiEnveloped
<
  //FreqCtrl_<
    SineOscillatorAlgo<T, eNormalizePolicy::FAST>
    //, InterpolatedFreq<T>
  //>
  , AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease>
>
>
//>
;


template<typename T>
struct TuneElementInitializer {
  TuneElementInitializer(int const sample_rate,
                            int const stride,
                            float const stereo_spread,
                            audioelement::AHDSR const & a,
                         std::vector<harmonicProperties_t> harmonics)
  : sample_rate(sample_rate)
  , stride(stride)
  , stereo_spread_(stereo_spread)
  , ahdsr(a)
  , m_harmonics(std::move(harmonics))
  {}
  
  void operator()(audioelement::TuneElement<T> & e) const {
    // order is important, first set harmonics then AHDSR
    e.editEnvelope().setHarmonics(m_harmonics,
                                  sample_rate);
    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    
    // limit the speed of volume adjustment:
    
    e.getVolumeAdjustment().setMaxFilterIncrement(2. / static_cast<double>(stride));
    
    // frequency control
    
    //e.getVolumeAdjustment().getOsc().getAlgo().getCtrl().setup(stride,
    //                                                           itp::LINEAR);
  }
  
  bool operator ==(TuneElementInitializer const& o) const {
    return
    std::make_tuple(sample_rate, stride, stereo_spread_, ahdsr) ==
    std::make_tuple(o.sample_rate, o.stride, o.stereo_spread_, o.ahdsr);
  }
  bool operator !=(TuneElementInitializer const& o) const {
    return !this->operator ==(o);
  }
  
private:
  int sample_rate;
  int stride;
  float stereo_spread_;
  audioelement::AHDSR ahdsr;
  std::vector<harmonicProperties_t> m_harmonics;
};

} // NS audioelement

template<typename PitchGen>
struct ShufflePattern{
  ShufflePattern(PitchGen & gen, std::vector<size_t> pattern)
  : m_pattern(std::move(pattern))
  , m_gen(gen)
  {
    auto maxIndex = 0ul;
    for(size_t i : m_pattern)
      maxIndex = std::max(maxIndex, i);
    m_nextValues.resize(maxIndex + 1);
    m_patternIndex = m_pattern.size();

    for(auto & val : m_nextValues)
      val = m_gen();
  }

  double operator()() {
    if(m_patternIndex >= m_pattern.size())
    {
      m_patternIndex = 0;
      std::rotate(m_nextValues.begin(), m_nextValues.begin() + 1, m_nextValues.end());
      m_nextValues.back() = m_gen();
    }
    return m_nextValues[m_pattern[m_patternIndex++]];
  }

private:
  PitchGen& m_gen;
  std::vector<double> m_nextValues;
  std::vector<size_t> m_pattern;
  size_t m_patternIndex{};
};

void tune(int const sample_rate) {
  auto mk_note_id = [] {
    static int64_t i = 0;
    ++i;
    return NoteId{i};
  };

  auto ms_to_frames = [=](float ms){
    return audio::ms_to_frames(ms, sample_rate);
  };

  static constexpr int nAudioOut = 2;

  static constexpr int nVoices = 128;

  using Synth = sine::Synth <
  nAudioOut,
  audioelement::TuneElement<double>,
  SynchronizePhase::Yes,
  DefaultStartPhase::Zero,
  HandleNoteOff::Yes,
  nVoices,
  audioelement::TuneElementInitializer<double>>;
  
  Synth synth;
  
  std::filesystem::path harmonicsFile{"/Users/Olivier/Dev/cpp.audio/Harmonics.txt"};
  
  std::filesystem::file_time_type lastWriteHarmonics = std::filesystem::last_write_time(harmonicsFile);

  auto mkHarmonics = [&]()
  {
    std::ifstream file(harmonicsFile);
    std::string str;
    
    std::vector<float> volumes;
    {
      float maxVol{};
      while (std::getline(file, str))
      {
        volumes.push_back(str.size());
        maxVol = std::max(maxVol, volumes.back());
      }
      if(maxVol)
        for(auto & vol : volumes)
          vol /= maxVol;
      else
      {
        // safety net
        volumes.clear();
        volumes.push_back(1.f);
      }
    }
    std::vector<harmonicProperties_t> res;
    for(const auto & vol : volumes)
      res.push_back(harmonicProperties_t{
        0., // phase
        vol  // volume
      });
    return res;
  };

  float pitchDrift = 0.f;
  constexpr auto constantDrift =
  //0.05
  //0.01
  //0.02
  0
  ;
  auto withPitchDrift = [&](float pitch)
  {
    pitchDrift += constantDrift;
    return pitchDrift + pitch;
  };

  Midi midi;
  
  constexpr auto& scale = well_tempered::c_minorScaleAsc;
  //constexpr auto& scale = well_tempered::c_majorScaleAsc;
  //const auto scale = just::mkMajorScaleAsc<Constexpr::Yes>();
  //const auto scale = pythagorean::mkMajorScaleAsc<Constexpr::Yes>();

  const int countOctaves = 2;
  MultiOctave multiOctave{
    scale.begin(),
    scale.end(),
    midi.get_pitch(NoteOctave{Note::La, 2}),
    countOctaves
  };
  
  //auto& pitchGen = multiOctave;
  auto shufflePattern = ShufflePattern{multiOctave, std::vector<size_t>{0, 2, 1, 3, 2, 4, 3, 2}};
  auto& pitchGen = shufflePattern;

  const float volume = 1.f;
  
  float timeScaleFactor = 0.09;
  
  const auto wait_after_note_off = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 300));
  const auto wait_after_note_on = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 800));
    
  auto updateInitializer = [&]()
  {
    std::vector<harmonicProperties_t> harmonics = mkHarmonics();
    
    auto initializer = audioelement::TuneElementInitializer<double>{
      sample_rate,
      1, // not used,
      0., // not used,
      // Todo read from JSON interactively
      audioelement::AHDSR{
        // attack
        ms_to_frames(timeScaleFactor * 150),
        itp::interpolation::LINEAR,
        // hold
        ms_to_frames(timeScaleFactor * 0),
        // decay
        ms_to_frames(timeScaleFactor * 150),
        itp::interpolation::EASE_OUT_CUBIC,
        // release
        ms_to_frames(timeScaleFactor * 500),
        itp::interpolation::EASE_OUT_CUBIC,
        // sustain
        0.6
      },
      harmonics
    };
    synth.setSynchronousElementInitializer(initializer);
  };

  static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

  using Ctxt = Context<
  AudioPlatform::PortAudio,
  Features::JustOut
  >;
  
  using Stepper = SimpleAudioOutContext<
  nAudioOut,
  audioEnginePolicy
  >;

  Ctxt ctxt;
  Stepper stepper(GlobalAudioLock<audioEnginePolicy>::get(),
                  Synth::n_channels * 4 /* one shot */,
                  1 /* a single compute is needed (global for the synth)*/);
  
  if (!ctxt.doInit(0.008,
                   sample_rate,
                   nAudioOut,
                   [&stepper,
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
    throw std::runtime_error("ctxt init failed");
  }

  synth.initialize(stepper);
  
  updateInitializer();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::optional<NoteId> noteid;

  std::optional<double> prevMidiPitch;

  while(true) {

    if(noteid) {

      auto res = synth.onEvent(sample_rate,
                               mkNoteOff(*noteid),
                               stepper,
                               stepper,
                               {});
      // std::cout << "XXX " << *noteid << " " << res << std::endl;
      noteid.reset();
      std::this_thread::sleep_for(wait_after_note_off);
  
    }

    const float midiPitch = withPitchDrift(pitchGen());
    const float frequency = midi.midi_pitch_to_freq(midiPitch);
    noteid = mk_note_id();
    auto const res  = synth.onEvent(sample_rate,
                                    mkNoteOn(*noteid,
                                             frequency,
                                             volume),
                                    stepper,
                                    stepper,
                                    {});


#if 0
    const auto noteWithDeviation = midi_pitch_to_note_deviation(midiPitch);
    std::cout << *noteid << ": " << frequency << " Hz (" << midiPitch << " " <<
    noteWithDeviation << " "
    //<< (prevMidiPitch.has_value() ? (midiPitch - *prevMidiPitch) : 0.) << ") "
    << res << std::endl;
#endif

    prevMidiPitch = midiPitch;
    
    std::filesystem::file_time_type newLastWriteHarmonics = std::filesystem::last_write_time(harmonicsFile);
    if(newLastWriteHarmonics > lastWriteHarmonics)
    {
      updateInitializer();
      lastWriteHarmonics = newLastWriteHarmonics;
    }
    std::this_thread::sleep_for(wait_after_note_on);
  }

  synth.finalize(stepper);

  std::this_thread::sleep_for(wait_after_note_on); // so that the audio produced during finalization has a chance to be played

  ctxt.doTearDown();
}

} // NS

int main() {
  imajuscule::audio::tune(96000);
  return 0;
}
