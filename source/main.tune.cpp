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


// Helper class to play a sequence of notes using a pattern of indexes.
//
// We play the sequence of notes using the index pattern once,
// then we increment all indexes,
// then repeat.
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


// Helper class to generate a slight pitch drift over time,
// mimicking what can happen on the cello when we don't play
// any open string for a long time.
struct PitchDrifter
{
  static constexpr auto constantDrift =
  //0.05
  //0.01
  //0.02
  0
  ;

  float operator()(float pitch)
  {
    pitchDrift += constantDrift;
    return pitchDrift + pitch;
  }

private:
  float pitchDrift = 0.f;
};

using Events = std::vector<std::pair<TimestampAndSource, Event>>;
struct Loop
{
  Loop(Events && e, uint64_t lNanos)
  : events(std::move(e))
  , lengthNanos(lNanos)
  {}

  Events events;
  uint64_t lengthNanos{};
};

Loop mkEvents(int countNotes = 100);
Loop eventsFrom(std::filesystem::path const&);

static NoteId mk_note_id() {
  static int64_t i = 0;
  ++i;
  return NoteId{i};
}

// Main class.
struct AppTune
{
  AppTune();
  ~AppTune();

  void setHarmonicsFile(std::filesystem::path e)
  {
    if(e == harmonicsFile)
      return;
    harmonicsFile = e;
    m_lastWriteSynthFiles.reset();
  }
  void setEnvelopeFile(std::filesystem::path e)
  {
    if(e == envelopeFile)
      return;
    std::cout << "Using envelope " << e << std::endl;
    envelopeFile = e;
    m_lastWriteSynthFiles.reset();
  }

  void playEvents(Loop && loop, uint64_t countLoops = 1000);

private:

  // This file defines the harmonics of the synthesizer.
  //
  // The following syntax is used in this file:
  //
  // ....   // 1st harmonic has a relative weight 4
  //        // no 2nd harmonic
  // .      // 3rd harmonic has a relative weight 1
  // ...    // 4th harmonic has a relative weight 3
  std::filesystem::path harmonicsFile{"/Users/Olivier/Dev/cpp.audio/Synth/Harmonics.txt"};
  
  // This file defines the envelope of the synthesizer.
  //
  // The following syntax is used in this file:
  // A .
  // H ..
  // D ....
  // S ...
  // R ...
  std::filesystem::path envelopeFile{"/Users/Olivier/Dev/cpp.audio/Synth/Envelope.txt"};

  static constexpr int m_sampleRate{96000};
  static constexpr int nAudioOut = 2;
  
  static constexpr int nVoices = 128;

  using Synth = sine::Synth <
  nAudioOut,
  audioelement::TuneElement<double>,
  TryAccountForTimeSourceJitter::No,
  SynchronizePhase::Yes,
  DefaultStartPhase::Zero,
  HandleNoteOff::Yes,
  nVoices,
  audioelement::TuneElementInitializer<double>>;
  
  static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;
  
  using Ctxt = Context<
  AudioPlatform::PortAudio,
  Features::JustOut,
  // This monotonic time ensures that the audio played will not depend on thread scheduler timing.
  TimeSource::Monotonic
  >;
  
  using Stepper = SimpleAudioOutContext<
  nAudioOut,
  audioEnginePolicy
  >;
  
  
  Ctxt m_ctxt;
  Synth m_synth;
  Stepper m_stepper;
    
  // Represents the min time of the harmonics/envelope files that were
  // last used to produce the AE initializer.
  std::optional<std::filesystem::file_time_type> m_lastWriteSynthFiles;
  
  void updateInitializerIfNeeded();

  auto ms_to_frames (float ms){
    return audio::ms_to_frames(ms, m_sampleRate);
  }
};

AppTune::AppTune()
: m_stepper(GlobalAudioLock<audioEnginePolicy>::get(),
            Synth::n_channels * 4 /* one shot */,
            1 /* a single compute is needed (global for the synth)*/)
{
  m_synth.initialize(m_stepper);

  if (!m_ctxt.doInit(0.008,
                     m_sampleRate,
                     nAudioOut,
                     [this]
                     (SAMPLE *outputBuffer,
                      int nFrames,
                      uint64_t const tNanos){
    m_stepper.step(outputBuffer,
                   nFrames,
                   tNanos);
  })) {
    throw std::runtime_error("ctxt init failed");
  }

  // I guess we need this wait because it could take some time for the audio callback to be
  // up and running.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

AppTune::~AppTune()
{
  m_synth.finalize(m_stepper);
  
  // We sleep so that the audio produced during finalization has a chance to be played
  // (taking into account the delay between when the audio is written in the buffer
  //  and when the audio is actually played in the speakers).

  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  
  m_ctxt.doTearDown();
}


void AppTune::updateInitializerIfNeeded()
{
  {
    const std::filesystem::file_time_type newLastWriteHarmonics =
    std::min(std::filesystem::last_write_time(harmonicsFile), std::filesystem::last_write_time(envelopeFile));
    if(m_lastWriteSynthFiles.has_value() && newLastWriteHarmonics < *m_lastWriteSynthFiles)
      // No need to updatem the files have not changed since last time the initializer has been updated.
      return;
    // This initializer update will affect any note that has not started yet.
    m_lastWriteSynthFiles = newLastWriteHarmonics;
  }

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
  
  auto mkEnvelope = [&]()
  {
    std::ifstream file(envelopeFile);
    std::string str;
    std::map<char, int> e;
    e['a'] = 0;
    e['h'] = 0;
    e['d'] = 0;
    e['s'] = 0;
    e['r'] = 0;
    const float constant = 10.f;
    while (std::getline(file, str))
    {
      if(!str.empty())
        e[tolower(str[0])] = constant * std::count(str.begin(), str.end(), '.');
    }
    return audioelement::AHDSR{
      // attack
      ms_to_frames(e['a']),
      itp::interpolation::LINEAR,
      // hold
      ms_to_frames(e['h']),
      // decay
      ms_to_frames(e['d']),
      itp::interpolation::EASE_OUT_CUBIC,
      // release
      ms_to_frames(e['r']),
      itp::interpolation::EASE_OUT_CUBIC,
      // sustain
      0.1f * e['s'] / constant
    };
  };
  std::vector<harmonicProperties_t> harmonics = mkHarmonics();
  audioelement::AHDSR env = mkEnvelope();
  
  auto initializer = audioelement::TuneElementInitializer<double>{
    m_sampleRate,
    1, // not used,
    0., // not used,
    env,
    harmonics
  };
  m_synth.setSynchronousElementInitializer(initializer);
}

Loop mkEvents(int countNotes)
{
  std::cout << "Generating " << countNotes << " notes" << std::endl;
  Midi m_midi;
  std::vector<std::pair<TimestampAndSource, Event>> events;
  
  PitchDrifter withPitchDrift;

  //constexpr auto& scale = well_tempered::c_minorScaleAsc;
  //constexpr auto& scale = well_tempered::c_majorScaleAsc;
  //const auto scale = just::mkMajorScaleAsc<Constexpr::Yes>();
  const auto scale = pythagorean::mkMajorScaleAsc<Constexpr::Yes>();
  
  const int countOctaves = 2;
  MultiOctave multiOctave{
    scale.begin(),
    scale.end(),
    m_midi.get_pitch(NoteOctave{Note::La, 2}),
    countOctaves
  };
  
  //auto& pitchGen = multiOctave;
  auto shufflePattern = ShufflePattern{multiOctave, std::vector<size_t>{0, 2, 1, 3, 2, 4, 3, 2}};
  auto& pitchGen = shufflePattern;
    
  // Scales time.
  static constexpr float timeScaleFactor = 0.09;
  // Duration of a note
  static constexpr auto wait_after_note_on = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 800));
  // Pause length between consecutive notes
  static constexpr auto wait_after_note_off = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 300));

  const float volume = 1.f;

  uint64_t curTimeNanos{};
  for(int i=0; i<countNotes; ++i)
  {
    const auto noteid = mk_note_id();
    {
      const float midiPitch = withPitchDrift(pitchGen());
      const float frequency = m_midi.midi_pitch_to_freq(midiPitch);
      
      // if needed we could generate the string below, and print it when we
      // schedule the note to be played
      // so that the user visualizes the pitch deviation.
#if 0
      const auto noteWithDeviation = midi_pitch_to_note_deviation(midiPitch);
      std::cout << *noteid << ": " << frequency << " Hz (" << midiPitch << " " <<
      noteWithDeviation << " "
      //<< (prevMidiPitch.has_value() ? (midiPitch - *prevMidiPitch) : 0.) << ") "
      << res << std::endl;
#endif
      events.emplace_back(TimestampAndSource{curTimeNanos, 0},
                          mkNoteOn(noteid,
                                   frequency,
                                   volume));
    }
    curTimeNanos += std::chrono::duration_cast<std::chrono::nanoseconds>(wait_after_note_on).count();
    events.emplace_back(TimestampAndSource{curTimeNanos, 0},
                        mkNoteOff(noteid));
    curTimeNanos += std::chrono::duration_cast<std::chrono::nanoseconds>(wait_after_note_off).count();
  }
  
  return Loop{std::move(events), curTimeNanos};
}

Loop eventsFrom(std::filesystem::path const& scoreFile)
{
  std::cout << "Generating " << scoreFile << std::endl;
  std::ifstream file(scoreFile);
  
  Midi m_midi;
  std::vector<std::pair<TimestampAndSource, Event>> events;
  
  // 0  Do
  // 1  Do#
  // 2  Re
  // 3  Re#
  // 4  Mi
  // 5  Fa
  // 6  Fa#
  // 7  Sol
  // 8  Sol#
  // 9  La
  // A  La#
  // B  Si
  
  std::vector<std::string> chars;
  {
    std::string str;
    while (std::getline(file, str))
      chars.emplace_back(str);
  }
  
  // Scales time.
  static constexpr float timeScaleFactor = 0.09;
  // Duration of a note
  static constexpr auto wait_after_note_on = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 800));
  // Pause length between consecutive notes
  static constexpr auto wait_after_note_off = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 300));
  
  const float volume = 1.f / std::max(1ul, chars.size());
  
  uint64_t maxCurTimeNanos{};
  
  uint64_t voice{};
  for(const auto & str : chars)
  {
    uint64_t curTimeNanos{};
    for(auto c : str)
    {
      const auto noteid = mk_note_id();
      const auto semitones = [=]()
      {
        if(c >= '0' && c <= '9')
          return c - '0';
        return 10 + c - 'A';
      }();
      
      const float midiPitch = semitones + A_pitch + 3;
      const float frequency = m_midi.midi_pitch_to_freq(midiPitch);
      events.emplace_back(TimestampAndSource{curTimeNanos, voice},
                          mkNoteOn(noteid,
                                   frequency,
                                   volume));
      curTimeNanos += std::chrono::duration_cast<std::chrono::nanoseconds>(wait_after_note_on).count();
      events.emplace_back(TimestampAndSource{curTimeNanos, voice},
                          mkNoteOff(noteid));
      curTimeNanos += std::chrono::duration_cast<std::chrono::nanoseconds>(wait_after_note_off).count();
    }
    maxCurTimeNanos = std::max(maxCurTimeNanos, curTimeNanos);
    ++voice;
  }
  return {std::move(events), maxCurTimeNanos};
}

void AppTune::playEvents(Loop && loop, uint64_t countLoops)
{
  if(!countLoops)
    return;

  auto & events = loop.events;
  std::sort(events.begin(), events.end(), [](const auto & t1, const auto & t2){ return t1.first.getNanosTime() < t2.first.getNanosTime(); });
  
  // Use a delay of one second
  constexpr uint64_t bufferNanos{static_cast<uint64_t>(1e9)};
  
  // The first noteId that is not used.
  // This assumes that noteids are small contiguous integers which is the case here.
  int64_t endNoteId{};
  for(auto&[_, event]:events)
    endNoteId = std::max(event.noteid.noteid + 1l, endNoteId);

  {
    const uint64_t ctxtCurTimeNanos = nanos_per_frame<double>(m_sampleRate) * m_ctxt.getCountFrames();

    // This is the offset we need to apply to events if we want them to start playing right away.
    // We add one second to that so that audio won't depend on thread scheduling.
    const uint64_t offsetTimeNanos = ctxtCurTimeNanos + bufferNanos;
    
    for(auto&[time, _]:events)
      time.offsetNanosTime(offsetTimeNanos);
  }
  
  std::optional<uint64_t> lastCtxtScheduleTimeNanos;
  for(size_t firstIndex=0, sz = events.size();;)
  {
    size_t count_yields{};
  retry:
    
    updateInitializerIfNeeded();
    
    const uint64_t ctxtCurTimeNanos = nanos_per_frame<double>(m_sampleRate) * m_ctxt.getCountFrames();

    if(firstIndex < sz)
    {
      // Try to schedule events that need to be played during the next "bufferNanos" time.
      
      auto endIndex = firstIndex;
      for(;endIndex < sz; ++endIndex)
      {
        if(events[endIndex].first.getNanosTime() >= ctxtCurTimeNanos + bufferNanos)
          break;
      }
      
      if(firstIndex != endIndex)
      {
        // std::cout << endIndex - firstIndex << " " << count_yields << std::endl;
        for(; firstIndex != endIndex; ++firstIndex)
          auto const res = m_synth.onEvent(m_sampleRate,
                                           events[firstIndex].second,
                                           m_stepper,
                                           m_stepper,
                                           events[firstIndex].first);
        lastCtxtScheduleTimeNanos = ctxtCurTimeNanos;
        continue;
      }
      else
      {
        // There are remaining events in this iteration of the loop but we cannot schedule them yet.        
      }
    }
    else
    {
      // All events in this iteration of the loop have been scheduled.

      // Some notes may not have started being played yet, eventhough they are scheduled.
      // These notes may start being played up to "bufferNanos" in the future.
      
      if(countLoops && --countLoops)
      {
        firstIndex = 0;
        for(auto&[time, event]:events)
        {
          time.offsetNanosTime(loop.lengthNanos);
          // we offset the noteids otherwise some noteoff events may be ignored.
          // when we have several noteon and noteoff
          // events in the queue for the same noteid.
          event.offsetNoteId(endNoteId);
        }
        goto retry;
      }
      else if(!lastCtxtScheduleTimeNanos.has_value() || ((*lastCtxtScheduleTimeNanos + 2*bufferNanos) < ctxtCurTimeNanos))
      {
        // By now, all notes of the loop iteration have started (and stopped).
        break;
      }
    }
    
    ++count_yields;
    std::this_thread::yield();
    goto retry;
  }
  
  // All notes have started (and stopped) but some envelope tails may still be active.

  while(!m_synth.allEnvelopesFinished())
    std::this_thread::yield();
}

} // NS

int main() {
  using namespace imajuscule::audio;
  using namespace std::filesystem;

  auto a = AppTune{};
  
  const auto synth = path{"/Users/Olivier/Dev/cpp.audio/Synth/"};
  const auto scores = path{"/Users/Olivier/Dev/cpp.audio/Scores/"};
  
  // The ZeroEnvelope file produces drum sounds.
  const path zeroEnv{synth / "EnvelopeZero.txt"};
  const path fastEnv{synth / "EnvelopeFast.txt"};
  const path slowEnv{synth / "EnvelopeSlow.txt"};

  const path harSimple{synth / "HarmonicsSimple.txt"};

  a.setHarmonicsFile(harSimple);

  for(auto const &env : {zeroEnv, fastEnv, slowEnv})
  {
    a.setEnvelopeFile(env);
    
    a.playEvents(eventsFrom(scores / "Phrase2.txt"), 4);
    a.playEvents(eventsFrom(scores / "Phrase.txt"), 4);
    a.playEvents(eventsFrom(scores / "StrangeBots.txt"), 4);
    
    a.playEvents(mkEvents(250), 1);
  }
  return 0;
}
