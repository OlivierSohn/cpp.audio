#if 0

Analyse d'un enregistrement pour la justesse, pour voir les tendances que j'ai.

Analyse en temps reel de la justesse?

#endif

namespace imajuscule::audio {

namespace audioelement {

template <typename T>
using TuneElement =
//StereoPanned<
LowPassAlgo<
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
,
2
>
//>
;


template<typename T>
struct TuneElementInitializer {
  TuneElementInitializer(int const sample_rate,
                         int const stride,
                         float const stereo_spread,
                         float lowPassFreq,
                         audioelement::AHDSR const & a,
                         std::vector<harmonicProperties_t> harmonics)
  : sample_rate(sample_rate)
  , stride(stride)
  , stereo_spread_(stereo_spread)
  , ahdsr(a)
  , m_harmonics(std::move(harmonics))
  , m_lowPassFreq(lowPassFreq)
  {}
  
  void operator()(audioelement::TuneElement<T> & e) const {
    // order is important, first set harmonics then AHDSR
    e.editEnvelope().setHarmonics(m_harmonics,
                                  sample_rate);
    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    
    // limit the speed of volume adjustment:
    
    e.getVolumeAdjustment().setMaxFilterIncrement(2. / static_cast<double>(stride));
    
    // filtering
    
    e.setFilterAngleIncrements(freq_to_angle_increment(m_lowPassFreq,
                                                       sample_rate));
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
  float m_lowPassFreq;
  audioelement::AHDSR ahdsr;
  std::vector<harmonicProperties_t> m_harmonics;
};

} // NS audioelement


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
  std::filesystem::path envelopeFile{"/Users/Olivier/Dev/cpp.audio/Synth/EnvelopeFast.txt"};
  
  std::filesystem::path lowPassFile{"/Users/Olivier/Dev/cpp.audio/Synth/LowPass.txt"};
  
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

  std::atomic<size_t> m_countCinChars{};
  std::atomic_bool m_runCinListener{true};
  std::thread m_cinListener;

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
, m_cinListener{[&]()
  {
    std::string str;
    while(m_runCinListener)
    {
      try
      {
        std::cin >> str;
        if(!str.empty())
          m_countCinChars += str.size();
      }
      catch(std::exception const&)
      {}
    }
  }}
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
  
  m_runCinListener = false;
  m_cinListener.join();
}


void AppTune::updateInitializerIfNeeded()
{
  {
    const std::filesystem::file_time_type newLastWriteHarmonics =
    std::min(std::min(std::filesystem::last_write_time(harmonicsFile),
             std::filesystem::last_write_time(envelopeFile)),
             std::filesystem::last_write_time(lowPassFile));
    if(m_lastWriteSynthFiles.has_value() && newLastWriteHarmonics < *m_lastWriteSynthFiles)
      // No need to updatem the files have not changed since last time the initializer has been updated.
      return;
    // This initializer update will affect any note that has not started yet.
    m_lastWriteSynthFiles = newLastWriteHarmonics;
  }
  
  auto mkLowPass = [&]()
  {
    std::ifstream file(lowPassFile);
    std::string str;
    
    while (std::getline(file, str))
    {
      try
      {
        if(!str.empty())
          return std::stof(str);
      }
      catch(std::exception const&)
      {}
    }
    return 440.f;
  };

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
    mkLowPass(),
    env,
    harmonics
  };
  m_synth.setSynchronousElementInitializer(initializer);
}

// ConsecutivePitches is a list of consecutive pitches.
struct ConsecutivePitches
{
  std::vector<MidiPitch> m_pitches;
};

// a Score is a list of voices playing simultaneously.
struct Score
{
  std::vector<ConsecutivePitches> m_voices;  
};

// "Simple ascii" pitches encoding is intentionally made somewhat obscure so that
// when you write a character you have little idea what pitch you are going to get.
// This way melodies created with it are varied and surprising.
//
// One benefit of this encoding is that it takes a single character to create a note.
// So notes vertically aligned in a simple fixed-width text editor will play simultaneously.
//
// ## Description
//
// C5 (Do5) is the reference "zero" pitch
//
// ## Numeric characters
//
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
//
// ## Non-numeric characters
//
// pitch = Do5 + 10 + c - 'A'
MidiPitch decodePitchFromSimpleAsciiEncoding(char c)
{
  const auto semitones = [=]()
  {
    if(c >= '0' && c <= '9')
      return c - '0';
    return 10 + c - 'A';
  }();
  
  return A_pitch + semitones + 3;  
}
char encodePitchAsSimpleAscii(MidiPitch p)
{
  const int semitones = static_cast<int>(p - A_pitch - 3. + 0.5);
  if(semitones >= 0 && semitones <= 9)
    return '0' + semitones;
  return semitones + 'A' - 10;
}

Score readScoreFromSimpleAsciiPitchesEncoding(std::filesystem::path const& scoreFile)
{
  Score score;

  std::cout << "Generating " << scoreFile << std::endl;
  std::ifstream file(scoreFile);

  std::vector<std::string> chars;
  {
    std::string str;
    while (std::getline(file, str))
      chars.emplace_back(str);
  }
  for(const auto & str : chars)
  {
    score.m_voices.emplace_back();
    for(auto c : str)
      score.m_voices.back().m_pitches.push_back(decodePitchFromSimpleAsciiEncoding(c));
  }  
  return score;
}

bool writeScoreInSimpleAsciiPitchesEncoding(Score const & score, std::filesystem::path const& scoreFile)
{
  if(std::filesystem::exists(scoreFile))
    return false;
  
  std::ofstream f(scoreFile);
  for(const auto & voice : score.m_voices)
  {
    for(const auto & pitch : voice.m_pitches)
      f << encodePitchAsSimpleAscii(pitch);
    f << std::endl;
  }
  return true;   
}

Score mkScore(int countNotes)
{
  Score score;

  std::cout << "Generating " << countNotes << " notes" << std::endl;
  Midi m_midi;
    
  
  score.m_voices.emplace_back();

  static constexpr auto constantDrift =
  //0.05
  //0.01
  //0.02
  0
  ;
  PitchDrifter withPitchDrift(constantDrift);
  
  const MidiPitch rootPitch = m_midi.get_pitch(NoteOctave{Note::La, 2});
  
  //constexpr auto& scaleOffsets = well_tempered::c_minorScaleAsc;
  //constexpr auto& scaleOffsets = well_tempered::c_majorScaleAsc;
  //const auto scaleOffsets = just::mkMajorScaleAsc<Constexpr::Yes>();
  const auto scaleOffsets = pythagorean::mkMajorScaleAsc<Constexpr::Yes>();
  
  const auto scale = toMidiPitches(rootPitch, scaleOffsets);
  const int countOctaves = 2;
  MultiOctave multiOctave{
    scale.begin(),
    scale.end(),
    countOctaves
  };
  
  //auto& pitchGen = multiOctave;
  auto shufflePattern = ShufflePattern{multiOctave, std::vector<size_t>{0, 2, 1, 3, 2, 4, 3, 2}};
  auto& pitchGen = shufflePattern;
  
  for(int i=0; i<countNotes; ++i)
  {
    const MidiPitch midiPitch = withPitchDrift(pitchGen());
    score.m_voices.back().m_pitches.push_back(midiPitch);
  }
  
  return score;
}


Loop loopFromScore(Score const & score)
{
  Midi m_midi;
  std::vector<std::pair<TimestampAndSource, Event>> events;
  
  // Scales time.
  static constexpr float timeScaleFactor = 0.09;
  // Duration of a note
  static constexpr auto wait_after_note_on = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 800));
  // Pause length between consecutive notes
  static constexpr auto wait_after_note_off = std::chrono::milliseconds(static_cast<int>(timeScaleFactor * 300));
  
  const float volume = 1.f / std::max(1ul, score.m_voices.size());
  
  uint64_t maxCurTimeNanos{};
  
  uint64_t voice{};
  for(const auto & consecutivePitches : score.m_voices)
  {
    uint64_t curTimeNanos{};
    for(auto midiPitch : consecutivePitches.m_pitches)
    {
      const auto noteid = mk_note_id();
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

const auto synth = std::filesystem::path{"/Users/Olivier/Dev/cpp.audio/Synth/"};
const auto scores = std::filesystem::path{"/Users/Olivier/Dev/cpp.audio/Scores/"};

Loop mkEvents(int countNotes)
{
  const Score score = mkScore(countNotes);
  writeScoreInSimpleAsciiPitchesEncoding(score, scores/("test" + std::to_string(countNotes) + ".txt"));
  return loopFromScore(score);  
}

Loop eventsFrom(std::filesystem::path const& scoreFile)
{
  const Score score = readScoreFromSimpleAsciiPitchesEncoding(scoreFile);
  
  return loopFromScore(score);
}

void AppTune::playEvents(Loop && loop, uint64_t countLoops)
{
  if(!countLoops)
    return;

  auto countCinChars = m_countCinChars.load(std::memory_order_relaxed);
  auto must_interrupt = [&]()
  {
    return countCinChars != m_countCinChars.load(std::memory_order_relaxed);
  };

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
  
    if(must_interrupt())
    {
      m_synth.allNotesOff(m_stepper);
      break;
    }

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
  {
    std::this_thread::yield();
  }
}

Loop moduloPitch(Loop && l)
{
  Midi midi;
  
  constexpr MidiPitch minPitch{50.};
  constexpr MidiPitch maxPitch{80.};

  for(auto & [_, e] : l.events)
  {
    switch(e.type)
    {
      case EventType::NoteOn:
      {
        if(auto pitch = midi.frequency_to_midi_pitch(e.noteOn.frequency))
        {
          *pitch -= 2.*num_halftones_per_octave;
          while(*pitch < minPitch)
            *pitch += num_halftones_per_octave;
          while(*pitch > maxPitch)
            *pitch -= num_halftones_per_octave;
          e.noteOn.frequency = midi.midi_pitch_to_freq(*pitch);
        }
        break;
      }
    }
  }
  return std::move(l);
}

} // NS

int main() {
  using namespace imajuscule::audio;
  using namespace std::filesystem;

  auto a = AppTune{};
    
  // The ZeroEnvelope file produces drum sounds.
  const path zeroEnv{synth / "EnvelopeZero.txt"};
  const path fastEnv{synth / "EnvelopeFast.txt"};
  const path slowEnv{synth / "EnvelopeSlow.txt"};

  const path harSimple{synth / "HarmonicsSimple.txt"};

  a.setHarmonicsFile(harSimple);

  for(auto const &env : {zeroEnv, fastEnv, slowEnv})
  {
    a.setEnvelopeFile(env);
    
    // When writing the score files it is easy to use midi pitches that are way off,
    // given the experimental nature of the grammar...
    //
    // This is as-designed, to allow for more creativity by breaking from
    // the usual ways in which music is traditionally written.
    //
    // moduloPitch is a way to reduce the range of generated pitches.
    // It is an approach somewhat complementary to low-pass filtering.

    a.playEvents(moduloPitch(eventsFrom(scores / "Phrase.txt")), 4);
    a.playEvents(eventsFrom(scores / "Phrase.txt"), 4);
    a.playEvents(moduloPitch(eventsFrom(scores / "Phrase2.txt")), 4);
    a.playEvents(eventsFrom(scores / "Phrase2.txt"), 4);
    a.playEvents(moduloPitch(eventsFrom(scores / "StrangeBots.txt")), 4);
    a.playEvents(eventsFrom(scores / "StrangeBots.txt"), 4);
    
    a.playEvents(moduloPitch(mkEvents(250)), 1);
    a.playEvents(mkEvents(250), 1);
  }
  return 0;
}
