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

MidiPitch decodePitchFromBinaryStatEncoding(size_t c)
{
  return MidiPitch{A_pitch - 24 + c + 3};
}

Score readScoreFromSimpleAsciiPitchesEncoding(std::filesystem::path const& scoreFile)
{
  Score score;

  std::cout << "Reading " << scoreFile << std::endl;
  std::ifstream file(scoreFile, std::ios::in);

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

struct ByteHistogram
{
  // Ordered from most frequent to least frequent
  //
  // Only contains existing bytes.
  std::vector<size_t> v;
};

struct FileStats
{
  FileStats() : m_byteFreq(256, 0) {}
  
  void feed(uint8_t c)
  {
    ++m_byteFreq[c];
    if(m_prevByte.has_value())
    {
      if(*m_prevByte == c)
      {
        ++m_curConsecutiveBytes;
        m_maxConsecutiveBytes = std::max(m_maxConsecutiveBytes, m_curConsecutiveBytes);        
      }
      else
      {
        m_maxConsecutiveBytes = std::max(m_maxConsecutiveBytes, m_curConsecutiveBytes);
        m_curConsecutiveBytes = 1;
      }
    }
    else
    {
      ++m_curConsecutiveBytes;
      m_maxConsecutiveBytes = std::max(m_maxConsecutiveBytes, m_curConsecutiveBytes);      
    }
    m_prevByte = c;
  }
    
  ByteHistogram mkHistogram() const
  {
    std::vector<size_t> v(m_byteFreq.size());
    std::iota(v.begin(), v.end(), 0);
    
    std::sort(v.begin(), v.end(), [&](size_t a, size_t b){ return m_byteFreq[a] > m_byteFreq[b];});
    
    for(size_t i=0; i<v.size(); ++i)
      if(!m_byteFreq[v[i]])
      {
        v.resize(i);
        break;
      }

    return ByteHistogram{std::move(v)};    
  }

  void show(std::ostream& os, const ByteHistogram& hist) const
  {    
    for(auto i : hist.v)
    {
      std::cout << std::setfill(' ') << std::setw(3) << i << " " << m_byteFreq[i] << std::endl;
    }
  }
  
  size_t getFreq(size_t i) const { return m_byteFreq[i]; }
  
  size_t getMaxConsecutiveBytes() const { return m_maxConsecutiveBytes; }
private:
  std::vector<size_t> m_byteFreq;
  std::optional<uint8_t> m_prevByte;
  size_t m_curConsecutiveBytes{};
  size_t m_maxConsecutiveBytes{};
};

// Infinitely iterates (ascending) a byte range
struct CyclicByteRangeIterator
{
  CyclicByteRangeIterator(uint8_t min=0, uint8_t max=1)
  : m_min(min)
  , m_max(max)
  , m_nextVal(min)
  {}

  uint8_t operator()()
  {
    auto v = m_nextVal;
    if(m_nextVal == m_max)
      m_nextVal = m_min;
    else
      ++m_nextVal;
    return v;    
  }
private:
  uint8_t m_min;
  uint8_t m_max;
  uint8_t m_nextVal;
};


// Defines the timing for converting a stream of midi pitches (in a Score) to a Loop.
struct LoopTiming
{
  LoopTiming(float timeScaleFactor = 0.09)
  : m_timeScaleFactor(timeScaleFactor)
  {}
  
  // Duration between note on and note off of the same note.
  auto wait_after_note_on() const
  {
    return std::chrono::milliseconds(static_cast<int>(m_timeScaleFactor * 800));
  }
  // Duration between note off of previous note and note on of next note.
  auto wait_after_note_off() const
  {
    return std::chrono::milliseconds(static_cast<int>(m_timeScaleFactor * 300));
  }
  
  auto note_period() const { return wait_after_note_on() + wait_after_note_off(); }
  
private:
  float m_timeScaleFactor;
};

struct BatchKey
{
  // the maximum byte frequence.
  size_t maxByteFreq{};

  // the maximum number of consecutive bytes that are the same.
  size_t maxConsecutiveBytes{};
  
  bool operator == (const BatchKey& o) const
  {
    return std::tie(maxByteFreq, maxConsecutiveBytes) == std::tie(o.maxByteFreq, o.maxConsecutiveBytes);
  }
  bool operator < (const BatchKey& o) const
  {
    return std::tie(maxByteFreq, maxConsecutiveBytes) < std::tie(o.maxByteFreq, o.maxConsecutiveBytes);
  }
};

std::map<BatchKey, std::vector<size_t>>
statsFromBinary(std::ostream& os, std::filesystem::path const& scoreBinaryFile, const size_t batchSize)
{
  os << "Stats of binary " << scoreBinaryFile << std::endl;

  std::ifstream file(scoreBinaryFile, std::ios::binary | std::ios::in);

  size_t sz{};
  {
    FileStats stats;
    {
      uint8_t c;
      for(;;)
      {
        file.read(reinterpret_cast<char*>(&c), 1);
        if(file.fail())
          break;
        stats.feed(c);
        ++sz;
      }
    }
    const auto hist = stats.mkHistogram();  
    os << "- Global stats" << std::endl;
    stats.show(os, hist);
  }

  std::map<BatchKey, std::vector<size_t>> batchesByMaxFreq;
  const size_t countbatches = 1 + ((sz-1) / batchSize);
  {
    file.clear();
    file.seekg(0, std::ios::beg);
    
    for(size_t batchIndex{0};batchIndex < countbatches; ++batchIndex)
    {
      FileStats stats;
      {
        uint8_t c;
        for(size_t i=0;i<batchSize;++i)
        {
          file.read(reinterpret_cast<char*>(&c), 1);
          if(file.fail())
            break;
          stats.feed(c);
        }        
      }
      const auto hist = stats.mkHistogram();
      
      // Given the way we convert byte sequences to pitches,
      // the music will be "interesting" when there is some structure
      // i.e the byte distribution is not uniform:
      // - few bytes have large frequencies
      // - many bytes have small frequencies.

      const auto maxFreq = stats.getFreq(hist.v.front());
      batchesByMaxFreq[BatchKey{maxFreq, stats.getMaxConsecutiveBytes()}].push_back(batchIndex);

      //const auto countFreq = hist.v.size();
      //os << "batch " << (batchIndex + 1) << '/' << countbatches << " ";
      //os << countFreq << " maxFreq:" << maxFreq << std::endl;
      //os << "- stats of batch " << (batchIndex + 1) << '/' << countbatches << std::endl;
      //stats.show(os, hist);
    }
  }
  return batchesByMaxFreq;
}

struct ByteRange {
  size_t m_beginIndex;
  size_t m_endIndex;
};


struct ByteRangeIterator
{
  ByteRangeIterator(ByteRange range)
  : m_range(range)
  {
    m_nextIndex = m_range.m_beginIndex;
  }
  
  std::optional<size_t> operator()()
  {
    auto res = m_nextIndex;
    if(res >= m_range.m_endIndex)
      return std::nullopt;
    ++m_nextIndex;
    return res;
  }
  
private:
  ByteRange m_range;
  size_t m_nextIndex;
};

struct ByteRangesIterator
{
  ByteRangesIterator(std::vector<ByteRange> ranges)
  : m_ranges(std::move(ranges))
  {
    m_it = m_ranges.begin();
  }

  std::optional<ByteRange> operator()()
  {
    auto it = m_it;
    if(m_it != m_ranges.end())
      ++m_it;
    if(it != m_ranges.end())
      return *it;
    else
      return std::nullopt;      
  }

private:
  std::vector<ByteRange> m_ranges;
  std::vector<ByteRange>::iterator m_it;
};

// Whether the CyclicByteRangeIterator(s) are reinitialized when we start a new range
enum class ReinitCycleAtRangeBoundary{Yes, No};

// Whether all cycles start at the same locations (min value of the cycle)
enum class UniformCycleInitialization{Yes, No};

// If a small number of bytes occur very often in the file,
// mapping a byte to a single MidiPitch would produce a boring melody.
//
// So we map each byte to the same range of MidiPitches
// to have some variations in the produced melody.
struct MidiPitchStreamFromBinary
{
  MidiPitchStreamFromBinary(std::filesystem::path const& scoreBinaryFile,
                            ByteRangesIterator ranges,
                            ReinitCycleAtRangeBoundary r,
                            UniformCycleInitialization u);
  
  std::optional<MidiPitch> operator()();
  
private:
  std::ifstream m_file;
  ByteRangesIterator m_ranges;
  std::optional<ByteRangeIterator> m_curRange;
  std::vector<CyclicByteRangeIterator> m_byteToByteIterator;
  ReinitCycleAtRangeBoundary m_reinitCycleAtRangeBoundary;
  UniformCycleInitialization m_uniformCycleInitialization;
  void reinitCycles();
};

MidiPitchStreamFromBinary::MidiPitchStreamFromBinary(std::filesystem::path const& scoreBinaryFile,
                                                     ByteRangesIterator ranges,
                                                     ReinitCycleAtRangeBoundary r,
                                                     UniformCycleInitialization u)
: m_file(scoreBinaryFile, std::ios::binary | std::ios::in)
, m_ranges(std::move(ranges))
, m_reinitCycleAtRangeBoundary(r)
, m_uniformCycleInitialization(u)
{
  std::cout << "Reading binary " << scoreBinaryFile << std::endl;
  
  reinitCycles();
}

void MidiPitchStreamFromBinary::reinitCycles()
{
  m_byteToByteIterator.clear();
  m_byteToByteIterator.resize(256, CyclicByteRangeIterator(static_cast<uint8_t>(0), static_cast<uint8_t>(48)));
  if(m_uniformCycleInitialization == UniformCycleInitialization::No)
  {
    for(size_t i = 1; i<m_byteToByteIterator.size(); ++i)
    {
      for(size_t j=0; j<i; ++j)
        m_byteToByteIterator[i]();
    }
  }
}

std::optional<MidiPitch>
MidiPitchStreamFromBinary::operator()()
{
  if(m_curRange.has_value())
  {
    if((*m_curRange)())
    {
    return_read:
      uint8_t c;
      m_file.read(reinterpret_cast<char*>(&c), 1);
      if(m_file.fail())
        // Should not occur, means one byte rage is too large for the file.
        return std::nullopt;
      return decodePitchFromBinaryStatEncoding(m_byteToByteIterator[c]());        
    }
  }

next_range:
  // the current range is finished.
  m_curRange = m_ranges();
  if(!m_curRange.has_value())
  {
    // we are done, all ranges have been visited.
    return std::nullopt;
  }
  else if(const auto beginIndex = (*m_curRange)())
  {
    // the new range is non-empty 
    m_file.seekg(*beginIndex);
    if(m_file.fail())
      // Should not occur, means the range was too large for the file.
      return std::nullopt;
    if(m_reinitCycleAtRangeBoundary == ReinitCycleAtRangeBoundary::Yes)
      reinitCycles();
    goto return_read;
  }
  else
  {
    // the new range was empty.
    goto next_range;
  }
}

Score readScoreFromBinaryPitchesEncoding(std::filesystem::path const& scoreBinaryFile, LoopTiming loopTiming)
{
  const size_t batchSize = 10000;
  const auto note_period = loopTiming.note_period();  
  const auto period_batch = note_period * batchSize;
  std::cout << "One batch corresponds to " << std::chrono::duration_cast<std::chrono::seconds>(period_batch).count() << " seconds" << std::endl;

  auto batchesByMaxFreq = statsFromBinary(std::cout, scoreBinaryFile, batchSize);

  std::cout << "# Batches by max freq / max consecutive bytes:" << std::endl;
  for(const auto &[batchKey, batches] : batchesByMaxFreq)
  {
    std::cout << batchKey.maxByteFreq << ", " << batchKey.maxConsecutiveBytes << ":" << batches.size();
    //for(const auto batchIndex : batches)
    //  std::cout << batchIndex << " ";
    std::cout << std::endl;
  }

  
  std::vector<std::pair<BatchKey, ByteRange>> notBoringFirst, boringFirst;
  auto tryUseBatch = [&](const BatchKey& key, const std::vector<size_t>& batchIndexes, std::vector<std::pair<BatchKey, ByteRange>>& res)
  {
    const auto maxFreq = key.maxByteFreq;
    // Skip batches that are too "boring" i.e
    // - some notes are too frequent, or
    // - at least one note is consecutively repeated too many times
    if(maxFreq > 0.1 * batchSize) // works well for batchSize = 100
      return;
    const auto maxRepetitions = key.maxConsecutiveBytes;
    // It could be simpler to ignore long repetitions when reading the file,
    // i.e build a set<ByteRange> of bytes to skip and ignore those when reading later on.
    if(maxRepetitions > batchSize/10.)  // works well for batchSize = 100
      return;
    for(auto batchIndex : batchIndexes)
    {
      // Todo: it could be interesting to include prev/next batches as well if they are not included?
      // Todo: it could be interesting to merge consecutive batches if they are all "interesting" enough.
      res.emplace_back(key,
                       ByteRange{
        batchIndex * batchSize,
        (batchIndex+1) * batchSize
      });
    }
  };
  for (auto it = batchesByMaxFreq.rbegin(); it != batchesByMaxFreq.rend(); it++)
    tryUseBatch(it->first, it->second, notBoringFirst);
  for (auto it = batchesByMaxFreq.begin(); it != batchesByMaxFreq.end(); it++)
    tryUseBatch(it->first, it->second, boringFirst);

  std::vector<ByteRange> byteRangesByByteMaxFreq;
  for(size_t i=0; i<std::max(notBoringFirst.size(), boringFirst.size()); ++i)
  {
    if(i<notBoringFirst.size())
    {
      auto batchKey = notBoringFirst[i].first;
      // for Feuillard4,
      // with batchSize = 10000,
      // the batch: maxByteFreq=138, maxConsecutiveBytes=11
      // works really well.
      std::cout << batchKey.maxByteFreq << ", " << batchKey.maxConsecutiveBytes << std::endl;
      byteRangesByByteMaxFreq.push_back(notBoringFirst[i].second);
    }
    if(i<boringFirst.size())
    {
      // alternating not boring with boring doesn't work
      //byteRangesByByteMaxFreq.push_back(boringFirst[i].second);
    }
  }

  Score score;
 
  auto stream = MidiPitchStreamFromBinary(scoreBinaryFile,
                                          ByteRangesIterator{std::move(byteRangesByByteMaxFreq)},
                                          ReinitCycleAtRangeBoundary::No,
                                          // when we use batches that are not "boring",
                                          // UniformCycleInitialization::Yes is good:
                                          // this way, we see a long term progression of the music from
                                          // quite uniform to quite varied.
                                          // with UniformCycleInitialization::No the music feels much more
                                          // "random" from the start so it is harder for the listener to
                                          // hear a progression in the unfolding of the music.
                                          UniformCycleInitialization::Yes);

  score.m_voices.emplace_back();
  while(auto pitch = stream())
  {
    score.m_voices.back().m_pitches.push_back(*pitch);
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

Loop loopFromScore(Score const & score, const LoopTiming& timing)
{
  Midi m_midi;
  std::vector<std::pair<TimestampAndSource, Event>> events;
    
  const auto wait_after_note_on = timing.wait_after_note_on();
  const auto wait_after_note_off = timing.wait_after_note_off();

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

Loop mkEvents(int countNotes, LoopTiming loopTiming = {})
{
  const Score score = mkScore(countNotes);
  writeScoreInSimpleAsciiPitchesEncoding(score, scores/("test" + std::to_string(countNotes) + ".txt"));
  return loopFromScore(score, loopTiming);  
}

Loop eventsFrom(std::filesystem::path const& scoreFile, LoopTiming loopTiming = {})
{
  const Score score = readScoreFromSimpleAsciiPitchesEncoding(scoreFile);
  
  return loopFromScore(score, loopTiming);
}

Loop eventsFromBinary(std::filesystem::path const& scoreFile, LoopTiming loopTiming = {})
{
  const Score score = readScoreFromBinaryPitchesEncoding(scoreFile, loopTiming);
  
  return loopFromScore(score, loopTiming);
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

  for(auto const &env : {slowEnv, zeroEnv, fastEnv})
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
    const auto file = "/Users/Olivier/Downloads/IMSLP874967-PMLP1036212-Feuillard_La_Technique_du_Violoncelle_Vol.4.pdf";
    a.playEvents(eventsFromBinary(file), 1);
    a.playEvents(moduloPitch(eventsFromBinary(file)), 1);
    a.playEvents(eventsFromBinary(scores / "Phrase.txt"), 1);
    a.playEvents(moduloPitch(eventsFromBinary(scores / "Phrase.txt")), 1);
    a.playEvents(eventsFrom(scores / "Phrase.txt"), 4);
    a.playEvents(moduloPitch(eventsFrom(scores / "Phrase.txt")), 4);

    a.playEvents(moduloPitch(eventsFrom(scores / "StrangeBots.txt")), 4);
    a.playEvents(eventsFrom(scores / "StrangeBots.txt"), 4);
    a.playEvents(moduloPitch(eventsFrom(scores / "Phrase2.txt")), 4);
    a.playEvents(eventsFrom(scores / "Phrase2.txt"), 4);
    
    a.playEvents(moduloPitch(mkEvents(250)), 1);
    a.playEvents(mkEvents(250), 1);
  }
  return 0;
}
