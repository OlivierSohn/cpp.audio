#if 0

automatiser la detection de samples. 

Tenir compte de la range de samples dans le streaming du pitch midi
pour produire des picthes valides.

Sampler le violoncelle pour utiliser les samples dans le pdf feuillard.
- sons courts, piano.

Analyse d'un enregistrement pour la justesse, pour voir les tendances que j'ai.

Analyse en temps reel de la justesse?

#endif

namespace imajuscule::audio {

namespace audioelement {

// The base oscillator (SineOscillatorAlgo) is a sine wave
// and we define harmonics (MultiEnveloped).
//
// An alternative approach would be to use a more complex base oscillator
// without explicitely defning harmonis, for example:
// FOscillatorAlgo<T, FOscillator::SAW, OscillatorUsage::Raw>
// FOscillatorAlgo<T, FOscillator::TRIANGLE, OscillatorUsage::Raw>
// FOscillatorAlgo<T, FOscillator::SQUARE, OscillatorUsage::Raw>

template <typename T>
using TuneOscElement =
//StereoPanned<
LowPassAlgo<
VolumeAdjusted<
//LoudnessVolumeAdjusted<
MultiEnveloped
<
//FreqCtrl_<
SineOscillatorAlgo<T, eNormalizePolicy::FAST>
//, InterpolatedFreq<T>
//>
, AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease, AllowZeroAttack::No>
>
>
, 2
>
//>
;

template<typename T>
struct TuneOscElementInitializer {
  TuneOscElementInitializer(int const sample_rate,
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
  
  void operator()(audioelement::TuneOscElement<T> & e) const {
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
  
  bool operator ==(TuneOscElementInitializer const& o) const {
    return
    std::tie(sample_rate, stride, stereo_spread_, m_lowPassFreq, ahdsr, m_harmonics) ==
    std::tie(o.sample_rate, o.stride, o.stereo_spread_, o.m_lowPassFreq, o.ahdsr, o.m_harmonics);
  }
  bool operator !=(TuneOscElementInitializer const& o) const {
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

// Use stereo samples.
constexpr int countSamplerChannels{2};

// TODO: later, find a way to artificially make the sample longer to play long notes.
template <typename T>
using TuneSamplerElement =
//StereoPanned<
//LoudnessVolumeAdjusted<
//LowPassAlgo< // is this needed?
VolumeAdjusted<
Enveloped <
//FreqCtrl_<
audioelement::SamplerAlgo<T, countSamplerChannels>
//, InterpolatedFreq<T>
, AHDSREnvelope<Atomicity::Yes, T, EnvelopeRelease::WaitForKeyRelease, AllowZeroAttack::Yes>
>
>
//, 2 >
//>
//>
;

template<typename T>
struct TuneSamplerElementInitializer {
  TuneSamplerElementInitializer(int const sample_rate,
                                int const stride,
                                audioelement::AHDSR const & a,
                                std::reference_wrapper<const std::map<T, std::vector<T>>> samples)
  : sample_rate(sample_rate)
  , stride(stride)
  , ahdsr(a)
  , m_samples(samples)
  {}
  
  void operator()(audioelement::TuneSamplerElement<T> & e) const {

    e.getOsc().getOsc().setSamples(&m_samples.get());

    e.editEnvelope().setAHDSR(ahdsr,
                              sample_rate);
    
    // limit the speed of volume adjustment:
    
    e.getVolumeAdjustment().setMaxFilterIncrement(2. / static_cast<double>(stride));
    
    // filtering
    
    //e.setFilterAngleIncrements(freq_to_angle_increment(m_lowPassFreq,
    //                                                   sample_rate));

    // frequency control
    
    //e.getVolumeAdjustment().getOsc().getAlgo().getCtrl().setup(stride,
    //                                                           itp::LINEAR);
  }
  
  bool operator ==(TuneSamplerElementInitializer const& o) const {
    return
    std::tie(sample_rate, stride, ahdsr, m_harmonics, m_samples) ==
    std::tie(o.sample_rate, o.stride, o.ahdsr, o.m_harmonics, o.m_samples);
  }
  bool operator !=(TuneSamplerElementInitializer const& o) const {
    return !this->operator ==(o);
  }
  
private:
  int sample_rate;
  int stride;
  audioelement::AHDSR ahdsr;
  std::vector<harmonicProperties_t> m_harmonics;
  std::reference_wrapper<const std::map<T, std::vector<T>>> m_samples;
};

} // NS audioelement


using Events = std::vector<std::pair<TimestampAndSource, Event>>;

struct LoopOffsets
{
  TimeNanos eventTimeOffsetNanos{};
  uint64_t eventNoteIdOffset{};    
};

struct Loop
{
  // The loop starts at time zero.
  static constexpr TimeNanos c_refTime{0};

  // The loop starts at time zero, hence times of events are zero-based.
  // The time of an event is offset in 'instantiateEvent' when we schedule the event to be played.
  Loop(Events && e, DurationNanos lengthNanos)
  : events(std::move(e))
  , m_lengthNanos(lengthNanos)
  {
    std::sort(events.begin(), events.end(), [](const auto & t1, const auto & t2){ return t1.first.getNanosTime() < t2.first.getNanosTime(); });
    
    m_maxNoteId = 0;
    for(const auto&[_, event]:events)
      m_maxNoteId = std::max(event.noteid.noteid, m_maxNoteId);
  }

  // Update the LoopOffsets to start playing the next iteration of the loop.
  void updateOffsetsForNextIteration(LoopOffsets& offsets) const
  {
    // The first noteId that is not used.
    // This assumes that noteids are small contiguous integers which is the case here.
    const int64_t endNoteId = m_maxNoteId + 1l;

    // we offset the noteids otherwise some noteoff events may be ignored.
    // when we have several noteon and noteoff
    // events in the queue for the same noteid.
    offsets.eventNoteIdOffset += endNoteId;

    offsets.eventTimeOffsetNanos += m_lengthNanos;    
  }
  
  std::pair<TimestampAndSource, Event> instantiateEvent(size_t eventIndex, const LoopOffsets& offsets) const
  {
    auto event = events[eventIndex];
    event.second.offsetNoteId(offsets.eventNoteIdOffset);
    event.first.offsetNanosTime(DurationNanos{offsets.eventTimeOffsetNanos.get()});
    return event;
  }

  size_t countEvents() const { return events.size(); }
  Event& mutEvent(size_t index) { return events[index].second; }

  const Events& getEvents() const { return events; }
    
private:
  // invariant : sorted by first.getNanosTime().
  Events events;

  DurationNanos m_lengthNanos{};
  
  int64_t m_maxNoteId;
};

static NoteId mk_note_id() {
  static int64_t i = 0;
  ++i;
  return NoteId{i};
}

enum class StreamStatus{OK, EndOfStream};

struct EventStream
{
  virtual ~EventStream() = default;

  // This is called at least once before materializeNextEvents.
  //
  // can be called multiple times to restart the stream.
  virtual void startStream(TimeNanos refTime) = 0;

  // Stops the stream.
  // May be called multiple times.
  virtual void stopStream() = 0;
  
  // startStream will be called before this method to give the reference "start" time.
  //
  // materializes events that have not yet been materialized yet and that will occur up to maxTime.
  //
  // For convenience, it is ok to materialize more events (i.e some events that would occur after maxTime).
  //
  // Calling materializeNextEvents after StreamStatus::EndOfStream
  // will return StreamStatus::EndOfStream until startStream is called.
  virtual StreamStatus materializeNextEvents(Events & events, TimeNanos maxTime) = 0;
};

struct NullEventStream: public EventStream
{
  void startStream(TimeNanos refTime) override {}
  
  void stopStream() override {}
  
  StreamStatus materializeNextEvents(Events & events, TimeNanos maxTime) override { return StreamStatus::EndOfStream; }
};

struct LoopEventStream : public EventStream
{
  LoopEventStream(Loop && loop, uint64_t countLoops = 1000)
  : m_loop(std::move(loop))
  , m_countRemainingIterations(countLoops)
  {
    m_sz = m_loop.countEvents();
  }
  
private:
  void startStream(TimeNanos refTime) override
  {
    m_offsets = {};
    m_offsets.eventTimeOffsetNanos = refTime;

    m_firstIndex = 0;
  }
  void stopStream() override
  {
  }

public:
  StreamStatus materializeNextEvents(Events & events, TimeNanos maxTime) override;

private:
  Loop m_loop;
  LoopOffsets m_offsets;
  int m_countRemainingIterations;
  size_t m_firstIndex{};
  size_t m_sz{};
};

StreamStatus
LoopEventStream::materializeNextEvents(Events & events, TimeNanos maxTime)
{
  if(m_sz == 0)
    return StreamStatus::EndOfStream;

  bool newIteration{};
  
  if(m_firstIndex >= m_sz)
  {
    // All events of this iteration of the loop have been materialized.
    m_firstIndex = 0;
    --m_countRemainingIterations;
    newIteration = true;
  }

  if(m_countRemainingIterations <= 0)
    // All iterations of the loop have been materialized.
    return StreamStatus::EndOfStream;

  if(newIteration)
    m_loop.updateOffsetsForNextIteration(m_offsets);
  
  for(;m_firstIndex < m_sz; ++m_firstIndex)
  {
    const auto event = m_loop.instantiateEvent(m_firstIndex, m_offsets);
    if(event.first.getNanosTime() >= maxTime)
      break;
    events.push_back(event);
  }
  return StreamStatus::OK;
}


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
  size_t getCurConsecutiveBytes() const { return m_curConsecutiveBytes; }
  
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


// Defines the timing for converting a stream of midi pitches (in a Score) to a stream of events.
struct EventsTiming
{
  EventsTiming(float timeScaleFactor = 0.09)
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

struct ByteRange
{
  size_t m_beginIndex;
  size_t m_endIndex;
  
  bool operator == (const ByteRange& o) const
  {
    return std::tie(m_beginIndex, m_endIndex) == std::tie(o.m_beginIndex, o.m_endIndex);
  }
  bool operator < (const ByteRange& o) const
  {
    return std::tie(m_beginIndex, m_endIndex) < std::tie(o.m_beginIndex, o.m_endIndex);
  }
};

// A single batch can correspond to multiple ranges: to 
// avoid having too may repetitions of the same byte we
// skip some bytes.
struct Batch
{
  std::vector<ByteRange> ranges;
};

std::map<BatchKey, std::vector<Batch>> batchesByMaxFreq;

struct MaxConsecutiveBytes{
  MaxConsecutiveBytes(size_t m)
  : max(m)
  {}
  size_t get() const { return max; }
  
private:
  size_t max;
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
    restart();
  }
  
  void restart()
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

// Helper to know whether bytes nust be skipped.
struct SkipBytes
{
  SkipBytes(std::vector<ByteRange> skipBytes)
  : m_skipBytes(std::move(skipBytes))
  {
    computeNextSkipped();
  }
  
  // The caller must guarantee that this method will be called with strictly increasing idx values.
  bool operator()(size_t idx)
  {
    if(m_nextSkipped.has_value())
    {
      if(*m_nextSkipped != idx)
      {
        return false;
      }
      computeNextSkipped();
      return true;
    }
    return false;
  }
  
private:
  ByteRangesIterator m_skipBytes;
  std::optional<ByteRangeIterator> m_curSkippedRange;
  std::optional<size_t> m_nextSkipped;
  
  void computeNextSkipped()
  {
    m_nextSkipped.reset();
    if(!m_curSkippedRange.has_value())
    {
    next_range:
      m_curSkippedRange = m_skipBytes();
    }
    if(!m_curSkippedRange.has_value())
      return;
    m_nextSkipped = (*m_curSkippedRange)();
    if(!m_nextSkipped.has_value())
    {
      goto next_range;
    }
  }
};

std::map<BatchKey, std::vector<Batch>>
statsFromBinary(std::ostream& os,
                std::filesystem::path const& scoreBinaryFile,
                const size_t batchSize,
                const MaxConsecutiveBytes maxConsecutiveBytes)
{
  os << "Stats of binary " << scoreBinaryFile << std::endl;
  
  std::ifstream file(scoreBinaryFile, std::ios::binary | std::ios::in);
  
  // These byte ranges should be skipped, they correspond to repetitions
  // of the same byte.
  
  std::vector<ByteRange> skipBytes;
  
  size_t sz{};
  {
    FileStats stats;
    size_t countBytesSkipped{};
    {
      ByteRange * curBytesSkipped{nullptr}; 
      
      uint8_t c;
      for(;;)
      {
        file.read(reinterpret_cast<char*>(&c), 1);
        if(file.fail())
          break;
        stats.feed(c);
        if(stats.getCurConsecutiveBytes() > maxConsecutiveBytes.get())
        {
          if(!curBytesSkipped)
          {
            skipBytes.push_back(ByteRange{sz, sz});
            curBytesSkipped = &skipBytes.back();
          }
          ++(curBytesSkipped->m_endIndex);
          ++countBytesSkipped;
        }
        else
        {
          curBytesSkipped = nullptr;
        }
        ++sz;
      }
      sz -= countBytesSkipped;
    }
    const auto hist = stats.mkHistogram();  
    os << "- Global stats" << std::endl;
    stats.show(os, hist);
  }
  
  auto skip = SkipBytes(std::move(skipBytes));
  
  std::map<BatchKey, std::vector<Batch>> batchesByMaxFreq;
  const size_t countbatches = 1 + ((sz-1) / batchSize);
  {
    file.clear();
    file.seekg(0, std::ios::beg);
    
    size_t byteIdx{};
    
    for(size_t batchIndex{0};batchIndex < countbatches; ++batchIndex)
    {
      Batch batch;
      
      FileStats stats;
      {
        uint8_t c;
        for(size_t i=0;i<batchSize;++i)
        {        
        nextByte:
          file.read(reinterpret_cast<char*>(&c), 1);
          if(file.fail())
            break;
          const auto candidateByteIdx = byteIdx++;
          if(skip(candidateByteIdx))
          {
            goto nextByte;
          }
          if(batch.ranges.empty() || (batch.ranges.back().m_endIndex != candidateByteIdx))
          {
            // new range
            batch.ranges.push_back(ByteRange{candidateByteIdx, candidateByteIdx});                            
          }
          ++batch.ranges.back().m_endIndex;
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
      batchesByMaxFreq[BatchKey{maxFreq, stats.getMaxConsecutiveBytes()}].push_back(batch);
      
      //const auto countFreq = hist.v.size();
      //os << "batch " << (batchIndex + 1) << '/' << countbatches << " ";
      //os << countFreq << " maxFreq:" << maxFreq << std::endl;
      //os << "- stats of batch " << (batchIndex + 1) << '/' << countbatches << std::endl;
      //stats.show(os, hist);
    }
  }
  return batchesByMaxFreq;
}

struct Polyphony
{
  Polyphony(size_t c)
  : count(c)
  {}
  size_t get() const { return count;}
private:
  size_t count;
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
                            UniformCycleInitialization u,
                            Polyphony countVoices);
  
  size_t countVoices() const { return m_byteToByteIterator.size(); }

  void restart();

  std::optional<MidiPitch> operator()(size_t voiceIndex);
  
private:
  std::ifstream m_file;
  ByteRangesIterator m_ranges;
  std::optional<ByteRangeIterator> m_curRange;
  // one vector per voice
  std::vector<std::vector<CyclicByteRangeIterator>> m_byteToByteIterator;
  ReinitCycleAtRangeBoundary m_reinitCycleAtRangeBoundary;
  UniformCycleInitialization m_uniformCycleInitialization;
  void reinitCycles();
};

MidiPitchStreamFromBinary::MidiPitchStreamFromBinary(std::filesystem::path const& scoreBinaryFile,
                                                     ByteRangesIterator ranges,
                                                     ReinitCycleAtRangeBoundary r,
                                                     UniformCycleInitialization u,
                                                     Polyphony countVoices)
: m_file(scoreBinaryFile, std::ios::binary | std::ios::in)
, m_ranges(std::move(ranges))
, m_reinitCycleAtRangeBoundary(r)
, m_uniformCycleInitialization(u)
, m_byteToByteIterator(countVoices.get())
{
  std::cout << "Reading binary " << scoreBinaryFile << std::endl;
  
  restart();
}

void
MidiPitchStreamFromBinary::restart()
{
  m_file.clear();
  m_file.seekg(0, std::ios::beg);
  
  reinitCycles();
  
  m_curRange.reset();
  m_ranges.restart();
}

void MidiPitchStreamFromBinary::reinitCycles()
{
  for(auto & iterators : m_byteToByteIterator)
  {
    iterators.clear();
    iterators.resize(256, CyclicByteRangeIterator(static_cast<uint8_t>(0), static_cast<uint8_t>(48)));
    if(m_uniformCycleInitialization == UniformCycleInitialization::No)
    {
      for(size_t i = 1; i<iterators.size(); ++i)
      {
        for(size_t j=0; j<i; ++j)
          iterators[i]();
      }
    }
  }
}

std::optional<MidiPitch>
MidiPitchStreamFromBinary::operator()(size_t voiceIndex)
{
  if(m_curRange.has_value())
  {
    if((*m_curRange)())
    {
    return_read:
      uint8_t c;
      m_file.read(reinterpret_cast<char*>(&c), 1);
      if(m_file.fail())
        // Should not occur, means one byte range is too large for the file.
        return std::nullopt;
      //std::cout << static_cast<int>(c) << ' ';
      //std::cout << c;
      return decodePitchFromBinaryStatEncoding(m_byteToByteIterator[voiceIndex][c]());        
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
    //std::cout << "start reading at " << *beginIndex << std::endl;
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

struct EventStreamFromBinary : public EventStream
{
  EventStreamFromBinary(MidiPitchStreamFromBinary && midiPitchStreamFromBinary, EventsTiming timing)
  : m_midiPitchStreamFromBinary(std::move(midiPitchStreamFromBinary))
  , m_timing(timing)
  {
    m_countVoices = m_midiPitchStreamFromBinary.countVoices();
    m_volume = m_countVoices ? (1.f/m_countVoices) : 1.f;
    m_nextEventTime.resize(m_countVoices);
  }
  
private:
  void startStream(TimeNanos refTime) override
  {
    for(auto & t : m_nextEventTime)
      t = refTime;
    m_midiPitchStreamFromBinary.restart();
  }
  void stopStream() override
  {
  }
  
public:
  StreamStatus materializeNextEvents(Events & events, TimeNanos maxTime) override;
  
private:
  MidiPitchStreamFromBinary m_midiPitchStreamFromBinary;
  size_t m_countVoices;
  // one per voice
  std::vector<TimeNanos> m_nextEventTime;
  EventsTiming m_timing;
  Midi m_midi;
  float m_volume;
};

StreamStatus
EventStreamFromBinary::materializeNextEvents(Events & events, TimeNanos maxTime)
{
  size_t countEOS{};
  for(size_t voice=0; voice<m_countVoices; ++voice)
  {
    auto & nextEventTime = m_nextEventTime[voice];
    while(nextEventTime < maxTime)
    {
      const auto midiPitch = m_midiPitchStreamFromBinary(voice);
      if(!midiPitch.has_value())
      {
        ++countEOS;
        break;
      }
      
      const auto noteid = mk_note_id();
      const float frequency = m_midi.midi_pitch_to_freq(*midiPitch);
      events.emplace_back(TimestampAndSource{nextEventTime, voice},
                          mkNoteOn(noteid,
                                   frequency,
                                   m_volume));
      nextEventTime += m_timing.wait_after_note_on();
      events.emplace_back(TimestampAndSource{nextEventTime, voice},
                          mkNoteOff(noteid));
      nextEventTime += m_timing.wait_after_note_off();
    }
  }
  if(countEOS == m_countVoices)
    return StreamStatus::EndOfStream;
  return StreamStatus::OK;
}

constexpr int nAudioOut = 2;
constexpr int m_sampleRate{
  //96000
  44100
};
auto ms_to_frames (float ms){
  return audio::ms_to_frames(ms, m_sampleRate);
}

struct SynthDef{
  // We probably only need a few voices,
  // in particular for synths that play one note at a time.
  static constexpr int nVoices = 32;
    
  // This file defines the envelope of the synthesizer.
  //
  // The following syntax is used in this file:
  // A .
  // H ..
  // D ....
  // S ...
  // R ...
  std::filesystem::path envelopeFile{"/Users/Olivier/Dev/cpp.audio/Synth/EnvelopeFast.txt"};
  
  // Represents the max time of the synth files that were
  // last used to produce the AE initializer.
  // This is a way to avoid reading multiple time the same file content.
  std::optional<std::filesystem::file_time_type> m_lastWriteSynthFiles;

  void setEnvelopeFile(std::filesystem::path e)
  {
    if(e == envelopeFile)
      return;
    std::cout << "Using envelope " << e << std::endl;
    envelopeFile = e;
    m_lastWriteSynthFiles.reset();
  }

  audioelement::AHDSR mkEnvelope() const;
};


audioelement::AHDSR SynthDef::mkEnvelope() const
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
}

struct OscSynthDef : public SynthDef
{
  using OscSynth = sine::Synth <
  nAudioOut,
  audioelement::TuneOscElement<double>,
  TryAccountForTimeSourceJitter::No,
  SynchronizePhase::Yes,
  DefaultStartPhase::Zero,
  HandleNoteOff::Yes,
  nVoices,
  audioelement::TuneOscElementInitializer<double>>;

  void updateInitializerIfNeeded();

  void setHarmonicsFile(std::filesystem::path e)
  {
    if(e == harmonicsFile)
      return;
    harmonicsFile = e;
    m_lastWriteSynthFiles.reset();
  }
  
  // This file defines the harmonics of the synthesizer.
  //
  // The following syntax is used in this file:
  //
  // ....   // 1st harmonic has a relative weight 4
  //        // no 2nd harmonic
  // .      // 3rd harmonic has a relative weight 1
  // ...    // 4th harmonic has a relative weight 3
  std::filesystem::path harmonicsFile{"/Users/Olivier/Dev/cpp.audio/Synth/Harmonics.txt"};
  
  std::filesystem::path lowPassFile{"/Users/Olivier/Dev/cpp.audio/Synth/LowPass.txt"};
    
  OscSynth m_synth;    
};


struct SamplerSynthDef : public SynthDef
{  
  using SamplerSynth = sine::Synth <
  nAudioOut,
  audioelement::TuneSamplerElement<double>,
  TryAccountForTimeSourceJitter::No,
  SynchronizePhase::Yes,
  DefaultStartPhase::Zero,
  HandleNoteOff::Yes,
  nVoices,
  audioelement::TuneSamplerElementInitializer<double>>;
  
  void updateInitializerIfNeeded(std::map<double, std::vector<double>> const& samples);
  
  SamplerSynth m_synth;    
};

struct SampleRange
{
  int32_t firstFrame;
  // last frame is included.
  int32_t lastFrame;
  
  int32_t getLength() const { return lastFrame - firstFrame + 1; }
};

struct SampleRanges
{
  // invariant: ranges are not overlapping.
  // invariant: ranges are ordered by increasing frame location.
  std::vector<SampleRange> ranges;
};

// Returns the union of ranges 'rangeA' and 'rangeB'.
SampleRanges unionRanges(const SampleRanges& rangeA, const SampleRanges& rangeB)
{
  std::vector<SampleRange> res;
  auto onRange = [&](SampleRange r)
  {
    if(res.empty() || (res.back().lastFrame < r.firstFrame))
      res.push_back(r);
    else
    {
      res.back().lastFrame = std::max(res.back().lastFrame, r.lastFrame);
    }
  };

  auto a = rangeA.ranges.begin();
  auto b = rangeB.ranges.begin();
  for(;;)
  {
    if(a != rangeA.ranges.end() && b != rangeB.ranges.end())
    {
      if(a->firstFrame < b->firstFrame)
        onRange(*a++);
      else
        onRange(*b++);
    }
    else if(a != rangeA.ranges.end())
    {
      onRange(*a++);      
    }
    else if(b != rangeB.ranges.end())
    {
      onRange(*b++);      
    }
    else
      break;
  }

  return SampleRanges{std::move(res)};
}

SampleRanges unionRanges(const std::vector<SampleRanges>& ranges)
{
  SampleRanges res;
  if(!ranges.empty())
  {
    res = ranges[0];
    for(size_t i=1; i<ranges.size(); ++i)
    {
      res = unionRanges(ranges[i], res);
    }
  }
  return res;
}

// Algorithm to find valid sample ranges:
//
// - find the start of a range using find_relevant_start_relaxed(abs_relevant_level=0.1, sliding_avg_size=15).
// - find the end of a range using
//   - fwd until signal (sliding average of absolute) is lower than 0.02 for a duration of at least "lookahead".
//   - from first location where signal started to be low, fwd until (sliding average of absolute) increases again.
SampleRanges
computeSampleRanges(const std::vector<double>& signal,
                    int const slidingAvgFrameCount,
                    const int32_t lookAheadFrameCount)
{
  SampleRanges res;
  
  auto it = signal.begin();
  auto end = signal.end();
  
  const double noise_level = compute_noise_floor(it, end, lookAheadFrameCount);
  
  const float maxSoundAmplitudeEnd = noise_level * 3.;
  const float minSoundAmplitudeStart = noise_level * 3.;

  for(;it != end;)
  {
    it = find_relevant_start_relaxed(it, end, minSoundAmplitudeStart, slidingAvgFrameCount);
    if(it == end)
      break;
    // by now 'it' is the start of the sample range.
    
    const int32_t startFrame(static_cast<int32_t>(std::distance(signal.begin(), it)));
    it = find_relevant_end_relaxed(it, end, maxSoundAmplitudeEnd, slidingAvgFrameCount, lookAheadFrameCount);
    
    int32_t lastFrame = static_cast<int32_t>(std::distance(signal.begin(), it));
    if(it == end)
      lastFrame -= 1;
    if(lastFrame > startFrame)
      res.ranges.push_back(SampleRange{startFrame, lastFrame});
  }
  return res;
}

SampleRanges removeShortRanges(const int32_t minSampleFrameCount, SampleRanges const & r)
{
  SampleRanges res;
  for(const auto & ra : r.ranges)
    if(ra.getLength() >= minSampleFrameCount)  
      res.ranges.push_back(ra);
  return res;
}

// Modifies an interlaced sample in case the channel count is not as expected.
//
// countSourceChannels : count of channels in |interlaced| as input.
// countTargetChannels : count of channels in |interlaced| as output.
void interlacedAdaptChannelCount(int countSourceChannels, int countTargetChannels, std::vector<double>& interlaced)
{
  if(countTargetChannels <= 0 || countSourceChannels <= 0 || interlaced.empty())
  {
    interlaced.clear();
    return;
  }
  
  const int64_t countFrames = interlaced.size() / countSourceChannels;
  
  if(countSourceChannels > countTargetChannels)
  {
    // we need to skip some channels.
    for(int64_t frame = 0; frame < countFrames; ++frame)
    {
      const size_t sourceBaseIndex = frame * countSourceChannels;
      const size_t targetBaseIndex = frame * countTargetChannels;
      for(int chanIdx = 0; chanIdx<countTargetChannels; ++chanIdx)
      {
        const size_t targetIdx = targetBaseIndex + chanIdx;
        const size_t sourceIdx = sourceBaseIndex + chanIdx;
        // sourceIdx >= targetIdx
        interlaced[targetIdx] = interlaced[sourceIdx];
      }
    }
    interlaced.resize(countFrames * countTargetChannels);
  }
  else if(countSourceChannels < countTargetChannels)
  {
    // we need to duplicate some channels.
    interlaced.resize(countFrames * countTargetChannels);
    for(int64_t frame = countFrames - 1ll; frame >= 0; --frame)
    {
      const size_t sourceBaseIndex = frame * countSourceChannels;
      const size_t targetBaseIndex = frame * countTargetChannels;
      for(int chanIdx = 0; chanIdx<countTargetChannels; ++chanIdx)
      {
        const size_t targetIdx = targetBaseIndex + chanIdx;
        const size_t sourceIdx = sourceBaseIndex + (chanIdx % countSourceChannels);
        // sourceIdx <= targetIdx
        interlaced[targetIdx] = interlaced[sourceIdx];
      }
    }    
  }
}

void
writeSamples(std::filesystem::path const & samplesFile,
             SampleRanges const & ranges,
             std::filesystem::path const & outDir)
{
  if(std::filesystem::exists(outDir) && !std::filesystem::is_empty(outDir))
    throw std::logic_error("out dir is not empty");
  
  std::filesystem::create_directories(outDir);
  
  auto reader = WAVReader(samplesFile);
  std::vector<double> interleaved;
  int const countChannels = read_wav_as_interleaved_floats(reader, interleaved);
  if(interleaved.empty())
    throw std::logic_error("Failed to read sample");
  interlacedAdaptChannelCount(countChannels, audioelement::countSamplerChannels, interleaved);

  const int32_t countFrames = interleaved.size() / countChannels;

  auto writeSample=[&](const SampleRange& r)
  {
    // Should some parts of the sample be avoided?
    // i.e in "Si", the "S" part is not relevant to the pitch.

    FrequenciesSqMag<double> freqs_sqmag;
    int constexpr zero_padding_factor = 1;

    std::vector<double> half_window;
    
    // cf. comment in testDeduceNotes() for the choice of these constants.
    const int windowLength = floor_power_of_two(std::min(8192*4, r.getLength()));
    const int sigmaFactor{8};
    
    half_gaussian_window<double>(sigmaFactor, windowLength/2, half_window);
    normalize_window(half_window);

    // offset the range to focus on the center of the sample which is more likely to contain
    // relevant information wrt pitch.
    const int startOffset = (r.getLength() - windowLength) / 2;

    auto itStart = interleaved.begin() + (countChannels * (r.firstFrame + startOffset));
    auto itEnd = interleaved.begin() + (countChannels * (r.firstFrame + startOffset + windowLength));
    const int windowed_signal_stride = countChannels; // because the buffer is interleaved.
    findFrequenciesSqMagSlow(itStart, itEnd, windowed_signal_stride, half_window, zero_padding_factor, freqs_sqmag);
    std::vector<FreqMag<double>> localMaxFreqsMags;
    extractLocalMaxFreqsMags(reader.getSampleRate(),
                             freqs_sqmag,
                             SqMagToDb<double>(),
                             localMaxFreqsMags);
    if(localMaxFreqsMags.empty())
      throw std::logic_error("Failed to compute local max freqs mags");
    
    std::optional<FreqMag<double>> fundamentalFreqMag = extractFundamental(localMaxFreqsMags);
    if(!fundamentalFreqMag.has_value())
      throw std::logic_error("Failed to compute fundamentalFreqMag");
    Midi midi;
    std::optional<MidiPitch> pitch = midi.frequency_to_midi_pitch(fundamentalFreqMag->freq);
    if(!pitch.has_value())
      throw std::logic_error("Failed to compute pitch");
    std::cout << "pitch " << pitch->get() << "  window: " << windowLength << std::endl;
    write_wav(outDir / (std::to_string(r.firstFrame) + std::string{"_"} + std::to_string(r.lastFrame) + std::string{".wav"}),
              interleaved.begin() + (countChannels * r.firstFrame),
              interleaved.begin() + (countChannels * (r.lastFrame + 1)),
              CountChannels{countChannels},
              reader.getSampleRate());
  };

  for(const auto & range : ranges.ranges)
  {
    if(range.lastFrame >= countFrames)
      throw std::logic_error("wav file has not enough frames");

    writeSample(range);
  }
}

void
makeSamples(std::filesystem::path const & samplesFile,
            std::filesystem::path const & outDir,
            const DurationNanos slidingAverageDuration = DurationNanos(0.34 * 1e6), // 15 / 44100 
            const DurationNanos minSampleDuration = DurationNanos(150 * 1e6),
            // Lookahead default : 40 milliseconds.
            // It is possible that the same sample decreases in amplitude drastically
            // and then increases within the lookahead.
            // It happens for "Sol" where there are 2 sound parts, "S" and "ol",
            // separated by ~15 milliseconds of low amplitude signal.
            const DurationNanos lookAhead = DurationNanos(40 * 1e6))
{
  // Do computeSampleRanges on each channel, then
  // - merge ranges that overlap across channels.
  // - discard ranges that are smaller than 'minSampleDuration'.
  
  auto reader = std::make_unique<WAVReader>(samplesFile);

  std::vector<std::vector<double>> deinterlaced;
  read_wav_as_floats(*reader, deinterlaced);
  if(deinterlaced.empty())
    throw std::logic_error("Failed to read sample");
  
  const int fileSampleRate = reader->getSampleRate();

  const int32_t minSampleFrameCount = nanoseconds_to_frames(minSampleDuration, fileSampleRate);
  const int32_t lookAheadFrameCount = nanoseconds_to_frames(lookAhead, fileSampleRate);
  
  std::vector<SampleRanges> sampleRangesPerChannel;
  const int countChannels = deinterlaced.size();
  sampleRangesPerChannel.reserve(countChannels);
  for(int chan = 0; chan < countChannels; ++chan)
    sampleRangesPerChannel.push_back(computeSampleRanges(deinterlaced[chan],
                                                         fileSampleRate * (slidingAverageDuration.get() / 1e9),
                                                         lookAheadFrameCount));

  SampleRanges sampleRanges = unionRanges(sampleRangesPerChannel);

  sampleRanges = removeShortRanges(minSampleFrameCount, sampleRanges);
  
  reader.reset();
  writeSamples(samplesFile, sampleRanges, outDir);
}

std::vector<std::pair<NoteOctave, std::filesystem::path>>
readSamples(std::filesystem::path const & dir)
{}

std::map<double, std::vector<double>>
readSamples()
{
  auto folder = std::filesystem::path{"/Users/Olivier/Music/Samples/Manual"};
  Midi midi;
  std::vector<std::pair<NoteOctave, std::filesystem::path>> files{
    {NoteOctave{Note::Do, 4}, "Do1.wav"},
    {NoteOctave{Note::Mi, 4}, "Mi1.wav"},
    {NoteOctave{Note::Fa, 4}, "Fa1.wav"},
    {NoteOctave{Note::Sol, 4}, "Sol1.wav"},
  };
  
  std::map<double, std::vector<double>> res;
  for(const auto & [noteOctave, f] : files)
  {
    auto reader = WAVReader(folder / f);
    
    std::vector<double> interlaced;
    int const countChannels = read_wav_as_interleaved_floats(reader, interlaced);
    if(interlaced.empty())
      throw std::logic_error("Failed to read sample");
    interlacedAdaptChannelCount(countChannels, audioelement::countSamplerChannels, interlaced);
    auto pitch = midi.get_pitch(noteOctave);
    auto freq = midi.midi_pitch_to_freq(pitch);
    auto ai = freq_to_angle_increment(freq, m_sampleRate);
    res[ai] = interlaced;
  }
  return res;
}

// Main class.
struct AppTune
{
  AppTune(size_t countOscSynths, size_t countSamplerSynths);
  ~AppTune();
  
  void playEventStreams(std::vector<std::unique_ptr<EventStream>>& streams);

  void playLoop(Loop && loop, uint64_t countLoops = 1000)
  {
    std::unique_ptr<EventStream> stream = std::make_unique<LoopEventStream>(std::move(loop), countLoops);
    std::vector<std::unique_ptr<EventStream>> streams;
    streams.push_back(std::move(stream));
    playEventStreams(streams);
  }
  
  OscSynthDef& oscSynth(size_t i) {return m_oscSynths[i];}
  SamplerSynthDef& samplerSynth(size_t i) {return m_samplerSynths[i];}
private:
  
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
  Stepper m_stepper;
  std::vector<OscSynthDef> m_oscSynths;
  std::vector<SamplerSynthDef> m_samplerSynths;

  std::map<double, std::vector<double>> m_samples;

  std::atomic<size_t> m_countCinChars{};
  std::atomic_bool m_runCinListener{true};
  std::thread m_cinListener;
  
  void updateInitializersIfNeeded();
  
  template<typename F>
  void forEachSynth(F && f)
  {
    for(auto & s : m_oscSynths)
      f(s);
    for(auto & s : m_samplerSynths)
      f(s);
  }

  void allNotesOff() {
    forEachSynth([&](auto & s){
      s.m_synth.allNotesOff(m_stepper);      
    });
  }  
};

AppTune::AppTune(size_t countOscSynths, size_t countSamplerSynths)
: m_stepper(GlobalAudioLock<audioEnginePolicy>::get(),
            countOscSynths * OscSynthDef::OscSynth::n_channels * 4 /* one shot */ +
            countSamplerSynths * SamplerSynthDef::SamplerSynth::n_channels * 4 /* one shot */,
            (countOscSynths + countSamplerSynths) * 1 /* a single compute is needed per synth*/)
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
, m_oscSynths(countOscSynths)
, m_samplerSynths(countSamplerSynths)
{
  if(countSamplerSynths)
  {
    m_samples = readSamples();
  }
  forEachSynth([&](auto&synth){
    synth.m_synth.initialize(m_stepper);
  });
  
  if (!m_ctxt.doInit(0.008,
                     m_sampleRate,
                     nAudioOut,
                     [this]
                     (SAMPLE *outputBuffer,
                      int nFrames,
                      TimeNanos const tNanos){
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
  forEachSynth([&](auto&synth){
    synth.m_synth.finalize(m_stepper);
  });
  
  // We sleep so that the audio produced during finalization has a chance to be played
  // (taking into account the delay between when the audio is written in the buffer
  //  and when the audio is actually played in the speakers).
  
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  
  m_ctxt.doTearDown();
  
  m_runCinListener = false;
  m_cinListener.join();
}

void AppTune::updateInitializersIfNeeded()
{
  for(auto & synth : m_oscSynths)
    synth.updateInitializerIfNeeded();
  for(auto & synth : m_samplerSynths)
    synth.updateInitializerIfNeeded(m_samples);
}

void OscSynthDef::updateInitializerIfNeeded()
{
  {
    const std::filesystem::file_time_type newLastWriteHarmonics =
    std::max(std::max(std::filesystem::last_write_time(harmonicsFile),
             std::filesystem::last_write_time(envelopeFile)),
             std::filesystem::last_write_time(lowPassFile));
    if(m_lastWriteSynthFiles.has_value() && newLastWriteHarmonics <= *m_lastWriteSynthFiles)
      // No need to update, the files have not changed since last time the initializer has been updated.
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
    
  auto initializer = audioelement::TuneOscElementInitializer<double>{
    m_sampleRate,
    1, // not used,
    0., // not used,
    mkLowPass(),
    mkEnvelope(),
    mkHarmonics()
  };
  m_synth.setSynchronousElementInitializer(initializer);
}


void SamplerSynthDef::updateInitializerIfNeeded(std::map<double, std::vector<double>> const& samples)
{
  {
    const std::filesystem::file_time_type newLastWriteHarmonics =
      std::filesystem::last_write_time(envelopeFile);
    if(m_lastWriteSynthFiles.has_value() && newLastWriteHarmonics <= *m_lastWriteSynthFiles)
      // No need to update, the files have not changed since last time the initializer has been updated.
      return;
    // This initializer update will affect any note that has not started yet.
    m_lastWriteSynthFiles = newLastWriteHarmonics;
  }
  
  auto initializer = audioelement::TuneSamplerElementInitializer<double>{
    m_sampleRate,
    1, // not used,
    mkEnvelope(),
    std::reference_wrapper{samples}
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

MidiPitchStreamFromBinary streamFromBinaryPitchesEncoding(std::filesystem::path const& scoreBinaryFile, EventsTiming loopTiming, Polyphony countVoices)
{
  const size_t batchSize = 10000;
  const auto maxConsecutiveBytes = MaxConsecutiveBytes{11};
  
  const auto note_period = loopTiming.note_period();  
  const auto period_batch = note_period * batchSize;
  std::cout << "One batch corresponds to " << std::chrono::duration_cast<std::chrono::seconds>(period_batch).count() << " seconds" << std::endl;
  
  auto batchesByMaxFreq = statsFromBinary(std::cout, scoreBinaryFile, batchSize, maxConsecutiveBytes);
  
  std::cout << "# Batches by max freq / max consecutive bytes:" << std::endl;
  for(const auto &[batchKey, batches] : batchesByMaxFreq)
  {
    std::cout << batchKey.maxByteFreq << ", " << batchKey.maxConsecutiveBytes << ":" << batches.size() << std::endl;
  }
  
  
  std::vector<std::pair<BatchKey, ByteRange>> notBoringFirst, boringFirst;
  auto tryUseBatch = [&](const BatchKey& key, const std::vector<Batch>& batches, std::vector<std::pair<BatchKey, ByteRange>>& res)
  {
    const auto maxFreq = key.maxByteFreq;
    // Skip batches that are too "boring" i.e
    // - some notes are too frequent, or
    // - at least one note is consecutively repeated too many times
    //
    // Note: the multiplicative constant was modified from 0.1 to 0.03
    // to select the interesting batch of Feuillard.
    // Ideally we should investigate why
    // the first select Feuillard with 0.1 sounds "boring"
    // and why the one selected for 0.03 is "interesting"
    // and deduce a new criteria for batch selection.
    // 
    // My intuition is that the batch size 10000 is so large (corresponds to 1000 seconds of audio)
    // that the global statistics of the batch are maybe not representative of the beginning of the batch.
    //
    // It would also be interesting to actually understand what kind of data is encoded in each batch:
    // for the Feuillard pdf do we have a contiguous portion of an image in each batch?
    if(maxFreq > 0.03 * batchSize)
      return;
    for(const auto& batch : batches)
    {
      // Todo: it could be interesting to include prev/next batches as well if they are not included?
      // Todo: it could be interesting to merge consecutive batches if they are all "interesting" enough.
      for(const auto & range : batch.ranges)
        res.emplace_back(key, range);
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
  
  return MidiPitchStreamFromBinary(scoreBinaryFile,
                                   ByteRangesIterator{std::move(byteRangesByByteMaxFreq)},
                                   ReinitCycleAtRangeBoundary::No,
                                   // when we use batches that are not "boring",
                                   // UniformCycleInitialization::Yes is good:
                                   // this way, we see a long term progression of the music from
                                   // quite uniform to quite varied.
                                   // with UniformCycleInitialization::No the music feels much more
                                   // "random" from the start so it is harder for the listener to
                                   // hear a progression in the unfolding of the music.
                                   UniformCycleInitialization::Yes,
                                   countVoices);
}

std::unique_ptr<EventStream> toEventStream(MidiPitchStreamFromBinary && s, EventsTiming loopTiming)
{
  return std::make_unique<EventStreamFromBinary>(std::move(s), loopTiming);
}

template<typename Stream>
Score scoreFromStream(Stream& stream)
{
  Score score;
  score.m_voices.emplace_back();
  while(std::optional<MidiPitch> pitch = stream())
  {
    score.m_voices.back().m_pitches.push_back(*pitch);
  }
  return score;
}
template<typename Stream>
Score scoreFromStream(Stream& stream, Polyphony countVoices)
{
  Score score;
  score.m_voices.resize(countVoices.get());
  for(;;)
  {
    size_t countEOS{};
    for(size_t voice=0; voice<countVoices.get(); ++voice)
    {
      if(std::optional<MidiPitch> pitch = stream(voice))
        score.m_voices[voice].m_pitches.push_back(*pitch);
      else
        ++countEOS;
    }
    if(countEOS == countVoices.get())
      break;
  }
  return score;
}

Score readScoreFromBinaryPitchesEncoding(std::filesystem::path const& scoreBinaryFile, EventsTiming loopTiming, Polyphony countVoices)
{
  auto stream = streamFromBinaryPitchesEncoding(scoreBinaryFile, loopTiming, countVoices);
  return scoreFromStream(stream, countVoices);
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

Loop loopFromScore(Score const & score, const EventsTiming& timing)
{
  Midi m_midi;
  std::vector<std::pair<TimestampAndSource, Event>> events;
    
  const auto wait_after_note_on = timing.wait_after_note_on();
  const auto wait_after_note_off = timing.wait_after_note_off();

  const float volume = 1.f / std::max(1ul, score.m_voices.size());
  
  DurationNanos maxCurDurationNanos{};
  
  uint64_t voice{};
  for(const auto & consecutivePitches : score.m_voices)
  {
    DurationNanos curDurationNanos{};
    for(auto midiPitch : consecutivePitches.m_pitches)
    {
      const auto noteid = mk_note_id();
      const float frequency = m_midi.midi_pitch_to_freq(midiPitch);
      events.emplace_back(TimestampAndSource{Loop::c_refTime + curDurationNanos, voice},
                          mkNoteOn(noteid,
                                   frequency,
                                   volume));
      curDurationNanos += wait_after_note_on;
      events.emplace_back(TimestampAndSource{Loop::c_refTime + curDurationNanos, voice},
                          mkNoteOff(noteid));
      curDurationNanos += wait_after_note_off;
    }
    maxCurDurationNanos = std::max(maxCurDurationNanos, curDurationNanos);
    ++voice;
  }
  return {std::move(events), maxCurDurationNanos};
}

const auto synth = std::filesystem::path{"/Users/Olivier/Dev/cpp.audio/Synth/"};
const auto scores = std::filesystem::path{"/Users/Olivier/Dev/cpp.audio/Scores/"};

Loop mkEvents(int countNotes, EventsTiming loopTiming = {})
{
  const Score score = mkScore(countNotes);
  writeScoreInSimpleAsciiPitchesEncoding(score, scores/("test" + std::to_string(countNotes) + ".txt"));
  return loopFromScore(score, loopTiming);  
}

Loop loopFromAscii(std::filesystem::path const& scoreFile, EventsTiming loopTiming = {})
{
  const Score score = readScoreFromSimpleAsciiPitchesEncoding(scoreFile);
  
  return loopFromScore(score, loopTiming);
}

Loop loopFromBinary(std::filesystem::path const& scoreFile, Polyphony countVoices, EventsTiming loopTiming = {})
{
  const Score score = readScoreFromBinaryPitchesEncoding(scoreFile, loopTiming, countVoices);
  
  return loopFromScore(score, loopTiming);
}


// Should we have one stream per synth, or one stream for all synths?
// one stream per synth feels more modular.
// one stream can do polyphony within the same synth.
void AppTune::playEventStreams(std::vector<std::unique_ptr<EventStream>>& streams)
{
  auto countCinChars = m_countCinChars.load(std::memory_order_relaxed);
  auto must_interrupt = [&]()
  {
    return countCinChars != m_countCinChars.load(std::memory_order_relaxed);
  };
  
  // Use a delay of one second
  constexpr DurationNanos bufferNanos{static_cast<uint64_t>(1e9)};
  {
    // This is the offset we need to apply to events if we want them to start playing right away.
    const TimeNanos ctxtCurTimeNanos(nanos_per_frame<double>(m_sampleRate) * m_ctxt.getCountFrames());

    // To ensure that the rythm of events will be ok, we add one second to that    
    // so that audio won't depend on thread scheduling.
    for(auto & stream : streams)
      stream->startStream(ctxtCurTimeNanos + bufferNanos);
  }

  const auto sz = streams.size();
  if(sz > (m_oscSynths.size() + m_samplerSynths.size()))
    throw std::logic_error("too many streams");
  const auto firstSamplerIdx = m_oscSynths.size();

  Events events;
  for(;;)
  {
    size_t count_yields{};
  retry:
  
    if(must_interrupt())
    {
      for(auto & stream : streams)
        stream->stopStream();
      allNotesOff();
      break;
    }

    updateInitializersIfNeeded();
    
    const TimeNanos ctxtCurTimeNanos(nanos_per_frame<double>(m_sampleRate) * m_ctxt.getCountFrames());
    
    size_t countEOS{};
    for(size_t i=0; i<sz; ++i)
    {
      // Materialize events that have not been materialized yet and need
      // to be played during the next "bufferNanos" time.
      events.clear();
      auto streamRes = streams[i]->materializeNextEvents(events, ctxtCurTimeNanos + bufferNanos);
      
      if(!events.empty())
      {
        for(const auto & event : events)
        {
          if(i < firstSamplerIdx)
          {
            auto const res = m_oscSynths[i].m_synth.onEvent(m_sampleRate,
                                                            event.second,
                                                            m_stepper,
                                                            m_stepper,
                                                            event.first);
          }
          else
          {
            auto const res = m_samplerSynths[i - firstSamplerIdx].m_synth.onEvent(m_sampleRate,
                                                                                  event.second,
                                                                                  m_stepper,
                                                                                  m_stepper,
                                                                                  event.first);            
          }
        }
        continue;
      }
      else if(streamRes == StreamStatus::EndOfStream)
      {
        countEOS++;
      }
    }
    
    if(countEOS == sz)
      break;
      
    ++count_yields;
    std::this_thread::yield();
    goto retry;
  }

  for(auto & stream : streams)
    stream->stopStream();

  // All notes have started (and stopped) but some envelope tails may still be active.
  
  forEachSynth([](auto&synth){
    while(!synth.m_synth.allEnvelopesFinished())
      std::this_thread::yield();
  });
}

// moduloPitch is a way to reduce the range of generated pitches.
// It is an approach somewhat complementary to low-pass filtering.
Loop moduloPitch(Loop && l)
{
  Midi midi;
  
  constexpr MidiPitch minPitch{50.};
  constexpr MidiPitch maxPitch{80.};

  for(size_t i=0, sz = l.countEvents(); i<sz; ++i)
  {
    auto & e = l.mutEvent(i);
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


// This function has as many explicit parameters as possible,
// so that if we change the defaults later on we will be able to replay
// the music.
void playFeuillardTwoVoices()
{
  using namespace imajuscule::audio;
  using namespace std::filesystem;

  const size_t countOscSynths{2};
  const size_t countSamplerSynths{0};
  auto a = AppTune{countOscSynths, countSamplerSynths};

  const path fastEnv{synth / "EnvelopeFast.txt"};
  const path fastFunEnv{synth / "EnvelopeFastFun.txt"};

  const path harSimple{synth / "HarmonicsSimple.txt"};
  const path har{synth / "Harmonics.txt"};

  const path lowPass{synth / "LowPass.txt"};
  
  a.oscSynth(0).lowPassFile = lowPass;
  a.oscSynth(0).setHarmonicsFile(harSimple);
  a.oscSynth(0).setEnvelopeFile(fastFunEnv);

  a.oscSynth(1).lowPassFile = lowPass;
  a.oscSynth(1).setHarmonicsFile(har);
  a.oscSynth(1).setEnvelopeFile(fastEnv);
  
  const auto file = "/Users/Olivier/Downloads/IMSLP874967-PMLP1036212-Feuillard_La_Technique_du_Violoncelle_Vol.4.pdf";
  
  const auto timing1 = EventsTiming{0.09};
  auto stream1 = toEventStream(streamFromBinaryPitchesEncoding(file, timing1, Polyphony{1}), timing1);
  const auto timing2 = EventsTiming{0.18};
  auto stream2 = toEventStream(streamFromBinaryPitchesEncoding(file, timing2, Polyphony{1}), timing2);
  // ^^ With:
  //              polyphony  speed
  // - stream1    1          2
  // - stream2    1          1
  //
  // Both stream play the same melody but one plays at half speed,
  // which results in interesting patterns.
  
  // ^^ With:
  //              polyphony  speed
  // - stream1    1          2   
  // - stream2    2          1
  //
  // The "slow" stream (stream2) plays simultaneously the 2 notes
  // that the fast stream (stream1) plays in sequence.
  // This is interesting but not as interesting as the previous set of settings.
  
  std::vector<std::unique_ptr<EventStream>> streams;
  streams.push_back(std::move(stream1));
  streams.push_back(std::move(stream2));
  a.playEventStreams(streams);
}

int main() {
#if 0
  playFeuillardTwoVoices();
#else
  using namespace imajuscule::audio;
  using namespace std::filesystem;

  makeSamples("/Users/Olivier/Music/Samples/gamme.wav", "/Users/Olivier/Music/Samples/gen");

  const size_t countOscSynths{2};
  const size_t countSamplerSynths{2};
  auto a = AppTune{countOscSynths, countSamplerSynths};
    
  // The ZeroEnvelope file produces drum sounds.
  const path zeroEnv{synth / "EnvelopeZero.txt"};
  const path oneEnv{synth / "EnvelopeOne.txt"};
  const path fastEnv{synth / "EnvelopeFast.txt"};
  const path fastFunEnv{synth / "EnvelopeFastFun.txt"};
  const path slowEnv{synth / "EnvelopeSlow.txt"};

  const path harSimple{synth / "HarmonicsSimple.txt"};

  a.samplerSynth(0).setEnvelopeFile(oneEnv);
  a.samplerSynth(1).setEnvelopeFile(oneEnv);

  a.oscSynth(0).setHarmonicsFile(harSimple);

  for(auto const &env : {zeroEnv, fastFunEnv})
  {
    a.oscSynth(0).setEnvelopeFile(env);

    const auto file = "/Users/Olivier/Downloads/IMSLP874967-PMLP1036212-Feuillard_La_Technique_du_Violoncelle_Vol.4.pdf";

    const auto timing1 = EventsTiming{0.09};
    auto stream1 = toEventStream(streamFromBinaryPitchesEncoding(file, timing1, Polyphony{1}), timing1);
    const auto timing2 = EventsTiming{0.18};
    auto stream2 = toEventStream(streamFromBinaryPitchesEncoding(file, timing2, Polyphony{1}), timing2);
    
    std::vector<std::unique_ptr<EventStream>> streams;
    // osc synth streams:
    streams.push_back(std::make_unique<NullEventStream>());
    streams.push_back(std::make_unique<NullEventStream>());
    // sampler synth streams:
    streams.push_back(std::move(stream1));
    streams.push_back(std::move(stream2));
    
    a.playEventStreams(streams);

/*
    a.playLoop(moduloPitch(loopFromBinary(file, Polyphony{1})), 1);

    a.playLoop(loopFromBinary(scores / "Phrase.txt", Polyphony{1}), 1);
    a.playLoop(moduloPitch(loopFromBinary(scores / "Phrase.txt", Polyphony{1})), 1);

    a.playLoop(loopFromAscii(scores / "Phrase.txt"), 4);
    a.playLoop(loopFromAscii(scores / "StrangeBots.txt"), 4);
    a.playLoop(loopFromAscii(scores / "Phrase2.txt"), 4);
    
    a.playLoop(mkEvents(250), 1);
 */
  }
#endif
  return 0;
}
