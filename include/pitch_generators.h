namespace imajuscule::audio {

// Designed for playing scales and arpegios on multiple octaves.
//
// Repeats the midi pitch sequence over multiple octaves,
// first ascending then descending:
//
// for example if the sequence is 1, 2, 3 and the number of octaves is 2:
//
// 1      // starts ascending
// 2
// 3
// 1 + 12
// 2 + 12
// 3 + 12
// 1 + 24  // reached 2 octavtes, now will be desceding
// 3 + 12
// 2 + 12
// 1 + 12
// 3
// 2
// 1      // ascend again
// 2
// etc...
struct MultiOctave {
  MultiOctave(const MidiPitch* midiPitchSeqBegin, const MidiPitch* midiPitchSeqEnd, int countOctaves)
  : m_midiPitchSeqBegin(midiPitchSeqBegin)
  , m_midiPitchSeqEnd(midiPitchSeqEnd)
  , m_endOctave(countOctaves)
  , m_nextPitch(midiPitchSeqBegin)
  {}
  
  MidiPitch operator()()
  {
    if(m_endOctave <= 0)
      // just return the base pitch
      goto return_base;
    
    if(m_asc)
    {
      // when we are ascending, m_nextPitch is the next pitch that will be played.
      if(m_nextPitch < m_midiPitchSeqEnd)
      {
      return_regular_asc:
        return *(m_nextPitch++) + (m_curOctave * num_halftones_per_octave);
      }
      else
      {
        // m_nextPitch == m_midiPitchSeqEnd
        if(m_curOctave < m_endOctave)
        {
          ++m_curOctave;
          if(m_curOctave < m_endOctave)
          {
            m_nextPitch = m_midiPitchSeqBegin;
            goto return_regular_asc;
          }
          else
          {
            // m_curOctave == m_endOctave
          return_base:
            return *m_midiPitchSeqBegin + (m_curOctave * num_halftones_per_octave);
          }
        }
        else
        {
          // m_curOctave == m_endOctave
          m_asc = false;
          m_curOctave = m_endOctave - 1;
        }
      }
    }
    
    // m_asc == false
    
    // when we are descending, m_nextPitch is the previous pitch that was played.
    if(m_nextPitch > m_midiPitchSeqBegin)
    {
    return_regular_desc:
      return *(--m_nextPitch) + (m_curOctave * num_halftones_per_octave);
    }
    else
    {
      // m_nextPitch == m_midiPitchSeqBegin
      if(m_curOctave > 0)
      {
        --m_curOctave;
        m_nextPitch = m_midiPitchSeqEnd;
        goto return_regular_desc;
      }
      else
      {
        // m_curOctave == 0
        m_asc = true;
        ++m_nextPitch; // because we have already played this.
        goto return_regular_asc;
      }
    }
  }
  
private:
  const MidiPitch* m_midiPitchSeqBegin;
  const MidiPitch* m_midiPitchSeqEnd;
  int m_endOctave;
  
  const MidiPitch* m_nextPitch;
  int m_curOctave = 0;
  bool m_asc = true;
};


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
  
  MidiPitch operator()() {
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
  std::vector<MidiPitch> m_nextValues;
  std::vector<size_t> m_pattern;
  size_t m_patternIndex{};
};


// Helper class to generate a slight pitch drift over time,
// mimicking what can happen on the cello when we don't play
// any open string for a long time.
struct PitchDrifter
{
  PitchDrifter(double constantDrift)
  : constantDrift(constantDrift)
  {}

  MidiPitch operator()(MidiPitch pitch)
  {
    pitchDrift += constantDrift;
    return pitch + pitchDrift;
  }
  
private:
  float pitchDrift = 0.f;
  double constantDrift;
};


}
