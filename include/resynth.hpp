
namespace imajuscule::audio {

template<typename T>
void resynth(std::vector<DeducedNote<T>> const & notes,
             int const stride,
             int const sampling_rate,
             DirectoryPath const & dir_wav,
             FileName const & file_wav) {
  // For now, simply "drive" audioelements, and manually mix them.

  using namespace audioelement;

  using AE = VolumeAdjusted<OscillatorAlgo<double>>; // no loudness volume adjustment
  using Envelope = AHDSREnvelope<Atomicity::No, double, EnvelopeRelease::WaitForKeyRelease>;
  using EnvelopedAE = Enveloped<AE, Envelope>;

  std::vector<std::unique_ptr<EnvelopedAE>> pool;

  auto getAvailableOscillator = [&pool] () {
    for (auto & p : pool) {
      if (!p->getEnvelope().isEnvelopeRTActive()) {
        return p.get();
      }
    }
    pool.push_back(std::make_unique<EnvelopedAE>());
    return pool.back().get();
  };

  auto wave_format = WaveFormat::IEEE_FLOAT;
  auto header = pcm(wave_format,
                    sampling_rate,
                    NChannels::ONE,
                    AudioSample<double>::format);

  WAVWriter writer(dir_wav, file_wav, header);
  writer.Initialize();

  auto recordFrame = [&pool, &writer] () {
    double res = 0.;
    bool empty = true;
    for (auto const & p : pool) {
      Assert(p);
      if (!p->getEnvelope().isEnvelopeRTActive()) {
        continue;
      }
      empty = false;
      p->step();
      res += p->imag();
    }
    writer.writeSample(res);
    return empty;
  };

  std::vector<DeducedNote<T>> byStartFrame = notes;

  // unstride frames

  for (auto & n : byStartFrame) {
    n.startFrame *= stride;
    n.endFrame *= stride;
  }

  // in order to minimize the max number of audioelements in the pool,
  // we sort by start time and process them in that order
  std::sort(byStartFrame.begin(),
            byStartFrame.end(),
            [](DeducedNote<double> const & n,
               DeducedNote<double> const & m) {
    return n.startFrame < m.startFrame;
  });

  int frame = 0;

  for (auto const & note : byStartFrame) {
    // byStartFrame is ordered by start frame so there is no other note that must start before noteStartFrame

    for (; frame < note.startFrame; ++frame) {
      recordFrame();
    }

    Assert(note.endFrame >= note.startFrame);

    EnvelopedAE * ae = getAvailableOscillator();
    ae->forgetPastSignals();
    ae->setAngleIncrements(freq_to_angle_increment(note.frequency, sampling_rate));
    ae->getAlgo().setVolumeTarget(DbToMag<double>()(note.amplitude));
    ae->editEnvelope().setAHDSR(AHDSR{
      1000, itp::LINEAR,
      0,
      1000, itp::LINEAR,
      10000, itp::LINEAR,
      0.7f
    }, sampling_rate);

    ae->onKeyPressed(0);
    ae->onKeyReleased(stride + note.endFrame - note.startFrame);
  }

  while (!recordFrame()) {}
}

} // NS
