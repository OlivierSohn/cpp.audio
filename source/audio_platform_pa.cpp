

namespace imajuscule::audio {

  void analyzeTime(uint64_t t, int nFrames) {
    static uint64_t next = 0;
    if(likely(next!= 0)) {
      // verify that 't' is approximately equal to next.
      uint64_t diff = cyclic_unsigned_dist(t,next);
      float nSamples = static_cast<float>(diff) / nanos_per_frame<float>();
      if(nSamples > 2) {
        LG(ERR,"Significant time deviation: %llu (%f samples)", diff, nSamples);
      }

    }

    next = t + static_cast<uint64_t>(0.5f + nFrames * nanos_per_frame<float>());
  }
}
