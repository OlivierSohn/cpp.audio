

namespace imajuscule::audio {


void printDevices() {
  int numDevices;
  numDevices = Pa_GetDeviceCount();
  if( numDevices < 0 )
  {
    std::cerr << "Pa_CountDevices returned " << numDevices << std::endl;
    return;
  }
  const   PaDeviceInfo *deviceInfo;
  for( int i=0; i<numDevices; i++ )
  {
    deviceInfo = Pa_GetDeviceInfo( i );
    std::cout << i << ":";
    if (!deviceInfo) {
      std::cout << "null device" << std::endl;
    } else {
      std::cout << "structVersion            :" << deviceInfo->structVersion << std::endl;
      std::cout << "name                     :" << deviceInfo->name << std::endl;
      std::cout << "hostApi                  :" << deviceInfo->hostApi << std::endl;
      std::cout << "maxInputChannels         :" << deviceInfo->maxInputChannels << std::endl;
      std::cout << "maxOutputChannels        :" << deviceInfo->maxOutputChannels << std::endl;
      std::cout << "defaultLowInputLatency   :" << deviceInfo->defaultLowInputLatency << std::endl;
      std::cout << "defaultLowOutputLatency  :" << deviceInfo->defaultLowOutputLatency << std::endl;
      std::cout << "defaultHighInputLatency  :" << deviceInfo->defaultHighInputLatency << std::endl;
      std::cout << "defaultHighOutputLatency :" << deviceInfo->defaultHighOutputLatency << std::endl;
      std::cout << "defaultSampleRate        :" << deviceInfo->defaultSampleRate << std::endl;
    }
    std::cout << std::endl;
  }
}

double getGoodSuggestedLatency(double suggestedLatency,
                               int const sample_rate) {
  
  // To optimize cache use, the callback should be asked to compute
  // a multiple of n_frames_per_buffer frames at each call.
  // So we round the latency such that this requirement is met.
  
  int suggestedLatencyInSamples = static_cast<int>(0.5f + suggestedLatency * static_cast<float>(sample_rate));
  LG(INFO, "suggested latency in samples       : %d", suggestedLatencyInSamples);
  using namespace audioelement;
  
  // we ceil to the next multiple of 'n_frames_per_buffer'
  int ceiledSLIS = n_frames_per_buffer * (1 + (suggestedLatencyInSamples-1) / n_frames_per_buffer);
  LG(INFO, " ceiled to the next multiple of %d : %d", n_frames_per_buffer, ceiledSLIS);
  
  // we have cracks on osx, rounding to the next power of 2 fixed this.
  ceiledSLIS = ceil_power_of_two(ceiledSLIS);
  LG(INFO, " ceiled to the next power of 2 : %d", ceiledSLIS);
  
  // The current portaudio doc says, about suggestedLatency:
  //   "implementations should round the suggestedLatency up to the next practical value
  //     - ie to provide an equal or higher latency than suggestedLatency wherever possible"
  //   http://portaudio.com/docs/v19-doxydocs/structPaStreamParameters.html#aa1e80ac0551162fd091db8936ccbe9a0
  //
  // But what I found is that the resulting latency is 'equal or lower' to the suggested one.
  // Hence, I add epsilon here instead of substracting it, to be sure I will get the right buffer sizes.
  double res = static_cast<double>(ceiledSLIS) / static_cast<double>(sample_rate) + 1e-6;
  LG(INFO, "p.suggestedLatency          : %f", res);
  return res;
}

#ifndef NDEBUG
std::atomic_int& PartialCallbackCheck::flag() {
  static std::atomic_int f{0};
  return f;
}
#endif

#ifndef NDEBUG
void analyzeTime(uint64_t t,
                 int nFrames,
                 int sample_rate,
                 PortaudioAsyncLogger & logger) {
  static uint64_t next = 0;
  if(likely(next!= 0)) {
    // verify that 't' is approximately equal to next.
    uint64_t diff = cyclic_unsigned_dist(t,next);
    float nSamples = static_cast<float>(diff) / nanos_per_frame<float>(sample_rate);
    if(nSamples > 2) {
      logger.sync_feed(SignificantTimeDeviation{diff, nSamples});
    }
    
  }
  
  next = t + static_cast<uint64_t>(0.5f + nFrames * nanos_per_frame<float>(sample_rate));
}
#endif

} // NS
