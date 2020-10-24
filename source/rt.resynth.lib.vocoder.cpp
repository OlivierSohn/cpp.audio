#define IMJ_DEBUG_VOCODER 1

namespace imajuscule::audio::rtresynth {

struct SetupParams {

  // Params for modulator:
  float env_follower_cutoff_ratio;
  float modulator_window_size_seconds;
  
  // Params for modulator and carrier:
  float stride_seconds;
  int count_bands;
  float min_freq;
  float max_freq;
  
  void fill_freqs(std::vector<double> & freqs) const {
    freqs.clear();
    reserve_no_shrink(freqs,
                      count_bands + 1);
    
    auto log_min = std::log(min_freq);
    auto log_max = std::log(max_freq);
    
    for (int i=0; i<=count_bands; ++i) {
      double const ratio = static_cast<double>(i) / count_bands;
      freqs.push_back(std::exp(log_min + ratio * (log_max-log_min)));
    }
  }
  
  bool operator !=(SetupParams const & o) const {
    return
    std::make_tuple(env_follower_cutoff_ratio,
                    modulator_window_size_seconds,
                    stride_seconds,
                    count_bands,
                    min_freq,
                    max_freq) !=
    std::make_tuple(o.env_follower_cutoff_ratio,
                    o.modulator_window_size_seconds,
                    o.stride_seconds,
                    o.count_bands,
                    o.min_freq,
                    o.max_freq);
  }
};

/*
template<int ORDER, typename T>
struct BandPass {
  void setup(int sample_rate, T freq_from, T freq_to) {
    hp_.initWithFreq(sample_rate, freq_from);
    lp_.initWithFreq(sample_rate, freq_to);
  }
  
  T feed(T sample) {
    hp_.feed(&sample);
    lp_.feed(hp_.filtered());
    return *(lp_.filtered());
  }

private:
  Filter<T, 1, FilterType::HIGH_PASS, ORDER> hp_;
  Filter<T, 1, FilterType::LOW_PASS, ORDER> lp_;
};

template<int ORDER, typename T>
struct EnvelopeFollower {
  void setup(int sample_rate, T freq) {
    lp_.initWithFreq(sample_rate, freq);
  }
  
  T feed(T sample) {
    lp_.feed(&sample);
    return *(lp_.filtered());
  }

private:
  Filter<T, 1, FilterType::LOW_PASS, ORDER> lp_;
};
*/

/*
 "Good" in the sense that crossfades have an even half length
 */
inline int good_stride(float const seconds,
                       int sample_rate) {
  int res = std::max(1,
                     static_cast<int>(0.5f + seconds * sample_rate));
  if (0 == res % 2) {
    ++res; // EqualGainXFade requires that the xfade size is even
  }
  return res;
}

constexpr std::size_t modulator_max_fft_size = pow2(16);
constexpr std::size_t carrier_max_fft_size = 2 * modulator_max_fft_size;
constexpr std::size_t max_stride_size = pow2(16);

constexpr std::size_t max_count_bands = 50;

template<typename T>
struct FFTModulator {
  FFTModulator() {
    bands_amplitudes.reserve(max_count_bands);
    
    freqs.reserve(max_count_bands + 1);
  }

  void init_dynamic_allocs(int sample_rate) {
    periodic_fft.setLambdas([this, sample_rate]() {
      int candidate_sz = std::max(1,
                                  static_cast<int>(0.5f + sample_rate * window_size_seconds));
      if (1 == candidate_sz % 2) {
        ++candidate_sz;
      }
      Assert(candidate_sz <= static_cast<int>(modulator_max_fft_size)); // to avoid dynamic allocations in real time thread
      return std::min(static_cast<int>(modulator_max_fft_size),
                      candidate_sz);
    },
                            [this, sample_rate]() {
      return getStride(sample_rate);
    },
                            [this, sample_rate](int const window_center_stride,
                                                FrequenciesSqMag<double> const & frequencies_sqmag) {
      std::fill(bands_amplitudes.begin(),
                bands_amplitudes.end(),
                static_cast<T>(0));
      
      Assert(frequencies_sqmag.get_fft_length() <= static_cast<int>(modulator_max_fft_size));
      double const bin_to_Hz = frequencies_sqmag.bin_index_to_Hz(sample_rate);
      
      auto itLow = freqs.begin();
      auto end = freqs.end();
      Assert (itLow != end);
      auto itHigh = itLow + 1;
      Assert (itHigh != end);
      int band_idx = 0;
      
      for (int i=0, sz = static_cast<int>(frequencies_sqmag.frequencies_sqmag.size()); i < sz; ++i) {
        double const Hz = bin_to_Hz * i;
        if (Hz <= *itLow) {
          continue;
        }
        while (Hz > *itHigh) {
          ++band_idx;
          ++itHigh;
          ++itLow;
          if (itHigh == end) {
            break;
          }
        }
        if (itHigh == end) {
          break;
        }
        Assert(Hz <= *itHigh);
        Assert(Hz > *itLow);
        bands_amplitudes[band_idx] += frequencies_sqmag.frequencies_sqmag[i];
        //bands_amplitudes[band_idx] = std::max(new_bands_amplitudes[band_idx],
        //                                      frequencies_sqmag.frequencies_sqmag[i]);
      }
      for (auto & a : bands_amplitudes) {
        a = std::sqrt(a);
      }
    }
                            );
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input = std::make_unique<AsyncWavWriter>(1,
                                                                  sample_rate,
                                                                  "debug_modulator_input");
    for (std::size_t i=0; i<max_count_bands; ++i) {
      wav_writer_modulator_envelopes.push_back(std::make_unique<AsyncWavWriter>(1,
                                                                                sample_rate,
                                                                                "debug_modulator_envelope_" + std::to_string(i)));
    }
#endif
  }

  void setup(SetupParams const & p) {
    p.fill_freqs(freqs);
    sz = static_cast<int>(freqs.size()) - 1;
    Assert(sz > 0);
    
    Assert(static_cast<int>(bands_amplitudes.capacity()) >= sz);
    bands_amplitudes.clear();
    bands_amplitudes.resize(sz);

    window_size_seconds = p.modulator_window_size_seconds;
    window_center_stride_seconds = p.stride_seconds;
  }
  
  void on_dropped_frame() {
    periodic_fft.reset_samples();
  }
  
  void feed(std::pair<T, SampleContinuity> const & sample) {
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input->sync_feed_frame(&sample.first);
#endif
    
    if (unlikely(sample.second == SampleContinuity::No)) {
      periodic_fft.reset_samples();
    }
    periodic_fft.feed(sample.first);
    
#if IMJ_DEBUG_VOCODER
    for (int i=0; i<sz; ++i) {
      wav_writer_modulator_envelopes[i]->sync_feed_frame(&bands_amplitudes[i]);
    }
#endif
  }
  
  int getStride(int sample_rate) const {
    return good_stride(window_center_stride_seconds,
                       sample_rate);
  }

  std::vector<T> const & getBandsAmplitudes() const {
    return bands_amplitudes;
  }

  void getBandsAmplitudes(std::vector<T> & v) const {
    int s = sz;
    Assert(static_cast<int>(v.capacity()) >= s);
    v.clear();
    for (int i=0; i<s; ++i) {
      v.push_back(bands_amplitudes[i]);
    }
  }
  
  void getBandsFreqs(std::vector<T> & v) const {
    int s = sz;
    Assert(static_cast<int>(v.capacity()) > s);
    v.clear();
    for (int i=0; i<=s; ++i) {
      v.push_back(freqs[i]);
    }
  }

private:
  int sz; // number of bands
  std::vector<T> freqs;
  PeriodicFFT<SqMagFftOperation<Window::Gaussian, double>> periodic_fft{modulator_max_fft_size};
  
  std::vector<T> bands_amplitudes;

  double window_size_seconds;
  double window_center_stride_seconds;

#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_modulator_input;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_modulator_envelopes;
#endif
};

/*
template<typename T>
struct FFT2Modulator {
  
  static double constexpr window_size_seconds = 0.05;
  
  // This dynamically allocates memory
  void setup_bands(int sample_rate,
                   std::vector<T> const & band_freqs) {
    periodic_fft.setLambdas([sample_rate]() { return std::max(1,
                                                              static_cast<int>(0.5f + window_size_seconds * sample_rate)); },
                            [sample_rate]() { return std::max(1,
                                                              static_cast<int>(0.5f + window_size_seconds * sample_rate)); },
                            [](int const window_center_stride,
                               auto const & ) {});
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input = std::make_unique<AsyncWavWriter>(1,
                                                                  sample_rate,
                                                                  "debug_modulator_input");
#endif
  }
  
  void setup(int sample_rate,
             std::vector<T> const & freqs,
             SetupParams const &) {
    // Not applicable
  }
  
  void on_dropped_frame() {
    periodic_fft.reset_samples();
  }
  
  void feed(std::pair<T, SampleContinuity> const & sample,
            std::vector<T> const &) {
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input->sync_feed_frame(&sample.first);
#endif
    
    if (unlikely(sample.second == SampleContinuity::No)) {
      periodic_fft.reset_samples();
    }
    periodic_fft.feed(sample.first);
  }
  
  auto const & getFreqBins() const {
    return periodic_fft.getResults();
  }

private:
  PeriodicFFT<FftOperation<double>> periodic_fft{modulator_max_fft_size};
  
#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_modulator_input;
#endif
};
*/
/*
template<int ORDER, typename T>
struct Modulator {

  // This dynamically allocates memory
  void setup_bands(int sample_rate,
                   std::vector<T> const & freqs) {
    sz = static_cast<int>(freqs.size()) - 1;
    Assert(sz);
    band_pass.clear();
    band_pass.resize(sz);
    env_follower.clear();
    env_follower.resize(sz);
    for (int i=0; i<sz; ++i) {
      band_pass[i].setup(sample_rate,
                         freqs[i],
                         freqs[i+1]);
    }

#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input = std::make_unique<AsyncWavWriter>(1,
                                                                  sample_rate,
                                                                  "debug_modulator_input");
    for (int i=0; i<sz; ++i) {
      wav_writer_modulator_bands.push_back(std::make_unique<AsyncWavWriter>(1,
                                                                            sample_rate,
                                                                            "debug_modulator_band_" + std::to_string(i)));
      wav_writer_modulator_envelopes.push_back(std::make_unique<AsyncWavWriter>(1,
                                                                                sample_rate,
                                                                                "debug_modulator_envelope_" + std::to_string(i)));
    }
#endif
  }

  // This does not allocate memory
  //
  // @param factor : low-pass cutoff = factor * band start freq
  void setup(int sample_rate,
             std::vector<T> const & freqs,
             SetupParams const & setup) {
    Assert(sz == static_cast<int>(freqs.size()) - 1);
    Assert(env_follower.size() == (freqs.size() - 1));
    for (int i=0; i<sz; ++i) {
      env_follower[i].setup(sample_rate,
                            //1. // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2730710/
                            freqs[i] * setup.env_follower_cutoff_ratio
                            );
    }
  }

  void feed(std::pair<T, SampleContinuity> const & sample,
            std::vector<T> & res) {
    Assert(res.size() == band_pass.size());

#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input->sync_feed_frame(&sample.first);
#endif

    for (int i=0; i<sz; ++i) {
      T const filtered = band_pass[i].feed(sample);

#if IMJ_DEBUG_VOCODER
      wav_writer_modulator_bands[i]->sync_feed_frame(&filtered);
#endif

      res[i] = env_follower[i].feed(std::abs(filtered));

#if IMJ_DEBUG_VOCODER
      wav_writer_modulator_envelopes[i]->sync_feed_frame(&res[i]);
#endif
    }
  }

private:
  int sz;
  std::vector<BandPass<ORDER, T>> band_pass;
  std::vector<EnvelopeFollower<1, T>> env_follower;

#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_modulator_input;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_modulator_bands;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_modulator_envelopes;
#endif
};
*/

template<typename T>
struct FFTCarrier {

  using FreqBinsImpl = typename fft::RealFBins_<fft::Fastest, T, a64::Alloc>;
  using FreqBins = typename FreqBinsImpl::type;
  
  FFTCarrier(FFTModulator<T> & mod)
  : modulator(mod)
  {
    tmp_freq_bins.resize(carrier_max_fft_size);
    signal_old.resize(carrier_max_fft_size);
    signal_new.resize(carrier_max_fft_size);
    xfade.reserve(max_stride_size * 2);
    
    freqs.reserve(max_count_bands + 1);
  }
  
  void init_dynamic_allocs(int sample_rate) {
    periodic_fft.setLambdas([this, sample_rate]() {
      return 2 * modulator.getStride(sample_rate);
    },
                            [this, sample_rate]() {
      return modulator.getStride(sample_rate);
    },
                            [this, sample_rate](int const window_center_stride,
                                                FreqBins const & freq_bins) {
      Assert(modulator.getStride(sample_rate) == window_center_stride);
      
      sz_half_signal = window_center_stride;
      
      Assert(static_cast<int>(signal_new.size() / 2) >= sz_half_signal);
      Assert(static_cast<int>(signal_old.size() / 2) >= sz_half_signal);
      
      // TODO this could be very costly if stride is big. Instead, for very big strides we should limit the crossfade duration.
      xfade.set(sz_half_signal - 1,
                EqualGainCrossFade::Sinusoidal);
      
      int const fft_length = FreqBinsImpl::get_fft_length(freq_bins);
      Assert(fft_length <= static_cast<int>(carrier_max_fft_size));
      
      signal_old.swap(signal_new);
      
      using Algo = fft::Algo_<fft::Fastest, T>;
      using Contexts = fft::Contexts_<fft::Fastest, T>;
      
      Algo fft(Contexts::getInstance().getBySize(fft_length));
      
      T const scale_factor = 1/(Algo::scale * static_cast<T>(fft_length));
      
      new_signal_idx = 0;
      
      Assert(freq_bins.size() <= tmp_freq_bins.capacity());
      tmp_freq_bins.resize(freq_bins.size());
      
      FreqBinsImpl::copy_same_size(freq_bins,
                                   tmp_freq_bins);
      
      FreqBinsImpl::modulate_bands(sample_rate,
                                   modulator.getBandsAmplitudes(),
                                   freqs,
                                   tmp_freq_bins);
      if constexpr (Algo::inplace_dft) {
        fft.inverse(tmp_freq_bins.data(),
                    fft_length);
        for(int i=0; i<fft_length; ++i) {
          signal_new[i] = Algo::extractRealOutput(tmp_freq_bins.data(),
                                                  i,
                                                  fft_length);
        }
      }
      else {
        fft.inverse(tmp_freq_bins.data(),
                    signal_new.data(),
                    fft_length);
      }
      
      for(auto & v : signal_new) {
        v *= scale_factor;
      }
    });

#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input = std::make_unique<AsyncWavWriter>(1,
                                                                sample_rate,
                                                                "debug_carrier_input");
    wav_writer_carrier_bands_weighted_sum = std::make_unique<AsyncWavWriter>(1,
                                                                             sample_rate,
                                                                             "debug_carrier_bands_weighted_sum");
    wav_writer_carrier_bands_weighted_sum_new = std::make_unique<AsyncWavWriter>(1,
                                                                                 sample_rate,
                                                                                 "debug_carrier_bands_weighted_sum_new");
    wav_writer_carrier_bands_weighted_sum_old = std::make_unique<AsyncWavWriter>(1,
                                                                                 sample_rate,
                                                                                 "debug_carrier_bands_weighted_sum_old");
#endif
  }

  void setup(SetupParams const & p) {
    p.fill_freqs(freqs); // in the future, we could have a frequency shift between carrier and modulator
  }

  void on_dropped_frame() {
    // TODO : ? (will not happen in today's use cases, though)
  }
  
  T feed(std::pair<T, SampleContinuity> const & sample) {
    // TODO : handle the case where sample.first == SampledDiscontinuity::Yes
    // (will not happen in real life use cases today, though)
    
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input->sync_feed_frame(&sample.first);
#endif
    
    periodic_fft.feed(sample.first);

    Assert(signal_new.size() == signal_old.size());
    Assert(new_signal_idx < static_cast<int>((signal_new.size() / 2)));

    // cross-fade from the old signal with old bands amplitudes to the new signal with new bands amplitudes
    XFadeValues<T> xf = xfade.get(new_signal_idx + 1);

    T const res =
    signal_new[new_signal_idx] * xf.new_signal_mult +
    signal_old[new_signal_idx + sz_half_signal] * xf.old_signal_mult;

#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_bands_weighted_sum->sync_feed_frame(&res);
    wav_writer_carrier_bands_weighted_sum_new->sync_feed_frame(&signal_new[new_signal_idx]);
    wav_writer_carrier_bands_weighted_sum_old->sync_feed_frame(&signal_old[new_signal_idx + sz_half_signal]);
#endif

    ++new_signal_idx;
    
    return res;
  }
private:
  int sz_half_signal;
  int new_signal_idx = 0; // old_signal_idx = new_signal_idx + sz_half_signal
  EqualGainXFade<T> xfade;
  FFTModulator<T> & modulator;
  std::vector<T> freqs;
  FreqBins tmp_freq_bins;
  // The 2nd half of "old" and the 1st half of "new" are crossfaded:
  //   during its 2nd half, "old" contribution goes linearily from 1 to 0
  //   during its 1st half, "new" contribution goes linearily from 0 to 1
  // Then, old and new are swapped and a new "new" signal is computed.
  a64::vector<T> signal_old;
  a64::vector<T> signal_new;
  PeriodicFFT<FftOperation<double>> periodic_fft{carrier_max_fft_size};
#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_input;
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_bands_weighted_sum;
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_bands_weighted_sum_new;
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_bands_weighted_sum_old;
#endif
};

/*
// TODO : apply the xfade techniques seen in FFTCarrier, and compare the results with FFTCarrier
template<typename T>
struct FFT2Carrier {
  FFT2Carrier(FFT2Modulator<T> & mod)
  : modulator(mod)
  , signal_length(carrier_max_fft_size)
  {
    tmp_freq_bins.resize(carrier_max_fft_size);
    signal.resize(carrier_max_fft_size);
  }
  
  static constexpr double window_size_seconds = FFT2Modulator<T>::window_size_seconds;
  
  using FreqBinsImpl = typename fft::RealFBins_<fft::Fastest, T, a64::Alloc>;
  using FreqBins = typename FreqBinsImpl::type;
  
  void setup(int sample_rate,
             std::vector<T> const & band_freqs) {
    periodic_fft.setLambdas([sample_rate]() { return std::max(1,
                                                              static_cast<int>(0.5f + window_size_seconds * sample_rate)); },
                            [sample_rate]() { return std::max(1,
                                                              static_cast<int>(0.5f + window_size_seconds * sample_rate)); },
                            [this, sample_rate](int const window_center_stride,
                                                FreqBins const & freq_bins) {
      using Algo = fft::Algo_<fft::Fastest, T>;
      using Contexts = fft::Contexts_<fft::Fastest, T>;
      
      int const fft_length = FreqBinsImpl::get_fft_length(freq_bins);
      Assert(fft_length <= carrier_max_fft_size);
      
      Algo fft(Contexts::getInstance().getBySize(fft_length));
      
      T const scale_factor = 1/(Algo::scale * static_cast<T>(fft_length));
      
      signal_idx = 0;
      signal_length = fft_length;
      
      Assert(freq_bins.size() <= tmp_freq_bins.capacity());
      tmp_freq_bins.resize(freq_bins.size());
      
      FreqBinsImpl::multiply(tmp_freq_bins.data(),
                             freq_bins.data(),
                             modulator.getFreqBins().data(),
                             fft_length/2);
      if constexpr (Algo::inplace_dft) {
        fft.inverse(tmp_freq_bins.data(),
                    fft_length);
        for(int i=0; i<fft_length; ++i) {
          signal[i] = Algo::extractRealOutput(tmp_freq_bins.data(),
                                              i,
                                              fft_length);
        }
      } else {
        fft.inverse(tmp_freq_bins.data(),
                    signal.data(),
                    fft_length);
      }
      
      for(auto & v : signal) {
        v *= scale_factor;
      }
    });
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input = std::make_unique<AsyncWavWriter>(1,
                                                                sample_rate,
                                                                "debug_carrier_input");
    wav_writer_carrier_bands_weighted_sum = std::make_unique<AsyncWavWriter>(1,
                                                                             sample_rate,
                                                                             "debug_carrier_bands_weighted_sum");
#endif
  }
  
  void on_dropped_frame() {
    // TODO : ? (will not happen in today's use cases, though)
  }
  
  T feed(std::pair<T, SampleContinuity> const & sample,
         std::vector<T> const &) {
    // TODO : handle the case where sample.first == SampledDiscontinuity::Yes
    // (will not happen in real life use cases today, though)
    
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input->sync_feed_frame(&sample.first);
#endif
    
    periodic_fft.feed(sample.first);
    
    Assert(signal_idx < signal_length);
    
    T const res = signal[signal_idx];
    
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_bands_weighted_sum->sync_feed_frame(&res);
#endif
    
    ++signal_idx;
    
    return res;
  }
private:
  int signal_idx = 0;
  int signal_length = carrier_max_fft_size;
  FreqBins tmp_freq_bins;
  FFT2Modulator<T> & modulator;
  a64::vector<T> signal;
  PeriodicFFT<FftOperation<double>> periodic_fft{carrier_max_fft_size};
#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_input;
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_bands_weighted_sum;
#endif
};
*/

/*
template<int ORDER, typename T>
struct Carrier {
  void setup(int sample_rate,
             std::vector<T> const & freqs) {
    sz = static_cast<int>(freqs.size()) - 1;
    Assert(sz);
    band_pass.clear();
    band_pass.resize(sz);
    for (int i=0; i<sz; ++i) {
      band_pass[i].setup(sample_rate,
                         freqs[i],
                         freqs[i+1]);
    }
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input = std::make_unique<AsyncWavWriter>(1,
                                                                sample_rate,
                                                                "debug_carrier_input");
    wav_writer_carrier_bands_weighted_sum = std::make_unique<AsyncWavWriter>(1,
                                                                             sample_rate,
                                                                             "debug_carrier_bands_weighted_sum");
    
    for (int i=0; i<sz; ++i) {
      wav_writer_carrier_bands.push_back(std::make_unique<AsyncWavWriter>(1,
                                                                          sample_rate,
                                                                          "debug_carrier_band_" + std::to_string(i)));
    }
#endif
  }

  void on_dropped_frame() {
    // TODO : ? (will not happen in today's use cases, though)
  }

  T feed(std::pair<T, SampleContinuity> const & sample,
         std::vector<T> const & ponderation) {
    // TODO : handle the case where sample.first == SampledDiscontinuity::Yes
    // (will not happen in real life use cases today, though)
    
    Assert(ponderation.size() == band_pass.size());
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input->sync_feed_frame(&sample.first);
#endif
    T res{};
    for (int i=0; i<sz; ++i) {
      T const filtered = band_pass[i].feed(sample.first);
#if IMJ_DEBUG_VOCODER
      wav_writer_carrier_bands[i]->sync_feed_frame(&filtered);
#endif
      res += ponderation[i] * filtered;
    }
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_bands_weighted_sum->sync_feed_frame(&res);
#endif
    return res;
  }
private:
  int sz;
  std::vector<BandPass<ORDER, T>> band_pass;
#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_input;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_carrier_bands;
  std::unique_ptr<AsyncWavWriter> wav_writer_carrier_bands_weighted_sum;
#endif
};
*/

struct Vocoder {
  // we need at least order 4 to have a good separation between bands, but then the bands are not flat anymore, and the output is very low
  // so we use ffts instead
  static constexpr int order_filters_carrier = 1;
  static constexpr int order_filters_modulator = 1;

  struct Volumes {
    float modulator;
    float carrier;
    float vocoder;
  };
  struct Params {
    Volumes vol;
    SetupParams setup;
  };

  template<typename Out, typename P, typename S>
  void initialize(Out & ctxt,
                  P && get_params,
                  S && get_modulator_carrier_sample) {
    sample_rate = ctxt.getSampleRate();
    last_setup = get_params().setup;
    setup();

    Assert(state == SynthState::ComputeNotRegistered);
    ctxt.getStepper().enqueueOneShot([this,
                                      get_params,
                                      get_modulator_carrier_sample](auto & out, auto){
      if (!out.registerSimpleCompute([this,
                                      get_params,
                                      get_modulator_carrier_sample
                                      ](double * buf, int frames) mutable {
        if (state == SynthState::WaitingForComputeUnregistration) {
          state = SynthState::ComputeNotRegistered;
          return false;
        }

        Params const params = get_params();
        if (!last_setup || *last_setup != params.setup) {
          last_setup = params.setup;
          modulator.setup(params.setup);
          carrier.setup(params.setup);
        }
      
        Volumes const & volume = params.vol;
        
        for (int i=0; i<frames; ++i) {
          std::pair<
          std::optional<std::pair<double, SampleContinuity>>,
          std::optional<std::pair<double, SampleContinuity>>
          > const s = get_modulator_carrier_sample();
          
          auto const & res_modulator = s.first;
          auto const & res_carrier = s.second;
          if (unlikely(!res_modulator)) {
            // happens during initialization phase (if we use a queue)
            modulator.on_dropped_frame();
          } else {
            modulator.feed(*res_modulator);
          }
          if (unlikely(!res_carrier)) {
            // happens during initialization phase (if we use a queue)
            carrier.on_dropped_frame();
          } else {
            double const vocoded = carrier.feed(*res_carrier);
            if (res_modulator) {
              double const mixed =
              volume.modulator * res_modulator->first +
              volume.carrier * res_carrier->first +
              volume.vocoder * vocoded;
              for (int j=0; j<Out::nAudioOut; ++j) {
                buf[Out::nAudioOut * i + j] += mixed;
              }
            }
          }
        }
        return true;
      })) {
        throw std::runtime_error("failed to register compute");
      }
    });
    state = SynthState::ComputeRegistered;
  }
  
  template<typename Out>
  void finalize(Out & out) {
    Assert(state == SynthState::ComputeRegistered);
    
    state = SynthState::WaitingForComputeUnregistration;
    // block until the registered compute function returned false (to be removed from the context queue)
    while(state == SynthState::WaitingForComputeUnregistration) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
  
  void fetch_last_data(std::vector<double> & envelopes,
                       std::vector<double> & band_freqs) const {
    modulator.getBandsAmplitudes(envelopes);
    modulator.getBandsFreqs(band_freqs);
  }

private:
  std::atomic<SynthState> state = SynthState::ComputeNotRegistered;
  
  //Modulator<order_filters_modulator, double> modulator;
  FFTModulator<double> modulator;
  //FFT2Modulator<double> modulator;
  //Carrier<order_filters_carrier, double> carrier;
  FFTCarrier<double> carrier{modulator};
  //FFT2Carrier<double> carrier{modulator};
  
  std::optional<int> sample_rate;
  std::optional<SetupParams> last_setup;
  
  void setup() {
    modulator.init_dynamic_allocs(*sample_rate);
    carrier.init_dynamic_allocs(*sample_rate);

    modulator.setup(*last_setup);
    carrier.setup(*last_setup);
  }
};

} // NS
