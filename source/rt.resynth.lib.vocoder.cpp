#define IMJ_DEBUG_VOCODER 1

namespace imajuscule::audio::rtresynth {

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

constexpr std::size_t modulator_max_fft_size = pow2(13);
constexpr std::size_t carrier_max_fft_size = pow2(13);

template<typename T>
struct FFTModulator {
  
  static double constexpr window_size_seconds = 0.05;
  static double constexpr window_center_stride_seconds = 0.005;

  // This dynamically allocates memory
  void setup_bands(int sample_rate,
                   std::vector<T> const & band_freqs) {
    sz = static_cast<int>(band_freqs.size()) - 1;
    Assert(sz > 0);

    freqs = band_freqs;
    
    new_bands_amplitudes.clear();
    new_bands_amplitudes.resize(sz);
    old_bands_amplitudes.clear();
    old_bands_amplitudes.resize(sz);
    
    // TODO use hanning window (no need for "very fine" frequency peak measurement here)
    
    // TODO use different sizes of ftts to have a better latency on higher freqs or overlapp the ffts (more computationaly intensive, though?)

    periodic_fft.setLambdas([sample_rate]() { return sample_rate * window_size_seconds; },
                            [sample_rate]() { return good_stride(window_center_stride_seconds,
                                                                 sample_rate); },
                            [this, sample_rate](int const window_center_stride,
                                   FrequenciesSqMag<double> const & frequencies_sqmag) {
      itp_length = window_center_stride + 1;
      itp_index = 1;
      
      old_bands_amplitudes.swap(new_bands_amplitudes);
      
      std::fill(new_bands_amplitudes.begin(),
                new_bands_amplitudes.end(),
                static_cast<T>(0));

      Assert(frequencies_sqmag.get_fft_length() <= modulator_max_fft_size);
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
        new_bands_amplitudes[band_idx] += frequencies_sqmag.frequencies_sqmag[i];
        //new_bands_amplitudes[band_idx] = std::max(new_bands_amplitudes[band_idx],
          //                                        frequencies_sqmag.frequencies_sqmag[i]);
      }
      for (auto & a : new_bands_amplitudes) {
        a = std::sqrt(a);
      }
    }
                            );
    
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input = std::make_unique<AsyncWavWriter>(1,
                                                                  sample_rate,
                                                                  "debug_modulator_input");
    for (int i=0; i<sz; ++i) {
      wav_writer_modulator_envelopes.push_back(std::make_unique<AsyncWavWriter>(1,
                                                                                sample_rate,
                                                                                "debug_modulator_envelope_" + std::to_string(i)));
    }
#endif
  }
  
  void setup_env_followers(int sample_rate,
                           std::vector<T> const & freqs,
                           T const factor) {
    // Not applicable
  }
  
  void on_dropped_frame() {
    periodic_fft.reset_samples();
  }
  
  void feed(std::pair<T, SampleContinuity> const & sample,
            std::vector<T> & res) {
#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input->sync_feed_frame(&sample.first);
#endif
    
    if (unlikely(sample.second == SampleContinuity::No)) {
      periodic_fft.reset_samples();
    }
    periodic_fft.feed(sample.first);
    
    // interpolate linearily from the old to the new coefficients
    float const progress = static_cast<float>(itp_index) / itp_length;
    float const one_minus_progress = 1.f - progress;
    
    Assert(static_cast<int>(res.size()) == sz);
    for (int i=0; i<sz; ++i) {
      res[i] = progress * new_bands_amplitudes[i] + one_minus_progress * old_bands_amplitudes[i];
#if IMJ_DEBUG_VOCODER
      wav_writer_modulator_envelopes[i]->sync_feed_frame(&res[i]);
#endif
    }

    if (itp_index < itp_length) {
      ++ itp_index;
    }
  }
  
  std::vector<T> const & getNewBandsAmplitudes() const {
    return new_bands_amplitudes;
  }
  std::vector<T> const & getOldBandsAmplitudes() const {
    return old_bands_amplitudes;
  }

private:
  int sz; // number of bands
  int itp_length = 2; // includes start and end values
  int itp_index = 1;
  std::vector<T> freqs;
  PeriodicFFT<SqMagFftOperation<Window::Gaussian, double>> periodic_fft{modulator_max_fft_size};
  
  std::vector<T> old_bands_amplitudes;
  std::vector<T> new_bands_amplitudes;

#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_modulator_input;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_modulator_envelopes;
#endif
};

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
  
  void setup_env_followers(int sample_rate,
                           std::vector<T> const & freqs,
                           T const factor) {
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
  void setup_env_followers(int sample_rate,
                           std::vector<T> const & freqs,
                           T const factor) {
    Assert(sz == static_cast<int>(freqs.size()) - 1);
    Assert(env_follower.size() == (freqs.size() - 1));
    for (int i=0; i<sz; ++i) {
      env_follower[i].setup(sample_rate,
                            //1. // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2730710/
                            freqs[i] * factor
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

template<typename T>
struct FFTCarrier {
  FFTCarrier(FFTModulator<T> & mod)
  : modulator(mod)
  {
    tmp_freq_bins.resize(carrier_max_fft_size);
    signal_old.resize(carrier_max_fft_size);
    signal_new.resize(carrier_max_fft_size);
  }
  
  using FreqBinsImpl = typename fft::RealFBins_<fft::Fastest, T, a64::Alloc>;
  using FreqBins = typename FreqBinsImpl::type;
  
  void setup(int sample_rate,
             std::vector<T> const & band_freqs) {
    freqs = band_freqs;

    sz_half_signal = good_stride(FFTModulator<T>::window_center_stride_seconds,
                                 sample_rate);
    
    xfade.set(sz_half_signal - 1,
              EqualGainCrossFade::Sinusoidal);

    periodic_fft.setLambdas([this]() { return 2 * sz_half_signal; },
                            [this]() { return sz_half_signal; },
                            [this, sample_rate](int const window_center_stride,
                                                FreqBins const & freq_bins) {
      Assert(window_center_stride == sz_half_signal);

      int const fft_length = FreqBinsImpl::get_fft_length(freq_bins);
      Assert(fft_length <= carrier_max_fft_size);
      Assert(fft_length == ceil_power_of_two(2*sz_half_signal));

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
                                   modulator.getNewBandsAmplitudes(),
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

    Assert(signal_new.size() == signal_old.size());
    Assert(new_signal_idx < (signal_new.size() / 2));

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


struct Vocoder {
  static constexpr int count_bands = 5;
  
  static constexpr float min_freq = 100.f;
  static constexpr float max_freq = 20000.f;
  
  // we need at least order 4 to have a good separation between bands, but then the bands are not flat anymore, and the output is very low
  static constexpr int order_filters_carrier = 1;
  static constexpr int order_filters_modulator = 1;

  struct Volumes {
    float modulator;
    float carrier;
    float vocoder;
  };
  
  struct Params {
    Volumes vol;
    float env_follower_cutoff_ratio;
  };
  
  Vocoder()
  : amplitudes(count_bands) {
  }

  template<typename Out, typename P, typename S>
  void initialize(Out & ctxt,
                  P && get_params,
                  S && get_modulator_carrier_sample) {
    sample_rate = ctxt.getSampleRate();
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
        if (!last_env_follower_cutoff_ratio || *last_env_follower_cutoff_ratio != params.env_follower_cutoff_ratio) {
          last_env_follower_cutoff_ratio = params.env_follower_cutoff_ratio;
          modulator.setup_env_followers(*sample_rate,
                                        freqs,
                                        params.env_follower_cutoff_ratio);
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
            modulator.feed(*res_modulator,
                           amplitudes);
          }
          if (unlikely(!res_carrier)) {
            // happens during initialization phase (if we use a queue)
            carrier.on_dropped_frame();
          } else {
            double const vocoded = carrier.feed(*res_carrier,
                                                amplitudes);
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
  
  void fetch_last_data(std::vector<double> & envelopes, std::vector<double> & band_freqs) const {
    std::lock_guard l(mutex);

    envelopes = amplitudes;
    band_freqs = freqs;
  }

private:
  std::atomic<SynthState> state = SynthState::ComputeNotRegistered;
  
  //Modulator<order_filters_modulator, double> modulator;
  FFTModulator<double> modulator;
  //FFT2Modulator<double> modulator;
  std::vector<double> amplitudes;
  //Carrier<order_filters_carrier, double> carrier;
  FFTCarrier<double> carrier{modulator};
  //FFT2Carrier<double> carrier{modulator};
  std::vector<double> freqs;
  
  std::optional<int> sample_rate;
  std::optional<float> last_env_follower_cutoff_ratio;
  
  mutable std::mutex mutex; // protects amplitudes and freq allocation

  void setup() {
    std::lock_guard l(mutex);
    
    freqs.clear();
    freqs.reserve(count_bands + 1);
    
    auto log_min = std::log(min_freq);
    auto log_max = std::log(max_freq);
    
    for (int i=0; i<=count_bands; ++i) {
      double const ratio = static_cast<double>(i) / count_bands;
      freqs.push_back(std::exp(log_min + ratio * (log_max-log_min)));
    }

    modulator.setup_bands(*sample_rate,
                          freqs);
    carrier.setup(*sample_rate,
                  freqs);
  }
};

} // NS
