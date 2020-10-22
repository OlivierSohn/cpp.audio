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

template<typename T>
struct FFTModulator {
  
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
   
    double constexpr window_size_seconds = 0.05;
    double constexpr window_center_stride_seconds = 0.005;
    
    // TODO use hanning window (no need for "very fine" frequency peak measurement here)
    
    // TODO use different sizes of ftts to have a better latency on higher freqs or overlapp the ffts (more computationaly intensive, though?)

    periodic_fft.setLambdas([sample_rate]() { return sample_rate * window_size_seconds; },
                            [sample_rate]() { return std::max(1,
                                                              static_cast<int>(0.5f + window_center_stride_seconds * sample_rate)); },
                            [this, sample_rate](int const window_center_stride,
                                   FrequenciesSqMag<double> const & frequencies_sqmag) {
      itp_length = window_center_stride + 1;
      itp_index = 1;
      
      old_bands_amplitudes.swap(new_bands_amplitudes);
      
      std::fill(new_bands_amplitudes.begin(),
                new_bands_amplitudes.end(),
                static_cast<T>(0));

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
  
private:
  int sz; // number of bands
  int itp_length = 2; // includes start and end values
  int itp_index = 1;
  std::vector<T> freqs;
  PeriodicFFT periodic_fft{pow2(13)};
  
  std::vector<T> old_bands_amplitudes;
  std::vector<T> new_bands_amplitudes;

#if IMJ_DEBUG_VOCODER
  std::unique_ptr<AsyncWavWriter> wav_writer_modulator_input;
  std::vector<std::unique_ptr<AsyncWavWriter>> wav_writer_modulator_envelopes;
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
  std::vector<double> amplitudes;
  Carrier<order_filters_carrier, double> carrier;
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
/*
    std::cout << "Vocoder freqs :";
    for (auto const & f : freqs) {
      std::cout << f << " ";
    }
    std::cout << std::endl;
 */
    modulator.setup_bands(*sample_rate,
                          freqs);
    carrier.setup(*sample_rate,
                  freqs);
  }
};

} // NS
