#define IMJ_DEBUG_VOCODER 1

namespace imajuscule::audio {

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

  void feed(T sample,
            std::vector<T> & res) {
    Assert(res.size() == band_pass.size());

#if IMJ_DEBUG_VOCODER
    wav_writer_modulator_input->sync_feed_frame(&sample);
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

  T feed(T sample,
         std::vector<T> const & ponderation) {
    Assert(ponderation.size() == band_pass.size());
#if IMJ_DEBUG_VOCODER
    wav_writer_carrier_input->sync_feed_frame(&sample);
#endif
    T res{};
    for (int i=0; i<sz; ++i) {
      T const filtered = band_pass[i].feed(sample);
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
  static constexpr int count_bands = 20;
  
  static constexpr float min_freq = 100.f;
  static constexpr float max_freq = 10000.f;
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
          std::optional<std::pair<double, double>> const s = get_modulator_carrier_sample();
          if (!s) {
            // happens during initialization phase (if we use a queue)
            Assert(i == 0);
            return true;
          }
          auto [modulator_sample, carrier_sample] = *s;
          
          modulator.feed(modulator_sample,
                         amplitudes);
          
          double const vocoded = carrier.feed(carrier_sample,
                                              amplitudes);
          double const mixed =
          volume.modulator * modulator_sample +
          volume.carrier * carrier_sample +
          volume.vocoder * vocoded;
          for (int j=0; j<Out::nAudioOut; ++j) {
            buf[Out::nAudioOut * i + j] += mixed;
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
private:
  std::atomic<SynthState> state = SynthState::ComputeNotRegistered;
  
  Modulator<order_filters_modulator, double> modulator;
  std::vector<double> amplitudes;
  Carrier<order_filters_carrier, double> carrier;
  std::vector<double> freqs;
  
  std::optional<int> sample_rate;
  std::optional<float> last_env_follower_cutoff_ratio;

  void setup() {
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
