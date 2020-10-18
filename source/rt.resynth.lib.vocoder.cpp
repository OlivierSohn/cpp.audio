#define IMJ_DEBUG_VOCODER 1

namespace imajuscule::audio {

template<typename Q>
void drain_queue_until_size_smaller(Q & q,
                                    int const target_size) {
  while(true) {
    int const drop = static_cast<int>(q.was_size()) - target_size;
    if (drop <= 0) {
      return;
    }
    for (int i=0; i<drop; ++i) {
      typename Q::value_type var;
      q.try_pop(var);
    }
  }
}

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
    lp_.initWithFreq(sample_rate, freq); // TODO tune. The idea is that we don't want sinusoidal variations to be audible
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
  void setup(int sample_rate,
             std::vector<T> const & freqs) {
    sz = static_cast<int>(freqs.size()) - 1;
    Assert(sz);
    band_pass.resize(sz);
    env_follower.resize(sz);
    for (int i=0; i<sz; ++i) {
      band_pass[i].setup(sample_rate,
                         freqs[i],
                         freqs[i+1]);
      env_follower[i].setup(sample_rate,
                            //1. // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2730710/
                            freqs[i] / 20.
                            );
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
  static constexpr int count_bands = 40;
  
  static constexpr float min_freq = 100.f;
  static constexpr float max_freq = 5000.f;
  static constexpr int order = 2;
  
  Vocoder()
  : amplitudes(count_bands) {
  }

  template<typename Out, typename MetaQ, typename C>
  void initialize(MetaQ & q_modulator,
                  C && get_carrier_sample,
                  Out & ctxt,
                  std::atomic<float> & voice_volume,
                  std::atomic<float> & vocoded_volume) {
    setup(ctxt.getSampleRate());

    Assert(state == SynthState::ComputeNotRegistered);
    ctxt.getStepper().enqueueOneShot([this,
                                      &ctxt,
                                      &voice_volume,
                                      &vocoded_volume,
                                      &q_modulator,
                                      get_carrier_sample](auto & out, auto){
      if (!out.registerSimpleCompute([this,
                                      &ctxt,
                                      &voice_volume,
                                      &vocoded_volume,
                                      &q_modulator,
                                      get_carrier_sample
                                      ](double * buf, int frames){
        if (state == SynthState::WaitingForComputeUnregistration) {
          state = SynthState::ComputeNotRegistered;
          initialization = true;
          return false;
        }
        auto min_size_queue = [&q_modulator]{
          return q_modulator.queue.capacity() / 2;
        };
        // we synchronize here so as to:
        // - minimize audio latency, and
        // - avoid queue starvation (which would lead to audio cracks)
        //
        // To minimize audio latency, we should start consuming elements as soon as possible.
        // But to avoid starvation, we should start consuming elements only once the queue has a "big enough" size.
        // Taking into account jitter in both input and output, waiting for the queue to be of size
        //   '2 * max(delayIn, delayOut) * sample_rate' seems reasonnable.
        //
        // Note that queue capacity is
        //   '4 * max(delayIn, delayOut) * sample_rate'
        if (initialization) {
          if (q_modulator.queue.was_size() < min_size_queue()) {
            return true;
          }
          drain_queue_until_size_smaller(q_modulator.queue,
                                         min_size_queue());
          initialization = false;
        }
        //std::cout << "vocoder queue producer : size=" << q.queue.was_size() << std::endl;
        float const the_voice_volume = voice_volume;
        float const the_vocoded_volume = vocoded_volume;
        
        for (int i=0; i<frames; ++i) {
          typename MetaQ::QueueItem var_modulator;
          while(true) {
            if (unlikely(!q_modulator.queue.try_pop(var_modulator))) {
              // should never happen
              throw std::runtime_error("vocoder : the queue is empty");
            }
            if (unlikely(std::holds_alternative<CountDroppedFrames>(var_modulator))) {
              // happens when the cpu load of the callback is big, so some frames are dropped
#ifndef NDEBUG
              ctxt.asyncLogger().sync_feed(AudioQueueDroppedFrames{
                "vocoder",
                std::get<CountDroppedFrames>(var_modulator).count
              });
#endif
              // some frames have been dropped, so the queue is probably quite full now, we will
              // re-establish balance by dropping some items:
              drain_queue_until_size_smaller(q_modulator.queue, min_size_queue());
              continue;
            }
            break;
          }
          Assert(std::holds_alternative<InputSample>(var_modulator));
          auto const sample_modulator = std::get<InputSample>(var_modulator).value;
          modulator.feed(sample_modulator,
                         amplitudes);
          
          double const vocoded = carrier.feed(get_carrier_sample(), amplitudes);
          double const mixed = the_voice_volume * sample_modulator + the_vocoded_volume * vocoded;
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
  bool initialization = true;
  Modulator<order, double> modulator;
  std::vector<double> amplitudes;
  Carrier<order, double> carrier;

  void setup(int sample_rate) {
    std::vector<double> freqs;
    freqs.reserve(count_bands + 1);
    
    auto log_min = std::log(min_freq);
    auto log_max = std::log(max_freq);
    
    for (int i=0; i<=count_bands; ++i) {
      double const ratio = static_cast<double>(i) / count_bands;
      freqs.push_back(std::exp(log_min + ratio * (log_max-log_min)));
    }

    std::cout << "Vocoder freqs :";
    for (auto const & f : freqs) {
      std::cout << f << " ";
    }
    std::cout << std::endl;
    modulator.setup(sample_rate,
                    freqs);
    carrier.setup(sample_rate,
                  freqs);
  }
};

} // NS
