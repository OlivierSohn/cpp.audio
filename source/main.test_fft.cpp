

namespace imajuscule {
    template<typename CONTAINER>
    void remove_dc(CONTAINER & v) {
        Assert(!v.empty());
        typename CONTAINER::value_type avg = 0.f;
        for(auto e : v) {
            avg += e;
        }
        avg /= v.size();

        for(auto & e : v) {
            e -= avg;
        }
    }
    
    void testFFT()
    {
        using namespace imajuscule::fft;
        using namespace imajuscule::audio;
        
        constexpr auto length_fft = 4096;
        std::vector<float> signal_dc_removed;
        signal_dc_removed.resize(length_fft);
        
        constexpr auto n_samples = pow2(16);

        a64::vector<complex<double>> signal;
        signal.reserve(n_samples);

        std::vector<double> real_freq;

        for(auto i=1; i<=16; ++i) {
            auto const num_taps = pow2(i);
            
            a64::vector<double> real_signal;
            a64::vector<complex<double>> signal, frequencies;
            signal.reserve(n_samples);
            
            {
              auto noise = make_loudness_adapted_noise(getWhiteNoise, num_taps, num_taps);
                for(auto s=0; s<n_samples; ++s) {
                    noise.step();
                    real_signal.push_back(noise.get());
                }
            }
            
            write_wav({}, "signal_" + std::to_string(num_taps) + ".wav", real_signal, NChannels::ONE);
            
            frequencies.resize(length_fft);

            real_freq.resize(length_fft);
            std::fill(real_freq.begin(), real_freq.end(), float{});

            int n_superpositions = 0;
            
            auto it = real_signal.begin();
            
            while(1) {
                {
                    auto end = it + length_fft;
                    if(end > real_signal.end()) {
                        break;
                    }
                    std::copy(it, end, signal_dc_removed.begin());
                }
                
                remove_dc(signal_dc_removed);

                signal.clear();
                
                std::transform(signal_dc_removed.begin(),
                               signal_dc_removed.end(),
                               std::back_inserter(signal),
                               [] (auto value) { return complex<double>{value, 0.};});

                forward_fft(length_fft, signal, frequencies);
                apply_hann_window(frequencies.begin(), frequencies.end());

                std::transform(frequencies.begin(), frequencies.end(),
                               real_freq.begin(), // prev
                               real_freq.begin(),
                               [](auto v, auto prev) { return prev + (v * conj(v)).real(); });
                it += 10;
                ++ n_superpositions;
            }
            
            auto bin_freq_width = SAMPLE_RATE / static_cast<double>(length_fft);
            
            {
                real_freq.resize(real_freq.size()/2); // remove second half, which is symmetric to the first half.
                real_freq.erase(real_freq.begin());
                
                auto plot = StringPlot(66, real_freq.size());
                plot.drawLog(real_freq, default_curve_char, true);
                
                std::string fname = "spectral_density_" + std::to_string(num_taps) + ".txt";
                
                ScopedFileWrite f(fname);

                f << "n_superpositions = " << n_superpositions << std::endl << std::endl;
                f << "length_fft = " << length_fft << std::endl << std::endl;
                f << "num_taps = " << num_taps << std::endl << std::endl;
                f << "bin_freq_width = " << bin_freq_width << " Hz" << std::endl << std::endl;
                f << "The first is not displayed" << std::endl << std::endl;
                f << plot;
            }
        }
    }
}

int main(int argc, const char * argv[]) {
    using namespace imajuscule;
    testFFT();
    return 0;
}
