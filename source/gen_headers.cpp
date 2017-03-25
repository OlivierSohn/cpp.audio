
#include <iostream>

#include "public.h"
#include "../../os.storage/include/public.h"

namespace imajuscule {
    template<typename VEC>
    void add(float freq, VEC & normalized_frequencies, VEC & volumes) {
        using namespace imajuscule;
        using namespace imajuscule::loudness;
        
        auto vol = equal_loudness_volume(freq);
        auto normalized_f = freq / get_nyquist_frequency();
        normalized_frequencies.push_back(normalized_f);
        volumes.push_back(vol);
    }
    
    template<typename FILE>
    void write_header(FILE & file, std::string const & str) {
        file << "fprintf(fid, " << str << ");" << std::endl;
    }
    
    template<typename FILE>
    void check_warnings(FILE & file) {
        file << "[warn_msg, warn_id] = lastwarn();" << std::endl;
        file << "if !isempty(warn_msg)" << std::endl;
        write_header(file,
                     "\""
                     "// Octave generated the following warning:\\n\\n"
                     "// %s\\n\\n"
                     "\""
                     ", warn_msg");
        file << "    lastwarn(\"\");" << std::endl;
        file << "endif" << std::endl;
    }
    
    template<typename CONTAINER>
    void write_wav(std::string const & fname, CONTAINER const & samples) {
        using namespace imajuscule::audio;
        using SIGNED = int32_t;
        
        WAVWriter writer(DirectoryPath{}, fname, pcm(samples.size(), SAMPLE_RATE, NChannels::MONO, SignedSample<SIGNED>::format));
        auto res = writer.Initialize();
        if(res != ILE_SUCCESS) {
            throw;
        }
        for(auto sample : samples) {
            auto c = float_to_signed<SIGNED>(sample);
            writer.writeSample(c);
        }
    }
    
    void testFFT()
    {
        using namespace imajuscule::fft;
        
        constexpr auto length_fft = 4096;
        for(auto i=1; i<=16; ++i) {
            auto const num_taps = pow2(i);
            
            constexpr auto n_samples = pow2(16);
            std::vector<double> real_signal;
            std::vector<complex<double>> signal, frequencies;
            signal.reserve(n_samples);
            
            {
                auto noise = make_loudness_adapted_noise(getWhiteNoise, num_taps, num_taps);
                for(auto s=0; s<n_samples; ++s) {
                    noise.step();
                    real_signal.push_back(noise.get());
                }
            }
            
            signal.reserve(n_samples);
            
            std::transform(real_signal.begin(),
                           real_signal.end(),
                           std::back_inserter(signal),
                           [] (auto value) { return complex<double>{value, 0.};});
            
            write_wav("signal_" + std::to_string(num_taps) + ".wav", real_signal);
            
            frequencies.resize(length_fft);
            compute_fft(length_fft, signal.begin(), frequencies.begin());
            std::vector<float> real_freq;
            real_freq.reserve(length_fft);
            std::transform(frequencies.begin(), frequencies.end(),
                           std::back_inserter(real_freq),
                           [](auto v) { return (v * conj(v)).real(); });
            
            {
                auto plot = StringPlot(30, length_fft);
                plot.draw(real_freq);
                
                std::string fname = "spectral_density_" + std::to_string(num_taps) + ".txt";
                
                ScopedFileWrite f(fname);
                
                f << "length_fft = " << length_fft << std::endl << std::endl;
                f << "num_taps = " << num_taps << std::endl << std::endl;
                f << plot;
            }
        }
    }
}

void generateScript() {
    using namespace imajuscule;

    LG(INFO, "File will be written in shared/.../Debug/<thefile>");
    
    constexpr auto script_name = "make_loudness_filter_coefficients.m";
    ScopedFileWrite file(script_name);
    
    file << "# Generated, do not edit! The source is " << __FILE__ << std::endl << std::endl;
    
    file << "pkg load signal" << std::endl << std::endl;
    
    std::vector<float> volumes, normalized_frequencies;
    using namespace imajuscule::loudness;
    
    add(0.f, normalized_frequencies, volumes);
    add(freqs[0], normalized_frequencies, volumes);
    for(int i=0; i<n_freq-1; ++i) {
        add( freqs[i], normalized_frequencies, volumes );
        add( freqs[i+1], normalized_frequencies, volumes );
    }
    add(freqs[n_freq-1], normalized_frequencies, volumes);
    add(get_nyquist_frequency(), normalized_frequencies, volumes);
    
    file.write_vec(volumes, "volumes");
    
    file.write_vec(normalized_frequencies, "norm_frequencies");
    
    
    file << "path = \"" << __FILE__ << "\";"<< std::endl;
    file << "AUDIO = \"/audio/source/\";"<< std::endl;
    file << "positions = strfind(path,AUDIO);" << std::endl;
    file << "pos = positions(end);" << std::endl;
    file << "root = strtrunc(path, pos);" << std::endl;
    file << "RELATIVE = \"/audio/include/loudness_filter_coefficients_gen.h\";" << std::endl;
    file << "filename = [root RELATIVE];" << std::endl;
    file << "fid = fopen (filename, \"w\");" << std::endl;
    
    std::string header =
    std::string("\""
                "\\n// Generated, do not edit! The source is ")
    + __FILE__
    + "\\n\\n"
    + "\"";
    
    write_header(file, header);
    
    write_header(file,
                 "\""
                 "// Loudness level used for reference is : " + std::to_string(LN_default) + " phons\\n\\n"
                 "\"");
    
    check_warnings(file);
    
    file << "for i = [1:50]" << std::endl;
    
    // filter_length - 1 must be even
    file << "    filter_length = 2 * i * i + 1" << std::endl; // no ';' to log to console
    
    file << "    coefficients = firls(filter_length-1, norm_frequencies, volumes);" << std::endl << std::endl;
    
    write_header(file,
                 "\""
                 "constexpr std::array<double,%d> loudness_filter_coeffs_%d = {{\\n"
                 "\""
                 ", numel(coefficients), filter_length");
    file << "    for coeff = coefficients" << std::endl;
    {
        write_header(file,
                     "\""
                     "    %+.43g,\\n"
                     "\""
                     ", coeff");
    }
    file << "    endfor" << std::endl;
    
    write_header(file,
                 "\""
                 "}};\\n\\n"
                 "\"");
    
    check_warnings(file);
    
    file << "endfor" << std::endl;
    
    file << "fclose(fid);" << std::endl;
}

int main(int argc, const char * argv[]) {
    using namespace imajuscule;
    testFFT();
    generateScript();
    return 0;
}
