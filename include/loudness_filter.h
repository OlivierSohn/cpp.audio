namespace imajuscule::audio {
    namespace loudness {

        template<typename T>
        a64::vector<T> getLoudnessCompensationFIRCoefficients(int sample_rate, unsigned int fft_length, unsigned int NumTaps) {
            return fir_coefficients_by_f_sampling<T>(get_nyquist_frequency<T>(sample_rate),
                                                     [](auto v) { return equal_loudness_volume(v);},
                                                     fft_length,
                                                     NumTaps);
        }

    } // NS loudness
} // NS imajuscule
