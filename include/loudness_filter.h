namespace imajuscule {
    namespace loudness {

        template<typename T>
        std::vector<T> getLoudnessCompensationFIRCoefficients(unsigned int fft_length, unsigned int NumTaps) {
            return fir_coefficients_by_f_sampling<T>(get_nyquist_frequency<T>(),
                                                     [](auto v) { return equal_loudness_volume(v);},
                                                     fft_length,
                                                     NumTaps);
        }

    } // NS loudness
} // NS imajuscule
