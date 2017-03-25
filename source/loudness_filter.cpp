namespace imajuscule {
    namespace loudness {

        template<typename T>
        auto make_coefficients_by_least_squares() {
            ScopedLog l("Compute", "filter coeffs using least square method");

            auto vec = to_vector<T>(loudness_filter_coeffs_5001, [](T v){return v;});
         
            T max_ = {};
            for(auto c : vec) {
                max_ = std::max(std::abs(c), max_);
            }
            A(max_);
            std::transform(vec.begin(), vec.end(),
                           vec.begin(), [](auto v){ v/= max_; });
            
            return vec;
        }
        
        std::vector<double> const & getLoudnessCompensationFIRCoefficients() {
            static auto coefficients = fir_coefficients_by_f_sampling<double>(get_nyquist_frequency(),
                                                                              [](auto v){ return equal_loudness_volume(v);});
            return coefficients;
        }
        
    } // NS loudness
} // NS imajuscule
