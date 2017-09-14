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
            Assert(max_);
            std::transform(vec.begin(), vec.end(),
                           vec.begin(), [max_](auto v){ v/= max_; });
            
            return vec;
        }
        
        
    } // NS loudness
} // NS imajuscule
