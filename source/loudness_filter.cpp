namespace imajuscule {
    namespace loudness {

        static void plotMagnitude(fft::FFTVec<double> const & v) {
            std::vector<double> mags;
            mags.reserve(v.size());
            for(auto c : v) {
                mags.push_back(abs(c));
            }
            StringPlot<30,1024> plot;
            plot.draw(mags);
            plot.log();
        }
        
        template<typename T, typename U, typename F>
        auto to_vector(U const & array, F f)
        {
            std::vector<T> res;
            res.reserve(array.size());
            for(auto const & v : array) {
                res.push_back(f(v));
            }
            return res;
        }

        template<typename T>
        auto make_coefficients_by_f_sampling() {
            // according to http://iowahills.com/FIRFiltersByFreqSampling.html
            // with the same number of taps as of fft size (we could try less)
            
            using namespace imajuscule::fft;
            
            constexpr auto N = 4096;
            FFTVec<double> res, input;
            res.resize(N);
            input.resize(N);
            
            auto nf = get_nyquist_frequency();
            
            auto nyquist = N/2;
            
            auto NumTaps = N;
            
            double RadPerSample = -M_PI; // Odd tap count
            if(0 == NumTaps % 2) {
                RadPerSample *= (N - 1.0)/N; // Even tap count
            }
            for(int i=0; i<=nyquist; ++i) {
                auto f = nf * i / nyquist;
                auto magnitude = equal_loudness_volume(f);
                auto cplx = magnitude * polar(RadPerSample*i);
                input[i] = cplx;
                if(i && i != nyquist) {
                    auto conji = N-i;
                    input[conji] = cplx;
                }
            }
            
            auto roots = compute_roots_of_unity<double>(N);
            Algo<double> a(roots);
            a.run(input.begin(), res.begin(), N, 1);
            auto scale = (double)N;
            // scaling + windowing
            int i=0;
            for(auto & r : res) {
                r /= scale;
                //            W(n) = cos(n/NumTaps · π/2)
                r *= cos( std::abs(nyquist-i)/(double)(NumTaps/2) * M_PI_2);
                ++i;
            }
            
            plotMagnitude(res);
            return to_vector<double>(res, [](auto const &val) { return val.real(); });
        }
        
        
        template<typename T>
        auto make_coefficients_by_least_squares() {
            auto vec = to_vector<T>(loudness_filter_coeffs_5001, [](T v){return v;});
         
            T max_ = {};
            for(auto c : vec) {
                max_ = std::max(std::abs(c), max_);
            }
            A(max_);
            for(auto &c : vec) {
                c /= max_;
            }
            
            return vec;
        }
        
        std::vector<double> const & getLoudnessCompensationFIRCoefficients() {
            static auto coefficients = make_coefficients_by_f_sampling<double>();
            return coefficients;
        }
        
    } // NS loudness
} // NS imajuscule
