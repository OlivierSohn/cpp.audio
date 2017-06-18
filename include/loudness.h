namespace imajuscule {
    namespace loudness {
        
        // based on http://mariobon.com/Glossario/Loudness_ISO_226_2003b.pdf
        
        constexpr auto n_freq = 29;
        
        constexpr std::array<float,n_freq> freqs {{
            20.f,
            25.f,
            31.5f,
            40.f,
            50.f,
            63.f,
            80.f,
            100.f,
            125.f,
            160.f,
            200.f,
            250.f,
            315.f,
            400.f,
            500.f,
            630.f,
            800.f,
            1000.f,
            1250.f,
            1600.f,
            2000.f,
            2500.f,
            3150.f,
            4000.f,
            5000.f,
            6300.f,
            8000.f,
            10000.f,
            12500.f,
        }};
        
        constexpr std::array<float,n_freq> alpha_f_ {{
            .532f,
            .506f,
            .480f,
            .455f,
            .432f,
            .409f,
            .387f,
            .367f,
            .349f,
            .330f,
            .315f,
            .301f,
            .288f,
            .276f,
            .267f,
            .259f,
            .253f,
            .250f,
            .246f,
            .244f,
            .243f,
            .243f,
            .243f,
            .242f,
            .242f,
            .245f,
            .254f,
            .271f,
            .301f,
        }};
        
        constexpr std::array<float,n_freq> Lu_ {{
            -31.6f,
            -27.2f,
            -23.0f,
            -19.1f,
            -15.9f,
            -13.0f,
            -10.3f,
            -8.1f,
            -6.2f,
            -4.5f,
            -3.1f,
            -2.0f,
            -1.1f,
            -0.4f,
            0.f,
            .3f,
            .5f,
            0.f,
            -2.7f,
            -4.1f,
            -1.0f,
            1.7f,
            2.5f,
            1.2f,
            -2.1f,
            -7.1f,
            -11.2f,
            -10.7f,
            -3.1f
        }};
        
        constexpr std::array<float,n_freq> Tf_ {{
            78.5f,
            68.7f,
            59.5f,
            51.1f,
            44.0f,
            37.5f,
            31.5f,
            26.5f,
            22.1f,
            17.9f,
            14.4f,
            11.4f,
            8.6f,
            6.2f,
            4.4f,
            3.0f,
            2.2f,
            2.4f,
            3.5f,
            1.7f,
            -1.3f,
            -4.2f,
            -6.0f,
            -5.4f,
            -1.5f,
            6.0f,
            12.6f,
            13.9f,
            12.3f
        }};
        
        // using binary search,
        // ratio is the proportion of "returned index",
        // 1-ratio is the proportion of "returned index-1"
        static inline int closest_freq(float freq, float & ratio) {
            auto min_ = 0;
            auto max_ = n_freq-1;
            do {
                auto center = (min_ + max_) / 2;
                if(freq < freqs[center]) {
                    A(max_ != center);
                    max_ = center;
                }
                else {
                    A(min_ != center);
                    min_ = center;
                }
            }
            while(max_ - min_ >= 2);

            A(max_ == min_ + 1);
            A(freq >= freqs[min_] || min_ == 0);
            A(freq <= freqs[max_] || max_ == n_freq - 1);
            if(freq <= freqs[min_]) {
                A(min_==0 || freq == freqs[min_]);
                ratio = 1.f;
                return min_;
            }
            if(freq >= freqs[max_]) {
                A(max_ == n_freq-1);
                ratio = 1.f;
                return max_;
            }
            ratio = (freq - freqs[min_]) / (freqs[max_] - freqs[min_]);
            return max_;
        }

        static inline float compute_equal_loudness_volume(int i, float LN)
        {
            auto alpha_f = alpha_f_[i];
            auto Lu = Lu_[i];
            auto Tf = Tf_[i];
            
            auto Af = 4.47e-3f * (powf(10.f, .025f * LN) - 1.14f) + powf(.4f * powf(10.f, ((Tf + Lu) * .1f) - 9.f), alpha_f);
            
            auto Lp = 94.f - Lu + (10.f/alpha_f)* std::log(Af) / std::log(10.f);
            return Lp;
        }
        
        static inline auto compute_elv(float level) {
            std::array<float, n_freq> v;
            int i=0;
            for(auto & e : v) {
                e = compute_equal_loudness_volume(i, level);
                ++i;
            }
            return v;
        }

        static inline auto compute_elvs() {
            std::array<std::array<float, n_freq>, 9> elvs;
            int l = 0;
            for(auto & a : elvs) {
                a = compute_elv((l+2)*10.f);
                ++l;
            }
            return elvs;
        }
        
        static auto elvs = compute_elvs();
        
        static inline float equal_loudness_volume_db(float freq, int level) {
            float ratio;
            
            auto & elv = elvs[level];
            auto i = closest_freq(freq, ratio);
            if(ratio == 1.f) {
                return elv[i];
            }
            A(ratio < 1.f);
            A(ratio >= 0.f);
            return ratio * elv[i] + (1.f-ratio) * elv[i-1];
        }

        constexpr auto LN_default = 40.f; // unit : phons

        static inline float equal_loudness_volume(float freq, int index_freq_ref = 0, float log_ratio = 1.f, float level = LN_default) {
            // 20 .. 100 -> 0 .. 8;
            int i = static_cast<int>(level * .1f) - 2;
            i = std::max(0, i);
            i = std::min(static_cast<int>(elvs.size()) - 1, i);

            auto max_db = elvs[i][index_freq_ref];
            
            auto db = equal_loudness_volume_db(freq, i);
            if(db > max_db) {
                return 1.f;
                // equivalent to:
                // db = max_db;
            }
            
            return powf(10.f, log_ratio * (db-max_db)/20.f);
        }
}
} // NS imajuscule
