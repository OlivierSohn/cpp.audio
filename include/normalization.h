
namespace imajuscule {
    namespace audio {
        
        template<int m, int M>
        constexpr auto do_normalize_i(float v) { return (v-m) / (M-m); }
        
        template<float const * const m, float const * const M>
        constexpr auto do_normalize_f(float v) { return (v-*m) / (*M-*m); }
        
        template<int m, int M>
        constexpr auto do_denormalize_i(float v) { return m + v * (M-m); }
        
        template<float const * const m, float const * const M>
        constexpr auto do_denormalize_f(float v) { return *m + v * (*M-*m); }
     
        
        struct NormalizedParamLimits {
            static const float m;
            static const float M;
        };
    }
} // namespaces
