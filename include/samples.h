namespace imajuscule {
    namespace audio {
        
        
        template<int n_bytes>
        struct Uint8array_to_int32;
        
        template<>
        struct Uint8array_to_int32<1> {
            static int32_t run(uint8_t d[1]) {
                return (d[0] << 24) >> 24;
            }
        };
        
        template<>
        struct Uint8array_to_int32<2> {
            static int32_t run(uint8_t d[2]) {
                return ((d[1] << 24) | (d[0] << 16)) >> 16;
            }
        };
        
        template<>
        struct Uint8array_to_int32<3> {
            static int32_t run(uint8_t d[3]) {
                return ((d[2] << 24) | (d[1] << 16) | (d[0] << 8)) >> 8;
            }
        };
        
        template<>
        struct Uint8array_to_int32<4> {
            static int32_t run(uint8_t d[4]) {
                return ((d[3] << 24) | (d[2] << 16) | (d[1] << 8) | d[0]);
            }
        };
        
        template<int n_bytes>
        constexpr int32_t uint8array_to_int32(uint8_t array[n_bytes]) {
            return Uint8array_to_int32<n_bytes>::run(array);
        }
        
        template<
        typename SIGNED,
        typename FLT,
        SIGNED M = std::numeric_limits<SIGNED>::max(),
        SIGNED m = std::numeric_limits<SIGNED>::min()
        >
        static SIGNED float_to_signed(FLT flt) {
            if(flt > NumTraits<FLT>::zero()) {
                if(flt > NumTraits<FLT>::one()) {
                    return M;
                }
                constexpr auto mult = NumTraits<FLT>::half() + M;
                return static_cast<SIGNED>(flt * mult);
            }
            else {
                if(flt < -NumTraits<FLT>::one()) {
                    return m;
                }
                constexpr auto mult = NumTraits<FLT>::half() - m;
                return static_cast<SIGNED>(flt * mult);
            }
        }
        
        template<
        typename FLT,
        typename SIGNED,
        SIGNED M = std::numeric_limits<SIGNED>::max(),
        SIGNED m = std::numeric_limits<SIGNED>::min()
        >
        static auto signed_to_float(SIGNED s) -> FLT {
            if(s > 0) {
                return s / static_cast<FLT>(M);
            }
            else {
                return -s / static_cast<FLT>(m);
            }
        }
    } // NS audio
} // NS imajuscule
