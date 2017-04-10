namespace imajuscule {
    namespace audio {
        
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
