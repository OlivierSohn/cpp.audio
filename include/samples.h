namespace imajuscule {
    namespace audio {
        
        template<typename SIGNED, typename FLT>
        static SIGNED float_to_signed(FLT flt) {
            if(flt > NumTraits<FLT>::zero()) {
                if(flt > NumTraits<FLT>::one()) {
                    return std::numeric_limits<SIGNED>::max();
                }
                constexpr auto mult = NumTraits<FLT>::half() + std::numeric_limits<SIGNED>::max();
                return static_cast<SIGNED>(flt * mult);
            }
            else {
                if(flt < -NumTraits<FLT>::one()) {
                    return std::numeric_limits<SIGNED>::min();
                }
                constexpr auto mult = NumTraits<FLT>::half() - std::numeric_limits<SIGNED>::min();
                return static_cast<SIGNED>(flt * mult);
            }
        }
        
        template<typename FLT, typename SIGNED>
        static auto signed_to_float(SIGNED s) -> FLT {
            if(s > 0) {
                return s / static_cast<float>(std::numeric_limits<SIGNED>::max());
            }
            else {
                return -s / static_cast<float>(std::numeric_limits<SIGNED>::min());
            }
        }
    } // NS audio
} // NS imajuscule
