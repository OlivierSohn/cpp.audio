// this header should be included inside namespaces like:
// imajuscule::audio::<pluginnamespace>
// where ImplParams is an enum defined in that namespace

template<ImplParams e>
struct Limits;

template<ImplParams N>
struct Normalizer {
    template<typename lim = Limits<N>, int m = lim::m, int M = lim::M>
    static float run(float v)
    {
        return do_normalize_i<m, M>(v);
    }
    
    template<typename lim = Limits<N>, float const * const m = &lim::m, float const * const M = &lim::M>
    static float run(float v)
    {
        return do_normalize_f<m, M>(v);
    }
};

template<ImplParams N>
struct Denormalizer {
    template<typename lim = Limits<N>, int m = lim::m, int M = lim::M>
    static float run(float v)
    {
        return do_denormalize_i<m, M>(v);
    }
    
    template<typename lim = Limits<N>, float const * const m = &lim::m, float const * const M = &lim::M>
    static float run(float v)
    {
        return do_denormalize_f<m, M>(v);
    }
};

template<ImplParams N>
constexpr auto normalize(float v) {
    return Normalizer<N>::run(v);
}

template<ImplParams N>
constexpr auto denormalize(float v) {
    return Denormalizer<N>::run(v);
}
