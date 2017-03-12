template<> struct Limits<GAIN> {
    static const float m;
    static const float M; };

template<> struct Limits<LOUDNESS_LEVEL> {
    static const float m;
    static const float M; };

template<> struct Limits<LOUDNESS_REF_FREQ_INDEX> {
    static constexpr auto m = 0;
    static constexpr auto M = 10; };

template<> struct Limits<LOUDNESS_COMPENSATION_AMOUNT> : public NormalizedParamLimits {};
