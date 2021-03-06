
namespace imajuscule {
    namespace audio {
        namespace voice {

#include "../include/loudness_enum_limits_impl.h"

            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>::m = 0.f;
            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MIN>::M = 5.f;
            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>::m = 0.f;
            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH_MAX>::M = 5.f;

            const float Limits<CENTER_OCTAVE_MIN_LONG_TERM>::m = 0.f;
            const float Limits<CENTER_OCTAVE_MIN_LONG_TERM>::M = 10.f;
            const float Limits<CENTER_OCTAVE_MAX_LONG_TERM>::m = 0.f;
            const float Limits<CENTER_OCTAVE_MAX_LONG_TERM>::M = 10.f;

            const float Limits<PAN>::m = -1.f;
            const float Limits<PAN>::M = 1.f;

            const float Limits<HARMONIC_ATTENUATION>::m = 0.5f;
            const float Limits<HARMONIC_ATTENUATION>::M = 0.98f;

            const float Limits<LENGTH>::m = 10.f;
            const float Limits<LENGTH>::M = 500.f; // see comment below

            const float Limits<LENGTH_EXPONENT>::m = 0.f;
            const float Limits<LENGTH_EXPONENT>::M = 7.f; // see comment below
            // chosen so that Limits<LENGTH>::M ^ (Limits<LENGTH_EXPONENT>::M * 2) < std::numeric_limits<int>::max()
            // *2 in exponent to take into account exponent variation

            const float Limits<LOW_FREQ>::m = 10.f;
            const float Limits<LOW_FREQ>::M = 10000.f;
            const float Limits<HIGH_FREQ>::m = 20.f;
            const float Limits<HIGH_FREQ>::M = 40000.f;

        }
    }
} // namespaces
