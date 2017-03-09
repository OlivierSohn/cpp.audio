
namespace imajuscule {
    namespace audio {
        namespace voice {
            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH>::m = 0.f;
            const float Limits<PINK_NOISE_BP_OCTAVE_WIDTH>::M = 10.f;
            
            const float Limits<GAIN>::m = 1.f; // gain is multiplicative
            const float Limits<GAIN>::M = 60.f;
            
            const float Limits<PAN>::m = -1.f;
            const float Limits<PAN>::M = 1.f;
            
            const float Limits<LOUDNESS_LEVEL>::m = 20.f;
            const float Limits<LOUDNESS_LEVEL>::M = 100.f;

            const float Limits<HARMONIC_ATTENUATION>::m = 0.f;
            const float Limits<HARMONIC_ATTENUATION>::M = 0.98f;
            
            const float Limits<LENGTH>::m = 10.f;
            const float Limits<LENGTH>::M = 500.f; // see comment below
            
            const float Limits<LENGTH_EXPONENT>::m = 0.f;
            const float Limits<LENGTH_EXPONENT>::M = 7.f; // see comment below
            // chosen so that Limits<LENGTH>::M ^ (Limits<LENGTH_EXPONENT>::M * 2) < std::numeric_limits<int>::max()
            // *2 in exponent to take into account exponent variation
        }
    }
} // namespaces
