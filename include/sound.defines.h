namespace imajuscule {

using SAMPLE = float;

#ifndef CUSTOM_SAMPLE_RATE

constexpr int SAMPLE_RATE = 44100;
# define MAYBE_CONSTEXPR_SAMPLE_RATE constexpr

#else // CUSTOM_SAMPLE_RATE

extern int SAMPLE_RATE;
# define MAYBE_CONSTEXPR_SAMPLE_RATE

#endif // CUSTOM_SAMPLE_RATE

}
