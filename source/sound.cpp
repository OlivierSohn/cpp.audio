
namespace imajuscule::audio {

    constexpr auto noise_duration = .05f;
    // take into account the fact that to compute gray noise, a filter needs to be initialized using some pink noise samples
    constexpr auto grey_noise_duration = 2.f * noise_duration;

    soundBuffer<double> const & getPinkNoise(int sample_rate) {
        static soundBuffer<double> n(soundId{sample_rate, Sound::PINK_NOISE, noise_duration});
        return n;
    }

    soundBuffer<double> const & getGreyNoise(int sample_rate) {
        static soundBuffer<double> n(soundId{sample_rate, Sound::GREY_NOISE, grey_noise_duration});
        return n;
    }

    soundBuffer<double> const & getWhiteNoise(int sample_rate) {
        static soundBuffer<double> n(soundId{sample_rate, Sound::NOISE, noise_duration});
        return n;
    }

    template<typename T>
    T getAbsMean(soundBuffer<T> const & b) {
        T sum = 0;
        for(auto v : b) {
            sum += std::abs(v);
        }
        assert(!b.empty());
        sum /= b.size();
        LG(INFO, "abs mean: %f", sum);
        return sum;
    }

    float getWhiteNoiseAbsMean(int sample_rate) {
        static auto val = getAbsMean(getWhiteNoise(sample_rate));
        return val;
    }

    float getPinkNoiseAbsMean(int sample_rate) {
        static auto val = getAbsMean(getPinkNoise(sample_rate));
        return val;
    }

    float getGreyNoiseAbsMean(int sample_rate) {
        static auto val = getAbsMean(getGreyNoise(sample_rate));
        return val;
    }

static float triangle_( float angle_radians ) {
    Assert(angle_radians >= 0.f);
    Assert(angle_radians <= 2.f * (float)M_PI);

    static const float inv_pi = 1.f / (float)M_PI;

    angle_radians *= inv_pi;

    return triangle(angle_radians);
}
static float saw_( float angle_radians ) {
    Assert(angle_radians >= 0.f);
    Assert(angle_radians <= 2.f * (float)M_PI);

    constexpr float inv_pi = 1.f / (float)M_PI;

    angle_radians *= inv_pi;
    return saw(angle_radians);
}

static float square_( float angle_radians ) {
    Assert(angle_radians >= 0.f);
    Assert(angle_radians <= 2.f * (float)M_PI);

    static const float inv_pi = 1.f / (float)M_PI;

    angle_radians *= inv_pi;

    return square(angle_radians);
}


template<typename T>
template<typename F>
void soundBuffer<T>::generate( int period, F f ) {

    // Let's compute the waveform. First sample is non zero, last sample is zero, so the mapping is:
    //
    //  sample(int) [0 .. period - 1]  ->  radian(float) [2*pi/period .. 2*pi]
    //
    float increment = 2.f * (float)M_PI / (float) period;

    for( int i=0; i<period;) {
        i++;
        values.emplace_back( f( increment * (float)i ) );
    }

    Assert( (int)values.size() == period );
}

template<typename T>
template<typename F>
void soundBuffer<T>::generate_with_smooth_transition(int period, F f) {
    constexpr auto min_transition_length = 10;
    constexpr auto transition_ratio = 10;

    auto transition_length = min_transition_length + period / transition_ratio;
    transition_length = std::min(transition_length, period);

    Assert(transition_length > 0);

    generate(transition_length, f);
    auto pre = std::move(values);
    values.clear();
    generate(period, f);
    for(int i=0; i<transition_length; ++i) {
        auto ratio_pre = (i+1) / (float)(transition_length+1); // we want the ratio to start after 0 and never reach 1
        Assert(ratio_pre > 0.f);
        Assert(ratio_pre < 1.f);

        // we consider the two signals are not correlated (this is - hopefully - the case for noises!)
        // and we want to keep an equal power during fade, so the sum of the squared gains should be 1.
        // throughout the fade. We use sin / cos instead of sqrt to have a smoother start and end.

        auto angle = ratio_pre * (float)M_PI_2;
        auto gain_pre = std::sin(angle);
        auto gain_v = std::cos(angle);

        auto & v = values[values.size()-transition_length+i];
        v = gain_pre * pre[i] + gain_v * v;
        Assert(values[values.size()-transition_length+i] == v); // verify we used the right operator [] !
    }
}

template<typename T>
soundBuffer<T>::soundBuffer( soundId const & id ) {
    values.reserve( id.period_length );

    if(id.sound.type < Sound::END_NOISE) {
        switch (id.sound.type) {
            case Sound::NOISE:
            {
                ScopedLog l("Generating", "White Noise");
                // no need to generate_with_smooth_transition : white gaussian noise has no correlation between samples
                generate( id.period_length, white_gaussian_noise );
                logSummary();
                break;
            }
            case Sound::ATOM_NOISE:
            {
                ScopedLog l("Generating", "Atom Noise");
                // no need to generate_with_smooth_transition : white atom noise has no correlation between samples
                generate( id.period_length, white_atom_noise );
                logSummary();
                break;
            }
            case Sound::PINK_NOISE:
            {
                ScopedLog l("Generating", "Pink Noise");
                // using generate_with_smooth_transition because pink noise has some correlation between samples
                generate_with_smooth_transition( id.period_length, [sample_rate = id.sample_rate_](float){
                    static GaussianPinkNoiseAlgo a(sample_rate);
                    a.step();
                    return a.get();
                } );
                normalize();
                logSummary();
                break;
            }
            case Sound::GREY_NOISE:
            {
                ScopedLog l("Generating", "Grey Noise");
                // using generate_with_smooth_transition because grey noise has some correlation between samples
                generate_with_smooth_transition( id.period_length, [sample_rate = id.sample_rate_](float){
                    constexpr auto length_ftt =
#ifdef NDEBUG
                    4096;
#else
                    256;
#endif
                  static auto a = make_loudness_adapted_noise<T>(sample_rate, NoiseType::Pink, length_ftt, length_ftt);
                  return a.step();
                } );
                normalize();
                logSummary();
                break;
            }

            default:
              Assert(0);
              break;
        }
        if( id.period_length < 20 ) {
            {
                // center
                auto avg_ = std::accumulate(values.begin(), values.end(), 0.f) /
                            static_cast<float>(values.size());

                for( auto & v : values ) {
                    v -= avg_;
                }
            }
            {
                // maximize
                T M(0);
                for (auto const & v : values) {
                    M = std::max( M, std::abs( v ) );
                }
                if( M < 0.5 ) {
                    T fact = 0.7/M;
                    for( auto & v : values ) {
                        v *= fact;
                    }
                }
            }
        }
        return;
    }
    switch (id.sound.type) {

        case Sound::SINE:
            generate( id.period_length, sinf ); // todo measure if it is faster to use a temporary oscillator to generate the values
            break;

        case Sound::TRIANGLE:
            generate( id.period_length, triangle_ );
            break;

        case Sound::SAW:
            generate( id.period_length, saw_ );
            break;

        case Sound::SQUARE:
            generate( id.period_length, square_ );
            break;

        case Sound::SILENCE:
            generate( id.period_length, [](float){ return 0.f; } );
            break;

        case Sound::ONE:
            generate( id.period_length, [](float){ return 1.f; } );
            break;

        default:
            Assert(0);
            break;
    }
}

template<typename T>
void soundBuffer<T>::logSummary(int nsamples_per_extremity) const {
    std::vector<float> startend;
    startend.reserve(nsamples_per_extremity*2);
    for(int i=0; i < nsamples_per_extremity; ++i) {
        startend.push_back(values[i]);
    }
    for(int i=0; i < nsamples_per_extremity; ++i) {
        startend.push_back(values[values.size()-nsamples_per_extremity+i]);
    }
    StringPlot s(10, 10);
    s.draw(startend);
    s.log();

    auto power = 0.f;
    range<float> range;
    for(auto v : values) {
        range.extend(v);
        power += v*v;
    }
    power /= values.size();
    LG(INFO, "signal range [%.2f %.2f] avg power %.3f", range.getMin(), range.getMax(), power);
}

template<typename T>
void soundBuffer<T>::normalize() {
    if(values.empty()) {
        return;
    }
    range<float> r;
    for(auto v: values) {
        r.extend(v);
    }
    Assert(!r.empty());
    if(r.delta() < 0.f) {
        return;
    }

    auto M = std::max(-r.getMin(), r.getMax());
    float just_below_one = 1.f - FLOAT_EPSILON;
    M = just_below_one / M; // just_below_one is to be sure the signal doesn't go over 1 with numerical errors
    for(auto & v: values) {
        v *= M;
    }
}

template class soundBuffer<double>;
template class soundBuffer<float>;

  namespace detail {
    std::map< soundId, soundBuffer<double> > & getSounds() {
      static std::map< soundId, soundBuffer<double> > sounds;
      return sounds;
    }
  }
}
