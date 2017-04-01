
namespace imajuscule {
    
    constexpr auto noise_duration = .05f;
    // take into account the fact that to compute gray noise, a filter needs to be initialized using some pink noise samples
    constexpr auto grey_noise_duration = 2.f * noise_duration;
    
    soundBuffer const & getPinkNoise() {
        static soundBuffer n(soundId{Sound::PINK_NOISE, noise_duration});
        return n;
    }
    
    soundBuffer const & getGreyNoise() {
        static soundBuffer n(soundId{Sound::GREY_NOISE, grey_noise_duration});
        return n;
    }
    
    soundBuffer const & getWhiteNoise() {
        static soundBuffer n(soundId{Sound::NOISE, noise_duration});
        return n;
    }
}

using namespace imajuscule;

static float triangle_( float angle_radians ) {
    A(angle_radians >= 0.f);
    A(angle_radians <= 2.f * (float)M_PI);
    
    static const float inv_pi = 1.f / (float)M_PI;
    
    angle_radians *= inv_pi;
    if( angle_radians < 0.5f ) {        // 0 .. 0.5   ->  0 .. 1
        return 2.f * angle_radians;
    } else if( angle_radians < 1.5f ) { // 0.5 .. 1.5 ->  1 .. -1
        return 2.f - 2.f * angle_radians;
    } else {                            // 1.5 .. 2   ->  -1 .. 0
        A( angle_radians <= 2.f );
        return -4.f + 2.f * angle_radians;
    }
}
static float saw_( float angle_radians ) {
    A(angle_radians >= 0.f);
    A(angle_radians <= 2.f * (float)M_PI);
    
    constexpr float inv_pi = 1.f / (float)M_PI;
    
    angle_radians *= inv_pi;
    if( angle_radians <= 1.f ) {        // 0 .. 1   ->  0 .. 1
        return angle_radians;
    } else {                            // 1 .. 2   ->  -1 .. 0
        A( angle_radians <= 2.f );
        return -2.f + angle_radians;
    }
}

static float square_( float angle_radians ) {
    A(angle_radians >= 0.f);
    A(angle_radians <= 2.f * (float)M_PI);
    
    static const float inv_pi = 1.f / (float)M_PI;
    
    angle_radians *= inv_pi;

    return square(angle_radians);
}


template < typename F >
void soundBuffer::generate( int period, F f ) {
    
    // Let's compute the waveform. First sample is non zero, last sample is zero, so the mapping is:
    //
    //  sample(int) [0 .. period - 1]  ->  radian(float) [2*pi/period .. 2*pi]
    //
    float increment = 2.f * (float)M_PI / (float) period;
    
    for( int i=0; i<period;) {
        i++;
        values.emplace_back( f( increment * (float)i ) );
    }
    
    A( (int)values.size() == period );
}

template<typename F>
void soundBuffer::generate_with_smooth_transition(int period, F f) {
    constexpr auto min_transition_length = 10;
    constexpr auto transition_ratio = 10;
    
    auto transition_length = min_transition_length + period / transition_ratio;
    transition_length = std::min(transition_length, period);
    
    A(transition_length > 0);
    
    generate(transition_length, f);
    auto pre = std::move(values);
    values.clear();
    generate(period, f);
    for(int i=0; i<transition_length; ++i) {
        auto ratio_pre = (i+1) / (float)(transition_length+1); // we want the ratio to start after 0 and never reach 1
        static_assert(std::is_floating_point<decltype(ratio_pre)>::value, "");
        A(ratio_pre > 0.f);
        A(ratio_pre < 1.f);
        
        // we consider the two signals are not correlated (this is - hopefully - the case for noises!)
        // and we want to keep an equal power during fade, so the sum of the squared gains should be 1.
        // throughout the fade. We use sin / cos instead of sqrt to have a smoother start and end.
        
        auto angle = ratio_pre * (float)M_PI_2;
        auto gain_pre = std::sin(angle);
        auto gain_v = std::cos(angle);
        
        auto & v = values[values.size()-transition_length+i];
        v = gain_pre * pre[i] + gain_v * v;
        A(values[values.size()-transition_length+i] == v); // verify we used the right operator [] !
    }
}

soundBuffer::soundBuffer( soundId const & id ) {
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
                generate_with_smooth_transition( id.period_length, [](float){
                    static GaussianPinkNoiseAlgo a;
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
                generate_with_smooth_transition( id.period_length, [](float){
                    constexpr auto length_ftt =
#ifdef NDEBUG
                    4096;
#else
                    256;
#endif
                    static auto a = make_loudness_adapted_noise(getPinkNoise, length_ftt, length_ftt);
                    a.step();
                    return a.get();
                } );
                normalize();
                logSummary();
                break;
            }
        }
        if( id.period_length < 20 ) {
            {
                // center
                auto avg(0.f);
                for( auto const & v : values ) {
                    avg += v;
                }
                avg /= (float)values.size();
                for( auto & v : values ) {
                    v -= avg;
                }
            }
            {
                // maximize
                auto M(0.f);
                for (auto const & v : values) {
                    M = std::max( M, std::abs( v ) );
                }
                if( M < 0.5f ) {
                    auto fact = 0.7f/M;
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
            A(0);
            break;
    }
}

void soundBuffer::logSummary(int nsamples_per_extremity) const {
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

void soundBuffer::normalize() {
    if(values.empty()) {
        return;
    }
    range<float> r;
    for(auto v: values) {
        r.extend(v);
    }
    A(!r.empty());
    if(r.delta() < 0.f) {
        return;
    }
    
    auto M = std::max(-r.getMin(), r.getMax());
    M = 1.f / M;
    for(auto & v: values) {
        v *= M;
    }
}

soundBuffer & Sounds::get(soundId id ) {
    {
        auto it = sounds.find(id);
        if( it != sounds.end() ) {
            return it->second;
        }
    }
    auto it = sounds.emplace(id, id);
    A(it.second);
    return it.first->second;
}

