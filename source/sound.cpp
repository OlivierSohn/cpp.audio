
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

soundBuffer::soundBuffer( soundId const & id ) {
    values.reserve( id.period_length );

    if(id.sound.type < Sound::END_NOISE) {
        switch (id.sound.type) {
            case Sound::NOISE:
                generate( id.period_length, white_gaussian_noise );
                break;
            case Sound::ATOM_NOISE:
                generate( id.period_length, white_atom_noise );
                break;
            case Sound::PINK_NOISE:
                generate( id.period_length, [](float){
                    static GaussianPinkNoiseAlgo a;
                    a.step();
                    return a.get();
                } );
                break;
            case Sound::GREY_NOISE:
                generate( id.period_length, [](float){
                    static GaussianGreyNoiseAlgo a;
                    a.step();
                    return a.get();
                } );
                break;
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

