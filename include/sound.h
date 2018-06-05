
namespace imajuscule {

    struct Sound {
        enum Type : unsigned char {
            NOISE, // white, gaussian
            ATOM_NOISE, // white, -1 or 1
            PINK_NOISE, // pink, gaussian
            GREY_NOISE, // grey, gaussian
            
            END_NOISE,
            
            SINE = END_NOISE,
            TRIANGLE,
            SAW,
            SQUARE,
            SILENCE,
            ONE
        } type : 4;
        
        static constexpr auto ConstantSoundDummyFrequency = 1.f;
        
        constexpr bool zeroOnPeriodBoundaries() const { return type == SINE || type == TRIANGLE; }
        constexpr bool operator == (const Sound & other) const { return type == other.type; }
        constexpr bool operator < (const Sound & other) const { return type < other.type; }
        Sound(Type t) : type(t) {}
        Sound() = default;

        constexpr int minimalPeriod() const {
            switch(type) {
                case SINE:
                    return 3;
                case SQUARE:
                    return 3;
                case TRIANGLE:
                    return 2;
                case SAW:
                    return 3; // 2 would give the same result as a triangle
                case NOISE:
                case ATOM_NOISE:
                case PINK_NOISE:
                case GREY_NOISE:
                    return 1;
                case SILENCE:
                    return 0;
                case ONE:
                    return 0;
            }
            return 1;
        }
    };
    
    struct soundId {
        soundId() = default;
        
        soundId( Sound sound, float freq_hz = 1.f )
        :
        sound(sound),
        period_length( (sound == Sound::SILENCE) ? 1 : freq_to_int_period( freq_hz ) )
        {}
        
        Sound sound;
        int32_t period_length;
        bool operator < (const soundId & other) const {
            if(sound == other.sound) {
                return ( period_length < other.period_length );
            }
            return( sound < other.sound );
        }
    };
    
    struct soundBuffer {
        using value_type = float;
        using FPT = value_type;
        
        using buffer = a64::vector<value_type>;
        
        // no copy
        soundBuffer(const soundBuffer &) = delete;
        soundBuffer & operator=(const soundBuffer&) = delete;

        soundBuffer(soundBuffer &&) = default;
        soundBuffer& operator = (soundBuffer &&) = default;
        
        bool empty() const { return values.empty(); }
        auto size() const { return values.size(); }
        
        auto begin() const { return values.begin(); }
        auto end() const { return values.end(); }
        
        auto operator [] (int i) const { return values[i]; }
        
        soundBuffer(size_t n, float value) : values(n, value) {}
        
        soundBuffer( soundId const & );
        
        auto & getBuffer() { return values; }
        
        void logSummary(int nsamples_per_extremity = 3) const;
    private:
        template < typename F >
        void generate( int period, F );
        
        template < typename F >
        void generate_with_smooth_transition( int period, F );

        auto begin() { return values.begin(); }
        auto end() { return values.end(); }

        auto & operator [] (int i) { return values[i]; }

        buffer values;
        
        void normalize();
    };
    
    template<typename T, size_t N>
    T * editInactiveAudioElement(std::array<T, N> & aes) {
        auto it = std::find_if(aes.begin(), aes.end(), [](T const & elt){ return elt.isInactive(); });
        return (it == aes.end()) ? nullptr : &*it;
    }

    
    
    template<Sound::Type SOUND>
    struct FGetBuffer;
    
    template<Sound::Type SOUND>
    struct BufferIter {
        using F_GET_BUFFER = FGetBuffer<SOUND>;
        using FPT = float;
        
        BufferIter() {
            initializeForRun();
        }
        
        void initializeForRun() {
            it = F_GET_BUFFER()().begin();
            // randomize start position
            auto add = static_cast<int>(std::uniform_real_distribution<>{
                0.f,
                static_cast<float>(F_GET_BUFFER()().size()-1)
            }(mersenne<SEEDED::No>()));
            
            it += add;
            Assert(it < end);
        }
        
        void log() const {
            LG(INFO, "pink noise iterator @[%d]", getPosition());
        }
        
        void operator ++() {
            ++it;
            if(it == end) {
                it = F_GET_BUFFER()().begin();
            }
        }
        
        float operator *() const {
            auto v = *it;
            Assert(v <=  1.f);
            Assert(v >= -1.f);
            return v;
        }
        
        int getPosition() const { return static_cast<int>(std::distance(F_GET_BUFFER()().begin(), it)); }

        float getAbsMean() const { return F_GET_BUFFER().getAbsMean(); }
    private:
        decltype(F_GET_BUFFER()().begin()) it, end = F_GET_BUFFER()().end();
    };

    /////////////////
    // instantiations
    /////////////////
    
    soundBuffer const & getWhiteNoise();
    float getWhiteNoiseAbsMean();
    
    soundBuffer const & getPinkNoise();
    float getPinkNoiseAbsMean();

    soundBuffer const & getGreyNoise();
    float getGreyNoiseAbsMean();
    
    template<>
    struct FGetBuffer<Sound::PINK_NOISE> {
        auto const & operator()() {
            return getPinkNoise();
        }
        float getAbsMean() {
            return getPinkNoiseAbsMean();
        }
    };
    template<>
    struct FGetBuffer<Sound::GREY_NOISE> {
        auto const & operator()() {
            return getGreyNoise();
        }
        float getAbsMean() {
            return getGreyNoiseAbsMean();
        }
    };

    using PinkNoiseIter = BufferIter<Sound::PINK_NOISE>;
    using GreyNoiseIter = BufferIter<Sound::GREY_NOISE>;
}
