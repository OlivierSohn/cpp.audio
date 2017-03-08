
namespace imajuscule {

    struct Sound {
        enum Type : unsigned char {
            NOISE, // white, gaussian
            ATOM_NOISE, // white, -1 or 1
            PINK_NOISE, // pink, gaussian
            
            END_NOISE,
            
            SINE = END_NOISE,
            TRIANGLE,
            SAW,
            SQUARE,
            SILENCE,
            ONE
        } type : 3;
        
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
                    return 1;
                case ATOM_NOISE:
                    return 1;
                case PINK_NOISE:
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
        
        using buffer = cacheline_aligned_allocated::vector<value_type>;
        
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
        
    private:
        template < typename F >
        void generate( int period, F );

        auto begin() { return values.begin(); }
        auto end() { return values.end(); }

        buffer values;
    };
    
    template<typename T, size_t N>
    size_t countActiveAudioElements(std::array<T, N> & aes) {
        return std::count_if(aes.begin(), aes.end(), [](T const & elt){ return elt.isActive(); });
    }
    template<typename T, size_t N>
    T * editInactiveAudioElement(std::array<T, N> & aes) {
        auto it = std::find_if(aes.begin(), aes.end(), [](T const & elt){ return elt.isInactive(); });
        return (it == aes.end()) ? nullptr : &*it;
    }
    
    template<typename T, typename F>
    typename T::value_type * editInactiveAudioElementContainer(T & container, F f_get_audioelement) {
        auto it = std::find_if(container.begin(),
                               container.end(),
                               [f_get_audioelement](auto & elt) {
                                   return f_get_audioelement(elt).isInactive();
                               });
        return (it == container.end()) ? nullptr : &*it;
    }

    template<typename T, typename F>
    typename T::value_type * editAudioElementContainer_if(T & container, F f_get_audioelement) {
        auto it = std::find_if(container.begin(),
                               container.end(),
                               [f_get_audioelement](auto & elt) {
                                   return f_get_audioelement(elt);
                               });
        return (it == container.end()) ? nullptr : &*it;
    }
    
}
