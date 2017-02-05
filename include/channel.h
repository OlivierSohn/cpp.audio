
namespace imajuscule {

    struct MakeVolume {
        template<int J>
        static
        typename std::enable_if<J == 1, Volumes<J>>::type
        run(float volume, StereoGain const & ) {
            return {{volume}};
        }
        
        template<int J>
        static
        typename std::enable_if<J == 2, Volumes<J>>::type
        run(float volume, StereoGain const & gain) {
            return {{gain.left*volume, gain.right*volume}};
        }
    };
    
    /*
     * Ensures that xfade length is odd,
     * and at least 3
     */
    template<class T>
    constexpr int clamp_xfade(T xfade_) {
        int xfade = xfade_;
        if(xfade < 3) {
            return 3;
        }
        if(0 == xfade % 2) {
            ++xfade;
        }
        return xfade;
    }
    
    enum class XfadePolicy {
        SkipXfade, // internal fades are skipped but volume can act as fade in / out
        UseXfade
    };

    template<int nAudioOut, XfadePolicy XF = XfadePolicy::UseXfade>
    struct Channel : public NonCopyable {
        using Volumes = Volumes<nAudioOut>;
        using QueuedRequest = QueuedRequest<nAudioOut>;
        using Request = Request<nAudioOut>;
    
        static constexpr unsigned int default_volume_transition_length = 2000;
        static constexpr unsigned int min_xfade_size = 3;
        // make sure we'll have no overflow on volume_transition_remaining
        static_assert(default_volume_transition_length < (1 << 16), "");

    public:
        
        uint16_t volume_transition_remaining;

    private:
        // if next is false, the current crossfade is between two requests,
        // else the current crossfade is from or to 'empty'
        bool next : 1;
        bool active:1;
        
        unsigned int total_n_writes : relevantBits(audioelement::n_frames_per_buffer);
        unsigned int initial_audio_element_consummed : relevantBits(audioelement::n_frames_per_buffer - 1);
        
        QueuedRequest current;
        QueuedRequest previous; // not for SkipXfade
        int size_half_xfade; // not for SkipXfade
        int32_t remaining_samples_count = 0;
        int32_t current_next_sample_index = 0;
        int32_t other_next_sample_index = 0; // not for SkipXfade
        
        struct Volume {
            float current = 1.f;
            float increments = 0.f;
        };
        Volume chan_vol;

        std::queue<QueuedRequest> requests;

    public:
        Channel() : volume_transition_remaining(0), next(false), active(true) {
            A(!isPlaying());
        }
        
        void reset() {
            *this = {}; // to release all audio elements
            A(!isPlaying());
            A(!shouldReset());
            A(isActive());
        }
        
        // prefer using toVolume if channel is Playing
        void setVolume(float volume) {
            chan_vol.current = volume;
        }
        
        void toVolume(float volume, int nSteps) {
            A(nSteps);
            volume_transition_remaining = nSteps;
            auto invStep = 1.f / nSteps;
            chan_vol.increments = (volume - chan_vol.current) * invStep;
        }
        
        void set_xfade(int const size_xfade) {
            A(XF == XfadePolicy::UseXfade);
            A(!isPlaying()); // do not call this method while playing
            A( 1 == size_xfade % 2);
            
            size_half_xfade = (size_xfade-1) / 2;
            A(size_half_xfade > 0); // todo make a channel that has no xfade
        }
        
        int get_size_xfade() const {
            A(XF == XfadePolicy::UseXfade);
            return 1 + 2 * size_half_xfade;
        }
        
        float get_xfade_increment() const {
            A(XF == XfadePolicy::UseXfade);
            return 1.f / (get_size_xfade() - 1);
        };
        
        float duration_millis_xfade() const {
            A(XF == XfadePolicy::UseXfade);
            return frames_to_ms(static_cast<float>(get_size_xfade()));
        }
        
        void step(SAMPLE * outputBuffer, int nFrames, unsigned int audio_element_consummed);
        
        bool addRequest(Request r) {
            if(XF == XfadePolicy::UseXfade) {
                if(r.duration_in_frames < 2*get_size_xfade()) {
                    return false;
                }
            }
            A(r.valid());
            requests.emplace(std::move(r));
            return true;
        }

        void stopPlayingByXFadeToZero(int nSteps) {
            A(isPlaying()); // caller should check
            A(active);
            active = false;
            if(nSteps < 0) {
                A(XF==XfadePolicy::UseXfade);
                nSteps = get_size_xfade();
            }
            toVolume({}, nSteps);
        }
        
        bool isPlaying() const {
            if(shouldReset()) {
                return false;
            }
            return remaining_samples_count != 0 || !requests.empty() || current.buffer;
        }

        bool shouldReset() const {
            if(active) {
                return false;
            }
            if(chan_vol.increments < 0.f && std::abs(chan_vol.increments) < std::abs(chan_vol.current)) {
                return false;
            }
            return true;
        }
        
        bool isActive() const { return active; }

    private:
        
        bool consume(int n_writes_remaining) {
            A(remaining_samples_count == 0);
            if(XF==XfadePolicy::SkipXfade) {
                if(requests.empty()) {
                    return false;
                }
                current = std::move(requests.front());
                requests.pop();
                remaining_samples_count = current.duration_in_frames;
                if(current.buffer.isSoundBuffer()) {
                    current_next_sample_index = 0;
                }
                else {
                    current_next_sample_index = initial_audio_element_consummed; // keep separate to make the type conversion
                    A(n_writes_remaining <= total_n_writes); // make sure it's safe to do the following substraction, total_n_writes being unsigned
                    current_next_sample_index += total_n_writes - n_writes_remaining;
                    A(current_next_sample_index < audioelement::n_frames_per_buffer);
                }
                
                return true;
            }

            auto current_next_sample_index_backup = current_next_sample_index;
            if (requests.empty()) {
                A(!next); // because we have started the crossfade and have detected that there is no more requests to process
                if(!current.buffer) {
                    previous = std::move(current);
                    return false;
                }
                previous = std::move(current);
                A(!current.buffer); // move reset it
                // emulate a right xfade 'to zero'
                current.duration_in_frames = get_size_xfade()-1; // to do the right xfade
                remaining_samples_count = size_half_xfade;  // to do the right xfade
                current_next_sample_index = 0;
            }
            else if(!next && !current.buffer) {
                previous = std::move(current);
                // emulate a left xfade 'from zero'
                current.duration_in_frames = 2 * get_size_xfade(); // to skip the right xfade
                remaining_samples_count = size_half_xfade + 1; // to skip the normal writes and begin the left xfade
            }
            else {
                previous = std::move(current);
                current = std::move(requests.front());
                requests.pop();
                
                A(current.duration_in_frames >= 0);
                remaining_samples_count = current.duration_in_frames;
                current_next_sample_index = next ? other_next_sample_index : 0;
            }
            other_next_sample_index = current_next_sample_index_backup;
            return true;
        }
        
        bool done(int n_writes_remaining) {
            if(shouldReset()) {
                // to avoid residual noise due to very low volume
                return true;
            }
            return remaining_samples_count == 0 && !consume(n_writes_remaining);
        }
        
        void write_single(SAMPLE * outputBuffer, int n_writes) {
            A(n_writes > 0);
            if(current.buffer.isSoundBuffer()) {
                write_single_SoundBuffer(outputBuffer, n_writes, current.buffer.asSoundBuffer(), current.volumes);
            }
            else if(current.buffer.is32()) {
                write_single_AudioElement(outputBuffer, n_writes, current.buffer.asAudioElement32(), current.volumes);
            }
            else {
                write_single_AudioElement(outputBuffer, n_writes, current.buffer.asAudioElement64(), current.volumes);
            }
        }
        
        void write_single_SoundBuffer(SAMPLE * outputBuffer, int n_writes, soundBuffer::buffer const & buf, Volumes const & volume) {
            auto const s = (int) buf.size();
            for( int i=0; i<n_writes; ++i) {
                if( current_next_sample_index == s ) {
                    current_next_sample_index = 0;
                }
                A(current_next_sample_index < s);
                
                A(crossfading_from_zero_remaining() <= 0);
                A(std::abs(buf[current_next_sample_index]) < 1.1f);
                stepVolume();
                auto val = buf[current_next_sample_index] * chan_vol.current;
                for(auto i=0; i<nAudioOut; ++i) {
                    *outputBuffer += volume[i] * val;
                    ++outputBuffer;
                }
                ++current_next_sample_index;
            }
        }

        template<typename T>
        void write_single_AudioElement(SAMPLE * outputBuffer, int n_writes, T const & buf, Volumes const & volume) {
            for( int i=0; i<n_writes; ++i) {
                A(current_next_sample_index < audioelement::n_frames_per_buffer);
                
                A(XF==XfadePolicy::SkipXfade || crossfading_from_zero_remaining() <= 0);
                A(std::abs(buf[current_next_sample_index]) < 1.1f);
                stepVolume();
                auto val = chan_vol.current * static_cast<float>(buf[current_next_sample_index]);
                for(auto i=0; i<nAudioOut; ++i) {
                    *outputBuffer += val * volume[i];
                    ++outputBuffer;
                }
                ++current_next_sample_index;
            }

            if(current_next_sample_index == audioelement::n_frames_per_buffer) {
                current_next_sample_index = 0;
            }
        }
        
        void write_left_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes) {
            A(XF==XfadePolicy::UseXfade);
            A(n_writes > 0);
            //LG(INFO, "<<<<< %d", n_writes);
            A(n_writes <= remaining_samples_count);
            A(xfade_ratio >= 0.f);
            A(xfade_ratio <= 1.f);
            
            auto xfade_increment = get_xfade_increment();
            
            if(current.buffer.isSoundBuffer()) {
                if(!next || requests.front().buffer.isSoundBuffer()) {
                    auto * other = next ? &requests.front() : nullptr;
                    write_SoundBuffer_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment, other);
                }
                else {
                    A(next);
                    auto * other = &requests.front();
                    A(other->valid());
                    write_SoundBuffer_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment, other);
                }
            } else {
                auto * other = next ? &requests.front() : nullptr;
                
                if(other && other->buffer.isAudioElement()) {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment, other);
                }
                else {
                    write_AudioElement_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment, other);
                }
            }
        }

        void write_right_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes) {
            A(XF==XfadePolicy::UseXfade);
            A(n_writes > 0);
            //LG(INFO, ">>>>> %d", n_writes);
            A(n_writes <= crossfading_from_zero_remaining());
            A(xfade_ratio >= 0.f);
            A(xfade_ratio <= 1.f);
            
            auto xfade_decrement = - get_xfade_increment();
            xfade_ratio = 1-xfade_ratio;
            
            // end crossfade with other only if we started with him
            auto const * other = (next || !current.buffer) ? &previous : nullptr;
            
            if(!other || other->buffer.isSoundBuffer()) {
                if(!current.buffer || current.buffer.isSoundBuffer()) {
                    write_SoundBuffer_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_decrement, other);
                }
                else {
                    write_AudioElement_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_decrement, other);
                }
            } else {
                if(current.buffer && current.buffer.isAudioElement()) {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_decrement, other);
                }
                else {
                    write_SoundBuffer_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_decrement, other);
                }
            }
        }
        
        void write_SoundBuffer_2_SoundBuffer_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                   Request const * other) {
            A(XF==XfadePolicy::UseXfade);
            A(!other || !other->buffer || other->buffer.isSoundBuffer());
            A(!current.buffer || current.buffer.isSoundBuffer());

            int const s = current.buffer ? (int) current.buffer.asSoundBuffer().size() : 0;
            int const other_s = (other && other->buffer) ? safe_cast<int>(other->buffer.asSoundBuffer().size()) : 0;
            for( int i=0; i<n_writes; i++ ) {
                stepVolume();
                Volumes val{0.f};
                if(s) {
                    if( current_next_sample_index == s ) {
                        current_next_sample_index = 0;
                    }
                    A(current_next_sample_index < s);
                    A(std::abs(current.buffer.asSoundBuffer()[current_next_sample_index]) < 1.1f);
                    val = current.volumes * (xfade_ratio * chan_vol.current * current.buffer.asSoundBuffer()[current_next_sample_index]);
                    ++current_next_sample_index;
                }
                if(other_s) {
                    A(other_next_sample_index >= 0);
                    A(other_next_sample_index <= other_s);
                    if(other_next_sample_index == other_s) {
                        other_next_sample_index = 0;
                    }
                    A(other_next_sample_index <= other_s);
                    A(std::abs((other->buffer.asSoundBuffer())[other_next_sample_index] < 1.1f));
                    val += other->volumes * ((1.f - xfade_ratio) * chan_vol.current * (other->buffer.asSoundBuffer())[other_next_sample_index]);
                    ++other_next_sample_index;
                }
                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }
        }
        
        void write_SoundBuffer_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                    Request const * other) {
            A(XF==XfadePolicy::UseXfade);
            A(!current.buffer || current.buffer.isSoundBuffer());
            A(other->buffer.isAudioElement());
            if(other->buffer.is32()) {
                write_SoundBuffer_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                       other->buffer.asAudioElement32(),
                                                       other->volumes);
            }
            else {
                write_SoundBuffer_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                       other->buffer.asAudioElement64(),
                                                       other->volumes);
            }
        }

        template<typename T>
        void write_SoundBuffer_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                    T const & buf2, Volumes const & volBuf2) {
            A(XF==XfadePolicy::UseXfade);
            int const s = current.buffer ? (int) current.buffer.asSoundBuffer().size() : 0;
            for( int i=0; i<n_writes; i++ ) {
                stepVolume();

                A(other_next_sample_index >= 0);
                A(other_next_sample_index < audioelement::n_frames_per_buffer);
                A(std::abs(buf2[other_next_sample_index]) < 1.1f);
                auto val = volBuf2 * ((1.f - xfade_ratio) * chan_vol.current * static_cast<float>(buf2[other_next_sample_index]));
                ++other_next_sample_index;
                
                if(s) {
                    if( current_next_sample_index == s ) {
                        current_next_sample_index = 0;
                    }
                    A(current_next_sample_index < s);
                    A(std::abs(current.buffer.asSoundBuffer()[current_next_sample_index]) < 1.1f);
                    val += current.volumes * xfade_ratio * chan_vol.current * current.buffer.asSoundBuffer()[current_next_sample_index];
                    ++current_next_sample_index;
                }

                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }
            
            if(other_next_sample_index == audioelement::n_frames_per_buffer) {
                other_next_sample_index = 0;
            }
        }

        void write_AudioElement_2_SoundBuffer_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                    Request const * other) {
            A(XF==XfadePolicy::UseXfade);
            A(current.buffer.isAudioElement());
            A(!other || !other->buffer || other->buffer.isSoundBuffer());
            if(current.buffer.is32()) {
                write_AudioElement_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                       current.buffer.asAudioElement32(),
                                                       current.volumes,
                                                       other);
            }
            else {
                write_AudioElement_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                       current.buffer.asAudioElement64(),
                                                       current.volumes,
                                                       other);
            }
        }
        
        template<typename T>
        void write_AudioElement_2_SoundBuffer_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                        T const & buf1, Volumes const & volBuf1
                                                        , Request const * other) {
            A(XF==XfadePolicy::UseXfade);
            int const other_s = (other && other->buffer) ? safe_cast<int>(other->buffer.asSoundBuffer().size()) : 0;
            for( int i=0; i<n_writes; ++i, ++current_next_sample_index) {
                stepVolume();

                A(current_next_sample_index >= 0);
                A(current_next_sample_index < audioelement::n_frames_per_buffer);
                A(std::abs(buf1[current_next_sample_index]) < 1.1f);
                auto val = volBuf1 * (xfade_ratio * chan_vol.current * buf1[current_next_sample_index]);
                
                if(other_s) {
                    A(other_next_sample_index >= 0);
                    A(other_next_sample_index <= other_s);
                    if(other_next_sample_index == other_s) {
                        other_next_sample_index = 0;
                    }
                    A(other_next_sample_index <= other_s);
                    A(std::abs((other->buffer.asSoundBuffer())[other_next_sample_index]) < 1.1f);
                    val += other->volumes * ((1.f - xfade_ratio) * chan_vol.current * (other->buffer.asSoundBuffer())[other_next_sample_index]);
                    ++other_next_sample_index;
                }
                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }
            
            if(current_next_sample_index == audioelement::n_frames_per_buffer) {
                current_next_sample_index = 0;
            }
        }

        void write_AudioElement_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                     Request const * other)
        {
            A(XF==XfadePolicy::UseXfade);
            A(other->buffer.isAudioElement());
            A(current.buffer.isAudioElement());
            if(current.buffer.is32()) {
                if(other->buffer.is32()) {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                            current.buffer.asAudioElement32(),
                                                            current.volumes,
                                                            other->buffer.asAudioElement32(),
                                                            other->volumes);
                }
                else {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                            current.buffer.asAudioElement32(),
                                                            current.volumes,
                                                            other->buffer.asAudioElement64(),
                                                            other->volumes);
                }
            }
            else {
                if(other->buffer.is32()) {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                            current.buffer.asAudioElement64(),
                                                            current.volumes,
                                                            other->buffer.asAudioElement32(),
                                                            other->volumes);
                }
                else {
                    write_AudioElement_2_AudioElement_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment,
                                                            current.buffer.asAudioElement64(),
                                                            current.volumes,
                                                            other->buffer.asAudioElement64(),
                                                            other->volumes);
                }
            }
        }
        
        template<typename T1, typename T2>
        void write_AudioElement_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                         T1 const & buf1, Volumes const & volBuf1,
                                                         T2 const & buf2, Volumes const & volBuf2) {
            A(XF==XfadePolicy::UseXfade);
            for( int i=0; i<n_writes; ++i, ++current_next_sample_index, ++other_next_sample_index, xfade_ratio -= xfade_increment) {
                stepVolume();

                A(current_next_sample_index >= 0);
                A(current_next_sample_index < audioelement::n_frames_per_buffer);
                A(other_next_sample_index >= 0);
                A(other_next_sample_index < audioelement::n_frames_per_buffer);
                
                A(std::abs(buf1[current_next_sample_index]) < 1.1f);
                A(std::abs(buf2[other_next_sample_index]) < 1.1f);

                write_value(
                            volBuf1 * (xfade_ratio * chan_vol.current * static_cast<float>(buf1[current_next_sample_index])) +
                            volBuf2 * ((1.f - xfade_ratio) * chan_vol.current * static_cast<float>(buf2[other_next_sample_index]))
                            ,
                            outputBuffer);
            }
            
            if(current_next_sample_index == audioelement::n_frames_per_buffer) {
                current_next_sample_index = 0;
            }
            if(other_next_sample_index == audioelement::n_frames_per_buffer) {
                other_next_sample_index = 0;
            }
        }
        
        void stepVolume() {
            if( 0 == volume_transition_remaining ) {
                return;
            }
            volume_transition_remaining--;
            chan_vol.current += chan_vol.increments;
        }

        void write_value(Volumes v, SAMPLE *& outputBuffer) {
            for(auto i=0; i<nAudioOut; ++i) {
                *outputBuffer += v[i];
                ++outputBuffer;
            }
        }
        
        int crossfading_from_zero_remaining() const {
            if(XF==XfadePolicy::UseXfade && next) {
                return size_half_xfade - (current.duration_in_frames - remaining_samples_count);
            }
            else {
                return (get_size_xfade()-1) - (current.duration_in_frames - remaining_samples_count);
            }
        }
        
        void onBeginToZero(int n_writes_remaining) {
            A(XF==XfadePolicy::UseXfade);
            if((next = !requests.empty())) {
                
                if(requests.front().buffer.isSoundBuffer()) {
                    // soundBuffer are "synchronized" when possible : a sinus will start at the first positive value of sin, and end at 0
                    // so we want to start playing the next soundBuffer so that at the middle of the crossfade, it is exactly
                    // at the first sample of the buffer.
                
                    int sz_buffer = safe_cast<int>(requests.front().buffer.asSoundBuffer().size());
                    other_next_sample_index = sz_buffer - 1 - size_half_xfade;
                    while(other_next_sample_index < 0) {
                        other_next_sample_index += sz_buffer;
                    }
                }
                else {
                    A(n_writes_remaining > 0);
                    
                    other_next_sample_index = initial_audio_element_consummed; // keep separate to make the type conversion
                    A(n_writes_remaining <= total_n_writes); // make sure it's safe to do the following substraction, total_n_writes being unsigned
                    other_next_sample_index += total_n_writes - n_writes_remaining;
                    A(other_next_sample_index < audioelement::n_frames_per_buffer);
                }
                A(other_next_sample_index >= 0);
            }
        }
        
        bool handleToZero(SAMPLE *& outputBuffer, int & n_max_writes);
            
    };
    
    template<int nAudioOut, XfadePolicy XF>
    bool Channel<nAudioOut, XF>::handleToZero(SAMPLE *& outputBuffer, int & n_max_writes) {
        A(XF==XfadePolicy::UseXfade);
        if(remaining_samples_count == size_half_xfade + 1) {
            onBeginToZero(n_max_writes);
        }
        auto xfade_ratio = 0.5f + (float)(remaining_samples_count-1) / (float)(2*size_half_xfade);
        auto xfade_written = std::min(remaining_samples_count, n_max_writes);
        write_left_xfade( outputBuffer, xfade_ratio, xfade_written );
        n_max_writes -= xfade_written;
        remaining_samples_count -= xfade_written;
        if(n_max_writes <= 0) {
            return false;
        }
        outputBuffer += xfade_written * nAudioOut;
        A(remaining_samples_count == 0); // we are sure the xfade is finished
        return consume(n_max_writes);
    }
    
    template<int nAudioOut, XfadePolicy XF>
    void Channel<nAudioOut, XF>::step(SAMPLE * outputBuffer, int n_max_writes, unsigned int audio_element_consummed_frames)
    {
        A(n_max_writes <= audioelement::n_frames_per_buffer);
        A(audio_element_consummed_frames < audioelement::n_frames_per_buffer);
        
        initial_audio_element_consummed = audio_element_consummed_frames;
        total_n_writes = n_max_writes;
        
        if(done(n_max_writes)) {
            return;
        }
        
        while(1)
        {
            while(remaining_samples_count < n_max_writes)
            {
                if(XF==XfadePolicy::UseXfade)
                {
                    auto xfade_written = crossfading_from_zero_remaining();
                    if(xfade_written > 0) {
                        auto xfade_ratio = (float)(xfade_written-1) / (float)(2 * size_half_xfade);
                        xfade_written = std::min(xfade_written, remaining_samples_count);
                        write_right_xfade( outputBuffer, xfade_ratio, xfade_written );
                        remaining_samples_count -= xfade_written;
                        n_max_writes -= xfade_written;
                        if(done(n_max_writes)) {
                            return;
                        }
                        A(n_max_writes > 0);
                        outputBuffer += xfade_written * nAudioOut;
                        A(crossfading_from_zero_remaining() <= 0);
                    }
                }
                A(remaining_samples_count >= 0);
                {
                    auto remaining_normal = remaining_samples_count;
                    if(XF==XfadePolicy::UseXfade) {
                        remaining_normal -= size_half_xfade + 1;
                    }
                    else {
                        A(remaining_samples_count>0);
                    }
                    if(XF==XfadePolicy::SkipXfade || remaining_normal > 0) {
                        write_single( outputBuffer, remaining_normal );
                        n_max_writes -= remaining_normal;
                        remaining_samples_count -= remaining_normal;
                        if(n_max_writes <= 0) {
                            return;
                        }
                        outputBuffer += remaining_normal * nAudioOut;
                    }
                }
                A(remaining_samples_count >= 0);
                A(n_max_writes > 0);
                if(XF==XfadePolicy::UseXfade) {
                    A(remaining_samples_count <= size_half_xfade + 1);
                    if(!handleToZero(outputBuffer, n_max_writes)) {
                        return;
                    }
                }
                else {
                    A(remaining_samples_count == 0);
                    if(!consume(n_max_writes)) {
                        return;
                    }
                }
            }
            
            if(XF==XfadePolicy::UseXfade)
            {
                auto xfade_written = crossfading_from_zero_remaining();
                if(xfade_written > 0) {
                    auto xfade_ratio = (float)(xfade_written-1) / (float)(2 * size_half_xfade);
                    xfade_written = std::min(xfade_written, n_max_writes);
                    write_right_xfade( outputBuffer, xfade_ratio, xfade_written );
                    remaining_samples_count -= xfade_written;
                    n_max_writes -= xfade_written;
                    if(done(n_max_writes)) {
                        return;
                    }
                    if(n_max_writes <= 0) {
                        return;
                    }
                    outputBuffer += xfade_written * nAudioOut;
                    A(crossfading_from_zero_remaining() <= 0); // we are sure the xfade is finished
                    
                    if(remaining_samples_count < n_max_writes) {
                        continue;
                    }
                }
            }
            {
                auto remaining_normal = remaining_samples_count;
                if(XF==XfadePolicy::UseXfade) {
                    remaining_normal -= size_half_xfade + 1;
                }
                else {
                    A(remaining_samples_count>0);
                }
                if(XF==XfadePolicy::SkipXfade || remaining_normal > 0) {
                    if(remaining_normal <= n_max_writes) {
                        write_single( outputBuffer, remaining_normal );
                        n_max_writes -= remaining_normal;
                        remaining_samples_count -= remaining_normal;
                        if(n_max_writes <= 0) {
                            return;
                        }
                        outputBuffer += remaining_normal * nAudioOut;
                    }
                    else {
                        write_single( outputBuffer, n_max_writes );
                        remaining_samples_count -= n_max_writes;
                        return;
                    }
                }
            }
            if(XF==XfadePolicy::UseXfade) {
                A(remaining_samples_count >= 0);
                if(remaining_samples_count <= size_half_xfade + 1) {
                    if(!handleToZero(outputBuffer, n_max_writes)) {
                        return;
                    }
                    continue;
                }
            }
            return;
        }
    }
}
