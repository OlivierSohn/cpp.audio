
#ifndef NDEBUG
#define DO_LOG_XFADE 0
#else
#define DO_LOG_XFADE 0
#endif

#if DO_LOG_XFADE
#define LG_XFADE( x , y, ...) LG( x , y ,##__VA_ARGS__)
#else
#define LG_XFADE(...)
#endif

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

        template<int J>
        static
        typename std::enable_if<J == 1, Volumes<J>>::type
        run(float volume, float pan ) {
            return {{volume}};
        }

        template<int J>
        static
        typename std::enable_if<J == 2, Volumes<J>>::type
        run(float volume, float pan) {
            return run<J>(volume, stereo(pan));
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
        SkipXfade, // internal fades are skipped, we rely solely on volume to act as fade in / out.
                  // this is usefull for MonoNoteChannel usecase.
        UseXfade // use internal fades
    };

    enum class MaxQueueSize {
        One
      , Infinite
    };

    template<typename T, MaxQueueSize s>
    struct Queue;

    template <typename T>
    struct Queue<T, MaxQueueSize::One> {
      using type = fifo1<T>;
    };

    template <typename T>
    struct Queue<T, MaxQueueSize::Infinite> {
      using type = fifo<T>;
    };

    template <typename T, MaxQueueSize MQS>
    using queue_t = typename Queue<T,MQS>::type;

    template<int nAudioOut, XfadePolicy XF, MaxQueueSize MQS>
    struct Channel : public NonCopyable {
        using Volumes = Volumes<nAudioOut>;
        using QueuedRequest = QueuedRequest<nAudioOut>;
        using Request = Request<nAudioOut>;
        static constexpr auto XFPolicy = XF;

        static constexpr unsigned int default_volume_transition_length = 2000;
        static constexpr unsigned int min_xfade_size = 3;
        // make sure we'll have no overflow on volume_transition_remaining
        static_assert(default_volume_transition_length < (1 << 16));

        /*
         * used in case we don't know, at the time the request is added
         * to the channel, how long it will be.
         * After this is called, the xfade starts immediately
         *
         * Prerequisite: no xfade has begun, else the functionality is not guaranteed
         */
        void xfade_now() {
            Assert(XF!=XfadePolicy::SkipXfade);
            Assert(isPlaying());
            Assert(!StrictlyInsideXfade());
            auto new_c = 1 + size_half_xfade;
            if(!requests.empty()) {
                Assert(remaining_samples_count >= new_c);
                remaining_samples_count = new_c;
                Assert(current.duration_in_frames >= get_size_xfade());
                current.duration_in_frames = get_size_xfade();
            }
            else {

                //current.duration_in_frames = get_size_xfade(); // doesn't work
                // unchanged current.duration_in_frames works
                remaining_samples_count = new_c;
            }
        };

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
        int32_t size_half_xfade; // not for SkipXfade
        int32_t remaining_samples_count = 0;
        int32_t current_next_sample_index = 0;
        int32_t other_next_sample_index = 0; // not for SkipXfade

        struct Volume {
            float current = 1.f;
            float increments = 0.f;
        };
        Volume chan_vol;

        queue_t<QueuedRequest, MQS> requests;
      
    public:
        Channel() {
          reset();
        }

        auto & edit_requests() { return requests; }
        auto const & get_requests() const { return requests; }
        auto const & get_current() const { return current; }

        auto const get_size_half_xfade() const { Assert(XF == XfadePolicy::UseXfade); return size_half_xfade; }

        auto const get_remaining_samples_count() const { return remaining_samples_count; }

        // debugging function, do not use in production!
        void show() {
          using namespace std;
          cout << "channel:" << endl;
          cout << "volume_transition_remaining:" << volume_transition_remaining << endl;
          cout << "next:" << next << endl;
          cout << "active:" << active << endl;
          cout << "total_n_writes:" << total_n_writes << endl;
          cout << "initial_audio_element_consummed:" << initial_audio_element_consummed << endl;
          // TODO show current / previous if needed
          cout << "size_half_xfade:" << size_half_xfade << endl;
          cout << "remaining_samples_count:" << remaining_samples_count << endl;
          cout << "current_next_sample_index:" << current_next_sample_index << endl;
          cout << "other_next_sample_index:" << other_next_sample_index << endl;
          cout << "chan_vol current:" << chan_vol.current << endl;
          cout << "chan_vol increments:" << chan_vol.increments << endl;
          cout << "n queued requests:" << requests.size() << endl;
        }

        void reset() {
          volume_transition_remaining = 0;
          next = false;
          active = true;
          
          // will mark buffers of used requests as "inactive"
          current = {};
          previous = {};
          requests.reset();

          chan_vol = {};
          remaining_samples_count = 0;
          current_next_sample_index = 0;
          other_next_sample_index = 0;
        
          Assert(!isPlaying());
          Assert(!shouldReset());
          Assert(isActive());
        }

        void setVolume(float volume) {
            Assert(!isPlaying());
            chan_vol.current = volume;
            chan_vol.increments = 0.f;
            volume_transition_remaining = 0;
        }

        void toVolume(float volume, int nSteps) {
            Assert(nSteps);
            volume_transition_remaining = nSteps;
            auto invStep = 1.f / nSteps;
            chan_vol.increments = (volume - chan_vol.current) * invStep;
        }

        void set_xfade(int const size_xfade) {
            Assert(XF == XfadePolicy::UseXfade);
            Assert(!isPlaying()); // do not call this method while playing
            Assert( 1 == size_xfade % 2);

            size_half_xfade = (size_xfade-1) / 2;
            Assert(size_half_xfade > 0); // todo make a channel that has no xfade
        }

        int get_size_xfade() const {
            Assert(XF == XfadePolicy::UseXfade);
            return 1 + 2 * size_half_xfade;
        }

        float get_xfade_increment() const {
            Assert(XF == XfadePolicy::UseXfade);
            return 1.f / (get_size_xfade() - 1);
        };

        float duration_millis_xfade() const {
            Assert(XF == XfadePolicy::UseXfade);
            return frames_to_ms(static_cast<float>(get_size_xfade()));
        }

        void step(SAMPLE * outputBuffer, int nFrames, unsigned int audio_element_consummed);

        bool addRequest(Request && r) {
            if constexpr (XF == XfadePolicy::UseXfade) {
                if(r.duration_in_frames < 2*get_size_xfade()) {
                    return false;
                }
            }
            Assert(r.valid());
            requests.emplace(std::move(r));
            return true;
        }

        void stopPlayingByXFadeToZero(int nSteps) {
            Assert(isPlaying()); // caller should check
            Assert(active);
            active = false;
            if(nSteps < 0) {
                Assert(XF==XfadePolicy::UseXfade);
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

        bool StrictlyInsideXfade() const {
            Assert(XF==XfadePolicy::UseXfade);
            return (remaining_samples_count <= size_half_xfade) || (crossfading_from_zero_remaining() > 0);
        }
    private:

        bool consume(int n_writes_remaining) {
            Assert(remaining_samples_count == 0);
            if constexpr (XF==XfadePolicy::SkipXfade) {
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
                    Assert(n_writes_remaining <= total_n_writes); // make sure it's safe to do the following substraction, total_n_writes being unsigned
                    current_next_sample_index += total_n_writes - n_writes_remaining;
                    Assert(current_next_sample_index < audioelement::n_frames_per_buffer);
                }
            }
            else {
                auto current_next_sample_index_backup = current_next_sample_index;
                previous = std::move(current);
                Assert(!current.buffer); // the move constructor has reset it
                if (requests.empty()) {
                    Assert(!next); // because we have started the crossfade and have detected that there is no more requests to process
                    if(!previous.buffer) {
                        return false;
                    }
                    // emulate a right xfade 'to zero'
                    current.duration_in_frames = get_size_xfade()-1; // to do the right xfade
                    remaining_samples_count = size_half_xfade;  // to do the right xfade
                    current_next_sample_index = 0;
                }
                else if(!next && !current.buffer) {
                    // emulate a left xfade 'from zero'
                    current.duration_in_frames = 2 * get_size_xfade(); // to skip the right xfade
                    remaining_samples_count = size_half_xfade + 1; // to skip the normal writes and begin the left xfade
                }
                else {
                    current = std::move(requests.front());
                    requests.pop();

                    Assert(current.duration_in_frames >= 0);
                    remaining_samples_count = current.duration_in_frames;
                    current_next_sample_index = next ? other_next_sample_index : 0;
                }
                other_next_sample_index = current_next_sample_index_backup;
            }
            return true;
        }

        bool done(int n_writes_remaining) {
            if(shouldReset()) {
                // to avoid residual noise due to very low volume
                requests.reset();
                current = {};
                previous = {};
                return true;
            }
            return remaining_samples_count == 0 && !consume(n_writes_remaining);
        }

        void write_single(SAMPLE * outputBuffer, int n_writes) {
            Assert(n_writes > 0);
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
                Assert(current_next_sample_index < s);

                Assert(crossfading_from_zero_remaining() <= 0);
                Assert(std::abs(buf[current_next_sample_index]) < 100000.f);
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
                Assert(current_next_sample_index < audioelement::n_frames_per_buffer);

                Assert(XF==XfadePolicy::SkipXfade || crossfading_from_zero_remaining() <= 0);
                Assert(std::abs(buf[current_next_sample_index]) < 100000.f);
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
            Assert(XF==XfadePolicy::UseXfade);
            Assert(n_writes > 0);
            //LG(INFO, "<<<<< %d", n_writes);
            Assert(n_writes <= remaining_samples_count);
            Assert(xfade_ratio >= 0.f);
            Assert(xfade_ratio <= 1.f);

            auto xfade_increment = get_xfade_increment();

            if(current.buffer.isSoundBuffer()) {
                if(!next || requests.front().buffer.isSoundBuffer()) {
                    auto * other = next ? &requests.front() : nullptr;
                    write_SoundBuffer_2_SoundBuffer_xfade(outputBuffer, xfade_ratio, n_writes, xfade_increment, other);
                }
                else {
                    Assert(next);
                    auto * other = &requests.front();
                    Assert(other->valid());
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
            Assert(XF==XfadePolicy::UseXfade);
            Assert(n_writes > 0);
            //LG(INFO, ">>>>> %d", n_writes);
            Assert(n_writes <= crossfading_from_zero_remaining());
            Assert(xfade_ratio >= 0.f);
            Assert(xfade_ratio <= 1.f);

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
            Assert(XF==XfadePolicy::UseXfade);
            Assert(!other || !other->buffer || other->buffer.isSoundBuffer());
            Assert(!current.buffer || current.buffer.isSoundBuffer());

            LG_XFADE(INFO, ".xfade %.5f", xfade_ratio);
            int const s = current.buffer ? (int) current.buffer.asSoundBuffer().size() : 0;
            int const other_s = (other && other->buffer) ? safe_cast<int>(other->buffer.asSoundBuffer().size()) : 0;
            for( int i=0; i<n_writes; i++ ) {
                stepVolume();
                Volumes val{0.f};
                if(s) {
                    if( current_next_sample_index == s ) {
                        current_next_sample_index = 0;
                    }
                    Assert(current_next_sample_index < s);
                    Assert(std::abs(current.buffer.asSoundBuffer()[current_next_sample_index]) < 100000.f);
                    val = current.volumes * (xfade_ratio * chan_vol.current * current.buffer.asSoundBuffer()[current_next_sample_index]);
                    ++current_next_sample_index;
                }
                if(other_s) {
                    Assert(other_next_sample_index >= 0);
                    Assert(other_next_sample_index <= other_s);
                    if(other_next_sample_index == other_s) {
                        other_next_sample_index = 0;
                    }
                    Assert(other_next_sample_index <= other_s);
                    Assert(std::abs((other->buffer.asSoundBuffer())[other_next_sample_index]) < 100000.f);
                    val += other->volumes * ((1.f - xfade_ratio) * chan_vol.current * (other->buffer.asSoundBuffer())[other_next_sample_index]);
                    ++other_next_sample_index;
                }
                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }
            LG_XFADE(INFO, " xfade %.5f", xfade_ratio);
        }

        void write_SoundBuffer_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                    Request const * other) {
            Assert(XF==XfadePolicy::UseXfade);
            Assert(!current.buffer || current.buffer.isSoundBuffer());
            Assert(other->buffer.isAudioElement());
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
            LG_XFADE(INFO, ".xfade %.5f", xfade_ratio);
            Assert(XF==XfadePolicy::UseXfade);
            int const s = current.buffer ? (int) current.buffer.asSoundBuffer().size() : 0;
            for( int i=0; i<n_writes; i++ ) {
                stepVolume();

                Assert(other_next_sample_index >= 0);
                Assert(other_next_sample_index < audioelement::n_frames_per_buffer);
                Assert(std::abs(buf2[other_next_sample_index]) < 100000.f);
                auto val = volBuf2 * ((1.f - xfade_ratio) * chan_vol.current * static_cast<float>(buf2[other_next_sample_index]));
                ++other_next_sample_index;

                if(s) {
                    if( current_next_sample_index == s ) {
                        current_next_sample_index = 0;
                    }
                    Assert(current_next_sample_index < s);
                    Assert(std::abs(current.buffer.asSoundBuffer()[current_next_sample_index]) < 100000.f);
                    val += current.volumes * xfade_ratio * chan_vol.current * current.buffer.asSoundBuffer()[current_next_sample_index];
                    ++current_next_sample_index;
                }

                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }

            if(other_next_sample_index == audioelement::n_frames_per_buffer) {
                other_next_sample_index = 0;
            }
            LG_XFADE(INFO, " xfade %.5f", xfade_ratio);
        }

        void write_AudioElement_2_SoundBuffer_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                    Request const * other) {
            Assert(XF==XfadePolicy::UseXfade);
            Assert(current.buffer.isAudioElement());
            Assert(!other || !other->buffer || other->buffer.isSoundBuffer());
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
            LG_XFADE(INFO, ".xfade %.5f", xfade_ratio);
            Assert(XF==XfadePolicy::UseXfade);
            int const other_s = (other && other->buffer) ? safe_cast<int>(other->buffer.asSoundBuffer().size()) : 0;
            for( int i=0; i<n_writes; ++i, ++current_next_sample_index) {
                stepVolume();

                Assert(current_next_sample_index >= 0);
                Assert(current_next_sample_index < audioelement::n_frames_per_buffer);
                Assert(std::abs(buf1[current_next_sample_index]) < 100000.f);
                auto val = volBuf1 * (xfade_ratio * chan_vol.current * buf1[current_next_sample_index]);

                if(other_s) {
                    Assert(other_next_sample_index >= 0);
                    Assert(other_next_sample_index <= other_s);
                    if(other_next_sample_index == other_s) {
                        other_next_sample_index = 0;
                    }
                    Assert(other_next_sample_index <= other_s);
                    Assert(std::abs((other->buffer.asSoundBuffer())[other_next_sample_index]) < 100000.f);
                    val += other->volumes * ((1.f - xfade_ratio) * chan_vol.current * (other->buffer.asSoundBuffer())[other_next_sample_index]);
                    ++other_next_sample_index;
                }
                xfade_ratio -= xfade_increment;
                write_value(std::move(val), outputBuffer);
            }

            if(current_next_sample_index == audioelement::n_frames_per_buffer) {
                current_next_sample_index = 0;
            }
            LG_XFADE(INFO, " xfade %.5f", xfade_ratio);
        }

        void write_AudioElement_2_AudioElement_xfade(SAMPLE * outputBuffer, float xfade_ratio, int const n_writes, float xfade_increment,
                                                     Request const * other)
        {
            Assert(XF==XfadePolicy::UseXfade);
            Assert(other->buffer.isAudioElement());
            Assert(current.buffer.isAudioElement());
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
            LG_XFADE(INFO, ".xfade %.5f", xfade_ratio);
            Assert(XF==XfadePolicy::UseXfade);
            for( int i=0; i<n_writes; ++i, ++current_next_sample_index, ++other_next_sample_index, xfade_ratio -= xfade_increment) {
                stepVolume();

                Assert(current_next_sample_index >= 0);
                Assert(current_next_sample_index < audioelement::n_frames_per_buffer);
                Assert(other_next_sample_index >= 0);
                Assert(other_next_sample_index < audioelement::n_frames_per_buffer);

                Assert(std::abs(buf1[current_next_sample_index]) < 100000.f);
                Assert(std::abs(buf2[other_next_sample_index]) < 100000.f);

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
            LG_XFADE(INFO, " xfade %.5f", xfade_ratio);
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
            Assert(XF==XfadePolicy::UseXfade);
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
                    Assert(n_writes_remaining > 0);

                    other_next_sample_index = initial_audio_element_consummed; // keep separate to make the type conversion
                    Assert(n_writes_remaining <= total_n_writes); // make sure it's safe to do the following substraction, total_n_writes being unsigned
                    other_next_sample_index += total_n_writes - n_writes_remaining;
                    Assert(other_next_sample_index < audioelement::n_frames_per_buffer);
                }
                Assert(other_next_sample_index >= 0);
            }
        }

        bool duringRightXfade(int xfade_remaining, int max_xfade_writes, SAMPLE *& outputBuffer, int & n_max_writes)
        {
            {
                auto xfade_written = std::min(xfade_remaining, max_xfade_writes);
                {
                    auto xfade_ratio = (float)(xfade_remaining-1) / (float)(2 * size_half_xfade);
                    write_right_xfade( outputBuffer, xfade_ratio, xfade_written );
                }
                outputBuffer += xfade_written * nAudioOut;
                remaining_samples_count -= xfade_written;
                n_max_writes -= xfade_written;
                if(xfade_remaining == xfade_written) {
                    // this is the end of the right xfade, we release the previous request
                    // to avoid unnecessary audioelement computations
                    previous = {};
                }
            }
            return done(n_max_writes);
        }

        bool handleToZero(SAMPLE *& outputBuffer, int & n_max_writes);

    };

    template<int nAudioOut, XfadePolicy XF, MaxQueueSize MQS>
    bool Channel<nAudioOut, XF, MQS>::handleToZero(SAMPLE *& outputBuffer, int & n_max_writes) {
        Assert(XF==XfadePolicy::UseXfade);
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
        Assert(remaining_samples_count == 0); // we are sure the xfade is finished
        return consume(n_max_writes);
    }

    template<int nAudioOut, XfadePolicy XF, MaxQueueSize MQS>
    void Channel<nAudioOut, XF, MQS>::step(SAMPLE * outputBuffer, int n_max_writes, unsigned int audio_element_consummed_frames)
    {
        Assert(n_max_writes <= audioelement::n_frames_per_buffer);
        Assert(audio_element_consummed_frames < audioelement::n_frames_per_buffer);

        initial_audio_element_consummed = audio_element_consummed_frames;
        total_n_writes = n_max_writes;

        if(done(n_max_writes)) {
            return;
        }

        while(1)
        {
            while(remaining_samples_count < n_max_writes)
            {
                if constexpr (XF==XfadePolicy::UseXfade)
                {
                    auto xfade_remaining = crossfading_from_zero_remaining();
                    if(xfade_remaining > 0) {
                        if(duringRightXfade(xfade_remaining, remaining_samples_count, outputBuffer, n_max_writes)) {
                            return;
                        }
                        Assert(n_max_writes > 0);
                        Assert(crossfading_from_zero_remaining() <= 0);
                    }
                }
                Assert(remaining_samples_count >= 0);
                {
                    auto remaining_normal = remaining_samples_count;
                    if constexpr (XF==XfadePolicy::UseXfade) {
                        remaining_normal -= size_half_xfade + 1;
                    }
                    else {
                        Assert(remaining_samples_count>0);
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
                Assert(remaining_samples_count >= 0);
                Assert(n_max_writes > 0);
                if constexpr (XF==XfadePolicy::UseXfade) {
                    Assert(remaining_samples_count <= size_half_xfade + 1);
                    if(!handleToZero(outputBuffer, n_max_writes)) {
                        return;
                    }
                }
                else {
                    Assert(remaining_samples_count == 0);
                    if(!consume(n_max_writes)) {
                        return;
                    }
                }
            }

            if constexpr (XF==XfadePolicy::UseXfade)
            {
                auto xfade_remaining = crossfading_from_zero_remaining();
                if(xfade_remaining > 0) {
                    if(duringRightXfade(xfade_remaining, n_max_writes, outputBuffer, n_max_writes)) {
                        return;
                    }
                    Assert(n_max_writes >= 0);
                    if(0 == n_max_writes) {
                        return;
                    }
                    Assert(crossfading_from_zero_remaining() <= 0); // we are sure the xfade is finished

                    if(remaining_samples_count < n_max_writes) {
                        continue;
                    }
                }
            }
            {
                auto remaining_normal = remaining_samples_count;
                if constexpr (XF==XfadePolicy::UseXfade) {
                    remaining_normal -= size_half_xfade + 1;
                }
                else {
                    Assert(remaining_samples_count>0);
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
            if constexpr (XF==XfadePolicy::UseXfade) {
                Assert(remaining_samples_count >= 0);
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
