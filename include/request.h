
namespace imajuscule {
    namespace audioelement {
        template<typename T>
        void onQueued(T * buffer) {
            using AE = AEBuffer<T>;
            Assert(state(buffer) == AE::inactive()); // to be sure at most one channel uses it
            state(buffer) = AE::queued();
        }

        template<typename T>
        void onUnqueued(T * buffer) {
            using AE = AEBuffer<T>;
            Assert(state(buffer) != AE::inactive()); // to be sure at most one channel uses it
            // note that if state is AE::queued(), it means no computation occured on this buffer
            // (indicating the channel has been interrupted)
            state(buffer) = AE::inactive();
        }
    }

    using AE32Buffer = float *;
    using AE64Buffer = double *;

    template<Atomicity, int>
    struct QueuedRequest;

    struct TaggedBuffer {
        template<Atomicity, int>
        friend class QueuedRequest;

        template<typename T>
        explicit TaggedBuffer(T buf)
        : buffer(buf) {}

        void reset() {
            *this = TaggedBuffer{nullptr};
        }

        void reset(soundBuffer * buf) {
            *this = TaggedBuffer{buf};
        }

        soundBuffer::buffer & asSoundBuffer() const {
            auto ptr = buffer.soundBuffer();
            Assert(is32() && !isAudioElement() && ptr);
            return *ptr;
        }

        AE32Buffer asAudioElement32() const {
            auto ptr = buffer.audioElement32();
            Assert(is32() && isAudioElement() && ptr);
            return ptr;
        }

        AE64Buffer asAudioElement64() const {
            auto ptr = buffer.audioElement64();
            Assert(!is32() && isAudioElement() && ptr);
            return ptr;
        }

        bool null() const {
            // this is not strictly legal, as we dont check the tags
            // to see which of the union members is active...
            return !buffer.soundBuffer();
        }

        bool valid() const {
            if(isAudioElement()) {
                return !null();
            }
            if(!is32()) {
                return false;
            }
            auto ptr = buffer.soundBuffer();
            return ptr && !ptr->empty();
        }

        bool isSoundBuffer() const { return !buffer.flags.is_AudioElement; }
        bool isAudioElement() const { return buffer.flags.is_AudioElement; }

        bool is32() const { return buffer.flags.is_32; }

        bool isSilence() const {
            if(isAudioElement()) {
                // we cannot presume anything because
                // values are not yet computed
                return false;
            }
            if(!is32()) {
                Assert(0);
                return false;
            }
            if(!buffer.soundBuffer()) {
                return true;
            }
            for(auto b : *buffer.soundBuffer()) {
                if(b != 0.f) {
                    return false;
                }
            }
            return true;
        }

        operator bool() const {
            return !null();
        }

    private:
        union buffer {
            buffer(std::nullptr_t) : sound(nullptr) {
                Assert(as_uintptr_t == ptr());
                Assert(!flags.is_32);
                Assert(!flags.is_AudioElement);
            }
            buffer(soundBuffer * buf) : sound(buf ? &buf->getBuffer() : nullptr) {
                Assert(as_uintptr_t == ptr());
                flags.is_32 = true;
                Assert(!flags.is_AudioElement);
            }
            buffer(AE32Buffer buf) : audioelement_32(buf) {
                Assert(buf == audioElement32());
                flags.is_32 = true;
                flags.is_AudioElement = true;
            }
            buffer(AE64Buffer buf) : audioelement_64(buf) {
                Assert(buf == audioElement64());
                Assert(!flags.is_32);
                flags.is_AudioElement = true;
            }

            static constexpr auto n_low_bits_used = 2;

            uintptr_t ptr() const { return removeLowBits<n_low_bits_used>(as_uintptr_t); }

            soundBuffer::buffer * soundBuffer() const { return reinterpret_cast<soundBuffer::buffer *>(ptr()); }
            AE32Buffer audioElement32() const { return reinterpret_cast<AE32Buffer>(ptr()); }
            AE64Buffer audioElement64() const { return reinterpret_cast<AE64Buffer>(ptr()); }

            // all buffers are aligned on cachelines (64=2^6) meaning the 6 lower bits my be used to store information
            struct {
                bool is_AudioElement : 1;
                bool is_32 : 1;
            } flags;

            uintptr_t as_uintptr_t;
            soundBuffer::buffer * sound;
            AE32Buffer audioelement_32;
            AE64Buffer audioelement_64;
        } buffer;


        void onQueued() const {
            Assert(valid());
            if(isSoundBuffer()) {
                return;
            }
            if(is32()) {
                audioelement::onQueued(asAudioElement32());
            }
            else {
                audioelement::onQueued(asAudioElement64());
            }
        }

        void onUnqueued() {
            Assert(valid());
            if(isSoundBuffer()) {
                return;
            }
            if(is32()) {
                audioelement::onUnqueued(asAudioElement32());
            }
            else {
                audioelement::onUnqueued(asAudioElement64());
            }
        }
    };


    template<int nAudioOut>
    struct Volumes {
        std::array<float, nAudioOut> volumes;

        Volumes() = default;

        Volumes(float f) {
            volumes.fill(f);
        }
      
      static constexpr float uint8_to_float = 1 / 255.f;

      Volumes(std::array<float, nAudioOut> a) : volumes(std::move(a)) {}
      Volumes(std::array<uint8_t, nAudioOut> a) {
        for(int i=0; i<nAudioOut; ++i) {
          volumes[i] = uint8_to_float * static_cast<float>(a[i]);
        }
      }

        Volumes & operator =(float const f) {
            for(auto & v : volumes) {
                v = f;
            }
            return *this;
        }

        Volumes & operator *=(float const f)  {
            for(auto & v : volumes) {
                v *= f;
            }
            return *this;
        }

        Volumes operator *(float const f) const {
            Volumes v(*this);
            v *= f;
            return v;
        }

        void operator +=(Volumes const & o) {
            for(int i=0; i<volumes.size(); ++i) {
                volumes[i] += o.volumes[i];
            }
        }

        void operator +=(Volumes && o) {
            for(int i=0; i<volumes.size(); ++i) {
                volumes[i] += o.volumes[i];
            }
        }

        Volumes operator +(Volumes && o) {
            for(int i=0; i<volumes.size(); ++i) {
                o[i] += volumes[i];
            }
            return std::move(o);
        }

        bool is_zero() const {
            for(auto const v : volumes) {
                if(v!=0.f) {
                    return false;
                }
            }
            return true;
        }

        bool is_valid() const {
            // commented out : for virtual instruments we need to output more than 1.f
/*            for(auto const v : volumes) {
                if(v<-1.f) {
                    return false;
                }
                if(v>1.f) {
                    return false;
                }
            }
 */
            return true;
        }

        float & operator[](int i) {
            return volumes[i];
        }
        float operator[](int i) const {
            return volumes[i];
        }
    };

    template<Atomicity A, int nAudioOut>
    struct Request {
        using Volumes = Volumes<nAudioOut>;

        static constexpr float chan_base_amplitude = 0.3f; // ok to have3 chanels at max amplitude at the same time

        Request( Sounds<A> & sounds, Sound const sound, float freq_hz, Volumes vol, float duration_ms ) :
        volumes(vol*chan_base_amplitude),
        buffer(nullptr)
        {
            Assert(duration_ms >= 0.f);
            duration_in_frames = ms_to_frames(duration_ms);

            // we silence some sounds instead of just not playing them, in order to keep
            // the rythm

            soundId Id;
            bool silence = false;
            if(sound == Sound::SILENCE) {
                silence = true;
            }
            else if(freq_hz < 10.f) {
                LG(WARN, "silenced sound of inaudible (low) frequency '%.6f Hz'", freq_hz);
                silence = true;
            }
            else if(volumes.is_zero()) {
                silence = true;
            }
            else {
                Id = soundId{ sound, freq_hz };
                if(Id.period_length < sound.minimalPeriod()) {
                    silence = true;
                    LG(WARN, "silenced sound of inaudible (high) frequency '%.1f Hz'", freq_hz);
                }
            }
            if(silence) {
                buffer.reset(&sounds.get( {Sound::SILENCE} ));
                volumes = 0.f;
            }
            else {
                auto & b = sounds.get( std::move(Id) );
                buffer.reset(&b);

                if( sound.zeroOnPeriodBoundaries() ) {
                    const int period_size = (int)b.size();
                    if(period_size == 0) {
                        return;
                    }

                    if(0 == duration_in_frames) {
                        duration_in_frames = period_size;
                    }
                    else {
                        const int mod = duration_in_frames % period_size;
                        if(mod) {
                            duration_in_frames += period_size-mod;
                        }
                    }

                    Assert( 0 == duration_in_frames % period_size);
                }
            }
        }

        Request( soundBuffer * buffer, Volumes volume, int duration_in_frames) :
        buffer(buffer), volumes(volume*chan_base_amplitude), duration_in_frames(duration_in_frames) { }

        Request( AE32Buffer buffer, Volumes volume, int duration_in_frames) :
        buffer(buffer), volumes(volume*chan_base_amplitude), duration_in_frames(duration_in_frames) { }

        Request( AE64Buffer buffer, Volumes volume, int duration_in_frames) :
        buffer(buffer), volumes(volume*chan_base_amplitude), duration_in_frames(duration_in_frames) { }

        Request( AE32Buffer buffer, Volumes volume, float duration_in_ms) :
        Request(buffer, std::move(volume), ms_to_frames(duration_in_ms)) { }

        Request( AE64Buffer buffer, Volumes volume, float duration_in_ms) :
        Request(buffer, std::move(volume), ms_to_frames(duration_in_ms)) { }

        Request() : buffer(nullptr) {}

        void reset() { buffer.reset(); }

        bool valid() const { return duration_in_frames >= 1 && buffer.valid() && volumes.is_valid(); }

        bool isSilence() const {
            return volume_is_zero() || buffer.isSilence();
        }

        TaggedBuffer buffer;
        Volumes volumes;
        int32_t duration_in_frames;

    private:
        bool volume_is_zero() const {
            return volumes.is_zero();
        }
    };

    template<Atomicity A, int nAudioOut>
    struct QueuedRequest : public Request<A, nAudioOut> {
        using Request = Request<A, nAudioOut>;
        using Request::buffer;
        using Request::volumes;
        using Request::duration_in_frames;

        QueuedRequest() = default;

        // non copyable (cf. destructor implementation)
        QueuedRequest(const QueuedRequest &) = delete;
        QueuedRequest & operator=(const QueuedRequest&) = delete;

        // movable
        QueuedRequest(QueuedRequest && o) : Request(std::move(o)) {
            o.buffer.reset();
            Assert(!o.buffer);
        }

        QueuedRequest& operator =(QueuedRequest && o) {
            if (this != &o) {
                unqueue();
                buffer = o.buffer;
                volumes = o.volumes;
                duration_in_frames = o.duration_in_frames;
                o.buffer.reset();
                Assert(!o.buffer);
            }
            return *this;
        }

        QueuedRequest(Request const & o) : Request(o) {
            buffer.onQueued();
        }

        ~QueuedRequest() {
            unqueue();
        }

    private:
        void unqueue() {
            if(buffer) {
                buffer.onUnqueued();
            }
        }
    };
  
  template<int nAudioOuts>
  struct PackedRequestParams {
    int32_t length; // duration in samples
    std::array<uint8_t, nAudioOuts> volumes; // 0 = muted, 255 = full
  };

}
