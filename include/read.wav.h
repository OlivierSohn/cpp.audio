
namespace imajuscule {
    namespace audio {
        
        // http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/WAVE/WAVE.html
        
        enum class WaveFormat : uint16_t {
            PCM =          0x0001, // PCM (Linear quantization, represented using signed values)
            IEEE_FLOAT =   0x0003, // IEEE float
            ALAW =         0x0006, // 8-bit ITU-T G.711 A-law
            MULAW =        0x0007, // 8-bit ITU-T G.711 µ-law
            EXTENSIBLE =   0xFFFE  // Determined by SubFormat
        };
        
        struct WAVPCMHeader {
            int8_t chunk_id[4];
            int32_t chunk_size;
            int8_t format[4];
            int8_t subchunk1_id[4];
            int32_t subchunk1_size;
            WaveFormat audio_format;
            int16_t num_channels;
            int32_t sample_rate;            // sample_rate denotes the sampling rate.
            int32_t byte_rate;
            int16_t block_align;
            int16_t bits_per_sample;
            int8_t subchunk2_id[4];
            int32_t subchunk2_size; // subchunk2_size denotes the number of samples.
            
            unsigned int countFrames() const {
                if(0 == block_align) {
                    // header has not been read yet
                    return 0;
                }
                A(0 == subchunk2_size % block_align);
                A(block_align);
                return subchunk2_size / block_align;
            }
            
            unsigned int countSamples() const {
                return countFrames() * num_channels;
            }
            
            unsigned int getSampleSize() const {
                A(bits_per_sample % 8 == 0);
                return bits_per_sample / 8;
            }
        };
        
        enum class NChannels {
            ONE = 1,
            TWO = 2,
            MONO = 1,
            STEREO = 2
        };
        
        enum class WaveFileFormat {
            BeginSupported,
            
            RIFF = BeginSupported,
            WaveIR,
            
            EndSupported,
            
            Unknown = EndSupported
        };
        
        constexpr auto getWavChunkId(WaveFileFormat f) {
            switch(f) {
                case WaveFileFormat::RIFF:
                    return "RIFF";
                case WaveFileFormat::WaveIR:
                    return "wvIR";
            }
            return "XXXX";
        }
        constexpr auto getWavWAVEId(WaveFileFormat f) {
            switch(f) {
                case WaveFileFormat::RIFF:
                    return "WAVE";
                case WaveFileFormat::WaveIR:
                    return "ver1";
            }
            return "XXXX";
        }
        
        static bool isFormatSupported(WaveFormat format) {
            switch(format) {
                case WaveFormat::PCM:
                case WaveFormat::IEEE_FLOAT:
                    return true;
                default:
                    return false;
            }
        }
        
        static WAVPCMHeader pcm_(WaveFileFormat const file_format,
                                 WaveFormat const format,
                                 int const num_frames,
                                 int const sample_rate,
                                 NChannels const n_channels,
                                 int const bytes_per_sample) {
            A(bytes_per_sample);
            auto const num_channels = to_underlying(n_channels);
            auto const size_data = num_frames * num_channels * bytes_per_sample;
            WAVPCMHeader ret {
                {0,0,0,0},
                static_cast<int32_t>(sizeof(WAVPCMHeader) + size_data),
                {0,0,0,0},
                {'f','m','t',' '},
                16,
                format,
                static_cast<int16_t>(num_channels),
                sample_rate,
                sample_rate * bytes_per_sample * num_channels,
                static_cast<int16_t>(bytes_per_sample * num_channels),
                static_cast<int16_t>(bytes_per_sample * 8),
                {'d','a','t','a'},
                size_data
            };
            
            memcpy(ret.chunk_id, getWavChunkId(file_format), 4);
            memcpy(ret.format, getWavWAVEId(file_format), 4);
            
            return ret;
        }
        
        enum class SampleFormat {
            signed_16,
            signed_24,
            signed_32,
            float_32,
            float_64
        };
        
        struct int24_t {
            std::array<int8_t, 3> a;
            operator int32_t() const {
                int32_t v = ((a[2] << 24) | (a[1] << 16) | (a[0] << 8)) >> 8;
                return v;
            }
        };
        
        template<typename T>
        struct AudioSample;
        
        template<>
        struct AudioSample<int16_t> {
            static constexpr auto format = SampleFormat::signed_16;
        };
        
        template<>
        struct AudioSample<int24_t> {
            static constexpr auto format = SampleFormat::signed_24;
        };
        
        template<>
        struct AudioSample<int32_t> {
            static constexpr auto format = SampleFormat::signed_32;
        };

        template<>
        struct AudioSample<float> {
            static constexpr auto format = SampleFormat::float_32;
        };
        
        template<>
        struct AudioSample<double> {
            static constexpr auto format = SampleFormat::float_64;
        };
        
        template<int SAMPLE_SIZE>
        struct SignedTypeForSampleSize;
        
        template <>
        struct SignedTypeForSampleSize<1> {
            using type = int8_t;
            static_assert(sizeof(type) == 1, "");
        };
        
        template <>
        struct SignedTypeForSampleSize<2> {
            using type = int16_t;
            static_assert(sizeof(type) == 2, "");
        };
        
        template <>
        struct SignedTypeForSampleSize<3> {
            using type = int24_t;
            static_assert(sizeof(type) == 3, "");
        };
        
        template <>
        struct SignedTypeForSampleSize<4> {
            using type = int32_t;
            static_assert(sizeof(type) == 4, "");
        };
        
        
        template<int SAMPLE_SIZE>
        struct FloatTypeForSampleSize;
        
        template <>
        struct FloatTypeForSampleSize<4> {
            using type = float;
            static_assert(sizeof(type) == 4, "");
        };
        
        template <>
        struct FloatTypeForSampleSize<8> {
            using type = double;
            static_assert(sizeof(type) == 8, "");
        };
        
        constexpr auto bytes_per_sample(SampleFormat f) {
            switch(f) {
                case SampleFormat::signed_16:
                    return 2;
                case SampleFormat::signed_24:
                    return 3;
                case SampleFormat::signed_32:
                case SampleFormat::float_32:
                    return 4;
                case SampleFormat::float_64:
                    return 8;
            }
            return 0;
        }
        
        static
#ifdef NDEBUG
        constexpr
#endif
        bool is_signed(SampleFormat const f) {
            switch(f) {
                case SampleFormat::signed_16:
                case SampleFormat::signed_24:
                case SampleFormat::signed_32:
                    return true;
                case SampleFormat::float_32:
                case SampleFormat::float_64:
                    return false;
            }
   
            A(0);
            return false;
        }
        
        static
#ifdef NDEBUG
        constexpr
#endif
        bool is_float(SampleFormat const f) {
            switch(f) {
                case SampleFormat::signed_16:
                case SampleFormat::signed_24:
                case SampleFormat::signed_32:
                    return false;
                case SampleFormat::float_32:
                case SampleFormat::float_64:
                    return true;
            }

            A(0);
            return false;
        }
        
        static bool are_compatible(WaveFormat const format, SampleFormat const sf) {
            switch(format) {
                case WaveFormat::PCM:
                    return is_signed(sf);
                    
                case WaveFormat::IEEE_FLOAT:
                    return is_float(sf);
                    
                default:
                    LG(ERR, "unhandled wave format");
                    return false;
            }
        }
        
        static WAVPCMHeader pcm(WaveFormat format, int num_frames, int sample_rate, NChannels n_channels, SampleFormat f) {
            A(are_compatible(format, f));
            return pcm_(WaveFileFormat::RIFF,
                        format,
                        num_frames,
                        sample_rate,
                        n_channels,
                        bytes_per_sample(f));
        }
        
        struct WAVReader : public ReadableStorage {
            WAVReader(DirectoryPath const & d, FileName const & f) : ReadableStorage(d, f), header{} {}
            
            eResult Initialize();
            
            unsigned int getSampleSize() const { return header.getSampleSize(); }
            int countChannels() const { return header.num_channels; }
            int countSamples() const { return header.countSamples(); }
            int countFrames() const { return header.countFrames(); }
            int getSampleRate() const { return header.sample_rate; }
            float getDuration() const { return countFrames() / (float)getSampleRate(); }
            WaveFormat getFormat() const { return header.audio_format; }
            WaveFileFormat getFileFormat() const {
                auto candidate = WaveFileFormat::BeginSupported;
                for(;
                    candidate != WaveFileFormat::EndSupported;
                    increment(candidate))
                {
                    if(!memcmp(header.chunk_id, getWavChunkId(candidate), 4)) {
                        break;
                    }
                }
                return candidate;
            }
            
            // 'it' is the begin of the buffer to write to, 'end' is the end.
            // returns the end element
            
            template<typename ITER>
            ITER Read(ITER it, ITER end) {
                constexpr auto n_reads = sizeof(decltype(*it));
                A(n_reads == header.getSampleSize());
                auto dist = std::distance(it, end);
                auto bytes_to_read = dist * n_reads;
                A((audio_bytes_read + bytes_to_read) <= header.subchunk2_size);
                audio_bytes_read += bytes_to_read;
                while(it < end) {
                    ReadData(&*it, n_reads, 1);
                    ++it;
                }
                return it;
            }

            // todo templatize and make wrapper to abstract signed or float encoding
            template<typename ITER>
            ITER ReadSignedWithLinInterpStrideAsFloat(ITER it, ITER end, float const stride) {
                using VAL = typename ITER::value_type;

                auto n_channels = countChannels();
                
                std::vector<float> prev;
                prev.resize(n_channels, {});

                int k=0;
                int n_writes = 0;
                
                float i = 0.f;
                while(it != end && HasMore()) {
                    if(i <= 0.f) {
                        A(i > -1.f);
                        auto prev_ratio = -i;
                        auto ratio = 1.f - prev_ratio;
                        
                        i += stride - 1.f;
                        for(int j=0; j<n_channels; ++j) {
                            auto v = prev_ratio * prev[j];
                            prev[j] = ReadOneSignedAsOneFloat<VAL>();
                            *it = v + (ratio * prev[j]);
                            ++it;
                            ++n_writes;
                        }
                    }
                    else {
                        i -= 1.f;
                        for(int j=0; j<n_channels; ++j) {
                            prev[j] = ReadOneSignedAsOneFloat<VAL>();
                        }
                    }
                    
                    ++k;
                    //LG(INFO, "%d %d", k, n_writes);
                }
                return it;
            }
            
            bool HasMore() const;
            
        private:
            WAVPCMHeader header;
            unsigned int audio_bytes_read = 0;
            
            bool readHeader();
            
            template<typename FLT>
            FLT ReadOneSignedAsOneFloat() {
                A(audio_bytes_read < header.subchunk2_size);

                if(3 == header.getSampleSize()) {
                    constexpr auto n_bytes = 3;
                    uint8_t d[n_bytes];
                    ReadData(d, n_bytes, 1);
                    
                    audio_bytes_read += n_bytes;
                    A(audio_bytes_read <= header.subchunk2_size);

                    int32_t v = ((d[2] << 24) | (d[1] << 16) | (d[0] << 8)) >> 8;
                    constexpr int64_t n_different_values = pow2(8*n_bytes);
                    constexpr int32_t m = -n_different_values/2;
                    constexpr int32_t M = n_different_values/2 - 1;
                    A(v <= M);
                    A(v >= m);
                    
                    return signed_to_float<
                    FLT,
                    int32_t,
                    M,
                    m
                    >(v);
                }
                else {
                    A(0); // todo
                    return 0;
                }

            }
        };
        
        struct WAVWriter : public WritableStorage {
            WAVWriter(DirectoryPath d, FileName f, WAVPCMHeader h) : WritableStorage(d,f), header(h) {}

            ~WAVWriter() { Finalize(); }
            
            eResult Initialize() {
                return doSaveBegin();
            }

            template<typename T>
            void writeSample(T s) {
                // T can be integral, int24_t, float, double
                WriteData(&s, sizeof(T), 1);
            }
            
        protected:
            
            void DoUpdateFileHeader() override;
            
        private:
            WAVPCMHeader header;
        };

        template<typename CONTAINER>
        void write_wav(DirectoryPath dir, std::string const & fname, CONTAINER const & samples, NChannels n_channels) {
            using namespace imajuscule::audio;
            using SIGNED = int32_t;
            
            WAVWriter writer(dir,
                             fname,
                             pcm(samples.size() / to_underlying(n_channels),
                                 SAMPLE_RATE,
                                 n_channels,
                                 AudioSample<SIGNED>::format));
            auto res = writer.Initialize();
            if(res != ILE_SUCCESS) {
                throw;
            }
            for(auto sample : samples) {
                auto c = float_to_signed<SIGNED>(sample);
                writer.writeSample(c);
            }
        }
        
        template<typename VEC_VEC>
        void write_wav(DirectoryPath dir, std::string const & fname, VEC_VEC const & v_samples) {
            
            auto sz = v_samples[0].size();
            for(auto const & v : v_samples) {
                A(sz == v.size());
            }
            
            auto n_channels = v_samples.size();
            
            std::vector<float> interlaced;
            interlaced.reserve(sz * n_channels);
            for(int k=0; k<sz; ++k) {
                for(int i=0; i<n_channels; ++i) {
                    interlaced.push_back(v_samples[i][k]);
                }
            }
            
            write_wav(std::move(dir), fname, interlaced, static_cast<NChannels>(n_channels));
        }
    }
}
