
namespace imajuscule {
    namespace audio {
        
        // http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/WAVE/WAVE.html
        
        struct WAVPCMHeader {
            char chunk_id[4];
            int chunk_size;
            char format[4];
            char subchunk1_id[4];
            int subchunk1_size;
            short int audio_format;
            short int num_channels;
            int sample_rate;            // sample_rate denotes the sampling rate.
            int byte_rate;
            short int block_align;
            short int bits_per_sample;
            char subchunk2_id[4];
            int subchunk2_size; // subchunk2_size denotes the number of samples.
            
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
        
        static WAVPCMHeader pcm_(int const num_frames, int const sample_rate,
                                 NChannels n_channels, int const bytes_per_sample) {
            A(bytes_per_sample);
            auto const num_channels = to_underlying(n_channels);
            auto const size_data = num_frames * num_channels * bytes_per_sample;
            
            return {
                {'R','I','F','F'},
                static_cast<int32_t>(sizeof(WAVPCMHeader) + size_data),
                {'W','A','V','E'},
                {'f','m','t',' '},
                16,
                1,
                static_cast<int16_t>(num_channels),
                sample_rate,
                sample_rate * bytes_per_sample * num_channels,
                static_cast<int16_t>(bytes_per_sample * num_channels),
                static_cast<int16_t>(bytes_per_sample * 8),
                {'d','a','t','a'},
                size_data
            };
        }
        
        enum class SampleFormat {
            signed_16,
            signed_24,
            signed_32
        };
        
        // todo test...
        struct int24_t {
            std::array<int8_t, 3> a;
            operator int32_t() const {
                int32_t v = ((a[2] << 24) | (a[1] << 16) | (a[0] << 8)) >> 8;
                return v;
            }
        };
        
        template<typename T>
        struct SignedSample;
        
        template<>
        struct SignedSample<int16_t> {
            static constexpr auto format = SampleFormat::signed_16;
        };
        
        template<>
        struct SignedSample<int24_t> {
            static constexpr auto format = SampleFormat::signed_24;
        };
        
        template<>
        struct SignedSample<int32_t> {
            static constexpr auto format = SampleFormat::signed_32;
        };
        
        template<int SAMPLE_SIZE>
        struct TypeForSampleSize;
        
        template <>
        struct TypeForSampleSize<1> {
            using type = int8_t;
            static_assert(sizeof(type) == 1, "");
        };
        
        template <>
        struct TypeForSampleSize<2> {
            using type = int16_t;
            static_assert(sizeof(type) == 2, "");
        };
        
        template <>
        struct TypeForSampleSize<3> {
            using type = int24_t;
            static_assert(sizeof(type) == 3, "");
        };
        
        template <>
        struct TypeForSampleSize<4> {
            using type = int32_t;
            static_assert(sizeof(type) == 4, "");
        };
        
        constexpr auto bytes_per_sample(SampleFormat f) {
            switch(f) {
                case SampleFormat::signed_16:
                    return 2;
                case SampleFormat::signed_24:
                    return 3;
                case SampleFormat::signed_32:
                    return 4;
            }
            return 0;
        }
        
        static WAVPCMHeader pcm(int num_frames, int sample_rate, NChannels n_channels, SampleFormat f) {
            return pcm_(num_frames, sample_rate, n_channels, bytes_per_sample(f));
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

            template<typename ITER>
            ITER ReadWithLinInterpStrideAsFloat(ITER it, ITER end, float const stride) {
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
                            prev[j] = ReadOneFloat<VAL>();
                            *it = v + (ratio * prev[j]);
                            ++it;
                            ++n_writes;
                        }
                    }
                    else {
                        i -= 1.f;
                        for(int j=0; j<n_channels; ++j) {
                            prev[j] = ReadOneFloat<VAL>();
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
            FLT ReadOneFloat() {
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
                // commented out for int24_t (std::array<int8_t>)
                //static_assert(std::is_integral<T>::value, "");
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
                                 SignedSample<SIGNED>::format));
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
