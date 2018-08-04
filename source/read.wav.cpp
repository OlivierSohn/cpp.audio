using namespace imajuscule;
using namespace imajuscule::audio;

eResult WAVReader::Initialize()
{
    auto res = OpenForRead();
    if(res != ILE_SUCCESS) {
        return res;
    }
    if(!readHeader()) {
        LG(WARN, "this wav header is not supported");
        return ILE_NOT_IMPLEMENTED;
    }
    return ILE_SUCCESS;
}


bool WAVReader::readHeader() {
    bool is_wave_ir = false;
    ReadData(&header.chunk_id[0], 4, 1);
    {
        if(memcmp(header.chunk_id, getWavChunkId(WaveFileFormat::RIFF), 4)) {
            if(!memcmp(header.chunk_id, getWavChunkId(WaveFileFormat::WaveIR), 4)) {
                is_wave_ir = true;
            }
            else {
                std::string str(header.chunk_id, header.chunk_id+4);
                LG(ERR, "unhandled wav chunk_id '%s'", str.c_str());
                return false;
            }
        }
    }
    ReadData(&header.chunk_size, sizeof(int32_t), 1);
    ReadData(&header.format[0], 4, 1);
    {
        if(is_wave_ir) {
            if(memcmp(header.format, "ver1", 4)) {
                std::string str(header.format, header.format+4);
                LG(ERR, "unhandled wir format '%s'", str.c_str());
                return false;
            }
        }
        else if(memcmp(header.format, "WAVE", 4)) {
            std::string str(header.format, header.format+4);
            LG(ERR, "unhandled wav format '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.subchunk1_id[0], 4, 1);
    {
        if(memcmp(header.subchunk1_id, "fmt ", 4)) {
            std::string str(header.subchunk1_id, header.subchunk1_id+4);
            LG(ERR, "unhandled wav subchunk1_id '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.subchunk1_size, sizeof(int32_t), 1);
    ReadData(&header.audio_format, sizeof(uint16_t), 1);
    if(!isFormatSupported(header.audio_format)) {
        LG(ERR, "unhandled audio format %d", header.audio_format);
        return false;
    }
    ReadData(&header.num_channels, sizeof(int16_t), 1);
    ReadData(&header.sample_rate, sizeof(int32_t), 1);
    ReadData(&header.byte_rate, sizeof(int32_t), 1);
    ReadData(&header.block_align, sizeof(int16_t), 1);
    ReadData(&header.bits_per_sample, sizeof(int16_t), 1);


    if(is_wave_ir) {
        //
        // wir format found here : http://freeverb3vst.osdn.jp/tips.shtml says
        // encoding is IEEE_float 32
        //
        if(header.bits_per_sample == 23) {
            // handle a bug (might be an intentional bug to make it harder to decode?)
            header.bits_per_sample = 32;
        }
        else if(header.bits_per_sample != 32) {
            LG(ERR, "invalid bits_per_sample in wir header : %d", header.bits_per_sample);
            return false;
        }
    }

    // skip irrelevant subchunks, until "data" subchunk

    while(1) {
        ReadData(&header.subchunk2_id[0], 4, 1);
        ReadData(&header.subchunk2_size, sizeof(int32_t), 1);
        {
            if(!memcmp(header.subchunk2_id, "data", 4)) {
                break;
            }
            if(!memcmp(header.subchunk2_id, "fact", 4)) {
                // we skip that, cf. http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/WAVE/WAVE.html
                // contains the number of samples per channel
            }
            else if(!memcmp(header.subchunk2_id, "PEAK", 4)) {
            }
            else {
                LG(ERR, "unrecognized wav subchunk2_id '%s'. Contact your support team.",
                   std::string(header.subchunk2_id,
                               header.subchunk2_id+4).c_str());
                return false;
            }
            std::vector<uint8_t> skipped(header.subchunk2_size);
            ReadData(skipped.data(), skipped.size(), 1);
        }
    }

    auto s = header.getSampleSize();
    if(s < 2 || s > 4) {
        LG(ERR, "unhandled sample size %d", header.getSampleSize());
        return false;
    }
    return true;
}

namespace imajuscule {
    namespace audio {
        void makeDescription(std::string &desc, int16_t num_channels, float length_in_seconds, int32_t sample_rate) {
            desc.clear();
            desc.reserve(14 + 10);

            makeDescription(desc, num_channels, length_in_seconds);

            // to avoid float rounding error when doing '/ 1000.f' :
            auto s1 = sample_rate/1000;
            auto s2 = sample_rate - 1000 * s1;
            desc += " " + NumTraits<float>::toSignificantString(s1);
            if(s2) {
                auto str2 = NumTraits<float>::toSignificantString(s2);
                while(!str2.empty()) {
                    if(str2.back() != '0') {
                        break;
                    }
                    str2.pop_back();
                }
                desc += "." + str2;
            }
            desc += " kHz";
        }

        void makeDescription(std::string &desc, int16_t num_channels, float length_in_seconds) {
            desc.clear();
            desc.reserve(14);

            desc.append(num_channels, '.');
            desc += " ";
            constexpr auto n_decimals_total = 2;
            auto str_sec = NumTraits<float>::to_string_with_precision(length_in_seconds, n_decimals_total);
            {
                auto i = str_sec.find('.');
                if(i == std::string::npos) {
                    str_sec.push_back('.');
                }
            }
            {
                auto i = str_sec.find('.');
                int n_decimals = str_sec.size() - i - 1;
                if(n_decimals < n_decimals_total) {
                    str_sec.append(n_decimals_total - n_decimals, '0');
                }
            }
            desc += str_sec + " s";
        }
    }
}

void WAVPCMHeader::makeDescription(std::string & desc, bool with_sample_rate) const {
    if(with_sample_rate) {
        ::makeDescription(desc, num_channels, getLengthInSeconds(), sample_rate);
    }
    else {
        ::makeDescription(desc, num_channels, getLengthInSeconds());
    }
}

bool WAVReader::HasMore() const {
    Assert(audio_bytes_read <= header.subchunk2_size);
    //LG(INFO, "%d %d", audio_bytes_read, header.subchunk2_size);
    return audio_bytes_read < header.subchunk2_size;
}

namespace imajuscule::audio {
  bool getConvolutionReverbSignature(std::string const & dirname, std::string const & filename, spaceResponse_t & r) {
    WAVReader reader(dirname, filename);

    auto res = reader.Initialize();

    if(ILE_SUCCESS != res) {
      LG(WARN, "Cannot read '%s' as a '.wav' file. If the file exists in '%s', it might be corrupted.", filename.c_str(), dirname.c_str());
      return false;
    }

    r.nChannels = reader.countChannels();
    r.sampleRate = reader.getSampleRate();
    r.nFrames = reader.countFrames();
    r.sampleSize = reader.getSampleSize();
    r.lengthInSeconds = reader.getLengthInSeconds();
    return true;
  }
}
