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


bool WAVReader::HasMore() const {
    A(audio_bytes_read <= header.subchunk2_size);
    //LG(INFO, "%d %d", audio_bytes_read, header.subchunk2_size);
    return audio_bytes_read < header.subchunk2_size;
}
