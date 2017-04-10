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
    ReadData(&header.chunk_id[0], 4, 1);
    {
        std::string str(header.chunk_id, header.chunk_id+4);
        if(str != "RIFF") {
            LG(ERR, "unhandled wav chunk_id '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.chunk_size, sizeof(int), 1);
    ReadData(&header.format[0], 4, 1);
    {
        std::string str(header.format, header.format+4);
        if(str != "WAVE") {
            LG(ERR, "unhandled wav format '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.subchunk1_id[0], 4, 1);
    {
        std::string str(header.subchunk1_id, header.subchunk1_id+4);
        if(str != "fmt ") {
            LG(ERR, "unhandled wav subchunk1_id '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.subchunk1_size, sizeof(int), 1);
    ReadData(&header.audio_format, sizeof(short int), 1);
    ReadData(&header.num_channels, sizeof(short int), 1);
    ReadData(&header.sample_rate, sizeof(int), 1);
    ReadData(&header.byte_rate, sizeof(int), 1);
    ReadData(&header.block_align, sizeof(short int), 1);
    ReadData(&header.bits_per_sample, sizeof(short int), 1);
    ReadData(&header.subchunk2_id[0], 4, 1);
    {
        std::string str(header.subchunk2_id, header.subchunk2_id+4);
        if(str != "data") {
            LG(ERR, "unhandled wav subchunk2_id '%s'", str.c_str());
            return false;
        }
    }
    ReadData(&header.subchunk2_size, sizeof(int), 1);
    
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
