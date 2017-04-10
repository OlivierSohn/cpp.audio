

using namespace imajuscule;

void WAVWriter::DoUpdateFileHeader()
{
    WriteData(&header.chunk_id[0], 4, 1);
    WriteData(&header.chunk_size, sizeof(int), 1);
    WriteData(&header.format[0], 4, 1);
    WriteData(&header.subchunk1_id[0], 4, 1);
    WriteData(&header.subchunk1_size, sizeof(int), 1);
    WriteData(&header.audio_format, sizeof(short int), 1);
    WriteData(&header.num_channels, sizeof(short int), 1);
    WriteData(&header.sample_rate, sizeof(int), 1);
    WriteData(&header.byte_rate, sizeof(int), 1);
    WriteData(&header.block_align, sizeof(short int), 1);
    WriteData(&header.bits_per_sample, sizeof(short int), 1);
    WriteData(&header.subchunk2_id[0], 4, 1);
    WriteData(&header.subchunk2_size, sizeof(int), 1);
}
