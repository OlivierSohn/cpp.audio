

namespace imajuscule {
    namespace audio {
        template<int SAMPLE_SIZE>
        void wir_2_wav(DirectoryPath const & dir, FileName filename, WAVReader & reader) {
            using namespace std;
            using namespace imajuscule;
            using namespace imajuscule::audio;

            auto n_channels = reader.countChannels();
            auto n_frames = reader.countFrames();
            
            using T = typename FloatTypeForSampleSize<SAMPLE_SIZE>::type;
            static_assert(sizeof(T) == SAMPLE_SIZE, "");
            
            std::vector<T> buf(n_frames*n_channels);
            
            {
                auto end = reader.Read(buf.begin(), buf.end());
                A(end == buf.end());
                A(!reader.HasMore());
            }
            

            auto header = pcm(reader.getFormat(),
                              buf.size() / n_channels,
                              reader.getSampleRate(),
                              NChannels(n_channels),
                              AudioSample<T>::format);

            auto start_pos = filename.find(".wir");
            if(start_pos == std::string::npos) {
                cerr << "invalid wir filename, could not find .wir extension" << endl;
                throw;
            }
            filename.replace(start_pos, 4, ".wav");
            
            WAVWriter writer(dir, filename, header);
            auto res = writer.Initialize();
            
            if(ILE_SUCCESS != res) {
                cerr << "could not create file" << endl;
                throw;
            }
            
            for(auto c : buf) {
                writer.writeSample(c);
            }
        }
    }
}

void printUsage() {
    using namespace std;
    cout << "- wir_2_wav usage : " << endl;
    cout << "- pass one argument containing the path to the wir file." << endl;
}

/*
 * .wir files are 'waves' impulse response files
 */
int main(int argc, const char * argv[]) {
    using namespace std;
    using namespace imajuscule;
    using namespace imajuscule::audio;
    
    if(argc != 2) {
        cerr << "wir_2_wav takes 1 argument, " << argc-1 << " given" << endl;
        printUsage();
        throw;
    }
    
    DirectoryPath dir;
    FileName filename;
    if(!split_path(argv[1], dir, filename)) {
        cerr << "invalid path : " << argv[1] << endl;
        throw;
    }
    
    WAVReader reader(dir, filename);
    
    auto res = reader.Initialize();
    
    if(ILE_SUCCESS != res) {
        cerr << "error opening file for read" << endl;
        throw;
    }
    
    if(WaveFileFormat::WaveIR != reader.getFileFormat()) {
        cerr << "file format is not WaveIR" << endl;
        throw;
    }
    
    switch(reader.getSampleSize()) {
        case 4:
            wir_2_wav<4>(dir, std::move(filename), reader);
            break;
        case 8:
            wir_2_wav<8>(dir, std::move(filename), reader);
            break;
        default:
            throw;
    }
    

    return 0;
}
