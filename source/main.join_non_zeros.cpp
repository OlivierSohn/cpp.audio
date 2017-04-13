

namespace imajuscule {
    namespace audio {
        template<int SAMPLE_SIZE>
        void join_non_zeros(DirectoryPath const & dir, FileName const & filename, WAVReader & reader) {
            using namespace std;
            using namespace imajuscule;
            using namespace imajuscule::audio;

            auto n_channels = reader.countChannels();
            auto n_frames = reader.countFrames();
            
            using T = typename SignedTypeForSampleSize<SAMPLE_SIZE>::type;
            static_assert(sizeof(T) == SAMPLE_SIZE, "");
            
            std::vector<T> buf(n_frames*n_channels);
            
            {
                auto end = reader.Read(buf.begin(), buf.end());
                A(end == buf.end());
                A(!reader.HasMore());
            }
            
            std::vector<T> buf_nozero;
            
            auto it = buf.begin();
            auto end = buf.end();
            while(it != end) {
                bool ok = false;
                for(int j=0; j<n_channels; j++) {
                    if(0 != *(it+j)) {
                        ok = true;
                        break;
                    }
                }
                if(ok) {
                    for(int j=0; j<n_channels; j++) {
                        buf_nozero.push_back(*(it+j));
                    }
                }
                it += n_channels;
            }
            

            auto header = pcm(reader.getFormat(),
                              buf_nozero.size() / n_channels,
                              reader.getSampleRate(),
                              NChannels(n_channels),
                              AudioSample<T>::format);
            
            WAVWriter writer(dir, "joined_" + filename, header);
            auto res = writer.Initialize();
            
            if(ILE_SUCCESS != res) {
                cerr << "could not create file" << endl;
                throw;
            }
            
            for(auto c : buf_nozero) {
                writer.writeSample(c);
            }
        }
    }
}

void printUsage() {
    using namespace std;
    cout << "- join_non_zero usage : " << endl;
    cout << "- pass one argument containing the path to the wav file." << endl;
}

int main(int argc, const char * argv[]) {
    using namespace std;
    using namespace imajuscule;
    using namespace imajuscule::audio;
    
    if(argc != 2) {
        cerr << "join_non_zero takes 1 argument, " << argc-1 << " given" << endl;
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
    
    switch(reader.getSampleSize()) {
        case 2:
            join_non_zeros<2>(dir, filename, reader);
            break;
        case 3:
            join_non_zeros<3>(dir, filename, reader);
            break;
        case 4:
            join_non_zeros<4>(dir, filename, reader);
            break;
        default:
            throw;
    }
    

    return 0;
}
