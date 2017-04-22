

namespace imajuscule {
    namespace audio {
        
        static inline void join_non_zeros(DirectoryPath const & dir, FileName const & filename) {
            filter_frames(dir,
                          filename,
                          "joined_" + filename,
                          [](auto & frame) {
                for(auto c : frame) {
                    if(0 != c) {
                        return true;
                    }
                }
                return false;
            } );
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
   
    join_non_zeros(dir, filename);

    return 0;
}
