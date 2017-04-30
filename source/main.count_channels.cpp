
#include "public.h"

namespace imajuscule {
    namespace audio {
        int countChannels(DirectoryPath const & dir, FileName const & source) {
            using namespace std;
            
            WAVReader reader(dir, source);
            {
                auto res = reader.Initialize();
                
                if(ILE_SUCCESS != res) {
                    return 0;
                }
            }
            return reader.countChannels();
        }
    }
}

void printUsage() {
    using namespace std;
    cout << "- mod_wav usage : " << endl;
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
        cerr << "count_channel takes 1 argument, " << argc-1 << " given" << endl;
        printUsage();
        throw;
    }
    
    DirectoryPath dir;
    FileName source;
    if(!split_path(argv[1], dir, source)) {
        cerr << "invalid path : " << argv[1] << endl;
        throw;
    }
    
    cout << countChannels(dir, source);
    return 0;
}
