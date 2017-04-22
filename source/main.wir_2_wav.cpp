

namespace imajuscule {
    namespace audio {
        void wir_2_wav(DirectoryPath const & dir, FileName const & source, FileName const & dest) {
            filter_frames(dir, source, dest, [](auto &) { return true; });
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
    FileName source;
    if(!split_path(argv[1], dir, source)) {
        cerr << "invalid path : " << argv[1] << endl;
        throw;
    }
    
    FileName dest = source;
    auto start_pos = dest.find(".wir");
    if(start_pos == std::string::npos) {
        LG(WARN, "invalid wir filename, could not find .wir extension");
        dest.append(".wav");
    }
    else {
        dest.replace(start_pos, 4, ".wav");
    }

    wir_2_wav(dir, source, dest);

    return 0;
}
