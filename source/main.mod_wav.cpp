
#include "public.h"

namespace imajuscule {
    namespace audio {
        void mod_wav(DirectoryPath const & dir, FileName const & source, FileName const & dest) {
            filter_frames(dir, source, dest, [](auto & frame) {
                using T = typename std::remove_reference<decltype(frame)>::type::value_type;
                
                static std::vector<slidingAverage<T>> avgs;
                if(avgs.empty()) {
                    for(auto i=0; i<frame.size(); ++i) {
                        avgs.emplace_back(20);
                    }
                }
                int i=0;
                for(auto &c: frame) {
                    avgs[i].feed(c);
                    c = avgs[i].compute();
                    ++i;
                }
                return true;
            });
        }
        
        void rewrite_wav(DirectoryPath const & dir, FileName const & source, FileName const & dest) {
            rewrite_wav(dir, source, dest, [](auto & deinterlaced) {
                using CONTAINER = typename std::remove_reference<decltype(deinterlaced)>::type::value_type;
                using T = typename CONTAINER::value_type;
                
                T Max = {};
                
                for(auto & c: deinterlaced) {
                    CONTAINER input;
                    input.resize(2*c.size());
                    std::copy(c.begin(), c.end(), input.begin());
                    
                    CONTAINER output;
                    output.resize(c.size());
                    accelerate::API<T>::f_conv(&*input.begin(), 1,
                                               //&*(c.end()-1), -1,
                                               &*(c.begin()), 1,
                                               &*(output.begin()), 1, output.size(), c.size());
                    c = output;
                    auto it_max = std::max_element(c.begin(), c.end(), [](auto a, auto b) { return std::abs(a) < std::abs(b); });
                    Max = std::max(Max, *it_max);
                }
                LG(INFO, "Max : %f", Max);
                Max = 1/Max;
                for(auto & c: deinterlaced) {
                    for(auto & v : c) {
                        v *= Max;
                    }
                }
            });
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
        cerr << "mod_wav takes 1 argument, " << argc-1 << " given" << endl;
        printUsage();
        throw;
    }
    
    DirectoryPath dir;
    FileName source;
    if(!split_path(argv[1], dir, source)) {
        cerr << "invalid path : " << argv[1] << endl;
        throw;
    }
    
    FileName dest = "mod_" + source;

    //mod_wav(dir, source, dest);
    rewrite_wav(dir, source, dest);

    return 0;
}
