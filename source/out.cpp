
using namespace imajuscule;

DelayLine::DelayLine(int size, float attenuation): delay(size,{{}}), it(0), end(size), attenuation(attenuation) {}

void DelayLine::step(SAMPLE *outputBuffer, int nFrames) {
    for( int i=0; i < nFrames; i++ ) {
        auto & d = delay[it];
        for(auto j=0; j<nAudioOut; ++j) {
            auto delayed = d[j];
            d[j] = *outputBuffer;
            *outputBuffer += attenuation * delayed;
            ++outputBuffer;
        }
        ++it;
        if( unlikely(it == end) ) {
            it = 0;
        }
    }
}

