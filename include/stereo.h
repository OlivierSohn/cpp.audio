namespace imajuscule {
    
    struct StereoGain {
        double left, right;
        
        StereoGain opposite() const { return { right, left }; }
    };
    
    static inline StereoGain stereo(double pan) {
        pan = std::min(pan, 1.);
        pan = std::max(pan, -1.);
        
        // http://dsp.stackexchange.com/questions/21691/algorithm-to-pan-audio
        auto angle = 0.25 * (pan + 1.) * static_cast<double>(M_PI);
        
        return {cos(angle), sin(angle)};
    }
    
}
