namespace imajuscule {

struct StereoGain {
  static constexpr double quarter_pi = 0.25 * static_cast<double>(M_PI);
  
  double left() const { return gains[0]; }
  double right() const { return gains[1]; }
  double gain(int i) const {
    return gains[i];
  }

  StereoGain opposite() const { return { right(), left() }; }

  std::array<double, 2> gains{};
};


static inline StereoGain stereo(double pan) {
  pan = std::min(pan, 1.);
  pan = std::max(pan, -1.);
  
  // http://dsp.stackexchange.com/questions/21691/algorithm-to-pan-audio
  
  auto angle = StereoGain::quarter_pi * (pan + 1.);
  
  return {std::cos(angle), std::sin(angle)};
}

}
