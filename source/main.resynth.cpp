

namespace imajuscule::audio {

void testDeduceNotes() {
  int const window_center_stride = 400;
  int const windowed_signal_stride = 1;
  std::string const dir("/Users/Olivier/Documents/Quand je t'aime/Bounced Files/");
  std::string prefix("Quand je t'aime_1");
  
  // The frequency detection is too imprecise with hann window:
  
  //std::vector<double> half_window = half_hann_window<double>(400);
  //std::vector<double> half_window = half_hann_window<double>(4000);
  
  // The frequency detection is very precise with a gaussian window truncated at 8 sigma
  // however, low frequencies are incorrectly detected with such a narrow window, so
  // we can either:
  // - use a window with more points (but lose some temporal precision)
  // - or truncate the window at 4 sigma for example (but lose some frequency precision)
  //
  // TODO we could use a 8-sigma window for high frequencies, and a 4-sigma window for low frequencies,
  // this way we retain temporal precision in the high frequencies, and trade temporal precision
  // for frequency accuracy only for lower frequencies, where this is needed.
  
  std::vector<double> half_window;
  half_gaussian_window<double>(4, 400, half_window);
  //half_gaussian_window<double>(8, 400, half_window);
  //half_gaussian_window<double>(8, 4000, half_window);
  normalize_window(half_window);
  
  int const zero_padding_factor = 1;
  //int const zero_padding_factor = 10;
  auto reader = WAVReader(dir, prefix + ".wav");
  
  std::vector<std::vector<double>> deinterlaced;
  read_wav_as_floats(reader, deinterlaced);
  
  a64::vector<double> v;
  std::vector<DeducedNote<double>> notes = deduceNotesSlow(deinterlaced.begin()->begin(),
                                                           deinterlaced.begin()->end(),
                                                           reader.getSampleRate(),
                                                           windowed_signal_stride,
                                                           half_window,
                                                           window_center_stride,
                                                           zero_padding_factor,
                                                           v,
                                                           0.05776226504);
  
  // formula in http://support.ircam.fr/docs/AudioSculpt/3.0/co/Window%20Size.html
  // uses 5 as constant factor
  const double lowest_detectable_frequency = 4. * reader.getSampleRate() / (2*half_window.size() * windowed_signal_stride);
  std::cout << "lowest detectable freq : " << lowest_detectable_frequency << " Hz" << std::endl;
  
  std::vector<DeducedNote<double>> filtered_notes;
  filtered_notes.reserve(notes.size());
  constexpr double maxDbSpan = 60.;
  Optional<double> max_mag;
  for (auto const & val : notes) {
    if (!max_mag || val.amplitude > *max_mag) {
      max_mag = val.amplitude;
    }
  }
  double const min_allowed_mag = *max_mag - maxDbSpan;
  for (auto const & n : notes) {
    if (n.amplitude < min_allowed_mag) {
      continue;
    }
    if (n.frequency == 0) {
      continue;
    }
    filtered_notes.push_back(n);
  }
  
  drawDeducedNotes(filtered_notes,
                   lowest_detectable_frequency,
                   "/Users/Olivier/" + prefix + ".notes.bmp");
  
  resynth(filtered_notes,
          window_center_stride,
          44100.,
          "/Users/Olivier/",
          prefix + ".resynth.wav");
}

} // NS

int main() {
  imajuscule::audio::testDeduceNotes();
  return 0;
}
