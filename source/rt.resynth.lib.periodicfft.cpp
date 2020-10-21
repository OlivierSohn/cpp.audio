
namespace imajuscule::audio::rtresynth {

void setDuration(std::optional<float> secs,
                 std::atomic<float> & atom) {
  if (secs) {
    atom = *secs;
  } else {
    atom = -1.f;
  }
}

struct PeriodicFft {
  static int constexpr zero_padding_factor = 1;
  static int constexpr windowed_signal_stride = 1;

  PeriodicFft() {
    frequencies_sqmag.frequencies_sqmag.reserve(200);
  }
  
  using OnFftResult = folly::Function<void(int const, FrequenciesSqMag<double> const &)>;
  using GetWindowSizeFunc = folly::Function<int()>;
  using GetWindowCenterStrideFramesFunc = folly::Function<int()>;
  
  // if window size (=get_window_size()) never changes, no dynamic memory allocation will happen outside this initialization method.
  void setLambdas(GetWindowSizeFunc && get_window_size,
                  GetWindowCenterStrideFramesFunc && get_window_center_stride_frames,
                  OnFftResult && on_fft_result) {
    if (get_window_size.heapAllocatedMemory()) {
      throw std::runtime_error("no allocation is allowed in functors");
    }
    if (get_window_center_stride_frames.heapAllocatedMemory()) {
      throw std::runtime_error("no allocation is allowed in functors");
    }
    if (on_fft_result.heapAllocatedMemory()) {
      throw std::runtime_error("no allocation is allowed in functors");
    }
    this->get_window_size = std::move(get_window_size);
    this->on_fft_result = std::move(on_fft_result);
    this->get_window_center_stride_frames = std::move(get_window_center_stride_frames);
    
    ignore_frames = 0;
    // force initialization
    init_data(true);
  }
  
  void feed(float const sample) {
    if (ignore_frames > 0) {
      --ignore_frames;
      return;
    }
    
    samples[end] = sample;
    ++ end;
    if (end == window_size_) {
      end = 0;
      onFullBuffer();
    }
  }
  
  void on_dropped_frames(int count) {
    ignore_frames -= count;
  }
  
  std::optional<float> getDurationFft() {
    float const f = dt_fft_seconds;
    if (f < 0) {
      return {};
    }
    return f;
  }
  std::optional<float> getDurationCopy() {
    float const f = dt_copy_seconds;
    if (f < 0) {
      return {};
    }
    return f;
  }
  
private:
  int ignore_frames = 0;
  int end = 0;
  int window_size_ = 0;
  using Sample = double;
  using SampleVector = std::vector<Sample>;
  using SampleVectorConstIterator = SampleVector::const_iterator;
  SampleVector samples;
  
  std::vector<double> half_window;
  
  a64::vector<double> work_vector_signal;
  typename fft::RealFBins_<fft::Fastest, double, a64::Alloc>::type work_vector_freqs;
  
  FrequenciesSqMag<double> frequencies_sqmag;
  
  OnFftResult on_fft_result;
  
  GetWindowSizeFunc get_window_size;
  
  GetWindowCenterStrideFramesFunc get_window_center_stride_frames;
  
  std::atomic<float> dt_fft_seconds = -1.f;
  std::atomic<float> dt_copy_seconds = -1.f;
  
  void init_data(bool const force) {
    window_size_ = get_window_size();
    
    bool reinit_samples = force;
    {
      Assert(0 == window_size_%2);
      if (static_cast<int>(half_window.size()) != window_size_/2) {
        half_gaussian_window<double>(4,
                                     window_size_/2,
                                     half_window);
        normalize_window(half_window);
        Assert(static_cast<int>(half_window.size()) == window_size_/2);
        reinit_samples = true;
      }
    }
    if (reinit_samples) {
      samples.clear();
      samples.resize(window_size_, {});
    }
  }
  
  void onFullBuffer() {
    // The stride between the current buffer and the one to come
    int const window_center_stride = get_window_center_stride_frames();
    
    {
      std::optional<float> duration_fft_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        
        findFrequenciesSqMag<fft::Fastest>(samples.begin(),
                                           samples.begin() + window_size_,
                                           windowed_signal_stride,
                                           half_window,
                                           zero_padding_factor,
                                           work_vector_signal,
                                           work_vector_freqs,
                                           frequencies_sqmag);
      }
      if (dt) duration_fft_seconds = dt->count() / 1000000.;
      setDuration(duration_fft_seconds, dt_fft_seconds);
    }
    
    on_fft_result(window_center_stride,
                  frequencies_sqmag);
    
    init_data(false);
    
    {
      std::optional<float> duration_copy_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        int const windowoverlapp = window_size_ - window_center_stride;
        if (windowoverlapp >= 0) {
          const int offset = window_size_-windowoverlapp;
          for (; end<windowoverlapp; ++end) {
            samples[end] = samples[end + offset];
          }
          ignore_frames = 0;
        } else {
          ignore_frames = -windowoverlapp;
        }
      }
      if (dt) duration_copy_seconds = dt->count() / 1000000.;
      setDuration(duration_copy_seconds, dt_copy_seconds);
    }
  }
};

} // NS
