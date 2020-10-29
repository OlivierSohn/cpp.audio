
namespace imajuscule::audio::rtresynth {

inline void setDuration(std::optional<float> secs,
                 std::atomic<float> & atom) {
  if (secs) {
    atom = *secs;
  } else {
    atom = -1.f;
  }
}

template<typename FftOperation>
struct PeriodicFFT {
  using FftOp = FftOperation;
  using FPT = typename FftOperation::FPT;
  using Result = typename FftOperation::Result;

  template <typename... Args>
  PeriodicFFT(int reserve_for_max_fft_size,
              Args&&... args)
  : fft_op(reserve_for_max_fft_size,
           std::forward<Args>(args)...)
  {
    samples.reserve(reserve_for_max_fft_size);
  }
  
  using OnFftResult = folly::Function<void(int const, Result const &)>;
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
  
    init_data();
    reset_samples();
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
  
  void on_dropped_frames(std::optional<int> count) {
    if (count) {
      ignore_frames -= *count;
      if (ignore_frames >= 0) {
        // Luckily, the dropped frames have been ignored, so we don't need to reset the samples
        return;
      }
    }
    reset_samples();
  }
  
  void reset_samples() {
    samples.clear();
    samples.resize(window_size_, {});
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
  
  auto const & getResults() const {
    return fft_op.get_results();
  }
  
private:
  int ignore_frames = 0;
  int end = 0;
  int window_size_ = 0;
  
  std::vector<FPT> samples;
  
  FftOperation fft_op;
  
  OnFftResult on_fft_result;
  
  GetWindowSizeFunc get_window_size;
  
  GetWindowCenterStrideFramesFunc get_window_center_stride_frames;
  
  std::atomic<float> dt_fft_seconds = -1.f;
  std::atomic<float> dt_copy_seconds = -1.f;
  
  bool init_data() {
    bool changed = false;
    
    int new_size = get_window_size();
    if (new_size != window_size_) {
      window_size_ = new_size;
      // Returning true will zero the samples.
      // But we could be more subtle : if there are enough samples for the window overlap, we could reuse these samples.
      changed = true;
    }
    
    return fft_op.init_data(window_size_) || changed;
  }
  
  void onFullBuffer() {
    // The stride between the current buffer and the one to come
    int const window_center_stride = get_window_center_stride_frames();
    
    {
      std::optional<float> duration_fft_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        
        fft_op(samples.begin(),
               samples.begin() + window_size_);
      }
      if (dt) duration_fft_seconds = dt->count() / 1000000.;
      setDuration(duration_fft_seconds, dt_fft_seconds);
    }
    
    on_fft_result(window_center_stride,
                  fft_op.get_results());
    
    if (init_data()) {
      reset_samples();
    }
    
    Assert(end == 0);

    {
      std::optional<float> duration_copy_seconds;
      std::optional<profiling::CpuDuration> dt;
      {
        profiling::ThreadCPUTimer timer(dt);
        int const windowoverlapp = window_size_ - window_center_stride;
        if (windowoverlapp >= 0) {
          for (; end<windowoverlapp; ++end) {
            samples[end] = samples[end + window_center_stride];
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

template<typename T>
struct FftOperation {
  using FPT = T;
  using fftTag = fft::Fastest;

  using FreqBins = typename fft::RealFBins_<fftTag, T, a64::Alloc>::type;
  using Result = FreqBins;
  
  static int constexpr zero_padding_factor = 1;
  static int constexpr windowed_signal_stride = 1;

  FftOperation(int reserve_for_max_fft_size) {
    int const max_fft_length = get_fft_length_for(reserve_for_max_fft_size,
                                                  zero_padding_factor);
    signal.reserve(max_fft_length);
    result.reserve(max_fft_length);
  }
  
  bool init_data(int const window_size) {
    int const fft_length = get_fft_length_for(window_size,
                                              zero_padding_factor);
    Assert(fft_length <= static_cast<int>(signal.capacity()));
    signal.resize(fft_length);
    result.resize(fft_length);
    return false;
  }
  
  template<typename ITER>
  void operator()(ITER const & it,
                  ITER const & end) {
    using VAL = typename ITER::value_type;
    using Algo = fft::Algo_<fftTag, VAL>;

    int const numSamplesUnstrided = static_cast<int>(std::distance(it, end));
    int const numSamplesStrided = 1 + (numSamplesUnstrided-1) / windowed_signal_stride;
    int const fft_length = get_fft_length_for(numSamplesStrided, zero_padding_factor);
    
    using Contexts = fft::Contexts_<fftTag, VAL>;
    Algo fft(Contexts::getInstance().getBySize(fft_length));
    
    signal.clear();
    reserve_no_shrink(signal,
                      fft_length);
    
    // apply the rectangular window...
    apply_rectangular_window(it, end, windowed_signal_stride, signal);
    
    // ... and pad
    signal.resize(fft_length, 0.);
    
    result.resize(fft_length);
    fft.forward(signal.begin(), result.data(), fft_length);
  }
  
  FreqBins const & get_results() const {
    return result;
  }
  
private:
  a64::vector<T> signal;
  FreqBins result;
};

enum class Window {
  Rectangular,
  Gaussian
};

template<Window window_type, typename T>
struct SqMagFftOperation {
  using FPT = T;
  using fftTag = fft::Fastest;
  
  static int constexpr zero_padding_factor = 1;
  static int constexpr windowed_signal_stride = 1;
  
  using Result = FrequenciesSqMag<T>;
  
  SqMagFftOperation(int reserve_for_max_fft_size) {
    int const max_fft_length = get_fft_length_for(reserve_for_max_fft_size,
                                                  zero_padding_factor);
    
    std::size_t const reserve = fft::capacity_for_unwrap_frequencies_sqmag<fftTag>(frequencies_sqmag.frequencies_sqmag,
                                                                                   max_fft_length);
    frequencies_sqmag.frequencies_sqmag.reserve(reserve);
    
    work_vector_signal.reserve(max_fft_length);
    work_vector_freqs.reserve(max_fft_length);
  }
  
  bool init_data(int const window_size) {
    Assert(0 == window_size % 2);
    int const fft_length = get_fft_length_for(window_size,
                                              zero_padding_factor);
    Assert(fft_length <= work_vector_signal.capacity());
    reserve_no_shrink(work_vector_signal,
                      fft_length);
    reserve_no_shrink(work_vector_freqs,
                      fft_length);
    if (static_cast<int>(half_window.size()) != window_size/2) {
      switch(window_type) {
        case Window::Rectangular:
          half_rectangular_window<T>(window_size/2,
                                     half_window);
          break;
        case Window::Gaussian:
          half_gaussian_window<T>(4,
                                  window_size/2,
                                  half_window);
          break;
      }
      normalize_window(half_window);
      Assert(static_cast<int>(half_window.size()) == window_size/2);
      
      return true;
    }
    return false;
  }
  
  template<typename It>
  void operator()(It const & it,
                  It const & end) {
    findFrequenciesSqMag<fftTag>(it,
                                 end,
                                 windowed_signal_stride,
                                 half_window,
                                 zero_padding_factor,
                                 work_vector_signal,
                                 work_vector_freqs,
                                 frequencies_sqmag);
  }
  
  FrequenciesSqMag<T> const & get_results() const {
    return frequencies_sqmag;
  }
  
private:
  std::vector<T> half_window;
  
  FrequenciesSqMag<T> frequencies_sqmag;
  
  a64::vector<T> work_vector_signal;
  typename fft::RealFBins_<fftTag, T, a64::Alloc>::type work_vector_freqs;
};

} // NS
