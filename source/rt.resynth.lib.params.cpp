
namespace imajuscule::audio::rtresynth {

using json = ::nlohmann::json;

template<typename T>
struct Params {
  void clear() {
    params.clear();
  }
  
  void set(std::string const & name, T const & val) {
    if (params.count(name)) {
      throw std::runtime_error("Duplicate param key");
    }
    params[name] = val;
  }
  
  template<typename U>
  void read(std::string const & name,
            std::atomic<U> & val) const {
    auto it = params.find(name);
    if (it == params.end()) {
      std::cerr << "Preset param '" << name << "' not found" << std::endl;
      return;
    }
    val = static_cast<U>(it->second);
  }

  void tojson(json& j) const {
    j = json(params);
  }
  
  void fromjson(const json& j) {
    params = j.get<Container>();
  }

private:
  using Key = std::string;
  using Container = std::unordered_map<Key,T>;
  Container params;
};

template<typename T>
void to_json(json& j, const Params<T>& p) {
  p.tojson(j);
}

template<typename T>
void from_json(const json& j, Params<T>& p) {
  p.fromjson(j);
}

/*
 Represents a preset.
 */
struct Preset {
  void clear() {
    bool_params.clear();
    enum_params.clear();
    int32_params.clear();
    uint64_params.clear();
    float_params.clear();
  }
  
  void set(std::string const & name, bool val) {
    bool_params.set(name, val);
  }
  template<typename E>
  std::enable_if_t<std::is_enum_v<E>>
  set(std::string const & name,
      E val) {
    enum_params.set(name,
                    to_underlying(val));
  }
  void set(std::string const & name, int32_t val) {
    int32_params.set(name, val);
  }
  void set(std::string const & name, uint64_t val) {
    uint64_params.set(name, val);
  }
  void set(std::string const & name, float val) {
    float_params.set(name, val);
  }

  void read(std::string const & name, std::atomic_bool & val) const {
    bool_params.read(name, val);
  }
  template<typename E>
  std::enable_if_t<std::is_enum_v<E>>
  read(std::string const & name,
      std::atomic<E> & val) const {
    enum_params.read(name,
                     val);
  }
  void read(std::string const & name, std::atomic<int32_t> & val) const {
    int32_params.read(name, val);
  }
  void read(std::string const & name, std::atomic<uint64_t> & val) const {
    uint64_params.read(name, val);
  }
  void read(std::string const & name, std::atomic<float> & val) const {
    float_params.read(name, val);
  }

  void tojson(json& j) const {
    json b;
    json e;
    json i;
    json u;
    json f;
    
    to_json(b, bool_params);
    to_json(e, enum_params);
    to_json(i, int32_params);
    to_json(u, uint64_params);
    to_json(f, float_params);

    j = json{
      {"bool_params", b},
      {"enum_params", e},
      {"int32_params", i},
      {"uint64_params", u},
      {"float_params", f}
    };
  }

  void fromjson(const json& j) {
    bool_params = j.at("bool_params");
    enum_params = j.at("enum_params");
    int32_params = j.at("int32_params");
    uint64_params = j.at("uint64_params");
    float_params = j.at("float_params");
  }

private:
  Params<bool> bool_params;
  Params<int32_t> enum_params;
  Params<int32_t> int32_params;
  Params<uint64_t> uint64_params;
  Params<float> float_params;
};

inline void to_json(json& j, const Preset& p) {
  p.tojson(j);
}

inline void from_json(const json& j, Preset& p) {
  p.fromjson(j);
}

enum class Postprocessing {
  None,
  Limit
};

inline std::ostream & operator <<(std::ostream & os, Postprocessing const& p) {
  switch(p) {
    case Postprocessing::None:
      os << "none";
      break;
    case Postprocessing::Limit:
      os << "limit";
      break;
  }
  return os;
}
inline void from_string(std::string const & str, Postprocessing & p) {
  if (str == "none") {
    p = Postprocessing::None;
  } else if(str == "limit") {
    p = Postprocessing::Limit;
  } else {
    throw std::runtime_error("bad encoding");
  }
}

class RtResynthOfflineJob;

/*
 Represents a job config for RtResynth.
 */
struct RtResynthOfflineJobConfig {
  friend class RtResynthOfflineJob;
  
  void tojson(json& j) const {
    std::ostringstream os_post;
    os_post << post;
    
    j = json{
      {"preset_file", preset_file},
      {"input_voice_file", input_voice_file},
      {"input_carrier_file", input_carrier_file},
      {"output_file", output_file},
      {"post", os_post.str()}
    };
  }
  
  void fromjson(const json& j) {
    preset_file = j.at("preset_file");
    input_voice_file = j.at("input_voice_file");
    input_carrier_file = j.at("input_carrier_file");
    output_file = j.at("output_file");
    std::string str = j.at("post");
    from_string(str, post);
  }
  
private:
  std::string preset_file;
  Postprocessing post;
  std::string input_voice_file, input_carrier_file;
  std::string output_file;
};

inline void to_json(json& j, const RtResynthOfflineJobConfig& p) {
  p.tojson(j);
}

inline void from_json(const json& j, RtResynthOfflineJobConfig& p) {
  p.fromjson(j);
}


template<typename T>
void readFromJsonFile(std::string const & str,
                      T & obj) {
  json j;
  {
    std::ifstream file;
    file.open (str);
    file >> j;
  }
  from_json(j, obj);
}

template<typename T>
void writeToJsonFile(std::string const & str,
                     T const & obj) {
  std::ofstream file;
  file.open (str);
  
  json j;
  to_json(j, obj);
  
  file << std::setw(2) << j << std::endl;
}


struct RtResynthOfflineJob {
  using output_T = float; // TODO use double
  
  RtResynthOfflineJob(RtResynthOfflineJobConfig const & c)
  : post(c.post) {
    try {
      readFromJsonFile(c.preset_file,
                       preset);
    } catch(std::exception const & e) {
      std::cerr << "Reading preset file '" << c.preset_file << "': " << e.what() << std::endl;
      throw;
    }
    
    load_reader(c.input_voice_file,
                input_voice_reader);
    load_reader(c.input_carrier_file,
                input_carrier_reader);
    
    if (input_voice_reader) {
      sample_rate = input_voice_reader->getSampleRate();
    }
    if (input_carrier_reader) {
      auto r = input_voice_reader->getSampleRate();
      if (sample_rate && *sample_rate != r) {
        throw std::runtime_error("sample rate mismatch between carrier and voice");
      }
      sample_rate = r;
    }
    if (!sample_rate) {
      throw std::runtime_error("must havee at least one of carrier or voice");
    }

    make_writer(c.output_file,
                output_writer);
  }
  
  int get_samplerate() const {
    return *sample_rate;
  }
  
  bool has_more_carrier() const {
    return input_carrier_reader && input_carrier_reader->HasMore();
  }

  bool has_more_voice() const {
    return input_voice_reader && input_voice_reader->HasMore();
  }

  template<typename T>
  void read_carrier(T & v) {
    if (input_carrier_reader && input_carrier_reader->HasMore()) {
      v = input_carrier_reader->ReadAsOneFloat<T>();
    } else {
      v = 0;
    }
  }

  template<typename T>
  void read_voice(T & v) {
    if (input_voice_reader && input_voice_reader->HasMore()) {
      v = input_voice_reader->ReadAsOneFloat<T>();
    } else {
      v = 0;
    }
  }
  
  int count_outputs() const {
    Assert(output_writer);
    return output_writer->count_channels();
  }

  void write_output(output_T * v, int nSamples) {
    for (int i=0; i<nSamples; ++i) {
      output_writer->writeSample(v[i]);
    }
  }

  Preset const & getPreset() const {
    return preset;
  }
  
  Postprocessing getPostprocessing() const {
    return post;
  }

private:
  Preset preset;
  Postprocessing post;
  std::unique_ptr<WAVReader> input_carrier_reader;
  std::unique_ptr<WAVReader> input_voice_reader;
  std::unique_ptr<WAVWriter> output_writer;
  std::optional<int> sample_rate;

  void make_writer(std::string const & filename,
                   std::unique_ptr<WAVWriter> & writer) {
    if (filename.empty()) {
      throw std::runtime_error("no output file");
    }
    std::filesystem::path p(filename);
    
    
    auto header = pcm(WaveFormat::IEEE_FLOAT,
                      *sample_rate,
                      CountChannels{2},
                      AudioSample<output_T>::format);
    
    writer = std::make_unique<WAVWriter>(p.parent_path().native(),
                                         p.filename(),
                                         header);
    
    try {
      writer->Initialize();
    }
    catch(std::exception const & e) {
      throw std::runtime_error("failed to write wav file");
    }
  }
  void load_reader(std::string const & filename,
                   std::unique_ptr<WAVReader> & reader) {
    if (filename.empty()) {
      return;
    }
    std::filesystem::path p(filename);
    
    reader = std::make_unique<WAVReader>(p.parent_path().native(),
                                         p.filename());
    
    try {
      reader->Initialize();
    }
    catch(std::exception const & e) {
      throw std::runtime_error("failed to read wav file '" + filename + "':" + e.what());
    }
    
    if (reader->countChannels() != 1) {
      throw std::runtime_error("single channel only");
    }
  }
  
};

}

