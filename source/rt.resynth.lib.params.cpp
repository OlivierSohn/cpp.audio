
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


inline void readPresetFromFile(std::string const & str,
                               Preset & preset) {
  json j;
  {
    std::ifstream file;
    file.open (str);
    file >> j;
  }
  from_json(j, preset);
}

inline void writePresetToFile(std::string const & str,
                              Preset const & preset) {
  std::ofstream file;
  file.open (str);
  
  json j;
  to_json(j, preset);
  
  file << std::setw(2) << j << std::endl;
}

}

