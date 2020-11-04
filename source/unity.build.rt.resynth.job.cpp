
#include "unity.build.rt.resynth.lib.cpp"


int main(int argc, char **argv) {
  using namespace imajuscule::audio::rtresynth;

  if (argc != 2) {
    throw std::runtime_error("wrong number of args");
  }

  std::string json_params(argv[1]);
  RtResynthOfflineJobConfig config;
  try {
    readFromJsonFile(json_params,
                     config);
  } catch (std::exception const & e) {
    std::cerr << "readFromJsonFile : " << e.what() << std::endl;
    throw;
  }
  try {
    RtResynthOfflineJob job(config);
    RtResynth resynth(job);
  } catch (std::exception const & e) {
    std::cerr << e.what() << std::endl;
    throw;
  }
  
  return 0;
}
