
namespace imajuscule {
    namespace audio {
     
        struct Program {
            using ARRAY = std::vector<float>;
            
            std::string name;
            ARRAY params;
        };
        using Programs = std::vector<Program>;
        
        struct ProgramsI {
            ProgramsI(std::initializer_list<Program> && i) : v(std::move(i)){
                std::sort(v.begin(), v.end(), [](auto const & p1, auto const & p2) { return p1.name < p2.name; });
            }
            std::vector<Program> v;
        };
        
    }
} // namespace
