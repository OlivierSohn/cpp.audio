
namespace imajuscule {
    namespace audio {
        
        enum class ParamType {
            Boolean,
            Enum,
            Continuous,
            Discrete
        };
        
        struct ParamSpec {
            using enumTraversal = imajuscule::enumTraversal;
            
            ParamSpec(const char * name) :
            name(name),
            type(ParamType::Boolean)
            {}
            
            ParamSpec(const char * name, enumTraversal const & e) :
            name(name),
            type(ParamType::Enum),
            enum_(&e) {}
            
            ParamSpec(const char * name, int Min, int Max) :
            name(name),
            min_(Min),
            max_(Max),
            type(ParamType::Discrete),
            nsteps(Max-Min)
            {}
            
            ParamSpec(const char * name, float Min, float Max, int nsteps = 0) :
            name(name),
            min_(Min),
            max_(Max),
            type(nsteps ?
                 ParamType::Discrete:
                 ParamType::Continuous) ,
            nsteps(nsteps)
            {}
            
            const char * name;
            ParamType type;
            float min_, max_;
            int nsteps;
            enumTraversal const * enum_ = {};
        };
    
    }
}
