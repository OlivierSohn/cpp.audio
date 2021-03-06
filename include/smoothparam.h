
namespace imajuscule {
    
    template<typename BASE>
    struct SmoothedImpl : public BASE {
        using T = typename BASE::value_type;
        using BASE::cur;
        using BASE::target;
        using BASE::initialized;
        using BASE::changed;
        
        bool hasValue() const { return initialized;}
        
        bool didChange() const { return changed; }
        
        void setTarget(T t) {
            target = t;
            if(!initialized) {
                cur = t;
                initialized = true;
                changed = true;
            }
            else if(target != cur) {
                changed = true;
            }
        }
        
        T get() {
            Assert(initialized);
            return cur;
        }
        
        T operator ()() {
            Assert(initialized);
            return cur;
        }
    };
    
    template<typename T, int nBits>
    struct SmoothedIntegralPolicy {
        using value_type = T;
        
        static_assert(std::is_integral<T>::value);
        
        SmoothedIntegralPolicy() : initialized(false), changed(false) {}
        
        T step() {
            Assert(initialized);
            if(cur > target) {
                --cur;
                changed = true;
            }
            else if(cur < target) {
                ++cur;
                changed = true;
            }
            else {
                changed = false;
            }
            return cur;
        }
    protected:
        bool initialized : 1;
        bool changed : 1;
        T cur : nBits;
        T target: nBits;
    };
    
    template<const float * const increment>
    struct SmoothedFloatPolicy {
        using value_type = float;
        
        SmoothedFloatPolicy() : initialized(false), changed(false) {}
        
        value_type step() {
            Assert(initialized);
            if(cur > target) {
                cur -= *increment;
                if(cur < target) {
                    cur = target;
                }
                changed = true;
            }
            else if(cur < target) {
                cur += *increment;
                if(cur > target) {
                    cur = target;
                }
                changed = true;
            }
            else {
                changed = false;
            }
            return cur;
        }
    protected:
        bool initialized : 1;
        bool changed : 1;
        value_type cur, target;
    };
    
    template<typename T, int nBits>
    using Smoothed = SmoothedImpl<SmoothedIntegralPolicy<T, nBits>>;
    
    constexpr float step_1 = .001f; // works well for normalized parameters
    
    template<const float * const T>
    using SmoothedFloat = SmoothedImpl<SmoothedFloatPolicy<T>>;
    
} // namespaces
