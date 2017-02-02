
namespace imajuscule {
    namespace midi {
        // tuning in cents
        constexpr float tuned_note(int pitch, float tuning) {
            return pitch + tuning*0.01f;
        }
    }
   
}
