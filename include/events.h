
namespace imajuscule {
    namespace audio {
        
        enum class onEventResult {
            OK = 0,
            UNHANDLED,
            DROPPED_NOTE
        };
        
        static constexpr auto event_position_infinite = std::numeric_limits<int>::max();
        
        template<typename EventIterator>
        int getNextEventPosition(EventIterator it, EventIterator end) {
            if(it == end) {
                return event_position_infinite;
            }
            typename EventIterator::object event;
            it.dereference(event);
            return event.sampleOffset;
        }
        
    }
} // namespace
