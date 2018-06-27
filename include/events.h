
namespace imajuscule {
    namespace audio {

        enum class onEventResult {
            OK = 0,
            DROPPED_NOTE
        };

        constexpr auto event_position_infinite = std::numeric_limits<int>::max();

        template<typename EventIterator>
        int getNextEventPosition(EventIterator it, EventIterator end) {
            if(it == end) {
                return event_position_infinite;
            }
            typename EventIterator::object event;
            it.dereference(event);
            return event.sampleOffset;
        }

        template<typename IEventList>
        struct EventOf; // undefined

        template<typename IEventList>
        struct EventIterator {
            using Event = typename EventOf<IEventList>::type;

            using iterator = EventIterator;
            using object = Event;
            using reference = Event &;

            EventIterator(IEventList * l) : list(l) {}
            void asEnd() {
                if(list) {
                    cur = list->getEventCount();
                }
            }

            iterator& operator++() { // prefix increment
                ++cur;
                return *this;
            }

            bool operator ==(iterator const & o) const {
                return list == o.list && cur == o.cur;
            }
            void dereference(reference r) const {
                assert(list);
                list->getEvent(cur, r);
            }

            static EventIterator end(IEventList * l);
        private:
            int cur = 0;
            IEventList * list;
        };

        template<typename IEventList>
        static inline EventIterator<IEventList> begin(IEventList*l) {
            return {l};
        }

        template<typename IEventList>
        static inline EventIterator<IEventList> end_(IEventList*l) {
            EventIterator<IEventList> ret{l};
            ret.asEnd();
            return ret;
        }

    }
} // namespace
