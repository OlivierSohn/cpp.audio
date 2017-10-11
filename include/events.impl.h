
namespace imajuscule {
    namespace audio {
        
        struct NoteOnEvent
        {
            int16_t channel;			///< channel index in event bus
            int16_t pitch;			///< range [0, 127] = [C-2, G8] with A3=440Hz
            float tuning;			///< 1.f = +1 cent, -1.f = -1 cent
            float velocity;			///< range [0.0, 1.0]
            int32_t length;           ///< in sample frames (optional, Note Off has to follow in any case!)
            int32_t noteId;			///< note identifier (if not available then -1)
        };
        
        //------------------------------------------------------------------------
        /** Note-off event specific data. Used in \ref Event (union)*/
        struct NoteOffEvent
        {
            int16_t channel;			///< channel index in event bus
            int16_t pitch;			///< range [0, 127] = [C-2, G8] with A3=440Hz
            float velocity;			///< range [0.0, 1.0]
            int32_t noteId;			///< associated noteOn identifier (if not available then -1)
            float tuning;			///< 1.f = +1 cent, -1.f = -1 cent
        };
        
        struct Event
        {
            /**  Event Types - used for Event::type */
            enum EventTypes
            {
                kNoteOnEvent = 0,			///< is \ref NoteOnEvent
                kNoteOffEvent,				///< is \ref NoteOffEvent
            };
            
            uint16_t type;				///< a value from \ref EventTypes
            
            union
            {
                NoteOnEvent noteOn;								///< type == kNoteOnEvent
                NoteOffEvent noteOff;							///< type == kNoteOffEvent
            };
        };
        
        
        struct IEventList
        {
            int getEventCount () const { return events.size(); }
            
            void getEvent (int32_t index, Event& e /*out*/) {
                Assert(index < events.size());
                e = events[index];
            }
            
            std::vector<Event> events;
        };
        
        template<>
        struct EventOf<IEventList> {
            using type = Event;
        };

        using EventIteratorImpl = EventIterator<IEventList>;
        
        enum SymbolicSampleSizes
        {
            kSample32,		///< 32-bit precision
            kSample64		///< 64-bit precision
        };
        
        struct ProcessData
        {
            ProcessData ()
            : processMode (0), symbolicSampleSize (kSample32), numSamples (0), numInputs (0)
            , numOutputs (0)
            , inputEvents (0), outputEvents (0) {}
            
            //------------------------------------------------------------------------
            int32_t processMode;			///< processing mode - value of \ref ProcessModes
            int32_t symbolicSampleSize;   ///< sample size - value of \ref SymbolicSampleSizes
            int32_t numSamples;			///< number of samples to process
            int32_t numInputs;			///< number of audio input buses
            int32_t numOutputs;			///< number of audio output buses
            IEventList* inputEvents;				///< incoming events for this block (optional)
            IEventList* outputEvents;				///< outgoing events for this block (optional)
        };
        
        template<int nAudioOut, audio::SoundEngineMode MODE>
        using Voice = imajuscule::audio::voice::Impl_<nAudioOut, MODE, std::vector<float>, EventIteratorImpl, NoteOnEvent, NoteOffEvent, ProcessData>;
        
        
        ////////////////////
        
        struct Voicing {
            Voicing(int prog, int16_t midiPitch, float gain, float pan, bool random, int seed) : program(prog), pan(pan), gain(gain), random(random), seed(seed), midiPitch(midiPitch) {}
            int program;
            int16_t midiPitch;
            bool random;
            int seed;
            float gain;
            float pan;
        };
        
        template<typename Voice, typename OutputData>
        void playOneThing(Voice & v, OutputData & out, Voicing const & b) {
            
            v.initializeSlow(); // does something only the 1st time
            v.useProgram(b.program); // keep it first as it reinitializes params
            v.set_random_pan(false);
            v.set_random(b.random);
            if(!b.random) {
                v.set_seed(b.seed);
            }
            v.set_gain(b.gain);
            v.set_pan(b.pan);
            v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!
                        
            Event e;
            e.type = Event::kNoteOnEvent;
            e.noteOn = NoteOnEvent{0,b.midiPitch,0,1,0,0};
            IEventList l{{e}};
            v.onEvent(begin(&l), out);
        }
    }
}
