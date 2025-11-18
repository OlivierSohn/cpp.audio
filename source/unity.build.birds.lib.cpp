// inspired from : https://github.com/GoogleChromeLabs/web-audio-samples/blob/master/audio-worklet/design-pattern/wasm/SimpleKernel.cc

/*
 emscriptem compilation: ./emcc.bind.birds.sh

 // javascript:
 AudioWorklet worklet(birds.wasm)

 every second:
 trigger bird song

 (later, use https://developer.mozilla.org/en-US/docs/Web/API/MIDIInput)

 */

// It doesn't work with '1', with an obscure exception.
// Probably because writing a file in WebAssembly is not supported yet?
#define IMJ_DEBUG_AUDIO_OUT 0

//#define IMJ_LOG_AUDIO_TIME 1

#include <emscripten/bind.h>

#include <iostream>

#include "public.h"

#include "unity.build.cpp"
#include "../../cpp.algorithms/source/unity.build.cpp"

namespace imajuscule::audio {

namespace audioelement {

template<int nAudioOut, AudioOutPolicy policy, SoundEngineMode MODE>
using Voice = imajuscule::audio::voice::Impl_<
policy,
nAudioOut,
MODE,
HandleNoteOff::Yes,
std::vector<float>,
ProcessData
>;

} // NS audioelement

constexpr unsigned kRenderQuantumFrames = 128;
constexpr unsigned kBytesPerChannel = kRenderQuantumFrames * sizeof(float);


struct BirdsBase {
    virtual ~BirdsBase() = default;
    
    virtual void useProgram(int program_index) = 0;
    virtual void process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels) = 0;
    virtual void teardown() = 0;
};

template<audioelement::SoundEngineMode mode>
struct BirdsImpl : public BirdsBase {
    static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;
    
    static constexpr int nAudioOut = 1;
    
    using Synth = audioelement::Voice<nAudioOut, audioEnginePolicy, mode>;

    BirdsImpl(int const sample_rate);
    static int countPrograms() { return decltype(synth)::getPrograms().size(); }

    void useProgram(int program_index) override;
    void process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels) override;

    void teardown() override;

private:
    using Ctxt = Context<
    AudioPlatform::PortAudio,
    Features::JustOut
    >;

    using Stepper = SimpleAudioOutContext<
    nAudioOut,
    audioEnginePolicy
    >;

    int const sample_rate;
    Synth synth;
    std::optional<int> program;
    std::optional<NoteId> noteid;
    Stepper stepper;
    DurationNanos nanos_per_audioelement_buffer;

#if IMJ_DEBUG_AUDIO_OUT
    std::unique_ptr<AsyncWavWriter> async_wav_writer_out;
#endif
    
    auto mk_note_id() {
        static int64_t i = 0;
        ++i;
        return NoteId{i};
    }
};

struct Birds {
    Birds(int const sample_rate, int synthType) {
        switch(synthType) {
            case 0:
                impl = std::make_unique<BirdsImpl<audioelement::SoundEngineMode::BIRDS>>(sample_rate);
                break;
            case 1:
                impl = std::make_unique<BirdsImpl<audioelement::SoundEngineMode::ROBOTS>>(sample_rate);
                break;
            case 2:
                impl = std::make_unique<BirdsImpl<audioelement::SoundEngineMode::WIND>>(sample_rate);
                break;
        }
    }
    static int maxCountPrograms() {
        return std::max({
            BirdsImpl<audioelement::SoundEngineMode::BIRDS>::Synth::getPrograms().size(),
            BirdsImpl<audioelement::SoundEngineMode::ROBOTS>::Synth::getPrograms().size(),
            BirdsImpl<audioelement::SoundEngineMode::WIND>::Synth::getPrograms().size()
        });
    }
    
    void useProgram(int program_index) { impl->useProgram(program_index); }
    void process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels) {
        impl->process(inputBuffer, nInputChannels, outputBuffer, nOutputChannels);
    }
    
    void teardown() {
        impl->teardown();
        impl.reset();
    }

private:
    std::unique_ptr<BirdsBase> impl;
};

template<audioelement::SoundEngineMode mode>
BirdsImpl<mode>::BirdsImpl(int const sample_rate)
: nanos_per_audioelement_buffer(static_cast<uint64_t>(0.5f +
                                                      audio::nanos_per_frame<float>(sample_rate) *
                                                      static_cast<float>(audio::audioelement::n_frames_per_buffer)))
, stepper(GlobalAudioLock<audioEnginePolicy>::get(),
          Synth::n_channels * 4 /* one shot */,
          1 /* a single compute is needed (global for the synth)*/)
, sample_rate(sample_rate)
{
#if IMJ_DEBUG_AUDIO_OUT
    async_wav_writer_out = std::make_unique<AsyncWavWriter>(nAudioOut,
                                                            sample_rate,
                                                            "debug_audioout",
                                                            sample_rate * 3);
#endif
    synth.initialize(stepper);

    auto & v = synth;
    v.initializeSlow(); // does something only the 1st time
    v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!
}

/*
 If the program index is different from the current program, a new note is triggered
 */
template<audioelement::SoundEngineMode mode>
void BirdsImpl<mode>::useProgram(int program_index) {
    program_index = std::min(program_index,
                             synth.countPrograms()-1);
    if (program && *program == program_index)
        return;
    program = program_index;
    
    if (noteid) {
        auto res = synth.onEvent(sample_rate,
                                 mkNoteOff(*noteid),
                                 stepper,
                                 stepper,
                                 {});
        noteid.reset();
    }
    // for 7 (Light rain in a car) we use 1 ms per cb (filter order is 89 !!!).
    // for 6 (Light rain) we use 0.2 ms
    synth.useProgram(program_index); // keep it first as it reinitializes params
    //std::cout << "using program '" << synth.getProgram(program_index).name << "'" << std::endl;
    
    noteid = mk_note_id();
    float volume = 1.f;
    float frequency = 200.f;
    
    auto const res  = synth.onEvent(sample_rate,
                                    mkNoteOn(*noteid,
                                             frequency,
                                             volume),
                                    stepper,
                                    stepper,
                                    {});
    //std::cout << *noteid << ": pitch " << frequency << " vol " << volume << " " << res << std::endl;
}

template<audioelement::SoundEngineMode mode>
void BirdsImpl<mode>::process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels) {
    for (int i=0; i<nOutputChannels; ++i) {
        if (i==0) {
            stepper.step(reinterpret_cast<float*>(outputBuffer),
                         kRenderQuantumFrames,
                         TimeNanos(-1), // todo: can we pass 0 in constructor?
                         nanos_per_audioelement_buffer);
#if IMJ_DEBUG_AUDIO_OUT
            async_wav_writer_out->sync_feed_frames(reinterpret_cast<float*>(outputBuffer),
                                                   static_cast<int>(kRenderQuantumFrames));
#endif
        }
        else {
            float* destination = reinterpret_cast<float*>(outputBuffer) + i * kRenderQuantumFrames;
            for (int j=0; j<kRenderQuantumFrames; ++j) {
                destination[j] = 0.f;
            }
        }
    }
}

template<audioelement::SoundEngineMode mode>
void BirdsImpl<mode>::teardown() {
    synth.finalize(stepper);
}

} // NS

using namespace emscripten;
using namespace imajuscule::audio;

EMSCRIPTEN_BINDINGS(birdsModule) {
    class_<Birds>("Birds")
    .constructor<int const, int>()
    .class_function("maxCountPrograms", &Birds::maxCountPrograms)
    .function("useProgram", &Birds::useProgram)
    .function("process", &Birds::process, allow_raw_pointers())
    ;
}
