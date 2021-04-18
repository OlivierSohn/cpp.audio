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

struct Birds {
    Birds(int const sample_rate);

    void process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels);

    void teardown();

private:
    static constexpr auto audioEnginePolicy = AudioOutPolicy::MasterLockFree;

    static constexpr int nAudioOut = 1;

    using Ctxt = Context<
    AudioPlatform::PortAudio,
    Features::JustOut
    >;

    using Stepper = SimpleAudioOutContext<
    nAudioOut,
    audioEnginePolicy
    >;

    static constexpr audioelement::SoundEngineMode mode =
    audioelement::SoundEngineMode::ROBOTS;
    //audioelement::SoundEngineMode::WIND;
    //audioelement::SoundEngineMode::BIRDS;

    using Synth = audioelement::Voice<nAudioOut, audioEnginePolicy, mode>;

    Synth synth;
    Stepper stepper;
    uint64_t nanos_per_audioelement_buffer;

#if IMJ_DEBUG_AUDIO_OUT
    std::unique_ptr<AsyncWavWriter> async_wav_writer_out;
#endif
};

Birds::Birds(int const sample_rate)
: nanos_per_audioelement_buffer(static_cast<uint64_t>(0.5f +
                                                      audio::nanos_per_frame<float>(sample_rate) *
                                                      static_cast<float>(audio::audioelement::n_frames_per_buffer)))
, stepper(GlobalAudioLock<audioEnginePolicy>::get(),
          Synth::n_channels * 4 /* one shot */,
          1 /* a single compute is needed (global for the synth)*/)
{
#if IMJ_DEBUG_AUDIO_OUT
    async_wav_writer_out = std::make_unique<AsyncWavWriter>(nAudioOut,
                                                            sample_rate,
                                                            "debug_audioout",
                                                            sample_rate * 3);
#endif
    auto mk_note_id = [] {
        static int64_t i = 0;
        ++i;
        return NoteId{i};
    };

    synth.initialize(stepper);

    auto & v = synth;
    v.initializeSlow(); // does something only the 1st time

    std::optional<NoteId> noteid;

    const int program_index = std::min(6,
                                       v.countPrograms()-1);
    // for 7 (Light rain in a car) we use 1 ms per cb (filter order is 89 !!!).
    // for 6 (Light rain) we use 0.2 ms
    v.useProgram(program_index); // keep it first as it reinitializes params
    std::cout << "using program '" << v.getProgram(program_index).name << "'" << std::endl;

    v.set_loudness_compensation(.2f); // birds do not naturally emit loudness compensated frequencies!

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
    std::cout << *noteid << ": pitch " << frequency << " vol " << volume << " " << res << std::endl;
}

void Birds::process(uintptr_t inputBuffer, int nInputChannels, uintptr_t outputBuffer, int nOutputChannels) {
    for (int i=0; i<nOutputChannels; ++i) {
        if (i==0) {
            stepper.step(reinterpret_cast<float*>(outputBuffer),
                         kRenderQuantumFrames,
                         -1, // tNanos, only used for MIDI (?)
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

void Birds::teardown() {
    synth.finalize(stepper);
}

} // NS

using namespace emscripten;
using namespace imajuscule::audio;

EMSCRIPTEN_BINDINGS(birdsModule) {
    class_<Birds>("Birds")
    .constructor<int const>()
    .function("process", &Birds::process, allow_raw_pointers())
    ;
}
