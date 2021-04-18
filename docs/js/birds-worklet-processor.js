import Module from './birds.wasm.js';
import { RENDER_QUANTUM_FRAMES, MAX_CHANNEL_COUNT, HeapAudioBuffer }
    from './wasm-audio-helper.js';

/**
 * A simple demonstration of WASM-powered AudioWorkletProcessor.
 *
 * @class WASMWorkletProcessor
 * @extends AudioWorkletProcessor
 */
class WASMBirdWorkletProcessor extends AudioWorkletProcessor {
  static get parameterDescriptors () {
    return [{
      name: 'program',
      defaultValue: 0,
      minValue: 0,
      maxValue: Module.Birds.countPrograms() - 1,
      automationRate: 'k-rate'
    }]
  }

  /**
   * @constructor
   */
  constructor() {
    super();

    // Allocate the buffer for the heap access. Start with stereo, but it can
    // be expanded up to 32 channels.
    this._heapInputBuffer = new HeapAudioBuffer(Module, RENDER_QUANTUM_FRAMES,
                                                2, MAX_CHANNEL_COUNT);
    this._heapOutputBuffer = new HeapAudioBuffer(Module, RENDER_QUANTUM_FRAMES,
                                                 2, MAX_CHANNEL_COUNT);

    this._kernel = new Module.Birds(sampleRate);
  }

  /**
   * System-invoked process callback function.
   * @param  {Array} inputs Incoming audio stream.
   * @param  {Array} outputs Outgoing audio stream.
   * @param  {Object} parameters AudioParam data.
   * @return {Boolean} Active source flag.
   */
  process(inputs, outputs, parameters) {
    // Use the 1st input and output only to make the example simpler. |input|
    // and |output| here have the similar structure with the AudioBuffer
    // interface. (i.e. An array of Float32Array)
    let input = inputs[0];
    let output = outputs[0];

    let inputChannelCount = input.length;
    let outputChannelCount = output.length;

    // Prepare HeapAudioBuffer for the channel count change in the current
    // render quantum.
    this._heapInputBuffer.adaptChannel(inputChannelCount);
    this._heapOutputBuffer.adaptChannel(outputChannelCount);

    // Copy-in, process and copy-out.
    for (let channel = 0; channel < inputChannelCount; ++channel) {
      this._heapInputBuffer.getChannelData(channel).set(input[channel]);
    }
    this._kernel.useProgram(Math.round(parameters['program'][0]));
    this._kernel.process(this._heapInputBuffer.getHeapAddress(),
                         inputChannelCount,
                         this._heapOutputBuffer.getHeapAddress(),
                         outputChannelCount);
    for (let channel = 0; channel < outputChannelCount; ++channel) {
      output[channel].set(this._heapOutputBuffer.getChannelData(channel));
    }

    return true;
  }
}


registerProcessor('birds-worklet-processor', WASMBirdWorkletProcessor);
