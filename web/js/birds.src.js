var audioContext
onStartStop = async function() {
  button = document.getElementById("startStop")
  if (audioContext)
  {
    if (audioContext.state === "running"){
      audioContext.suspend()
      button.innerHTML = "Start sound"
      return
    }
    else
      audioContext.resume()
  }
  else
    audioContext = new AudioContext()
  button.innerHTML = "Stop sound"

  modname = 'birds-worklet-processor'

  try {
    await audioContext.audioWorklet.addModule('js/' + modname + '.js')
  } catch (error) {
    console.error("To build the WebAssembly part, see web/README.md");
    throw( error );
  }
  const whiteNoiseNode = new AudioWorkletNode(audioContext, modname)
  whiteNoiseNode.connect(audioContext.destination)
}
