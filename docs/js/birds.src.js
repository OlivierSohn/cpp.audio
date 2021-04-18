var audioContext
onStartStop = async function() {
  button = document.getElementById("startStop")
  if (audioContext)
  {
    if (audioContext.state === "running"){
      audioContext.suspend()
      button.innerHTML = "Start sound"
      let oldButtons = document.getElementsByClassName('.program')
      while(oldButtons && oldButtons.length){
        oldButtons[0].remove()
      }
      return
    }
    else
      audioContext.resume()
  }
  else {
    var AudioContext = window.AudioContext // Default
    || window.webkitAudioContext // Safari and old versions of Chrome
    || false;
    if (AudioContext) {
      audioContext = new AudioContext()
    } else {
      alert("Sorry, but the Web Audio API is not supported by your browser. Please, consider upgrading to the latest version or downloading Google Chrome or Mozilla Firefox");
    }
  }
  button.innerHTML = "Stop sound"

  modname = 'birds-worklet-processor'

  try {
    await audioContext.audioWorklet.addModule('js/' + modname + '.js')
  } catch (error) {
    alert("Sorry, but your browser might not be supported. Please, consider using Google Chrome instead.");
    throw( error );
  }
  const node = new AudioWorkletNode(audioContext, modname)
  const programParam = node.parameters.get('program')

  var activeBtn = null
  var i = Math.round(programParam.minValue);
  var end=Math.round(programParam.maxValue);
  for(; i <= end; ++i) {
    var btn = document.createElement('button')
    btn.innerHTML = 'Program ' + i;
    btn.classList.add('.program');
    (function () {
      var curBtn = btn
      var curI = i
      curBtn.onclick = function() {
        let buts = document.getElementsByClassName('.program')
        for (let item of buts)
          item.classList.remove('active');
        curBtn.classList.add('active');
        programParam.setValueAtTime(curI, audioContext.currentTime)
      };
    })();
    document.body.appendChild(btn);
    if (!activeBtn) {
      activeBtn = btn
    }
  }
  if (activeBtn)
    activeBtn.onclick()

  node.connect(audioContext.destination)
}
