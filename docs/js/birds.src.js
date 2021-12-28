var audioContext
var node
var curSynthType

removeButtons = function() {
  let oldButtons = document.getElementsByClassName('.program')
  while(oldButtons && oldButtons.length){
    oldButtons[0].remove()
  }
}

onStartStop = async function(synthType) {
  removeButtons()
  if(node) {
    node.disconnect()
    node = null
  }
  let sameSynth = false
  if (curSynthType !== null && (curSynthType == synthType)) {
    sameSynth = true
  }
  if (audioContext)
  {
    if (sameSynth)
    {
      if (audioContext.state === "running")
      {
        audioContext.suspend()
        curSynthType = null
        return
      }
    }
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
  curSynthType = synthType

  modname = 'birds-worklet-processor'

  try {
    await audioContext.audioWorklet.addModule('js/' + modname + '.js')
  } catch (error) {
    alert("Sorry, but your browser might not be supported. Please, consider using Google Chrome instead.");
    throw( error );
  }
  node = new AudioWorkletNode(audioContext, modname, {processorOptions:{'synthType':synthType}})
  const programParam = node.parameters.get('program')

  let programCount = [
    9,  // birds
    2,  // robots
    14 // wind
   ]

  var div = document.getElementById('programDiv');
  var activeBtn = null
  var i = Math.round(programParam.minValue);
  var end=programCount[synthType]-1//Math.round(programParam.maxValue);
  for(; i <= end; ++i) {
    var btn = document.createElement('button')
    btn.innerHTML = 'Program ' + (i + 1);
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
    div.appendChild(btn)
    if (!activeBtn) {
      activeBtn = btn
    }
  }
  if (activeBtn)
    activeBtn.onclick()

  node.connect(audioContext.destination)
}
