

namespace imajuscule {
    namespace audio {
        int initAudioSession() {
            
            if(AudioSessionInitialize(nullptr, nullptr, nullptr, nullptr) != noErr) {
                return 1;
            }
            
            if(AudioSessionSetActive(true) != noErr) {
                return 1;
            }
            
            UInt32 sessionCategory = kAudioSessionCategory_PlayAndRecord;
            if(AudioSessionSetProperty(kAudioSessionProperty_AudioCategory,
                                       sizeof(UInt32), &sessionCategory) != noErr) {
                return 1;
            }
            
            Float32 bufferSizeInSec = 0.005f;
            if(AudioSessionSetProperty(kAudioSessionProperty_PreferredHardwareIOBufferDuration,
                                       sizeof(Float32), &bufferSizeInSec) != noErr) {
                return 1;
            }
            
            UInt32 overrideCategory = 1;
            if(AudioSessionSetProperty(kAudioSessionProperty_OverrideCategoryDefaultToSpeaker,
                                       sizeof(UInt32), &overrideCategory) != noErr) {
                return 1;
            }
            
            // There are many properties you might want to provide callback functions for:
            // kAudioSessionProperty_AudioRouteChange
            // kAudioSessionProperty_OverrideCategoryEnableBluetoothInput
            // etc.
            
            return 0;
        }

        int initAudioStreams(Features feat, AudioUnit & audioUnit, void * chans,
                             AURenderCallback cb, int nOuts,
                             AudioStreamBasicDescription & streamDescription) {
            UInt32 audioCategory = (feat == Features::InAndOut)?
                kAudioSessionCategory_PlayAndRecord:
            // this looks wrong, maybe we can optimize : why should we need play and record for both audiounits??
            
            // cf http://stackoverflow.com/questions/27932879/remoteio-audiounit-playback-quality-not-tied-to-callback-runtime-but-to-somethi
                kAudioSessionCategory_MediaPlayback;
            
            if(AudioSessionSetProperty(kAudioSessionProperty_AudioCategory,
                                       sizeof(UInt32), &audioCategory) != noErr) {
                return 1;
            }
            /*
             UInt32 overrideCategory = 1;
             if(AudioSessionSetProperty(kAudioSessionProperty_OverrideCategoryDefaultToSpeaker,
             sizeof(UInt32), &overrideCategory) != noErr) {
             // Less serious error, but you may want to handle it and bail here
             }*/
            
            AudioComponentDescription componentDescription;
            componentDescription.componentType = kAudioUnitType_Output;
            componentDescription.componentSubType = kAudioUnitSubType_RemoteIO;
            componentDescription.componentManufacturer = kAudioUnitManufacturer_Apple;
            componentDescription.componentFlags = 0;
            componentDescription.componentFlagsMask = 0;
            auto component = AudioComponentFindNext(nullptr, &componentDescription);
            if(AudioComponentInstanceNew(component, &audioUnit) != noErr) {
                return 1;
            }
            
            
            AudioUnitElement const outputBus = 0;
            AudioUnitElement const inputBus = 1;
            
            //https://developer.apple.com/library/content/technotes/tn2091/_index.html
            UInt32 enable = (feat==Features::InAndOut)?1:0;
            
            if(auto err = AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_EnableIO,
                                               kAudioUnitScope_Input, inputBus, &enable, sizeof(UInt32))) {
                LG(ERR, "0: %d", err);
                return 1;
            }
            enable = 1;
            if(AudioUnitSetProperty(audioUnit, kAudioOutputUnitProperty_EnableIO,
                                    kAudioUnitScope_Output, outputBus, &enable, sizeof(UInt32)) != noErr) {
                return 1;
            }
            
            AURenderCallbackStruct callbackStruct;
            callbackStruct.inputProc = cb;
            callbackStruct.inputProcRefCon = chans;
            if(auto err = AudioUnitSetProperty(audioUnit, kAudioUnitProperty_SetRenderCallback,
                                               kAudioUnitScope_Input, outputBus, &callbackStruct,
                                               sizeof(AURenderCallbackStruct))) {
                LG(ERR, "1: %d", err);
                return 1;
            }
            
            // You might want to replace this with a different value, but keep in mind that the
            // iPhone does not support all sample rates. 8kHz, 22kHz, and 44.1kHz should all work.
            streamDescription.mSampleRate = SAMPLE_RATE;
            // Yes, I know you probably want floating point samples, but the iPhone isn't going
            // to give you floating point data. You'll need to make the conversion by hand from
            // linear PCM <-> float.
            streamDescription.mFormatID = kAudioFormatLinearPCM;
            // This part is important!
            streamDescription.mFormatFlags = kAudioFormatFlagIsSignedInteger |
            kAudioFormatFlagsNativeEndian |
            kAudioFormatFlagIsPacked;
            // Not sure if the iPhone supports recording >16-bit audio, but I doubt it.
            streamDescription.mBitsPerChannel = 16;
            streamDescription.mBytesPerFrame = nOuts*sizeof(SInt16);
            streamDescription.mChannelsPerFrame = nOuts;
            streamDescription.mFramesPerPacket = 1;
            streamDescription.mBytesPerPacket = streamDescription.mBytesPerFrame * streamDescription.mFramesPerPacket;
            streamDescription.mReserved = 0;
            
            if(AudioUnitSetProperty(audioUnit, kAudioUnitProperty_StreamFormat,
                                    kAudioUnitScope_Input, outputBus, &streamDescription, sizeof(streamDescription)) != noErr) {
                return 1;
            }
            
            if(auto err = AudioUnitSetProperty(audioUnit, kAudioUnitProperty_StreamFormat,
                                               kAudioUnitScope_Output, inputBus, &streamDescription, sizeof(streamDescription))) {
                LG(ERR, "2: %d", err);
                return 1;
            }
            
            return 0;
        }

        OSStatus startAudioUnit(AudioUnit audioUnit) {
            OSStatus res = AudioUnitInitialize(audioUnit);
            if( res != noErr ) {
                return res;
            }
            
            return AudioOutputUnitStart(audioUnit);
        }
        
        OSStatus stopProcessingAudio(AudioUnit audioUnit) {
            OSStatus res = AudioOutputUnitStop(audioUnit);
            if( res != noErr ) {
                return res;
            }
            
            res = AudioUnitUninitialize(audioUnit);
            if( res != noErr ) {
                return res;
            }
            
            audioUnit = nullptr;
            return noErr;
        }

    }
}
