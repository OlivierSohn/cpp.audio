



namespace imajuscule {
    namespace audio {

        AudioLockPolicyImpl<AudioOutPolicy::Master> & masterAudioLock() {
            static AudioLockPolicyImpl<AudioOutPolicy::Master> l;
            return l;
        }
        
#ifdef _WIN32
        int setenv(const char *name, const char *value, int overwrite)
        {
            int errcode = 0;
            if ( !overwrite ) {
                size_t envsize = 0;
                errcode = getenv_s(&envsize, nullptr, 0, name);
                if ( errcode || envsize ) return errcode;
            }
            return _putenv_s(name, value);
        }
#endif
        
        void setPortaudioEnvVars()
        {
#if TARGET_OS_IOS
#else
            // set minimum latency env var to speed things up
            const char * lat = "PA_MIN_LATENCY_MSEC";
            const char * latVal = "1";
            int erri = setenv( lat, latVal, true);
            if(unlikely(erri))
            {
                LG(ERR, "AudioIn::get : Could not set env variable PA_MIN_LATENCY_MSEC: %d", errno);
                Assert(0);
            }
            
            // verify that env var was set
#ifdef _WIN32
            char * test=0;
            size_t sz=0;
            if (0 != _dupenv_s(&test, &sz, lat) )
            {
                test = 0;
            }
#else
            const char * test = getenv (lat);
#endif
            
            Assert(test);
            Assert(!strcmp(test, latVal));
#ifdef _WIN32
            free(test);
#endif
#endif
        }
    }
}
