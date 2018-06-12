

#if defined _WIN32 && defined _UNICODE
#define PA_MIN_LATENCY_MSEC L"PA_MIN_LATENCY_MSEC"
#define PA_MIN_LATENCY_MSEC_V L"1"
#else
#define PA_MIN_LATENCY_MSEC "PA_MIN_LATENCY_MSEC"
#define PA_MIN_LATENCY_MSEC_V "1"
#endif

namespace imajuscule {
    namespace audio {


#ifdef _WIN32 // setenv
#  if defined _UNICODE
      int setenv(const wchar_t *name, const wchar_t *value, int overwrite)
      {
        int errcode = 0;
        if ( !overwrite ) {
          size_t envsize = 0;
          errcode = _wgetenv_s(&envsize, nullptr, 0, name);
          if ( errcode || envsize ) return errcode;
        }
        return _wputenv_s(name, value);
      }
#  else
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
#  endif
#endif

        void setPortaudioEnvVars()
        {
#if TARGET_OS_IOS
#else
            // set minimum latency env var to speed things up
            int erri = setenv( PA_MIN_LATENCY_MSEC, PA_MIN_LATENCY_MSEC_V, true);
            if(unlikely(erri))
            {
                LG(ERR, "AudioIn::get : Could not set env variable PA_MIN_LATENCY_MSEC: %d", errno);
                Assert(0);
            }
#endif
        }
    }
}
