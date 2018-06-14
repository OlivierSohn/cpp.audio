

#if defined _WIN32 && defined _UNICODE
#define PA_MIN_LATENCY_MSEC L"PA_MIN_LATENCY_MSEC"
#else
#define PA_MIN_LATENCY_MSEC "PA_MIN_LATENCY_MSEC"
#endif

namespace imajuscule {
    namespace audio {


#ifdef _WIN32 // setenv
#  if defined _UNICODE
      int setenv(const wchar_t *name, const char *orig, int overwrite)
      {
        // newsize describes the length of the
        // wchar_t string called wcstring in terms of the number
        // of wide characters, not the number of bytes.
        size_t newsize = strlen(orig) + 1;

        // The following creates a buffer large enough to contain
        // the exact number of characters in the original string
        // in the new format. If you want to add more characters
        // to the end of the string, increase the value of newsize
        // to increase the size of the buffer.
        auto vec = std::make_unique<std::vector<wchar_t>>();
        vec->reserve(newsize);
        wchar_t * wcstring = vec->data();

        // Convert char* string to a wchar_t* string.
        size_t convertedChars = 0;
        mbstowcs_s(&convertedChars, wcstring, newsize, orig, _TRUNCATE);

        int errcode = 0;
        if ( !overwrite ) {
          size_t envsize = 0;
          errcode = _wgetenv_s(&envsize, nullptr, 0, name);
          if ( errcode || envsize ) return errcode;
        }
        auto res = _wputenv_s(name, wcstring);
        return res;
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

        void setPortaudioLatencyMillis(int latency)
        {
            if(latency <= 0) {
                LG(ERR, "setPortaudioLatencyMillis : negative latency %d", latency);
                return;
            }
#if TARGET_OS_IOS
#else
            // set minimum latency env var to speed things up
            std::string lat = std::to_string(latency);
            int erri = setenv( PA_MIN_LATENCY_MSEC, lat.c_str(), true);
            if(unlikely(erri))
            {
                LG(ERR, "AudioIn::get : Could not set env variable PA_MIN_LATENCY_MSEC: %d", errno);
                Assert(0);
            }
#endif
        }
    }
}
