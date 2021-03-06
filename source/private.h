#include "../include/public.h"

#if __has_include(<fenv.h>)
#  include <fenv.h>
#endif

#if TARGET_OS_IOS
// on ios, these macros are hidden from fenv.h:

// from https://github.com/mstg/iOS-full-sdk/blob/master/iPhoneOS9.3.sdk/usr/include/fenv.h
extern const fenv_t _FE_DFL_DISABLE_DENORMS_ENV;
#define FE_DFL_DISABLE_DENORMS_ENV &_FE_DFL_DISABLE_DENORMS_ENV

// from https://github.com/mstg/iOS-full-sdk/blob/master/iPhoneOS9.3.sdk/usr/include/fenv.h
extern const fenv_t _FE_DFL_DISABLE_SSE_DENORMS_ENV;
#define FE_DFL_DISABLE_SSE_DENORMS_ENV  &_FE_DFL_DISABLE_SSE_DENORMS_ENV

#else

# if defined(__x86_64__)
// on ios produces errors like "Use of undeclared identifier __builtin_ia32_emms"
#  if __has_include(<xmmintrin.h>)
#   include <xmmintrin.h>
#  endif
# endif

#endif

#include <stdlib.h>
#include <string.h>
#include <cerrno>
#include <cstdlib>
