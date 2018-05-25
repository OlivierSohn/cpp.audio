#include "../include/public.h"

#if __APPLE__
#include <fenv.h>

#if TARGET_OS_IOS
// on ios, these macros are hidden from fenv.h:

// from https://github.com/mstg/iOS-full-sdk/blob/master/iPhoneOS9.3.sdk/usr/include/fenv.h
extern const fenv_t _FE_DFL_DISABLE_DENORMS_ENV;
#define FE_DFL_DISABLE_DENORMS_ENV &_FE_DFL_DISABLE_DENORMS_ENV

// from https://github.com/mstg/iOS-full-sdk/blob/master/iPhoneOS9.3.sdk/usr/include/fenv.h
extern const fenv_t _FE_DFL_DISABLE_SSE_DENORMS_ENV;
#define FE_DFL_DISABLE_SSE_DENORMS_ENV  &_FE_DFL_DISABLE_SSE_DENORMS_ENV

#endif

#endif

#define  _USE_MATH_DEFINES
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <cerrno>
#include <functional>
#include <map>
#include <cstdlib>
