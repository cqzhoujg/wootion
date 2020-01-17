#ifndef ELT_DEFINE_H
#define ELT_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif


#if !defined(__WINDOWS__) && (defined(WIN32) || defined(WIN64) || defined(_MSC_VER) || defined(_WIN32))
#define __WINDOWS__
#endif

#ifdef __WINDOWS__

#define ELT_SDK_CDECL __cdecl
#define ELT_SDK_STDCALL __stdcall

/* export symbols by default, this is necessary for copy pasting the C and header file */
#if !defined(ELT_SDK_HIDE_SYMBOLS) && !defined(ELT_SDK_IMPORT_SYMBOLS) && !defined(ELT_SDK_EXPORT_SYMBOLS)
#define ELT_SDK_EXPORT_SYMBOLS
#endif

#if defined(ELT_SDK_HIDE_SYMBOLS)
#define ELT_SDK_PUBLIC(type)   type ELT_SDK_STDCALL
#elif defined(ELT_SDK_EXPORT_SYMBOLS)
#define ELT_SDK_PUBLIC(type)   __declspec(dllexport) type ELT_SDK_STDCALL
#elif defined(ELT_SDK_IMPORT_SYMBOLS)
#define ELT_SDK_PUBLIC(type)   __declspec(dllimport) type ELT_SDK_STDCALL
#endif
#else /* !__WINDOWS__ */
#define ELT_SDK_CDECL
#define ELT_SDK_STDCALL

#if (defined(__GNUC__) || defined(__SUNPRO_CC) || defined (__SUNPRO_C)) && defined(ELT_SDK_API_VISIBILITY)
#define ELT_SDK_PUBLIC(type)   __attribute__((visibility("default"))) type
#else
#define ELT_SDK_PUBLIC(type) type
#endif
#endif


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "cJSON.h"

#define ELT_TRUE 1
#define ELT_FALSE 0

#define ELT_SUCCESS 1
#define ELT_FAILURE 0
#define ELT_ERROR -1

#ifdef __cplusplus
}
#endif

#endif // ELT_DEFINE_H
