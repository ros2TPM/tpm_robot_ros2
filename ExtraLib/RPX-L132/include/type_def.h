#ifndef __TYPE_DEF_H__
#define __TYPE_DEF_H__

#include <stdint.h>

typedef uint8_t         U8;
typedef int16_t         I16;
typedef uint16_t        U16;
typedef int32_t         I32;
typedef uint32_t        U32;
typedef float           F32;
typedef double          F64;

#if defined(_WIN32)
#   ifndef PASCAL
#   define PASCAL __stdcall
#   endif
#else
#   define PASCAL
#endif


#endif //__TYPE_DEF_H__
