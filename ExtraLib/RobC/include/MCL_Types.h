#ifndef _MCL_TYPES_H
#define _MCL_TYPES_H

#include "MCL_DEF_Custom.h"
#include <stdint.h>

#if defined(XMC4800_F144x2048) || defined(XMC4700_F144x2048)
	#include "type_def.h"
	//#define USE_F32 1
#else
	typedef int8_t			I8;
	typedef uint8_t			U8;
	typedef int16_t			I16;
	typedef uint16_t		U16;
	typedef int32_t			I32;
	typedef uint32_t		U32;
	typedef int64_t			I64;
	typedef uint64_t		U64;	
#endif


#if USE_F32
	typedef float                   FLT;
#else
	typedef double					FLT;
#endif

	typedef U8 MCL_BOOL;

#define TRUE  1
#define FALSE 0

#endif //_MCL_TYPES_H
