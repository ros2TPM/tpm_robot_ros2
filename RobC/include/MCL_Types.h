#ifndef _MCL_TYPES_H
#define _MCL_TYPES_H

#include "MCL_DEF_Custom.h"

#if defined(XMC4800_F144x2048) || defined(XMC4700_F144x2048)
	#include "type_def.h"
	//#define USE_F32 1
#else
	typedef signed char				I8;
	typedef unsigned char			U8;
	typedef signed short			I16;
	typedef unsigned short			U16;
	typedef signed long				I32;
	typedef unsigned long			U32;
	typedef signed long long		I64;
	typedef unsigned long long		U64;	
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
