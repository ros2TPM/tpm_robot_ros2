#ifndef _MCL_Types_H
#define _MCL_Types_H

#include "MCL_Types.h"
#include <string.h> //for memset & memcpy


#ifndef NULL
	#define NULL 0
#endif

#define TINY_VALUE 1e-5f
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

//=========== Compiler specific definition ==========

#ifdef _WIN32 

	#pragma warning(disable : 4996)

#else //__linux__ or XMC4800

#endif

#if defined(XMC4800_F144x2048) || defined(XMC4700_F144x2048)
	#include <DAVE.h>
	#define MCL_SLEEP(ms) Sleep(ms)
	#define MCL_INLINE  __attribute__((always_inline)) static inline
	#define PRAGMA_WARNING_PUSH _Pragma("GCC diagnostic push")
	#define PRAGMA_WARNING_POP  _Pragma("GCC diagnostic pop")
	#define PRAGMA_WARNING_IGNORE_UNUSED_PARAM   _Pragma("GCC diagnostic ignored \"-Wunused-variable\"") //_Pragma("GCC diagnostic ignored \"-Wunused-function\"")
	#define ATTRIBUTE_UNUSED	__attribute__((unused))
	#define MCLC_API

#elif defined __linux__
	#define MCL_SLEEP(ms) usleep(ms)
	#define MCL_INLINE inline
	#define PRAGMA_WARNING_PUSH _Pragma("warning( push )")
	#define PRAGMA_WARNING_POP  _Pragma("warning( pop )")
	#define PRAGMA_WARNING_IGNORE_UNUSED_PARAM  _Pragma("warning( disable : 4100 )")
	#define ATTRIBUTE_UNUSED
	#define MCLC_API

#else //_WIN32
	#include <Windows.h>
	#define MCL_SLEEP(ms) Sleep(ms)
	#define MCL_INLINE static _inline
	//#define PRAGMA_WARNING_PUSH _Pragma("warning( push )")
	//#define PRAGMA_WARNING_POP  _Pragma("warning( pop )")
	//#define PRAGMA_WARNING_IGNORE_UNUSED_PARAM  _Pragma("warning( disable : 4100 )")
	#define ATTRIBUTE_UNUSED

	#ifdef  MCLC_EXPORTS //todo: duplicate with MCLC
		#define MCLC_API __declspec(dllexport)
	#else
		#define MCLC_API //__declspec(dllimport)
	#endif

	// Custom Preprocessor: _MSC_PLATFORM_TOOLSET=$(PlatformToolsetVersion)
	#ifdef _MSC_PLATFORM_TOOLSET// &&  _MSC_PLATFORM_TOOLSET > 100 // larger than vc2010
		#define PRAGMA_WARNING_PUSH _Pragma("warning( push )")
		#define PRAGMA_WARNING_POP  _Pragma("warning( pop )")
		#define PRAGMA_WARNING_IGNORE_UNUSED_PARAM  _Pragma("warning( disable : 4100 )")
		
	#else
		#define PRAGMA_WARNING_PUSH 
		#define PRAGMA_WARNING_POP  
		#define PRAGMA_WARNING_IGNORE_UNUSED_PARAM  
	#endif

#endif


#if defined(XMC4800_F144x2048) || defined(XMC4700_F144x2048)
	#define STATIC_STRUCT_ARRAY(statement) __attribute__((aligned(4))) statement __attribute__((section("MNET_RAM")));

#else
	#define STATIC_STRUCT_ARRAY(statement) statement
#endif


#if defined(THREAD_SAFE)
// Note: if you realy want to do multi-thread with cross platform, 
// consider use 3rd party library. (ex. OpenMP https://www.openmp.org//)
	#if defined(_WIN32)
		#include <Windows.h>
		#define MCL_MUTEX					CRITICAL_SECTION
		#define MCL_INIT_MUTEX(obj, name) 	InitializeCriticalSection(obj)
		#define MCL_LOCK(obj)				EnterCriticalSection(obj)
		#define MCL_UNLOCK(obj)				LeaveCriticalSection(obj)

	#elif defined(__linux__)
		#define pthread_mutex_lock // something like this.

	#elif defined(XMC4800_F144x2048) || defined(XMC4700_F144x2048)
		#define MCL_MUTEX					osMutexId
		#define MCL_INIT_MUTEX(obj, name) 	osMutexDef (name); *obj = osMutexCreate (osMutex (name))
		#define MCL_LOCK(obj)				osMutexWait (*obj, osWaitForever)
		#define MCL_UNLOCK(obj)				osMutexRelease(*obj)
	#endif

#endif //THREAD_SAFE


#endif
