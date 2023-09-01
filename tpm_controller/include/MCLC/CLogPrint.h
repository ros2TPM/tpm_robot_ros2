#ifndef _MCLC_DBGPRINT_H
#define _MCLC_DBGPRINT_H

#include "MCL_Constant.h"
#include "MCL_Types.h"
#include "MCL_LOG_TAG.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_MAX_LEVEL_NUM 10
#define LOG_MAX_TAG_TYPE_NUM 50


typedef enum MCL_LOG_LEVEL
{
	MCL_LOG_LEVEL_INFO		= -2,
	MCL_LOG_LEVEL_WARNING	= -1,
	MCL_LOG_LEVEL_ERROR		= 0,
	MCL_LOG_LEVEL_DEBUG_1,
	MCL_LOG_LEVEL_DEBUG_2,
	MCL_LOG_LEVEL_DEBUG_3,
	MCL_LOG_LEVEL_DEBUG_4,
}MCL_LOG_LEVEL;

typedef struct _LOG_SETTING
{
	MCL_LOG_LEVEL level;
	MCL_BOOL tag[LOG_MAX_TAG_TYPE_NUM];
}LOG_SETTING;

// ==== Public functions ========
void	CLogPrint_Init(LOG_SETTING* logSetting); //if logSetting==NULL, use default setting.

void	CLogPrint_SetTypeEnable	(MCL_LOG_TAG_TYPE tag, MCL_BOOL isEnable);
void	CLogPrint_SetLevel		(MCL_LOG_LEVEL level);

MCLC_API MCL_BOOL CLogPrint(MCL_LOG_LEVEL level, MCL_LOG_TAG_TYPE tag, const char* format, ...);
MCLC_API MCL_BOOL CLogPrint_withInfo(char* fileName, int lineId, const char* funcName, MCL_LOG_LEVEL level, MCL_LOG_TAG_TYPE tag, char* format, ...);

LOG_SETTING* GetLogSetting();



#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#define LLOC printf("[%s(%d): %s]\n", __FILENAME__, __LINE__, __FUNCTION__)

#define LOG2(Level, tag, format, ...) \
	do{ \
	CLogPrint(MCL_LOG_LEVEL_##Level, MCL_LOG_TAG_##tag, format, ##__VA_ARGS__); \
	if (MCL_LOG_LEVEL_##Level## == MCL_LOG_LEVEL_ERROR) LLOC; \
	else printf("\n"); \
	} while (0)


/* Note: for GCC compiler:
 
 ##__VA_ARGS__   (o)
 __VA_ARGS__     (x)

 MCL_LOG_TAG_##tag,   (o)
 MCL_LOG_TAG_##tag##, (x)


 #define LOG_ERR(...) CLOG(...), LLOC  //try to put two actions in one line (x)
 #define LOG_ERR(...) CLogPrint_withInfo(...)  // use a new function to do make code in one line (o)
					  (has to put 'const' in Function signature. otherwise will generate warning.)
*/

#define CLOG(Level, tag, format, ...) CLogPrint(MCL_LOG_LEVEL_##Level, MCL_LOG_TAG_##tag, format, ##__VA_ARGS__)

//#define LOG_ERR(tag, format, ...) CLogPrint_withInfo( __FILENAME__, __LINE__, __FUNCTION__, MCL_LOG_LEVEL_ERROR, MCL_LOG_TAG_##tag, format, ##__VA_ARGS__) 


//return Errcode with itself as message. ex: return ERR(GMERR_xxx);
#define ERR(code)				 CLogPrint(MCL_LOG_LEVEL_ERROR, MCL_LOG_TAG_SYS, #code), code

//return Errcode with extra message. ex: return ERR_M(GMERR_xxx, "arg1=%d, should be %d", a, b);
#define ERR_M(code, format, ...) CLogPrint(MCL_LOG_LEVEL_ERROR, MCL_LOG_TAG_SYS, format, ##__VA_ARGS__), code



#ifdef __cplusplus
}
#endif

#endif