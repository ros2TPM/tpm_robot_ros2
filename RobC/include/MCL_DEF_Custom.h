#ifndef _MCL_CUSTOM_DEF_H
#define _MCL_CUSTOM_DEF_H


// This is the default version of MCL_DEF_Custom.
// Each project can have its own version.

#define USE_F32 1  // 1: float  0:double

#define MEM_AXIS_NUM 8 
#define MEM_GROUP_NUM 4
#define MEM_TOTAL_OBJ_NUM  (MEM_AXIS_NUM + MEM_GROUP_NUM)
#define MEM_BUF_SIZE 100 //must >= 3, otherwise Fillet will fail.
#define MEM_AXIS_PER_GRP 8

#define ERR_CODE_SHIFT 0 

// used in FEAT_CMPTRG
#define MEM_CMPTRG_NUM 8
#define MEM_CMP_ACTION_NUM 10

#ifdef VISUAL_TEST
#define DDA_TIME 0.001f //unit: sec
#define GROUP_NUM 2 // 0:singal axis, 1:group axis
#endif

#endif