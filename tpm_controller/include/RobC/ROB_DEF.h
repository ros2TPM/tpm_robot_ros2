#ifndef _ROB_DEF_H
#define _ROB_DEF_H

#include "Custom/MCL_FEATURES.h"

#define MAX_AXIS_PER_ROBOT 6
#define MAX_MSG_LENGTH	128

typedef enum {
	// range: [0~4999]

	ROB_OK = 0,
	// ========== same as MCLC ==========
		// ====== System Error =========
	ROB_ERR_NOT_IMPLEMENTED,
	ROB_ERR_NOT_INITIALIZED,	// need to call RobC_Init first.

		// ====== Pre-requisite fail ======
	ROB_ERR_PREREQUISITE = ROB_OK + 50,
	ROB_ERR_OBJ_DISABLED,		// can't do this operation when target object (Axis or Group) is disabled.
	ROB_ERR_OBJ_RUNNING,		// can't do this operation when target object (Axis or Group) is running.
	ROB_ERR_BUF_EXECMD_INVALID,	// try to manipulate current executing command while it is NULL or incorrect type.
	ROB_ERR_CMD_POOL,			// can't request memory from cmd memory pool.

	ROB_ERR_GRP_FULL,			// try to add axis to a group when group is full.
	ROB_ERR_GRP_AXIS_DUPLICATE, // try to add the same axis to a group twice.
	ROB_ERR_GRP_ENABLED,		// try to change the member axis while group is enabled
	ROB_ERR_GRP_AXIS_OCCUPIED,	// try to enable a group while at least one member axis is running, or occupied by another group.

	ROB_ERR_BUF_FULL,			// try to add new command when buffer is full.
	ROB_ERR_BUF_EMPTY,			// try to get command when buffer is empty.
	ROB_ERR_BUF_STOPPING,		// try to add new command when buffer is stopping previous command.
	ROB_ERR_BUF_CLEARING,		// try to add new command when buffer is clearing cmds in buffer.

	ROB_ERR_CANT_CHANGE_PROFILE,
	ROB_ERR_BACKLASH_AXIS_MOVING,	// can't set backlash information because the axis is moving

		// ====== Invalid Argument ===========
	ROB_ERR_INVALID_ARG = ROB_OK + 100,
	ROB_ERR_INVALID_GRPID,
	ROB_ERR_INVALID_AXISID,
	ROB_ERR_INVALID_CIR,
	ROB_ERR_MP_DIST_TOO_SHORT,

		// ====== Fillet =======
	ROB_ERR_FIL = ROB_OK + 150,
	ROB_ERR_FIL_BUSY,			// the Fillet processor is busy. Both C1 & C2 are not NULL
	ROB_ERR_FIL_SEG_INVALID,	// segment (line or circ) is invalid. (ex. too short)
	ROB_ERR_FIL_DIST_TOO_LARGE,	// target arc dist is too large
	ROB_ERR_FIL_LL_PARALLEL,	// two lines are parallel
	ROB_ERR_FIL_NOT_COPLANER,	// segments not on the same plane
	ROB_ERR_FIL_NO_SOLUTION,	// can't find solution in algebra.
	ROB_ERR_FIL_NOT_SUPPORT,	// this function currently not supported.


	// ====== Failure during execution =====
	ROB_ERR_AXIS_HWSTS = ROB_OK + 200,	// the axis/group can't move because general hardware status.(such as EMG, ALM, ...)
	ROB_ERR_AXIS_DISCONN,// the axis can't move because disconnect flag.
	ROB_ERR_AXIS_EMG,	 // the axis can't move because emergency flag.
	ROB_ERR_AXIS_ALM,	 // the axis can't move because alarm flag.
	ROB_ERR_AXIS_PEL,	 // the axis can't move because positive limit flag.
	ROB_ERR_AXIS_NEL,	 // the axis can't move because negative limit flag.


	ROB_ERR_HM = ROB_OK + 220,	// homing failed
	ROB_ERR_HM_ESC,		// homing failed because max escape sensor distance reached
	ROB_ERR_HM_PEL,		// homing failed because positive limit flag
	ROB_ERR_HM_MEL,		// homing failed because negative limit flag


	ROB_ERR_CB_CMD = ROB_OK + 240,
	ROB_ERR_CB_SETOUT,	// error in cmd SetOut.  (CB_SetIOOut return error) 
	ROB_ERR_CB_WAITINP,	// error in cmd WaitInp. (CB_GetIOInp return error)

#if FEAT_CMPTRG
	ROB_ERR_CMPTRG_COND_NOT_SET = ROB_OK + 400,	// try to enable comparator before condition is set.
	ROB_ERR_CMPTRG_CMP_ENABLED,			// try to add/modify/remove Action when Comparator is Enabled.
	ROB_ERR_CMPTRG_INVALID_CMPID,		// Invalid Comparator Id
	ROB_ERR_CMPTRG_INVALID_ACTID,		// Invalid Action Id
#endif

	// ======== Error code for robot only ========
	ROB_ERR_ROBOT_ONLY = 1000,
	ROB_ERR_ANGLE_LIMIT,
	ROB_ERR_AXIS_OVERSPEED,			// in dda cycle, the axis angle increment is too large.

	ROB_ERR_FK,
	ROB_ERR_IK,
	ROB_ERR_IK_OVERHEAD,
	ROB_ERR_IK_WRIST,
	ROB_ERR_IK_CANT_REACH,
	ROB_ERR_IK_END,

} ROB_ERR;

typedef enum {
	////ROB_DELTA,	// reserved.
	////ROB_SCARA,	// reserved.
	////ROB_5_AXIS,	// reserved.
	//ROB_6_AXIS,	
	////ROB_UR,		// reserved. (6-axis Universal Robot-like structure)

	ROB_SCARA      = 20, // reserved.
	ROB_Axis5      = 30, // reserved.
	ROB_Axis6      = 40, // traditional serial 6-axis arm	
	ROB_Axis7      = 50, // reserved.
	ROB_Ext        = 60, // reserved.
	ROB_Delta_Pris = 100, // Delta with prismatic actuator
	ROB_Delta_Rot  = 110, // reserved.

	ROB_Kohzu_USMT = 120, //parallel
	ROB_Kohzu_USM  = 121, //serial
	ROB_Kohzu_SMR  = 122, //serial

} ROB_KIN_TYPE;

typedef enum {
	ROB_FRAME_BASE,
	ROB_FRAME_TOOL
} ROB_FRAME_TYPE;


#endif