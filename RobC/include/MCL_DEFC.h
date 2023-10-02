#ifndef _MCL_DEFC_H
#define _MCL_DEFC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MCL_FEATURES.h"
#include "MCL_Types.h"
#include <string.h> // for memcpy. used in most files.

// version meaning : 0xaabbcc00 -> ver a.b.c
#define MCLC_VERSION 0x030F0100; // v3.16.0, Add Delta-end-rot to ROBC

// the initialize value is set at MCL_DEFC.c
extern FLT DDA_TIME;// = 0.001f;

extern U16 AVAIL_AXIS_NUM;
extern U16 AVAIL_GROUP_NUM;
extern U16 AVAIL_POOL_SIZE;
extern U16 AVAIL_AXIS_PER_GRP;

extern U16 AVAIL_CMPTRG_NUM;
extern U16 AVAIL_CMP_ACTION_NUM;


// ========= [Enum] =======
typedef enum MCL_ERR {
	// !!! Caution !!! when you modify here, plz also modify ROBC & MCL.
	MCL_OK = 0,

	// ====== System Error =========
	MCL_ERR_NOT_IMPLEMENTED,
	MCL_ERR_MALLOC_FAILED,

	// ====== Pre-requisite fail ======
	MCL_ERR_PREREQUISITE = MCL_OK + 50,
	MCL_ERR_OBJ_DISABLED,		// can't do this operation when target object (Axis or Group) is disabled.
	MCL_ERR_OBJ_RUNNING,		// can't do this operation when target object (Axis or Group) is running.
	MCL_ERR_OBJ_NO_AXIS,		// this object doesn't contain any axis.
	MCL_ERR_BUF_EXECMD_INVALID,	// try to manipulate current executing command while it is NULL or incorrect type.
	MCL_ERR_CMD_POOL,			// can't request memory from cmd memory pool.

	MCL_ERR_GRP_FULL,			// try to add axis to a group when group is full.
	MCL_ERR_GRP_AXIS_DUPLICATE, // try to add the same axis to a group twice.
	MCL_ERR_GRP_ENABLED,		// try to change the member axis while group is enabled
	MCL_ERR_GRP_AXIS_OCCUPIED,	// try to enable a group while at least one member axis is running, or occupied by another group.

	MCL_ERR_BUF_FULL,			// try to add new command when buffer is full.
	MCL_ERR_BUF_EMPTY,			// try to get command when buffer is empty.
	MCL_ERR_BUF_STOPPING,		// try to add new command when buffer is stopping previous command.
	MCL_ERR_BUF_CLEARING,		// try to add new command when buffer is clearing cmds in buffer.

	MCL_ERR_CANT_CHANGE_PROFILE,
	MCL_ERR_BACKLASH_AXIS_MOVING,	// can't set backlash information because the axis is moving

	MCL_ERR_ENABLED,			// Can't perform this operation while enabled
	MCL_ERR_CB_NOT_SET,			// callback is not registered

	MCL_ERR_PITCH_COMP_ENABLED,	// Can't perform this operation while pitch compensation is enabled
	MCL_ERR_SYNC_ENABLED,	// Can't perform this operation while synchronization is enabled

	// ====== Invalid Argument ===========
	MCL_ERR_INVALID_ARG = MCL_OK + 100,
	MCL_ERR_INVALID_GRPID,
	MCL_ERR_INVALID_AXISID,
	MCL_ERR_INVALID_OBJID,
	MCL_ERR_INVALID_CIR,
	MCL_ERR_MP_DIST_TOO_SHORT,
	MCL_ERR_NULL_POINTER,
	MCL_ERR_ITEM_NOT_FOUND,

	// ====== Fillet =======
	MCL_ERR_FIL = MCL_OK + 150,
	MCL_ERR_FIL_BUSY,			// the Fillet processor is busy. Both C1 & C2 are not NULL
	MCL_ERR_FIL_SEG_INVALID,	// segment (line or circ) is invalid. (ex. too short)
	MCL_ERR_FIL_DIST_TOO_LARGE,	// target arc dist is too large
	MCL_ERR_FIL_LL_PARALLEL,	// two lines are parallel
	MCL_ERR_FIL_NOT_COPLANER,	// segments not on the same plane
	MCL_ERR_FIL_NO_SOLUTION,	// can't find solution in algebra.
	MCL_ERR_FIL_NOT_SUPPORT,	// this function currently not supported.


	// ====== Failure during execution =====
	MCL_ERR_AXIS_HWSTS = MCL_OK + 200,	// the axis/group can't move because general hardware status.(such as EMG, ALM, ...)
	MCL_ERR_AXIS_DISCONN,	// the axis can't move because of disconnect flag.
	MCL_ERR_AXIS_EMG,		// the axis can't move because of emergency flag.
	MCL_ERR_AXIS_ALM,		// the axis can't move because of alarm flag.
	MCL_ERR_AXIS_PEL,		// the axis can't move because of positive limit flag.
	MCL_ERR_AXIS_NEL,		// the axis can't move because of negative limit flag.
	MCL_ERR_AXIS_SOFT_PEL,	// the axis can't move because of positive soft limit.
	MCL_ERR_AXIS_SOFT_MEL,	// the axis can't move because of negative soft limit.
	MCL_ERR_AXIS_MAX_VEL,	// the axis can't move because the velocity of cmd exceed the max velocity of the axis.
	MCL_ERR_AXIS_MAX_ACC,	// the axis can't move because the acceleration of cmd exceed the max acceleration of the axis.
	MCL_ERR_AXIS_MAX_DEC,	// the axis can't move because the deceleration of cmd exceed the max deceleration of the axis.


	MCL_ERR_HM = MCL_OK + 220,	// homing failed
	MCL_ERR_HM_ESC,		// homing failed because max escape sensor distance reached
	MCL_ERR_HM_PEL,		// homing failed because positive limit flag
	MCL_ERR_HM_MEL,		// homing failed because negative limit flag


	MCL_ERR_CB_CMD = MCL_OK + 240,
	MCL_ERR_CB_SETOUT,	// error in cmd SetOut.  (CB_SetIOOut return error)
	MCL_ERR_CB_WAITINP,	// error in cmd WaitInp. (CB_GetIOInp return error)
	MCL_ERR_CB_COND,	// error in cmd Wait.	 (CB_IsConditionMet return error)

	MCL_ERR_TIMEOUT,

	// ====== Synchronize control ======
	MCL_ERR_NOT_SYNC = MCL_OK + 300,
	MCL_ERR_SYNC_INVALID_MST,		// this axis can't be a master.
	MCL_ERR_SYNC_INVALID_SLV,		// this axis can't be a slave.
	MCL_ERR_SYNC_INVALID_GEARRATIO,	// GearRatio out of range: [0.001~ 10000]
	MCL_ERR_SYNC_CATCHUP,			// the slave is doing catchup motion.
	MCL_ERR_SYNC_PHASE_SHIFT,		// the slave is doing phase shift.
	MCL_ERR_SYNC_BUSY,				// the slave is running, can't start sync.

	#if FEAT_CMPTRG
	MCL_ERR_CMPTRG_COND_NOT_SET = MCL_OK + 400,	// try to enable comparator before condition is set.
	MCL_ERR_CMPTRG_CMP_ENABLED,			// try to add/modify/remove Action when Comparator is Enabled.
	MCL_ERR_CMPTRG_INVALID_CMPID,		// Invalid Comparator Id
	MCL_ERR_CMPTRG_INVALID_ACTID,		// Invalid Action Id
	#endif

	#if FEAT_MPG
	MCL_ERR_MPG = MCL_OK + 500,
	MCL_ERR_MPG_ENABLED,		// try to set MPG when MPG is Enabled
	MCL_ERR_MPG_DI_DUPLICATE,	// different functions are set to the same DI bit.
	MCL_ERR_MPG_CB_NOT_SET,		// callback not set
	#endif
} MCL_ERR;


// ========== [Enum] Action Type & Modes (From User input)  ======
typedef enum MCL_CMD_TYPE
{
	MCL_CMD_POS,
	MCL_CMD_LIN,
	MCL_CMD_CIR,
	MCL_CMD_SPR,	//spiral
	MCL_CMD_FRAME,
	MCL_CMD_CIRF,	//cir frame
	MCL_CMD_LCH,	//latch
	MCL_CMD_VEL,
	MCL_CMD_PVT,
	MCL_CMD_CSP_HM,	//homing

	// doesn't need axis.
	MCL_CMD_DELAY,
	MCL_CMD_SET_OUT,//set Digital output byte
	MCL_CMD_WAIT_INP,//wait for Digital input byte
	MCL_CMD_WAIT_COND,//wait for condition met
	

} MCL_CMD_TYPE; //command type

typedef enum MCL_STOP_TYPE {
	// we use 1 bit to represent this. so plz don't change the number. //todo: no use
	MCL_STOP_SMOOTH, //Absolute coordinates
	MCL_STOP_RAPID, //Relative coordinate
} MCL_STOP_TYPE;

typedef enum MCL_COORD_TYPE {
	MCL_COORD_ABS, //Absolute coordinates
	MCL_COORD_REL, //Relative coordinate
} MCL_COORD_TYPE;

typedef enum MCL_OVERLAP_TYPE {
	MCL_OVERLAP_DEC,  //overlap rate = (overlap time / deceleration time)  x 100%
	MCL_OVERLAP_TIME, //overlap rate = (overlap time / total time) x 100%
	MCL_OVERLAP_DIST, //overlap rate = (overlap dist / total dist) x 100%
} MCL_OVERLAP_TYPE;

typedef enum MCL_DIR_TYPE {
	MCL_DIR_POS = 1,	// positive 
	MCL_DIR_NEG = -1,	// negative
} MCL_DIR_TYPE; // Direction

typedef enum MCL_CIR_TYPE {
	MCL_CIR_CN_Ang,		// Specify Center, Norm and Angle
	MCL_CIR_CE_Dir,		// Specify Center, End and Direction
	MCL_CIR_ME,			// Specify Mid and End.
	MCL_CIR_CE_2D = -1,	// Specify Plane, Center, End and Direction
	MCL_CIR_RE_2D = -2,	// Specify Plane, Radius, End and Direction
	MCL_HELIX_CE_2D = -3,
	MCL_HELIX_RE_2D = -4
} MCL_CIR_TYPE; // Mode of Circular path

typedef enum MCL_PLANE_TYPE {
	MCL_PLANE_XY,
	MCL_PLANE_ZX,
	MCL_PLANE_YZ,
} MCL_PLANE_TYPE; // Plane of Circular path

typedef enum MCL_FIL_TYPE {
	MCL_FIL_RADIUS,	//Define the fillet using radius of the Arc
	MCL_FIL_ERROR,	//Define the fillet using error distance between Arc and edge point.
} MCL_FIL_TYPE;

typedef enum MCL_ORI_TYPE {
	MCL_ORI_CONST,	//No rotation
	MCL_ORI_LINEAR,	//Linear rotation between start and end
	MCL_ORI_PATH,	//
} MCL_ORI_TYPE; //Define the orientation type of cir move

typedef enum MCL_HOME_MODE {
	MCL_HOME_CURRENT_POS,
	MCL_HOME_EZ,
	MCL_HOME_PROBE,
	MCL_HOME_ORG,
	MCL_HOME_ORG_EZ,
	MCL_HOME_ORG_REZ,
	MCL_HOME_ORG_PROBE,
	MCL_HOME_ORG_ORG,
	MCL_HOME_ORGOFF,
	MCL_HOME_ORGOFF_EZ,
	MCL_HOME_ORGOFF_REZ,
	MCL_HOME_LS,
	MCL_HOME_LS_REZ,
} MCL_HOME_MODE;

typedef enum MCL_LCH_MODE {
	MCL_LCH_CONTINUE,	// Continue moving until the original cmd is done.
	MCL_LCH_STAY,		// Stop moving.
	MCL_LCH_EXTRA,		// Stop the original cmd, Move an extra pos cmd.
}	MCL_LCH_MODE;	// Latch mode: The behavior of axis after HwSts is triggered.

typedef enum MCL_PROBE_SRC
{
	MCL_PROBE_TCH,	// touch probe
	MCL_PROBE_EZ,	// z-phase of encoder
}  MCL_PROBE_SRC; //Source of touch probe.

typedef enum MCL_PROBE_MODE
{
	MCL_PROBE_SINGLE,
	MCL_PROBE_MULTI
} MCL_PROBE_MODE;

typedef enum MCL_LS_MODE {
	MCL_LS_NONE,		// do nothing. the moving command will continue to send.
	MCL_LS_STOP_RAPID,	// stop immediately.
	MCL_LS_STOP_SMOOTH,	// stop with deceleration (this is the default value)
}	MCL_LS_MODE;	// Limit-Switch mode: The behavior of moving-axis when detect Limit-Switch.


//========== [Enum] System State ===================

typedef enum MCL_MOT_STATE {
	// ---- can be determined in Motion Profile -----
	MCL_MOT_NONE,		// obj is not in Discrete_Motion State.
	MCL_MOT_ACC,		// obj is accelerating
	MCL_MOT_MAXV,		// obj has reached the max speed of the moving command.
	MCL_MOT_DEC,		// obj is decelerating
	MCL_MOT_STOPPING,	// obj is stopping

	// ---- determined in Executor / Cmd --------
	MCL_MOT_HOLDING,	// obj is holding
	MCL_MOT_WAIT_INPOS, // obj is waiting for In-position
	MCL_MOT_DELAY,		// obj is delaying
	MCL_MOT_WAIT_IO,	// obj is waiting for an IO input
	MCL_MOT_WAIT_COND,	// obj is waiting for a condition to met
	
} MCL_MOT_STATE; // Object Motion Status

typedef enum MCL_HM_STATE {
	MCL_HM_NONE, // contains state: START and FINISH.
	MCL_HM_ESCAPE_SENSOR,
	MCL_HM_HOME_REVERSE,
	MCL_HM_REBOUND_LIMIT,
	MCL_HM_MOVE_PHASE_1,
	MCL_HM_MOVE_PHASE_2,
	MCL_HM_SLOW_DOWN,
	MCL_HM_HOME_SHIFT,
} MCL_HM_STATE; // The State of Homing State Machine

typedef enum MCL_CMDEND_TYPE {
	MCL_CMDEND_UNDEFIND , // undefined.
	MCL_CMDEND_COMPLETE , // Command complete
	MCL_CMDEND_STOP     , // Command being stopped during execution, or cleared before execution.
	MCL_CMDEND_ERROR	, // An error occurs in command calculation
	MCL_CMDEND_SKIP_WAIT, // Command being skipped from waiting.
} MCL_CMDEND_TYPE;// reason of terminating a command

typedef enum MCL_SYNC_STATE {
	MCL_SYNC_NONE, // Not sync
	MCL_SYNC_RUNNING,
	MCL_SYNC_STOPPING,
	MCL_SYNC_GEAR_SHIFTING,
	MCL_SYNC_PHASE_SHIFTING,
} MCL_SYNC_STATE; // The State of Sync State Machine

typedef enum MCL_MST_DESYNC_TYPE
{
	MCL_MST_DESYNC_NO_ACTION,
	MCL_MST_DESYNC_DEC,
	MCL_MST_DESYNC_RAPID,
	MCL_MST_DESYNC_RAPID_SYNC,
} MCL_MST_DESYNC_TYPE;

typedef enum MCL_SLV_DESYNC_TYPE
{
	MCL_SLV_DESYNC_NO_ACTION,
	MCL_SLV_DESYNC_RESOLVE_SYNC,
} MCL_SLV_DESYNC_TYPE;

// ======== [Struct] ===========

typedef	struct MCL_SYSTEM_INFO {

	MCL_BOOL IsInitialized;
	U16 MaxAxisNum;
	U16 MaxGroupNum;
	U16 MaxBufferSize;
	FLT DDA_TIME;
} MCL_SYSTEM_INFO;
	
typedef	struct MCL_MPDATA {
	FLT	Feed;		// Speed [ command unit / sec]	
	FLT	Accel;		// Acceleration [ command unit / sec^2]	
	FLT	Decel;		// Acceleration [ command unit / sec^2]	
	FLT	VelStart;	// Start Speed [ command unit / sec]		
	FLT	VelEnd;		// Stop  Speed [ command unit / sec]		
	FLT	SFactor;	//The smoothness of acceleration. Range:[0~1]. 0:degrads to T-curve. 1:pure S-curve;  	
	FLT	OverlapRate;//0~100%
	MCL_OVERLAP_TYPE OverlapType; // Overlap type [Dec, Time, Dist]
	MCL_COORD_TYPE	Coord;	// Coordinate system [Abs, Inc]
	MCL_BOOL IsRunMaxVel;	// FALSE : return error if exceed axis maxV. TRUE : Use the max speed of the axes as the motion speed. Only valid in pos/lin
	MCL_BOOL IsRunMaxAcc;	// FALSE : return error if exceed axis maxAcc or maxDec. TRUE : Use the max acc/dec of the axes as the motion profile. Only valid in pos/lin
	MCL_BOOL IsVsVeOnly;	// Only use dist, Vs and Ve to calculate motion profile
#if FEAT_INPOS
	MCL_BOOL IsCheckInPos;
#endif
} MCL_MPDATA; // data for Motion Profile.

typedef	struct MCL_POS_DATA {
	FLT pos;		// target position. could be absolute/relative, depends on motion profile. 
	MCL_MPDATA mp;	// motion profile

} MCL_POS_DATA; //data for pos Command.

typedef	struct MCL_FILLET {
	FLT dist;	//Note: the meaning of this value depends on 'type' argument
	MCL_FIL_TYPE type;
} MCL_FILLET;

typedef	struct MCL_CIR_DATA {
	MCL_CIR_TYPE	CirType:3;
	MCL_ORI_TYPE	OriType:3;
	MCL_DIR_TYPE	Direction;
	MCL_PLANE_TYPE	Plane;

	I16				Angle;
	U16				ExLoop;
	FLT				Pitch;
	FLT				Center[3];
	FLT				Mid[6];
	FLT				End[6];
	FLT				Norm[3];
	FLT				Radius;
	FLT				Height;
	
} MCL_CIR_DATA; //data for Circle Command.

typedef	struct MCL_SPR_DATA {
	U32	InnerR;		//radius of inner circle.
	U32	OuterR;		//radius of outer circle.
	U32	Interval;	//distance between each loop.

	I32	Height;
	U32	Pitch;

} MCL_SPR_DATA; //data for Spiral Command.

typedef	struct MCL_VEL_DATA {
	FLT Vel;	// Velocity. could be negative. unit: 1/sec			
	FLT Acc;	// Acceleration and Deceleration. *** must be positive *** . unit: 1/sec^2	
} MCL_VEL_DATA; // data for Velocity Command.

typedef struct MCL_HMDATA {
	MCL_HOME_MODE Mode;
	MCL_DIR_TYPE Direction;
	FLT Accel;		// Acceleration and Deceleration. *** must be positive *** .
	FLT ShiftDist;
	FLT ShiftVel;
	FLT HighVel;	// The velocity of Phase 1.
	FLT LowVel;		// The velocity of Phase 2.
	FLT EscapeDist;	// The maximum distance to escape sensor. If set to 0, means no limit.
	FLT ReverseDist;
	FLT HomePosition;// The Position after homing process is done.
	U8  MultiEZ;
} MCL_HMDATA;


typedef	struct MCL_IO_DATA {
	U8 SlaveId;	// count from 0
	U8 ByteId;	// count from 0
	U8 Value;	// represent 8 IO bit. for Set_IO_Out: the expect value;  for Wait_IO_Inp: wait until hardware matches this value.
	U8 Mask;	// for each bit:  1: consider this bit;  0: ignore this bit.
} MCL_IO_DATA; // data for Set_IO_Out & Wait_IO_Inp Command.

typedef struct MCL_IO_BIT_DATA {
	U8 SlaveId;		// count from 0
	U8 ByteOffset;	// count from 0
	U8 BitNo;		// target bit number
	MCL_BOOL OnOff;	// value of the bit when active
} MCL_IO_BIT_DATA;

typedef	struct MCL_LCH_DATA {
	MCL_LCH_MODE	mode;
	MCL_IO_BIT_DATA	ioData;
	FLT				latchedPos;
	MCL_BOOL		IsTriggered;
	MCL_BOOL		IsTchProbe;
} MCL_LCH_DATA; // Latch Data

typedef union MCL_HW_STS {
	U8 value;
	struct {
		U8 PEL : 1;// Positive limit
		U8 MEL : 1;// Negative limit
		U8 ORG : 1;// Origin/Home
		U8 ALM : 1;// Alarm
		U8 EMG : 1;// Emergency
		U8 DISCONN : 1;// Disconnect
	} bits;
}MCL_HW_STS; //Hardware status

typedef struct {
	FLT pos;
	FLT vel;// [ command unit / sec]
	FLT timeMs;
}MCL_PVT_POINT;


// ====== callback functions (return: 0:success.)===========

// -- cmd done --
// called when cmd is deleted
// purpose: to inform user that a cmd is done.
typedef union CUnionCmd CUnionCmd;
typedef void (*fCB_CmdDone) (CUnionCmd* cmd, U8 mclObjId, MCL_ERR errCode);

// -- dda error --
// called when an error occurs in DDA cycle (ex.End Limit).
// purpose: to inform user that an error is occurred.
typedef void (*fCB_DDAErr) (U8 mclObjId, MCL_ERR errCode);


// -- Set Hardware Target Position --
// called at the end of every dda
// client function should set the value to hardware.
typedef int (*fCB_SetTrgPosHw) (U8 axisId, I32 value);

// -- Get Hardware Actual Position --
// called in the beginning of every dda
// client function should fill 'out_value'-- the Actual position of axis. (received from hardware)
typedef int (*fCB_GetActPosHw) (U8 axisId, I32* out_value);

// -- Get Hardware Status (ex.PEL, MEL, ORG, ALM, ...) --
// called in the beginning of every dda
// client function should fill 'out_value'-- the signal of axis. (received from hardware)
typedef int (*fCB_GetHwStatus)	(U8 axisId, MCL_HW_STS* out_value);

// -- SetAxisHwSts IO output --
// called in cmds that sets DO value. (ex. SetOutBit, CmpTrg)
// client function should set DO bit according to MCL_IO_DATA parameter.
// return 0 if no error.
typedef int (*fCB_SetIOOut) (MCL_IO_DATA* data);

// -- Get IO --
// called in cmds that needs DIO value, the 'type' means get input or output (0: input, 1:output).
// client function should fill 'out_value' -- the current input/output value(in byte) of the target slvId & byteId. 
// return 0 if no error.
typedef int (*fCB_GetIO) (U8 slvId, U8 byteId, U8 type, U8* out_value);

// -- Get IO Input --
// called in cmds that needs DI value. (ex. WaitInpBit, Latch)
// client function should fill 'out_value' -- the current IO input value(in byte) of the target slvId & byteId. 
// return 0 if no error.
typedef int (*fCB_GetIOInp) (U8 slvId, U8 byteId, U8* out_value);

// -- Set TouchProbe function --
// called in cmds that needs TouchProbe or EZ (ex. some mode of CSPHoming) 
// client function should switch On/Off hardware touchProbe function respectively. 
typedef int (*fCB_EnableTchProbe)	(U8 axisId, MCL_BOOL isEnable, MCL_PROBE_SRC Src, MCL_PROBE_MODE mode);

// -- Get TouchProbe enable status --
// called in cmds that needs TouchProbe or EZ (ex. some mode of CSPHoming) 
// client function should switch On/Off hardware touchProbe function respectively. 
typedef int (*fCB_IsTchProbeEnabled) (U8 axisId, MCL_BOOL* out_isEnabled);

// -- Get TouchProbe/EZ trigger Status --
// called in cmds that needs TouchProbe or EZ (ex. some mode of CSPHoming) 
// client function should fill 'out_value'--whether it is triggered, and the latched position.
// 'out_value' are zero by default.
typedef int (*fCB_GetTchProbe)	(U8 axisId, MCL_PROBE_SRC Src, MCL_BOOL* out_isTriggered, I32* out_LatchedPos);

// -- get condition result --
// this is used for Wait-command.
// when cmd is executed, this callback will be called every dda. The cmd execute until callback returns true.
typedef MCL_ERR (*fCB_IsConditionMet) (MCL_BOOL* out_result);

// -- Get Decel --
// called in dda when SetNewProfile. used in GM to stop at full circle position
// client function should return a new decel.
typedef FLT(*fCB_GetNewDec) (void);

typedef void (*fCB_TimeTrigger) ();
typedef void (*fCB_TimeTrgDone) (U32 trgNum);

#if FEAT_CMPTRG
	typedef enum
	{
		GREATER_THAN,
		LESS_THAN
	} MCL_CMP_MODE;

	typedef enum
	{
		MCL_TRG_ONESHOT,
		MCL_TRG_LEVEL,
		MCL_TRG_NONE
	} MCL_TRG_MODE;

	typedef struct MCL_CMP_COND
	{
		MCL_CMP_MODE CompareMode;
		I32 TrgCount;
		I32 StartPos;
		I32 IntervalDist;
	}MCL_CMP_COND;

	typedef struct MCL_CMP_ACT
	{
		MCL_TRG_MODE TriggerMode;
		U32 DurationTime;
	}MCL_CMP_ACT;

	typedef struct MCL_CMP_INFO
	{
		U16 MaxNum_CmpTrg;	// max available number of compare triggers.
		U16 MaxNum_Action;	// max available number of actions per each compare trigger.
	}MCL_CMP_INFO;

	// -- Compare Trigger Do Action --
	// called in CmpTrg cmd that sets DO value.
	// client function should convert MCL_IO_DATA to IO_DO or PDO_DO according to the type of slave
	typedef int (*fCB_DoAction) (MCL_IO_BIT_DATA* data);
#endif

#if FEAT_TIMETRG
	typedef enum MCL_COND //after timetrigger is enabled, check an extra condition to start or stop. 
	{
		MCL_COND_NONE,		//no extra condition.
		MCL_COND_BUFFER,	//start when any buffer is not empty;  stop when all buffer is empty.
	}MCL_COND;

	typedef struct _MCL_REC_DATA {
		I32 values[4];
	}MCL_REC_DATA;

	// -- Record specific value --
	// called in recorder that needs record value
	// 'type' and 'subType' is defined by client
	typedef int (*fCB_GetRecValue)	(U8 valueId, U16 type, U16 subType, I32* out_value);

	// -- Record send values --
	// called in recorder after record value, to sent recorded values to client
	typedef int (*fCB_SendRecValues) (MCL_REC_DATA recData);
#endif

#if FEAT_MPG
	typedef enum {
		MCL_MPG_SELECTION_ONE_HOT,	// One bit represents one axis
		MCL_MPG_SELECTION_BINARY	// Use 3 DI bits to represent up to 7 axes
	} MCL_MPG_SELECTION_MODE;	// MPG selection mode

	typedef enum {
		NO_FILTER,
		AVERAGE,
	} MCL_MPG_FILTER_MODE;

	typedef struct MCL_MPG_STATE {
		MCL_BOOL IsEnabled;
		I32 Encoder;
		FLT Counter;
		U8 SelectedAxis;
		FLT SelectedRatio;
	} MCL_MPG_STATE;

	// -- MPG read counter value --
	// called in MPG that needs counter value
	typedef int (*fCB_GetCntValue) (U8 slvId, U8 channel, I32* out_value);
#endif

#if FEAT_GANTRY
	typedef struct MCL_GANTRY_STATE
	{
		U8 MstId;
		U8 SlvId;
		MCL_BOOL IsStart;
	} MCL_GANTRY_STATE;
	typedef	struct MCL_GANTRY_COMP 
	{
		FLT Kp;	//0~1
		FLT Ki;	//0~5
	} MCL_GANTRY_COMP; // data for Velocity Command.

	// -- Gantry Componsation --
	typedef void (*fCB_GantryComp) (U8 mstId, U8 slvId, FLT velComp);
#endif

#if FEAT_PITCH_COMP

	typedef struct MCL_PITCH_COMP_DATA
	{
		FLT startPos; 	// the position of the first point [user unit]
		FLT interval;	// the distance between each point [user unit]. must >= 0
		U16 pointNum; 	// total number of error points. Range:[1~1024]
		FLT* compValues; // the array that contains the offset-values for each point. these values will be \A1\A5added to\A1\A6 the command position before send to servo. The array length should be pointNum. [user unit]

	} MCL_PITCH_COMP_DATA;

	typedef struct MCL_PITCH_COMP_DATA_2D
	{
		FLT startPos[2];	// The position of the first point of each axis [user unit]
		FLT interval[2];	// the distance between each point of each axis [user unit]. must >= 0
		U16 pointNum[2];	// total number of error points of each axis. Range:[1~1024]
		FLT** compValues;	// the 2D array that contains the offset-values for each point. these values will be \A1\A5added to\A1\A6 the command position before send to servo. The array length should be pointNum. [user unit]
	} MCL_PITCH_COMP_DATA_2D;
#endif

#ifdef __cplusplus
}
#endif
#endif
