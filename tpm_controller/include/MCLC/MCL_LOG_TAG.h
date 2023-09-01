typedef enum MCL_LOG_TAG_TYPE
{
	MCL_LOG_TAG_ALL = -1, // turn on/off all tags

	MCL_LOG_TAG_SYS,	//system
	MCL_LOG_TAG_AXIS,	//operation on _axis
	MCL_LOG_TAG_GRP,	//operation on group
	MCL_LOG_TAG_CMD,	//cmd event in cmdBuffer, such as Create/Init/Finish/UnInit
	MCL_LOG_TAG_MOV,	//movement cmd and related calculation. such as MotionProfile
	MCL_LOG_TAG_EXE,	//Executor
	MCL_LOG_TAG_CSPHM,	//CSP Homing
	MCL_LOG_TAG_CmpTrg,	//Compare Trigger
	MCL_LOG_TAG_LCH,	//Latch
	MCL_LOG_TAG_MPG,	//Manual Pulse Generator

	MCL_LOG_TAG_GM,		//GMIPE 
}MCL_LOG_TAG_TYPE;