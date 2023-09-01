#ifndef _MCL_FEATURES
#define _MCL_FEATURES


// define Features that can be turned on/off. (1:on 0:off)  (turn off to save memory/time) 
#define FEAT_BACKLASH	1 // Backlash compensation.
#define FEAT_LATCH		1 // Latch (record position when DI triggered)
#define FEAT_CMDDONE	1 // call User-Callback function when cmd is done (removed from buffer).
#define FEAT_INPOS		1 // In-Position check.
#define FEAT_CMPTRG		1 // CompareTrigger.
#define FEAT_TIMETRG	1 // TimeTrigger.
#define FEAT_MPG		1 // Manual Pulse Generator
#define FEAT_SYNC		1 // Synchronization.
#define FEAT_GANTRY		1 // Gantry componsation
#define FEAT_POS_CHANGE	1 // Cmd Position- posChange
#define FEAT_FILLET		1 
#define FEAT_SINGLE_TURN 1 // let pos of rotational axis always kept in a specified range.
#define FEAT_ROBC		0 
// ============================


#endif
