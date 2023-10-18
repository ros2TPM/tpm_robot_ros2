#ifndef _MCL_FEATURES
#define _MCL_FEATURES


// define Features that can be turned on/off. (1:on 0:off)  (turn off to save memory/time) 
#define FEAT_BACKLASH	0 // Backlash compensation.
#define FEAT_LATCH		0 // Latch (record position when DI triggered)
#define FEAT_CMDDONE	1 // call User-Callback function when cmd is done (removed from buffer).
#define FEAT_INPOS		1 // In-Position check.
#define FEAT_CMPTRG		0 // CompareTrigger.
#define FEAT_TIMETRG	0 // TimeTrigger.
#define FEAT_POS_CHANGE	0 // Cmd Position- posChange
#define FEAT_FILLET		0 
#define FEAT_ROBC		1 
// ============================


#endif
