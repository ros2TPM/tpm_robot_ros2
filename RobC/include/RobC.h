#ifndef _ROBC_H
#define _ROBC_H

#include "ROB_DEF.h"
#include "MCL_DEFC.h" 

// ==== Compiler specific definition =====
#ifdef _WIN32
    #ifdef  ROBC_EXPORTS
    #define ROBC_API __declspec(dllexport)
    #else
    #define ROBC_API __declspec(dllimport)
    #endif
#else
    #define ROBC_API
#endif
//======================================
//Note: unless specified, all angle unit is [deg] (not radius);  all length unit is [mm].
#ifdef __cplusplus
extern "C" {
#endif

    ROBC_API U32     robc_version();
    ROBC_API ROB_ERR robc_init(ROB_KIN_TYPE Type, 
        FLT a[], FLT alpha[], FLT d[], FLT theta[], 
        FLT thetaShift[], FLT posLimit[], FLT negLimit[], FLT PulsePerDeg[]);

    ROBC_API ROB_ERR robc_set_dda_time(FLT ddaMs); //default: 1ms
    ROBC_API ROB_ERR robc_dda_cycle();

    // ===== Add Command functions ==========
    ROBC_API ROB_ERR robc_move_p2p_axis (U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, U8 Mask);
    ROBC_API ROB_ERR robc_move_p2p_pose (U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame);
    ROBC_API ROB_ERR robc_move_p2p_pt   (U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, FLT* Pose, U8 baseId);

    ROBC_API ROB_ERR robc_move_lin_pose (U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame);
    ROBC_API ROB_ERR robc_move_lin_pt   (U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 baseId);
    ROBC_API ROB_ERR robc_move_cir_ME   (U32 CmdId, MCL_MPDATA* MPData, FLT* mid, FLT* end, MCL_ORI_TYPE oriType, I16 angle, I16 exLoop);

    ROBC_API ROB_ERR robc_delay         (U32 CmdId, U32 delayTimeMs);
    ROBC_API ROB_ERR robc_set_io_out    (U32 CmdId, MCL_IO_DATA* ioData);
    ROBC_API ROB_ERR robc_wait_io_inp   (U32 CmdId, MCL_IO_DATA* ioData);

    // Note: Dist, Vel & Acc always > 0; Dist==0 means infinite dist.
    ROBC_API ROB_ERR robc_jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc);
    ROBC_API ROB_ERR robc_jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame);

    ROBC_API ROB_ERR robc_set_pvt_data (U8 AxisId, U32 PointNum, MCL_PVT_POINT* PvtPoints);
    ROBC_API ROB_ERR robc_move_pvt     (U32 CmdId, FLT StopDec, U8 Mask);

    // ===== Command control functions =========
    ROBC_API void    robc_hold          ();
    ROBC_API void    robc_resume        ();
    ROBC_API void    robc_feedrate      (FLT Feedrate); // value: [0~1]             
    ROBC_API void    robc_stop          (MCL_STOP_TYPE type);
    ROBC_API void    robc_block_buffer  (MCL_BOOL isToBlock); // when buffer is blocked, it will not provide next cmd to executor.
    
    // ====== Get / Set Property functions ==========
    ROBC_API void    robc_get_axis_pulses       (I32* out_values); //return an array. [unit:pulse sent to driver per dda]
    ROBC_API void    robc_get_axis              (FLT* out_values); //return an array. [unit:mm or deg]
    ROBC_API void    robc_get_pose              (FLT* out_values); //return an array (xyzabc). [unit:mm or deg]

    ROBC_API U32     robc_get_buffer_depth      ();
    ROBC_API U32     robc_get_now_cmd_id        ();
    ROBC_API MCL_MOT_STATE robc_get_motion_status();
    ROBC_API ROB_ERR robc_get_last_error        ();
    ROBC_API char*   robc_get_last_error_msg    ();
    ROBC_API void    robc_clear_last_error      ();

    ROBC_API void    robc_set_axis_position     (U8 AxisId, FLT Value); // for one axis. [unit:mm or deg]
    ROBC_API void    robc_set_axes_positions    (FLT* Values);          // for all axes. [unit:mm or deg]
    ROBC_API void    robc_set_base              (FLT* Pose); //(xyzabc) [unit:mm or deg]
    ROBC_API void    robc_set_tool              (FLT* Pose); //(xyzabc) [unit:mm or deg]

        // ====== Callback setting ===============
    ROBC_API void    robc_setCB_setIOOut        (fCB_SetIOOut cbFunc);
    ROBC_API void    robc_setCB_getIOInp        (fCB_GetIOInp cbFunc);
    ROBC_API void    robc_setCB_getHwStatus     (fCB_GetHwStatus cbFunc);

    // ====== System config ===============
    ROBC_API void    robc_check_cmd_valid       (MCL_BOOL isEnable); //whether to check IK/Angle-Limit when adding new cmd. default is TRUE. 
    ROBC_API void    robc_check_angle_limit     (MCL_BOOL isEnable); //whether to check Angle-Limit (at any time). default is TRUE.
    ROBC_API void    robc_check_axis_overspeed  (MCL_BOOL isEnable); //whether to check Axis-Overspeed (at dda cycle). default is TRUE.

#ifdef __cplusplus
}
#endif

#endif // _ROBC_H
