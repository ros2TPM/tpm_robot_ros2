#include "RPiRobIF.h"

    I16 rpi_robc_version(U32* Ver)
    { return -4; }
    I16 rpi_robc_init(ROB_KIN_TYPE Type, FLT a[], FLT alpha[], FLT d[], FLT theta[], FLT thetaShift[], FLT posLimit[], FLT negLimit[], FLT PulsePerDeg[])
    { return -4; }

    // ===== Add Command functions ==========
    I16 rpi_robc_move_p2p_axis(U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, U8 Mask)
    { return -4; }
    I16 rpi_robc_move_p2p_pose(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame)
    { return -4; }
    I16 rpi_robc_move_p2p_pt(U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, FLT* Pose, U8 baseId)
    { return -4; }

    I16 rpi_robc_move_lin_pose(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame)
    { return -4; }
    I16 rpi_robc_move_lin_pt(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 baseId)
    { return -4; }
    I16 rpi_robc_move_cir_ME(U32 CmdId, MCL_MPDATA* MPData, FLT* mid, FLT* end, MCL_ORI_TYPE oriType, I16 angle, I16 exLoop)
    { return -4; }

    I16 rpi_robc_delay(U32 CmdId, U32 delayTimeMs)
    { return -4; }
    I16 rpi_robc_set_io_out(U32 CmdId, MCL_IO_DATA* ioData)
    { return -4; }
    I16 rpi_robc_wait_io_inp(U32 CmdId, MCL_IO_DATA* ioData)
    { return -4; }

    // Note: Dist, Vel & Acc always > 0; Dist==0 means infinite dist.
    I16 rpi_robc_jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc)
    { return -4; }
    I16 rpi_robc_jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame)
    { return -4; }

    // ===== Command control functions =========
    I16 rpi_robc_hold() { return -4; }
    I16 rpi_robc_resume() { return -4; }
    I16 rpi_robc_feedrate(FLT Feedrate) { return -4; } // value: [0~1]
    I16 rpi_robc_stop(MCL_STOP_TYPE type) { return -4; }
    I16 rpi_robc_block_buffer(MCL_BOOL isToBlock) { return -4; } // when buffer is blocked, it will not provide next cmd to executor.

    // ====== Get / Set Property functions ==========
    I16 rpi_robc_get_axis(FLT* out_values) { return -4; } //return an array. [unit:mm or deg]
    I16 rpi_robc_get_pose(FLT* out_values) { return -4; } //return an array (xyzabc). [unit:mm or deg]

    I16 rpi_robc_get_buffer_depth(I32* Depth) { return -4; }
    I16 rpi_robc_get_now_cmd_id(U32* ID) { return -4; }
    I16 rpi_robc_get_last_error() { return -4; }
    I16 rpi_robc_get_last_error_msg(char* Msg) { return -4; }
    I16 rpi_robc_clear_last_error() { return -4; }

    I16 rpi_robc_set_axis_position(U8 AxisId, FLT Value) { return -4; } //[unit:mm or deg]
    I16 rpi_robc_set_base(FLT* Pose) { return -4; } //(xyzabc) [unit:mm or deg]
    I16 rpi_robc_set_tool(FLT* Pose) { return -4; } //(xyzabc) [unit:mm or deg]

    // ====== System config ===============
    I16 rpi_robc_check_cmd_valid(MCL_BOOL isEnable) { return -4; } //whether to check IK/Angle-Limit when adding new cmd. default is TRUE.
    I16 rpi_robc_check_angle_limit(MCL_BOOL isEnable) { return -4; } //whether to check Angle-Limit (at any time). default is TRUE.
    I16 rpi_robc_check_axis_overspeed(MCL_BOOL isEnable) { return -4; } //whether to check Axis-Overspeed (at dda cycle). default is TRUE.

    ////////////////////////////////////////////////////////////////////////////
    I16 rpi_ri_init(void) { return -4; }
    I16 rpi_ri_mnet_m1a_init(U16 RingNo, U16* SlaveIPs) { return -4; }
    I16 rpi_ri_mnet_m1a_enable(U8 Enable) { return -4; }
    I16 rpi_ri_mnet_dio_init(U16 RingNo, U16 SlaveIP, U8 IoSlaveId) { return -4; }
    I16 rpi_ri_set_tdda_ratio(F32 TddaRatio) { return -4; }
    I16 rpi_ri_sta_enable(U8 Enable) { return -4; }
    I16 rpi_ri_set_is_apply_to_real_motor(U8 Enable) { return -4; }
    I16 rpi_ri_get_data(ST_RIDT_t* Values) { return -4; }