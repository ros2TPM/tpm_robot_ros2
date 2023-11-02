#ifndef RPIROBIF_H_
#define RPIROBIF_H_

#include "type_def.h"
#include "MCL_DEFC.h"
#include "ROB_DEF.h"

////////////////////////////////////////////////////////////////////////////////
#define MAX_AXIS_PER_ROBOT 6
#define MAX_MSG_LENGTH  128

////////////////////////////////////////////////////////////////////////////////
#define MAX_DIO_SLV_PER_ROBOT   8

typedef struct ST_MCLC_AXIS
{
    // Note: use GetAxes() to get an array of CAxis objects directly. 
    // Instead of call mclc_axis_get_xxx one by one.
    F32 trgPosCmd;
    F32 actPosCmd;
    F32 velocity;
    U32 bufDepth;
} ST_MCLC_AXIS_t;

typedef struct {
    ST_MCLC_AXIS_t axes[MAX_AXIS_PER_ROBOT];
    I32 lastErr;
} ST_RIDT_MCLC, ST_RIDT_MCLC_t;

typedef struct {

    F32 axis[MAX_AXIS_PER_ROBOT]; //robc_get_axis
    F32 pose[MAX_AXIS_PER_ROBOT]; //robc_get_pose
    U32 bufDepth;                 //robc_get_buffer_depth
    U32 nowCmdId;                 //robc_get_now_cmd_id
    I32 lastErr;                  //robc_get_last_error

} ST_RIDT_ROBC, ST_RIDT_ROBC_t;

typedef struct {

    U32 ioSts;
    I32 encPos;

} ST_RIDT_MNET_M1A, ST_RIDT_MNET_M1A_t;

typedef struct {

    U8 port[4];

} ST_RIDT_MNET_DIO, ST_RIDT_MNET_DIO_t;

typedef struct {

    union {
        ST_RIDT_MCLC_t mclc;
        ST_RIDT_ROBC_t robc;
    };
    ST_RIDT_MNET_M1A_t m1a[MAX_AXIS_PER_ROBOT];
    ST_RIDT_MNET_DIO_t dio[MAX_DIO_SLV_PER_ROBOT];

} ST_RIDT, ST_RIDT_t;


////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif
    // ======================================
    // Note: unless specified, all angle unit is [deg] (not radius);  all length unit is [mm].

    I16 rpi_robc_version(U32* Ver);
    I16 rpi_robc_init(ROB_KIN_TYPE Type, FLT a[], FLT alpha[], FLT d[], FLT theta[], FLT thetaShift[], FLT posLimit[], FLT negLimit[], FLT PulsePerDeg[]);

    // ===== Add Command functions ==========
    I16 rpi_robc_move_p2p_axis(U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, U8 Mask);
    I16 rpi_robc_move_p2p_pose(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame);
    I16 rpi_robc_move_p2p_pt(U32 CmdId, MCL_MPDATA* MPData, FLT* Axis, FLT* Pose, U8 baseId);

    I16 rpi_robc_move_lin_pose(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 Mask, ROB_FRAME_TYPE frame);
    I16 rpi_robc_move_lin_pt(U32 CmdId, MCL_MPDATA* MPData, FLT* Pose, U8 baseId);
    I16 rpi_robc_move_cir_ME(U32 CmdId, MCL_MPDATA* MPData, FLT* mid, FLT* end, MCL_ORI_TYPE oriType, I16 angle, I16 exLoop);

    I16 rpi_robc_delay(U32 CmdId, U32 delayTimeMs);
    I16 rpi_robc_set_io_out(U32 CmdId, MCL_IO_DATA* ioData);
    I16 rpi_robc_wait_io_inp(U32 CmdId, MCL_IO_DATA* ioData);

    // Note: Dist, Vel & Acc always > 0; Dist==0 means infinite dist.
    I16 rpi_robc_jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc);
    I16 rpi_robc_jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame);

    I16 rpi_robc_set_pvt_data(U8 AxisId, U32 PointNum, MCL_PVT_POINT* PvtPoints);
    I16 rpi_robc_move_pvt(U32 CmdId, FLT StopDec, U8 Mask);

    // ===== Command control functions =========
    I16 rpi_robc_hold();
    I16 rpi_robc_resume();
    I16 rpi_robc_feedrate(FLT Feedrate); // value: [0~1]
    I16 rpi_robc_stop(MCL_STOP_TYPE type);
    I16 rpi_robc_block_buffer(MCL_BOOL isToBlock); // when buffer is blocked, it will not provide next cmd to executor.

    // ====== Get / Set Property functions ==========
    I16 rpi_robc_get_axis(FLT* out_values); //return an array. [unit:mm or deg]
    I16 rpi_robc_get_pose(FLT* out_values); //return an array (xyzabc). [unit:mm or deg]

    I16 rpi_robc_get_buffer_depth(U32* Depth);
    I16 rpi_robc_get_now_cmd_id(U32* ID);
    I16 rpi_robc_get_last_error();
    I16 rpi_robc_get_last_error_msg(char* Msg);
    I16 rpi_robc_clear_last_error();

    I16 rpi_robc_set_axis_position(U8 AxisId, FLT Value); //[unit:mm or deg]
    I16 rpi_robc_set_base(FLT* Pose); //(xyzabc) [unit:mm or deg]
    I16 rpi_robc_set_tool(FLT* Pose); //(xyzabc) [unit:mm or deg]

    // ====== System config ===============
    I16 rpi_robc_check_cmd_valid(MCL_BOOL isEnable); //whether to check IK/Angle-Limit when adding new cmd. default is TRUE.
    I16 rpi_robc_check_angle_limit(MCL_BOOL isEnable); //whether to check Angle-Limit (at any time). default is TRUE.
    I16 rpi_robc_check_axis_overspeed(MCL_BOOL isEnable); //whether to check Axis-Overspeed (at dda cycle). default is TRUE.

    ////////////////////////////////////////////////////////////////////////////
    I16 rpi_mclc_version(U32* Ver);
    I16 rpi_mclc_init(void);
    I16 rpi_mclc_uninit(void);
    I16 rpi_mclc_axis_move_pos(U8 AxisId, U32 CmdId, MCL_MPDATA* MPData, F32 Position);
    I16 rpi_mclc_axis_move_pvt(U8 AxisId, U32 CmdId, U32 pointNum, MCL_PVT_POINT* pvtPoints, F32 stopDec);
    I16 rpi_mclc_axis_resume(U8 AxisId);
    I16 rpi_mclc_axis_hold(U8 AxisId);
    I16 rpi_mclc_axis_set_feedrate(U8 AxisId, F32 Feedrate);
    I16 rpi_mclc_axis_stop(U8 AxisId, MCL_STOP_TYPE type);
    I16 rpi_mclc_axis_set_position(U8 AxisId, F32 position);
    I16 rpi_mclc_axis_get_buffer_depth(U8 AxisId, U32* Depth);
    I16 rpi_mclc_axis_get_trgPosCmd(U8 AxisId, F32* out_value);
    I16 rpi_mclc_axis_get_actPosCmd(U8 AxisId, F32* out_value);

    ////////////////////////////////////////////////////////////////////////////
    I16 rpi_ri_init(U8 alg_type); //0:MCLC, 1:ROBC
    I16 rpi_ri_deinit(void);
    I16 rpi_ri_mnet_m1a_init(U16 RingNo, U16* SlaveIPs);
    I16 rpi_ri_mnet_m1a_enable(U8 Enable);
    I16 rpi_ri_mnet_dio_init(U16 RingNo, U16 SlaveIP, U8 IoSlaveId);
    I16 rpi_ri_set_tdda_ratio(F32 TddaRatio);
    I16 rpi_ri_sta_enable(U8 Enable);
    I16 rpi_ri_set_is_apply_to_real_motor(U8 Enable);
    I16 rpi_ri_get_data(ST_RIDT_t* Values);

#ifdef __cplusplus
}
#endif

#endif /* RPIROBIF_H_ */
