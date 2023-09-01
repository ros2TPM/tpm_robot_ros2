#ifndef _MCLC_H
#define _MCLC_H


#include "MCL_Constant.h"
#include "MCL_DEFC.h"
#include "Custom/MCL_FEATURES.h"
#include "CLogPrint.h"

#ifdef __cplusplus
extern "C" {
#endif

    // ===== under test (for ECP) ===
    typedef struct CAxis CAxis;
    typedef struct CGroup CGroup;
    typedef struct CExecutor CExecutor;
    typedef struct CCmdAdder CCmdAdder;

    CAxis*     GetAxes();
    CGroup*    GetGrps();
    CExecutor* GetExes();
    CCmdAdder* GetCmdAdders();

    // ===== Main System =====
    MCLC_API U32     mclc_version();
    MCLC_API MCL_SYSTEM_INFO  mclc_get_systemInfo();
    MCLC_API void    mclc_init();
    MCLC_API void    mclc_uninit();
    MCLC_API MCL_ERR mclc_dda_cycle();

    MCLC_API void    mclc_set_cmdExeTimer(MCL_BOOL isEnable);
    MCLC_API int     mclc_get_lastErrMsg(char* out_msg, int len);

    // ===== Setting -- Available objects =====
    MCLC_API MCL_ERR mclc_set_available_ObjectNum (U16 maxAxis, U16 maxGrp, U16 maxAxisPerGrp);
    MCLC_API MCL_ERR mclc_set_available_BufferNum (I32 maxBuf);//-1: use max size.

    // ===== Setting -- Log ==========
    MCLC_API void    mclc_log_set_tag_enable(MCL_LOG_TAG_TYPE tagType, MCL_BOOL isEnable);
    MCLC_API void    mclc_log_set_level     (MCL_LOG_LEVEL level);

    // ===== Setting -- Callbacks ==========
    MCLC_API void    mclc_setCB_setTrgPosHw  (fCB_SetTrgPosHw cbFunc);
    MCLC_API void    mclc_setCB_getActPosHw  (fCB_GetActPosHw cbFunc);
    MCLC_API void    mclc_setCB_getHwStatus  (fCB_GetHwStatus cbFunc);
    MCLC_API void    mclc_setCB_getTchProbe  (fCB_GetTchProbe cbFunc);
    MCLC_API void    mclc_setCB_enableTchProbe(fCB_EnableTchProbe cbFunc);
    MCLC_API void    mclc_setCB_IsTchProbeEnabled(fCB_IsTchProbeEnabled cbFunc);

    MCLC_API void    mclc_setCB_setIOOut     (fCB_SetIOOut cbFunc);
    MCLC_API void    mclc_setCB_getIOInp     (fCB_GetIOInp cbFunc);

    MCLC_API void    mclc_setCB_getDec_atNewMP(fCB_GetNewDec cbFunc);

    MCLC_API MCL_ERR mclc_setCB_ddaErr       (fCB_DDAErr cbFunc); // can add multiple callbacks.
    MCLC_API MCL_ERR mclc_rmvCB_ddaErr       (fCB_DDAErr cbFunc); // remove callback.

    #if FEAT_CMDDONE
    MCLC_API void    mclc_setCB_cmdDone      (fCB_CmdDone cbFunc);
    #endif

    // ===== Axis Setting API ==========
    MCLC_API MCL_ERR mclc_axis_set_ratio            (U8 AxisId, FLT pulsePerUnit);
    MCLC_API MCL_ERR mclc_axis_get_ratio            (U8 AxisId, FLT* out_value);
    
        // Limit switch & Software limit
    MCLC_API MCL_ERR mclc_axis_set_LSDec            (U8 AxisId, FLT dec);
    MCLC_API MCL_ERR mclc_axis_set_HwLSMode         (U8 AxisId, MCL_LS_MODE mode);
    MCLC_API MCL_ERR mclc_axis_set_SoftLSMode       (U8 AxisId, MCL_LS_MODE mode);
    MCLC_API MCL_ERR mclc_axis_set_SoftLimitPos     (U8 AxisId, FLT pos, FLT neg);

    MCLC_API MCL_ERR mclc_axis_set_maxVel           (U8 AxisId, FLT  maxVel);
    MCLC_API MCL_ERR mclc_axis_get_maxVel           (U8 AxisId, FLT* out_value);
    MCLC_API MCL_ERR mclc_axis_set_maxAccDec        (U8 AxisId, FLT maxAcc, FLT maxDec);
    MCLC_API MCL_ERR mclc_axis_get_maxAccDec        (U8 AxisId, FLT* out_maxAcc, FLT* out_maxDec);

    #if FEAT_SINGLE_TURN
    MCLC_API MCL_ERR mclc_axis_set_rotary           (U8 AxisId, MCL_BOOL isEnable, U32 unitPerTurn);
    MCLC_API MCL_ERR mclc_axis_get_rotary_count     (U8 AxisId, I32* out_turn);
    MCLC_API MCL_ERR mclc_axis_reset_rotary_count   (U8 AxisId);
    #endif

    #if FEAT_INPOS
    MCLC_API MCL_ERR mclc_axis_set_inPosWindow      (U8 AxisId, U16 inPosWindow);
    #endif

    #if FEAT_BACKLASH
    MCLC_API MCL_ERR mclc_axis_backlash_set_Linear  (U8 AxisId, I16 backlashValue, U16 compTimeMs);
    MCLC_API MCL_ERR mclc_axis_backlash_set_TCurve  (U8 AxisId, I16 backlashValue, U16 accTimeMs, U16 constTimeMs, U16 decTimeMs);
    MCLC_API MCL_ERR mclc_axis_backlash_enable      (U8 AxisId, MCL_BOOL isEnable);
    MCLC_API MCL_ERR mclc_axis_backlash_get_comp    (U8 AxisId, FLT* out_value);
    #endif

    // ===== Setting -- Group API ==========
    MCLC_API MCL_ERR mclc_grp_add_axis      (U8 GrpId, U8 AxisId);
    MCLC_API MCL_ERR mclc_grp_clear_axes    (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_enable        (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_disable       (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_get_isEnable  (U8 GrpId, MCL_BOOL* out_value);
    MCLC_API MCL_ERR mclc_grp_get_axes_info (U8 GrpId, I32* out_AxesNum, I32* out_AxisIds); //Note:out_AxisIds can be NULL.

    // ===== Add Command API ==========
    //under test
    MCLC_API MCL_ERR mclc_set_cmdDone_info  (U64 fptr, void* listener);
        // single Axis Command
    MCLC_API MCL_ERR mclc_axis_move_pos     (U8 AxisId, U32 CmdId, MCL_MPDATA* mp, FLT Position);
    MCLC_API MCL_ERR mclc_axis_move_vel     (U8 AxisId, U32 CmdId, FLT Vel, FLT Acc); // Note: call this API again will override the previous cmd.
    MCLC_API MCL_ERR mclc_axis_move_pvt     (U8 AxisId, U32 CmdId, U32 pointNum, MCL_PVT_POINT* pvtPoints, FLT stopDec);
    MCLC_API MCL_ERR mclc_axis_move_home    (U8 AxisId, U32 CmdId, MCL_HMDATA* HMData);

    MCLC_API MCL_ERR mclc_axis_delay        (U8 AxisId, U32 CmdId, U32 delayTimeMs);
    MCLC_API MCL_ERR mclc_axis_set_io_out   (U8 AxisId, U32 CmdId, MCL_IO_DATA* ioData);
    MCLC_API MCL_ERR mclc_axis_wait_io_inp  (U8 AxisId, U32 CmdId, MCL_IO_DATA* ioData); //Note: this cmd will wait(suspend) until the hardware IO input matches ioData.value.
    MCLC_API MCL_ERR mclc_axis_wait         (U8 AxisId, U32 CmdId, fCB_IsConditionMet cbFunc, U32 timeoutMs);

        // Group Command
    MCLC_API MCL_ERR mclc_grp_move_lin       (U8 GrpId, U32 CmdId, MCL_MPDATA* mp, FLT* Positions, MCL_FILLET* fillet);
    MCLC_API MCL_ERR mclc_grp_move_lin_mask  (U8 GrpId, U32 CmdId, MCL_MPDATA* mp, FLT* Positions, U64 mask, MCL_FILLET* fillet);
    MCLC_API MCL_ERR mclc_grp_move_cirCE_2D  (U8 GrpId, U32 CmdId, FLT* center_rel, FLT* end, MCL_DIR_TYPE dir, MCL_PLANE_TYPE plane, U16 exLoop, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_cirRE_2D  (U8 GrpId, U32 CmdId, FLT  radius,     FLT* end, MCL_DIR_TYPE dir, MCL_PLANE_TYPE plane, U16 exLoop, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_helixCE_2D(U8 GrpId, U32 CmdId, FLT* center_rel, FLT* end, FLT height, MCL_DIR_TYPE dir, MCL_PLANE_TYPE plane, U16 exLoop, FLT pitch, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_helixRE_2D(U8 GrpId, U32 CmdId, FLT  radius,     FLT* end, FLT height, MCL_DIR_TYPE dir, MCL_PLANE_TYPE plane, U16 exLoop, FLT pitch, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_cirME     (U8 GrpId, U32 CmdId, FLT* mid   , FLT* end,                   U16 exLoop, MCL_FILLET* fillet, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_cirCE     (U8 GrpId, U32 CmdId, FLT* center, FLT* end, MCL_DIR_TYPE dir, U16 exLoop, MCL_FILLET* fillet, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_cirCN     (U8 GrpId, U32 CmdId, FLT* center, FLT* norm, I16 angle,       U16 exLoop, MCL_FILLET* fillet, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_helixME   (U8 GrpId, U32 CmdId, FLT* mid   , FLT* end,                   U16 exLoop, FLT height, FLT pitch, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_helixCE   (U8 GrpId, U32 CmdId, FLT* center, FLT* end, MCL_DIR_TYPE dir, U16 exLoop, FLT height, FLT pitch, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_helixCN   (U8 GrpId, U32 CmdId, FLT* center, FLT* norm, I16 angle,       U16 exLoop, FLT height, FLT pitch, MCL_MPDATA* mp);
    MCLC_API MCL_ERR mclc_grp_move_spr       (U8 GrpId, U32 CmdId, MCL_MPDATA* MPData, MCL_SPR_DATA* SprData);

    MCLC_API MCL_ERR mclc_grp_delay         (U8 GrpId, U32 CmdId, U32 delayTimeMs);
    MCLC_API MCL_ERR mclc_grp_set_io_out    (U8 GrpId, U32 CmdId, MCL_IO_DATA* ioData);
    MCLC_API MCL_ERR mclc_grp_wait_io_inp   (U8 GrpId, U32 CmdId, MCL_IO_DATA* ioData); //Note: this cmd will wait(suspend) until the hardware IO input matches ioData.value.
    MCLC_API MCL_ERR mclc_grp_wait          (U8 GrpId, U32 CmdId, fCB_IsConditionMet cbFunc, U32 timeoutMs);

    // ===== Command Manipulate API =========
    MCLC_API MCL_ERR mclc_axis_hold             (U8 AxisId);
    MCLC_API MCL_ERR mclc_axis_resume           (U8 AxisId);
    MCLC_API MCL_ERR mclc_axis_set_feedrate     (U8 AxisId, FLT Feedrate);       // value: [0~1]
    MCLC_API MCL_ERR mclc_axis_stop             (U8 AxisId, MCL_STOP_TYPE type); // can stop all kinds of cmds. will also clear buffer.[Caution]:Need 1 dda to operate.
    MCLC_API MCL_ERR mclc_axis_skipWait         (U8 AxisId);                     // skip wait Input Bit
    MCLC_API MCL_ERR mclc_axis_block_buffer     (U8 AxisId, MCL_BOOL isToBlock); // when buffer is blocked, it will not provide next cmd to executor.
    MCLC_API MCL_ERR mclc_axis_pos_change       (U8 AxisId, MCL_POS_DATA* posData); // change the current executing pos-cmd to a new posData. 

    MCLC_API MCL_ERR mclc_grp_hold              (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_resume            (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_set_feedrate      (U8 GrpId, FLT Feedrate);       // value: [0~1]
    MCLC_API MCL_ERR mclc_grp_stop              (U8 GrpId, MCL_STOP_TYPE type); // can stop all kinds of cmds. will also clear buffer.[Caution]:Need 1 dda to operate.
    MCLC_API MCL_ERR mclc_grp_skipWait          (U8 GrpId);                     // skip wait Input Bit
    MCLC_API MCL_ERR mclc_grp_clear_buffer      (U8 GrpId);
    MCLC_API MCL_ERR mclc_grp_block_buffer      (U8 GrpId, MCL_BOOL isToBlock); // when buffer is blocked, it will not provide next cmd to executor.

        //=== object version: union of Axis & Group. (Axis Id: [0~AxisNum-1], Grp Id: [AxisNum ~ AxisNum + GrpNum-1]
    MCLC_API MCL_ERR mclc_obj_stop              (U8 objId, MCL_STOP_TYPE type);
    MCLC_API MCL_ERR mclc_obj_clearNonRunningCmds(U8 objId);
    MCLC_API MCL_ERR mclc_obj_get_buffer_depth  (U8 objId, U32* out_value);

    // ===== Buffer Manipulate API =========
    MCLC_API MCL_ERR mclc_axis_get_buffer_depth (U8 AxisId, U32* out_value);
    MCLC_API MCL_ERR mclc_axis_get_feedrate     (U8 AxisId, FLT* out_value);
    MCLC_API MCL_ERR mclc_axis_get_now_cmd_id   (U8 AxisId, U32* out_value);
    MCLC_API MCL_ERR mclc_axis_get_now_cmd_type (U8 AxisId, MCL_CMD_TYPE* out_value);
    MCLC_API MCL_ERR mclc_axis_get_motion_status(U8 AxisId, MCL_MOT_STATE* out_value);

    MCLC_API MCL_ERR mclc_grp_get_buffer_depth  (U8 GrpId, U32* out_value);
    MCLC_API MCL_ERR mclc_grp_get_feedrate      (U8 GrpId, FLT* out_value);
    MCLC_API MCL_ERR mclc_grp_get_now_cmd_id    (U8 GrpId, U32* out_value);
    MCLC_API MCL_ERR mclc_grp_get_now_cmd_type  (U8 GrpId, MCL_CMD_TYPE* out_value);
    MCLC_API MCL_ERR mclc_grp_get_motion_status (U8 GrpId, MCL_MOT_STATE* out_value);

    MCLC_API MCL_ERR mclc_obj_get_now_cmd_id    (U8 ObjId, U32* out_value);
    MCLC_API MCL_ERR mclc_obj_get_now_cmd_type  (U8 ObjId, U32* out_value);

    // ====== Get / SetAxisHwSts Property API ==========   
    MCLC_API MCL_ERR mclc_axis_set_position     (U8 AxisId, FLT Position);// set trgPosCmd to new value. trgPosHw will not change.
    MCLC_API MCL_ERR mclc_grp_set_position      (U8 GrpId , FLT*Positions);

    MCLC_API MCL_ERR mclc_axis_get_pulse        (U8 AxisId, I32* out_value);
    MCLC_API MCL_ERR mclc_axis_get_trgPosCmd    (U8 AxisId, FLT* out_value); // target command position. 
    MCLC_API MCL_ERR mclc_axis_get_trgPosHw_flt (U8 AxisId, FLT* out_value); 
    MCLC_API MCL_ERR mclc_axis_get_trgPosHw_int (U8 AxisId, I32* out_value); // final targetPosition send to Hardware.(after compensation)
    MCLC_API MCL_ERR mclc_axis_get_actPosHw     (U8 AxisId, I32* out_value); // actual Position directly receive from Hardware.
    MCLC_API MCL_ERR mclc_axis_get_actPosCmd    (U8 AxisId, FLT* out_value); // processed actual position (ex. after un-Compensate, un-ratio)
    MCLC_API MCL_ERR mclc_axis_get_homeOffset   (U8 AxisId, FLT* out_value);
    MCLC_API MCL_ERR mclc_axis_get_velocity     (U8 AxisId, FLT* out_value);
    MCLC_API MCL_ERR mclc_axis_get_hmState      (U8 AxisId, MCL_HM_STATE* out_value);

        // ----- Special usage ----
    MCLC_API MCL_ERR mclc_axis_reset_all_pos_params(U8 AxisId, FLT newPosition);// set all position-related parameters -- trg/act and cmd/Hw, to new value.
    MCLC_API MCL_ERR mclc_axis_setActPos_as_TrgPos(U8 AxisId);

    // ========= Synchronization API ===========
    #if FEAT_SYNC
    MCLC_API MCL_ERR mclc_sync_start            (U8 MstId, U8 SlvId, FLT GearRatio, FLT acc);
    MCLC_API MCL_ERR mclc_sync_end_to_stop      (U8 SlvId, FLT Dec);
    MCLC_API MCL_ERR mclc_sync_end_to_new_vel   (U8 SlvId, FLT Feed, FLT Acc);
    MCLC_API MCL_ERR mclc_sync_end_to_remain_vel(U8 SlvId);

    MCLC_API MCL_ERR mclc_sync_get_phase        (U8 SlvId, FLT* out_Phase);
    MCLC_API MCL_ERR mclc_sync_set_abs_phase    (U8 SlvId, FLT absPhase, FLT Feed, FLT Acc);
    MCLC_API MCL_ERR mclc_sync_add_rel_phase    (U8 SlvId, FLT relPhase, FLT Feed, FLT Acc);

    MCLC_API MCL_ERR mclc_sync_get_master       (U8 SlvId, U8* out_MstId);
    MCLC_API MCL_ERR mclc_sync_get_slave_num    (U8 MstId, U8* out_SlvNum);
    MCLC_API MCL_ERR mclc_sync_get_slaves       (U8 MstId, U8 buffSize, U8* out_SlvId);
    MCLC_API MCL_ERR mclc_sync_get_state        (U8 SlvId, MCL_SYNC_STATE* out_State);

    MCLC_API MCL_ERR mclc_sync_set_mst_feedback   (U8 MstId, MCL_BOOL IsUseFeedback);
    MCLC_API MCL_ERR mclc_sync_get_mst_feedback   (U8 MstId, MCL_BOOL* out_value);
    MCLC_API MCL_ERR mclc_sync_set_mst_desync_type(U8 MstId, MCL_MST_DESYNC_TYPE DesyncType);
    MCLC_API MCL_ERR mclc_sync_get_mst_desync_type(U8 MstId, MCL_MST_DESYNC_TYPE* DesyncType);
    MCLC_API MCL_ERR mclc_sync_set_slv_desync_type(U8 SlvId, MCL_SLV_DESYNC_TYPE DesyncType);
    MCLC_API MCL_ERR mclc_sync_get_slv_desync_type(U8 SlvId, MCL_SLV_DESYNC_TYPE* DesyncType);

    // for test
    MCLC_API MCL_ERR mclc_sync_set_broken(U8 AxisId, MCL_BOOL IsBroken);
    MCLC_API MCL_ERR mclc_sync_get_broken(U8 AxisId, MCL_BOOL* IsBroken);
    #endif

    // ========= Gantry API ===========
    #if FEAT_GANTRY
    MCLC_API MCL_ERR mclc_gantry_start      (U8 MstId, U8 SlvId);
    MCLC_API MCL_ERR mclc_gantry_stop       ();
	MCLC_API MCL_ERR mclc_gantry_get_state  (MCL_GANTRY_STATE* out_state);
    MCLC_API MCL_ERR mclc_gantry_enable_comp(MCL_BOOL isEnable);
    MCLC_API MCL_ERR mclc_gantry_set_comp   (MCL_GANTRY_COMP* compData);
    MCLC_API MCL_ERR mclc_gantry_get_comp   (MCL_BOOL* out_isEnable, MCL_GANTRY_COMP* out_comp);
    MCLC_API void    mclc_setCB_gantryComp  (fCB_GantryComp cbFunc);
    #endif

    // ========= Special movement API =========
    #if FEAT_LATCH
    MCLC_API MCL_ERR mclc_axis_latch_Setting_ExtDI  (U8 AxisId, MCL_IO_BIT_DATA* ioData);
    MCLC_API MCL_ERR mclc_axis_latch_Setting_TchPrb (U8 AxisId);
    MCLC_API MCL_ERR mclc_axis_latch_Move       (U8 AxisId, U32 CmdId, MCL_POS_DATA* posData, MCL_LCH_MODE mode, MCL_POS_DATA* posDataRet);
    MCLC_API MCL_ERR mclc_axis_latch_GetPos     (U8 AxisId, FLT* out_value, MCL_BOOL* out_isTriggered);
    #endif

    // ========= Comparator Trigger API =========
    #if FEAT_CMPTRG

    MCLC_API void    mclc_cmpTrg_setCB_doAction(fCB_DoAction cbFunc);

    MCLC_API MCL_ERR mclc_cmpTrg_set_available_num  (U16 maxCmpTrg, U16 maxActPerCmp);
    MCLC_API MCL_ERR mclc_cmpTrg_get_info           (MCL_CMP_INFO* out_info);

    MCLC_API MCL_ERR mclc_cmptrg_set_enable         (U8 CmpId, MCL_BOOL isEnable);
    MCLC_API MCL_ERR mclc_cmptrg_set_condition      (U8 CmpId, U8 AxisId, MCL_CMP_COND* cmpMode);
    MCLC_API MCL_ERR mclc_cmptrg_get_action         (U8 CmpId, U8 ActId, MCL_BOOL* isEnabled, MCL_CMP_ACT* actMode, MCL_IO_BIT_DATA* ioData);
    MCLC_API MCL_ERR mclc_cmptrg_enable_action      (U8 CmpId, U8 ActId, MCL_CMP_ACT* actMode, MCL_IO_BIT_DATA* ioData);
    MCLC_API MCL_ERR mclc_cmptrg_disable_action     (U8 CmpId, U8 ActId);
    MCLC_API MCL_ERR mclc_cmptrg_disable_all_action (U8 CmpId);

    #endif

    // ========= Time Trigger API =========
    #if FEAT_TIMETRG
    MCLC_API MCL_ERR mclc_timeTrg_set_enable    (MCL_BOOL isEnable, U16 intervalTimeMs, U16 triggerNum);
    MCLC_API MCL_ERR mclc_timeTrg_set_start_condition (MCL_COND startCondition);
    MCLC_API MCL_ERR mclc_timeTrg_set_stop_condition  (MCL_COND stopCondition);
    MCLC_API void    mclc_setCB_timeTriggered   (fCB_TimeTrigger cbFunc);
    MCLC_API void    mclc_setCB_timeTriggerDone (fCB_TimeTrgDone cbFunc);
    #endif

    // ========= Manual Pulse Generator API ========
    #if FEAT_MPG
    MCLC_API MCL_ERR mclc_mpg_enable           (MCL_BOOL isEnable);
    MCLC_API MCL_ERR mclc_mpg_record_to_file   (MCL_BOOL isEnable);
    MCLC_API MCL_ERR mclc_mpg_SetDISource      (U8 slvId);
    MCLC_API MCL_ERR mclc_mpg_SetCountSource   (U8 slvId, U8 channel);
    MCLC_API MCL_ERR mclc_mpg_SetAxesMode1     (U8* axesArr, U8* bitArr, U8 arrLen);
    MCLC_API MCL_ERR mclc_mpg_SetAxesMode2     (U8* axesArr, U8 arrLen, U8 bit0, U8 bit1, U8 bit2);
    MCLC_API MCL_ERR mclc_mpg_SetRatioArr      (FLT* ratioArr, U8* bitArr, U8 arrLen);
    MCLC_API MCL_ERR mclc_mpg_SetJog           (I16 posBit, I16 negBit);
    MCLC_API MCL_ERR mclc_mpg_SetFilter        (MCL_MPG_FILTER_MODE mode);
    MCLC_API MCL_ERR mclc_mpg_SetFreqDivide    (U8 divideShift);
    MCLC_API MCL_ERR mclc_mpg_SetGlobleRatio   (FLT globleRatio);
    MCLC_API MCL_ERR mclc_mpg_SetHardwareEnable(I16 bit);
    MCLC_API MCL_ERR mclc_mpg_SetCntThreshold  (I32 threshold);

    MCLC_API MCL_ERR mclc_mpg_GetTargetAxesArr(U8* axesArr, U8* bitArr, U8 arrLen);
    MCLC_API U8      mclc_mpg_GetTagetAxesNum ();

    MCLC_API MCL_ERR mclc_mpg_GetCurrentState(MCL_MPG_STATE* out_value);
    MCLC_API void    mclc_setCB_getCntValue  (fCB_GetCntValue cbFunc);
    #endif

    // ========= System ========
    MCLC_API MCL_ERR mclc_get_usedCmdMemory     (U16* out_value);

    // ====== Special purpose API (currently not in use) =========
    // Extra-info: it is a memory space carried by a cmd, for custom definition. (ex: robot related info)
    MCLC_API void mclc_grp_set_cmd_extra_info   (U8 GrpId, U8 value); // the next cmd added in buffer will carry this extra-info
    MCLC_API U8   mclc_grp_get_cmd_extra_info   (U8 GrpId); // get the extra-info of current executing cmd.

    

#ifdef __cplusplus
}
#endif

#endif // _MCLC_H
