#include "hwLib.hpp"
#include "RPiL132.h" //for rpi_master
#include "RPiMNet.h" //for Mnet
#include "myRobot.h" //for pulsePerDeg
#include "global_config.hpp" //for Config:: in RI APIs
#include "global_instance.hpp" //for MAX_AXIS_NUM
#include "def_macro.h"

namespace tpm_core
{
  #pragma region RI
  short HwLib_Real::connect()
  {
    RIDT = new ST_RIDT_t();

    TRY_AND_PRINT(rpi_master_initialize());
    TRY_AND_PRINT(rpi_mnet_open());
    TRY_AND_PRINT(rpi_mnet_set_ring_config(ringNo, Config::baudrate));
    TRY_AND_PRINT(rpi_mnet_reset_ring(ringNo));
    TRY_AND_PRINT(rpi_mnet_start_ring(ringNo));

    TRY_AND_PRINT(rpi_ri_init());

    std::vector<UINT8> vec_ioIP;
    vec_axisIP.clear();

    U32 devTable[2] = {0};
    TRY_AND_PRINT(rpi_mnet_get_ring_active_table(ringNo, devTable));

    for (int i = 0; i < 64; i++)
    {
        if ((devTable[i/32] & (1 << (i%32))) <= 0) 
            continue;

        UINT8 slvType = 0;
        TRY(rpi_mnet_get_slave_type(ringNo, i, &slvType));
        if (slvType == 0xA2){
            ROS_PRINT("==== find Axis slave (IP=%d) ====", i);
            TRY(init_axis(i));
            vec_axisIP.push_back(i);
            
        }
        else if (slvType >= 0xB0 && slvType < 0xD0){
            ROS_PRINT("==== find DIO slave (IP=%d) ====", i);
            vec_ioIP.push_back(i);
        }
    }
    ROS_PRINT("==== end finding slaves ====");
    

    // set axis
    U16 axisIPs[6];
    for (int i = 0; i < Global::MAX_AXIS_NUM; i++)
    {
        if (i < (int)vec_axisIP.size())
            axisIPs[i] = vec_axisIP.at(i);
        else
            axisIPs[i] = 255;
    }

    TRY_AND_PRINT(rpi_ri_mnet_m1a_init(ringNo, axisIPs));
    

    // set dio
    for (size_t i = 0; i < vec_ioIP.size(); i++)
        TRY_AND_PRINT(rpi_ri_mnet_dio_init(ringNo, vec_ioIP.at(i), i));

    TRY_AND_PRINT(rpi_ri_set_tdda_ratio(0.95));
    TRY_AND_PRINT(rpi_ri_sta_enable(false));
    TRY_AND_PRINT(rpi_ri_set_is_apply_to_real_motor(true));
    TRY_AND_PRINT(rpi_ri_mnet_m1a_enable(true));//endable DDA mode
    return 0;
  }
  short HwLib_Real::disconnect()
  {
    rpi_mnet_close();
    rpi_master_finalize();
    delete RIDT;

    return 0;
  }
  short HwLib_Real::ri_enable_DDAMode(bool enable)
  {
    return rpi_ri_mnet_m1a_enable(enable);
  }
  ST_RIDT_t* HwLib_Real::ri_get_RIDT()
  {
    rpi_ri_get_data(RIDT);
    return RIDT;
  }

  //===== private ======
  short HwLib_Real::init_axis(UINT16 ip)
  {
    TRY(rpi_mnet_reset_ring_error_counter(ringNo));
    TRY(rpi_mnet_m1a_initial(ringNo, ip));

    TRY(rpi_mnet_m1a_set_pls_iptmode(ringNo, ip, Config::ipt_mode, 0));
    TRY(rpi_mnet_m1a_set_pls_outmode(ringNo, ip, Config::out_mode));
    TRY(rpi_mnet_m1a_set_feedback_src(ringNo, ip, Config::feedback_src)); //0:external. 1:internal

    TRY(rpi_mnet_m1a_set_erc(ringNo, ip, 1, 0, 0));
    TRY(rpi_mnet_m1a_set_inp(ringNo, ip, 0, 0));
    TRY(rpi_mnet_m1a_set_sd(ringNo, ip, 0, 0, 1, 1));

    TRY(rpi_mnet_m1a_set_alm(ringNo, ip, Config::alm_logic, 0));//logic 0:low active. 1:high active //mode:
    TRY(rpi_mnet_m1a_set_ltc_logic(ringNo, ip, 1));
    
    UINT16 home_mode=0;
    TRY(rpi_mnet_m1a_set_home_config(ringNo, ip, home_mode, Config::org_logic, 1, 0, 0));//logic 0:low active. 1:high active //mode:


    TRY(rpi_mnet_m1a_reset_command(ringNo, ip));
    TRY(rpi_mnet_m1a_reset_position(ringNo, ip));
    TRY(rpi_mnet_m1a_reset_error_counter(ringNo, ip));
    return 0;
  }

  #pragma endregion

  #pragma region ROBC
  short HwLib_Real::init_inner(ROB_KIN_TYPE type, FLT* a, FLT* alpha, FLT* d, FLT* theta, FLT* thetaShift, FLT* posLimit, FLT* negLimit, FLT* pulsePerDeg)
  {
    auto rc = rpi_robc_init(type, a, alpha, d, theta, thetaShift, posLimit, negLimit, pulsePerDeg);
    printf("(%d)HwLib_Real::init\n", rc);
    return rc;
  }
  short HwLib_Real::stop(UINT8 stopType)
  {
    return rpi_robc_stop((MCL_STOP_TYPE)stopType);
  }

  short HwLib_Real::set_axis_position(U8 AxisId, FLT value)
  {
    return rpi_robc_set_axis_position(AxisId, value);
  }
  short HwLib_Real::jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc) 
  {
    return rpi_robc_jog_axis(AxisId, dir, Dist, Vel, Acc);
  }
  short HwLib_Real::jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame) 
  {
    return rpi_robc_jog_pose(PoseId, dir, Dist, Vel, Acc, frame);
  }
  short HwLib_Real::move_p2p_axis(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pos, UINT8 mask)
  {
    return rpi_robc_move_p2p_axis(cmdId, &mpData, pos, mask);
  }
  short HwLib_Real::move_lin_pose(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pose, UINT8 mask)
  {
    return rpi_robc_move_lin_pose(cmdId, &mpData, pose, mask, ROB_FRAME_BASE);
  }
  short HwLib_Real::get_axis(FLT* values)
  {
    return rpi_robc_get_axis(values);
  }
  short HwLib_Real::get_buffer_depth(U32* buffDepth)
  {
    I32 buff;
    short err = rpi_robc_get_buffer_depth(&buff);
    *buffDepth = buff;
    return err;
  }
  std::string HwLib_Real::get_last_err_msg()
  {
    return "";
   // return rpi_robc_get_last_error_msg();
  }

  short HwLib_Real::hold()
  {
    return rpi_robc_hold();
  }
  short HwLib_Real::resume()
  {
    return rpi_robc_resume();
  }
  short HwLib_Real::feedrate(double feedrate)
  {
    return rpi_robc_feedrate(feedrate);
  }
  #pragma endregion 

  #pragma region Mnet
  int HwLib_Real::mnet_m1a_set_svon(UINT16 ip, UINT16 on_off)
  {
    return rpi_mnet_m1a_set_svon(ringNo, ip, on_off);
  }
  int HwLib_Real::mnet_m1a_set_ralm(UINT16 ip, UINT16 on_off)
  {
    return rpi_mnet_m1a_set_ralm(ringNo, ip, on_off);
  }
  int HwLib_Real::mnet_m1a_home_search(UINT16 ip, UINT8 Dir, UINT32 StrVel, UINT32 MaxVel, float Tacc, INT32 ORGOffset)
  {
    return rpi_mnet_m1a_home_search(ringNo, ip, Dir, StrVel, MaxVel, Tacc, ORGOffset);
  }
  int HwLib_Real::mnet_m1a_reset_all(UINT16 ip)
  {
    rpi_mnet_m1a_reset_command    (ringNo, ip);
    rpi_mnet_m1a_reset_position   (ringNo, ip);
    rpi_mnet_m1a_reset_error_counter(ringNo, ip);    
    return 0;
  }
  int HwLib_Real::mnet_m1a_motion_done(UINT16 ip, UINT16 *MoSt)
  {
    return rpi_mnet_m1a_motion_done(ringNo, ip, MoSt);
  }
  int HwLib_Real::mnet_m1a_sd_stop(UINT16 ip)
  {
    return rpi_mnet_m1a_sd_stop(ringNo, ip);
  }
  #pragma endregion
}
