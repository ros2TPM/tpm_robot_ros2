#include "hwLib.hpp"
#include "myRobot.h" //for get_pulse_per_deg in init();
#include "def_macro.h"
#include "global_instance.hpp" //for MAX_AXIS_NUM
#include <thread> //for RI-DDA_Cycle()

namespace tpm_core
{
  HwLib_Sim::HwLib_Sim()
  {
  }
  #pragma region RI
  static bool is_need_stop_dda;
  static int temp_counter = 0;
  static std::thread ddaThread;

  void HwLib_Sim::DDA_Cycle()
  {
      while(!is_need_stop_dda)
      {
          short rc = dda_cycle();
          if (rc != 0){
              ROS_PRINT("dda_cycle error: %d", rc);
              ROS_PRINT("%s", robc_get_last_error_msg());
          }
          
          get_axis(RIDT->robc.axis);
          get_pose(RIDT->robc.pose);

          U32 buffDepth = 0;
          get_buffer_depth(&buffDepth);
          RIDT->robc.bufDepth = buffDepth;

          // ROS_PRINT("Axis:[%.3f, %.3f, %.3f] Pose[%.3f, %.3f, %.3f]",
          //   RIDT->robc.axis[0], RIDT->robc.axis[1], RIDT->robc.axis[2],
          //   RIDT->robc.pose[0], RIDT->robc.pose[1], RIDT->robc.pose[2]
          // );
              
          RIDT->m1a[0].encPos = (temp_counter++)/1000; //generate some fake data

          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
  }
  short HwLib_Sim::connect()
  {
    RIDT = new ST_RIDT_t();

    is_need_stop_dda = false;
    ddaThread = std::thread(&HwLib_Sim::DDA_Cycle, this);

    //set fake vec_axisIP
    for(int i=0; i<Global::MAX_AXIS_NUM; i++)
        vec_axisIP.push_back(i);

    return 0;
  }
  short HwLib_Sim::disconnect()
  {
    is_need_stop_dda = true;
    ddaThread.join();
    delete RIDT;
    return 0;
  }

  #pragma endregion

  #pragma region ROBC
  
  short HwLib_Sim::init_inner(ROB_KIN_TYPE type, FLT* a, FLT* alpha, FLT* d, FLT* theta, FLT* thetaShift, FLT* posLimit, FLT* negLimit, FLT* pulsePerDeg)
  {
    auto rc = robc_init(type, a, alpha, d, theta, thetaShift, posLimit, negLimit, pulsePerDeg);
    printf("(%d)HwLib_Sim::init\n", rc);
    return rc;
  }

  short HwLib_Sim::dda_cycle()  // todo: remove this use DDA_Cycle
  {
    //printf("HwLib_Sim::dda_cycle\n");
    return robc_dda_cycle();
  }

  short HwLib_Sim::stop(UINT8 stopType)
  {
    printf("HwLib_Sim::stop\n");
    robc_stop((MCL_STOP_TYPE)stopType);
    return 0;
  }

  short HwLib_Sim::set_axis_position(U8 AxisId, FLT value)
  {
    robc_set_axis_position(AxisId, value);
    return 0;
  }
  short HwLib_Sim::jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc) 
  {
    //ROS_PRINT("AxisId:%d, Dir:%d, Dist:%.3f, Vel:%.3f, Acc:%.3f", AxisId, dir, Dist, Vel, Acc);
    return robc_jog_axis(AxisId, dir, Dist, Vel, Acc);
  }
  short HwLib_Sim::jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame) 
  {
    return robc_jog_pose(PoseId, dir, Dist, Vel, Acc, frame);
  }
  short HwLib_Sim::move_p2p_axis(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pos, UINT8 mask)
  {
    // for (int i = 0; i < 6; i++)
    // {
    //   if (mask & (1 << i)) continue;
    //   axis_[i] = pos[i];
    // }

    // printf("HwLib_Sim::move_p2p_axis\n");
    // for (int i = 0; i < 6; i++)
    //   printf("%f, ", pos[i]);
    // printf("\n");

    return robc_move_p2p_axis(cmdId, &mpData, pos, mask);
  }
  short HwLib_Sim::move_lin_pose(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pose, UINT8 mask)
  {
    return robc_move_lin_pose(cmdId, &mpData, pose, mask, ROB_FRAME_BASE);
  }
  short HwLib_Sim::get_axis(FLT* values)
  {
    // for (int i = 0; i < 6; i++)
    //   values[i] = axis_[i];

    //printf("HwLib_Sim::get_axis\n");
    robc_get_axis(values);
    return 0;
  }
  short HwLib_Sim::get_pose(FLT* values)
  {
    robc_get_pose(values);
    return 0;
  }

  short HwLib_Sim::get_buffer_depth(U32* buffDepth)
  {
    // *buffDepth = 0;

    //printf("HwLib_Sim::get_buffer_depth\n");
    *buffDepth = robc_get_buffer_depth();
    return 0;
  }
  std::string HwLib_Sim::get_last_err_msg()
  {
    return robc_get_last_error_msg();
  }

  short HwLib_Sim::hold()
  {
    printf("HwLib_Sim::hold\n");
    robc_hold();
    return 0;
  }
  short HwLib_Sim::resume()
  {
    printf("HwLib_Sim::resume\n");
    robc_resume();
    return 0;
  }
  short HwLib_Sim::feedrate(double feedrate)
  {
    printf("HwLib_Sim::feedrate\n");
    robc_feedrate(feedrate);
    return 0;
  }


  short HwLib_Sim::set_pvt_data(U8 AxisId, U32 PointNum, MCL_PVT_POINT* PvtPoints)
  {
    return robc_set_pvt_data(AxisId, PointNum, PvtPoints);
  }
  short HwLib_Sim::move_pvt(FLT StopDec, U8 Mask)
  {
    return robc_move_pvt(0, StopDec, Mask);
  }
  #pragma endregion
 
}
