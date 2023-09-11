#include "hwLib.hpp"
#include "myRobot.h" //for get_pulse_per_deg in init();
#include "def_macro.h"
#include "def_RIDT.h"
#include "global_instance.hpp" //for MAX_AXIS_NUM
#include "global_config.hpp" //for RobotSpec
#include <thread> //for RI-DDA_Cycle()

#include "robot_kinematics/robotKinematics.hpp"

namespace tpm_core
{
  HwLib& HwLib::Instance(){
    #ifdef USE_REAL_MNET
        static HwLib_Real instance;
    #else 
        static HwLib_Sim instance;
    #endif

        return instance;
    }
  HwLib_Sim::HwLib_Sim()
  {
    ROS_PRINT("*** HwLib: use simulation mode ****");
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
          if (rc != 0)
              ROS_PRINT("dda_cycle error: %d", rc);
          
          get_axis(RIDT->robc.axis);
          get_pose(RIDT->robc.pose);

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
  short HwLib::init()
  {
    ROB_KIN_TYPE robotType = (ROB_KIN_TYPE)RobotSpec::robot_type;

    FLT posLimit   [MAX_AXIS_PER_ROBOT];
    FLT negLimit   [MAX_AXIS_PER_ROBOT];
    FLT a          [MAX_AXIS_PER_ROBOT];
    FLT alpha      [MAX_AXIS_PER_ROBOT];
    FLT d          [MAX_AXIS_PER_ROBOT];
    FLT theta      [MAX_AXIS_PER_ROBOT];
    FLT thetaShift [MAX_AXIS_PER_ROBOT];
    FLT pulsePerDeg[MAX_AXIS_PER_ROBOT];

    for (int i = 0; i < MAX_AXIS_PER_ROBOT; i++)
    {
      posLimit[i] = RobotSpec::pos_limit[i];
      negLimit[i] = RobotSpec::neg_limit[i];
      a[i] = RobotSpec::a[i];
      alpha[i] = RobotSpec::alpha[i];
      d[i] = RobotSpec::d[i];
      theta[i] = RobotSpec::theta[i];
      thetaShift[i] = RobotSpec::theta_shift[i];
      pulsePerDeg[i] = RobotSpec::pulse_per_unit[i];
    }

    auto rc = init_inner(robotType, a, alpha, d, theta, thetaShift, posLimit, negLimit, pulsePerDeg);

    RobotKinematics::SelectKinematic(robotType);
    RobotKinematics::GetInstance()->Init(a, alpha, d, theta);

    return rc;

  }
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

  short HwLib_Sim::get_buffer_depth(INT32* buffDepth)
  {
    // *buffDepth = 0;

    //printf("HwLib_Sim::get_buffer_depth\n");
    *buffDepth = robc_get_buffer_depth();
    return 0;
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
  #pragma endregion

  #pragma region MCLC
  /*
  short HwLib_Sim::init()
  {
    mclc_init();
    return 0;
  }
  short HwLib_Sim::dda_cycle()
  {
    return mclc_dda_cycle();
  }*/
  short HwLib_Sim::axis_move_pos(U8 AxisId, U32 CmdId, MCL_MPDATA mp, FLT pos)
  {
    return mclc_axis_move_pos(AxisId, CmdId, &mp, pos);
  }
  short HwLib_Sim::axis_move_pvt(U8 AxisId, U32 CmdId, U32 pointNum, MCL_PVT_POINT* pvtPoints, FLT stopDec)
  {
    return mclc_axis_move_pvt(AxisId, CmdId, pointNum, pvtPoints, stopDec);
  }
  short HwLib_Sim::axis_stop(U8 AxisId, U8 StopType)
  {
    return mclc_axis_stop(AxisId, (MCL_STOP_TYPE)StopType);
  }
  short HwLib_Sim::axis_get_buffer_depth(U8 AxisId, U32* buffDepth)
  {
    return mclc_axis_get_buffer_depth(AxisId, buffDepth);
  }
  short HwLib_Sim::axis_get_trgPosCmd(U8 AxisId, FLT* pos)
  {
    return mclc_axis_get_trgPosCmd(AxisId, pos);
  }
  short HwLib_Sim::axis_get_actPosCmd(U8 AxisId, FLT* pos)
  {
    return mclc_axis_get_actPosCmd(AxisId, pos);
  }
  #pragma endregion

 
}
