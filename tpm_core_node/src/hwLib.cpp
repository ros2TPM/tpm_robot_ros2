#include "hwLib.hpp"
#include "global_config.hpp"
#include "robot_kinematics/robotKinematics.hpp"

namespace tpm_core
{
    HwLib& HwLib::Instance(){
      static HwLib_Sim instance_sim;
      static HwLib_Real instance_real;

      if (Config::use_sim)
        return instance_sim;
      else
        return instance_real;
    }
    
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
}
