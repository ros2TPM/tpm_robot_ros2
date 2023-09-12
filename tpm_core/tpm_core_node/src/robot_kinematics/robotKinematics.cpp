#include "robot_kinematics/robotKinematics.hpp"
#include "def_math.h"

using namespace std;

namespace tpm_core
{
  RobotKinematics* RobotKinematics::_ptr = NULL;

  void RobotKinematics::SelectKinematic(ROB_KIN_TYPE type)
  {
    delete(_ptr);

    switch (type)
    {
    case ROB_Delta_Pris:
      _ptr = new DeltaKinematics();
      break;
    
    default:
      _ptr = new SerialKinematics();
      break;
    }
  }

  void SerialKinematics::Init(FLT a[], FLT alpha[], FLT d[], FLT theta[])
  {
    _joint_names.clear();
    _joint_names = {
      "joint1", "joint2", "joint3",
      "joint4", "joint5", "joint6",
      //"x", "y", "z", "a", "b", "c"
    };
  }

  void SerialKinematics::GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues)
  {
    jointValues.clear();
    jointNames = this->_joint_names;

    for(int i = 0; i < MAX_AXIS_PER_ROBOT; i++)
      jointValues.push_back(axes[i] * M_PI / 180);

    // for(int i = 0; i < MAX_AXIS_PER_ROBOT; i++)
    //   jointValues.push_back(pos[i]);
  }

  void DeltaKinematics::Init(FLT a[], FLT alpha[], FLT d[], FLT theta[])
  {
    _joint_names.clear();
    _joint_names = {
      "joint1", "elbow1_a", "elbow1_b",
      "joint2", "elbow2_a", "elbow2_b",
      "joint3", "elbow3_a", "elbow3_b",
      "x", "y", "z"
    };

    this->theta = alpha[0] * DEG2RAD;
    this->R = d[0];
    this->l_shift = d[1];
    this->L = d[2];
    this->r = d[4];
    this->H = theta[1];

    this->phi[0] = -150 * DEG2RAD;
    this->phi[1] = -30  * DEG2RAD;
    this->phi[2] =  90  * DEG2RAD;
  }

  void DeltaKinematics::GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues)
  {
    jointValues.clear();
    jointNames = this->_joint_names;

    FLT l[3] = {
      axes[0] + this->l_shift,
      axes[1] + this->l_shift,
      axes[2] + this->l_shift
    };

    // location of the slide
    FLT B[3][3] = {0};
    for(int i = 0; i < 3; i++)
    {
      B[i][0] = (this->R - l[i] * cos(theta)) * cos(phi[i]);
      B[i][1] = (this->R - l[i] * cos(theta)) * sin(phi[i]);
      B[i][2] = -l[i] * sin(theta);
    }

    // location of plane and link joints
    FLT P[3][3] = {0};
    for(int i = 0; i < 3; i++)
    {
      P[i][0] = pos[0] + r * cos(phi[i]);
      P[i][1] = pos[1] + r * sin(phi[i]);
      P[i][2] = pos[2];
    }

    // vector BP
    FLT v[3][3] = {0};
    FLT tmp[3] = {0};
    FLT rotZ[3] = { -30 * DEG2RAD, -150 * DEG2RAD,  -270 * DEG2RAD};
    FLT alpha[3] = {0};
    FLT beta[3] = {0};

    // elbow angles
    for (int i = 0; i < 3; i++)
    {
      Vec3_Sub(tmp, P[i], B[i]);
      Vec3_RotZYX(v[i], tmp, rotZ[i], -this->theta, 0);
      Vec3_NormalizeSelf(v[i]);

      alpha[i] = -atan2(v[i][2], v[i][0]);
      beta[i]  =  atan2(v[i][1], sqrt(v[i][0] * v[i][0] + v[i][2] * v[i][2]));
    }

    jointValues = {
      axes[0] / 1000, alpha[0], beta[0],
      axes[1] / 1000, alpha[1], beta[1],
      axes[2] / 1000, alpha[2], beta[2],
       pos[0] / 1000,
       pos[1] / 1000,
      -pos[2] / 1000,
    };
  }
}