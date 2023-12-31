#include "manager_service.hpp"
#include "archive.hpp"
#include "hwLib.hpp"
#include "global_config.hpp"
#include "def_type.h"
#include "def_macro.h"
#include "myRobot.h"
#include "myAxis.h"

namespace tpm_core
{
  Robot* pRobot;
  typedef short (*opFunc2)(int);

  enum eRobotParam
  {
    e_move_speed,
    e_jog_dist,
  };

  Manager_Service::Manager_Service(rclcpp::Node::SharedPtr node)
  {
    using namespace std::placeholders;

    pRobot = &Robot::getInstance();

    //======== general API ======
    services_.push_back(node->create_service<AxisOperation>( "/tpm/axis_operation",
      std::bind(&Manager_Service::axis_operation, this, _1, _2)));
  
    //======== ROBC API=======
    //fureture todo: hold, resume
    //fureture todo: delay, set io out/wait io in
    services_.push_back(node->create_service<RobotOperation>(
      "/tpm/robot_operation",
      std::bind(&Manager_Service::robot_operation, this, _1, _2)
    ));

    services_.push_back(node->create_service<RobotMove>(
      "/rob/move",
      std::bind(&Manager_Service::rob_move, this, _1, _2)
    ));

    services_.push_back(node->create_service<JogPose>(
      "/rob/jog_pose",
      std::bind(&Manager_Service::jog_pose, this, _1, _2)
    ));
    
  }
  
  short Manager_Service::axis_operation(const AxisOperation::Request::SharedPtr req, AxisOperation::Response::SharedPtr res)
  {
    TRY(pRobot->do_axis_action(req->function, req->axis_id));
    res->ok = true;
    return 0;
  }

  short Manager_Service::robot_operation(const RobotOperation::Request::SharedPtr req, RobotOperation::Response::SharedPtr res)
  {
    TRY(pRobot->do_action(req->function, req->arg1));
    res->ok = true;
    return 0;
  }
 
  short Manager_Service::rob_move(const RobotMove::Request::SharedPtr req, RobotMove::Response::SharedPtr res)
  {
    MCL_MPDATA mp;
    mp.Feed  = req->mp_data.feed;
    mp.Accel = req->mp_data.accel;
    mp.Decel = req->mp_data.decel;
    mp.Coord = (MCL_COORD_TYPE)req->mp_data.coord;
    mp.VelStart = req->mp_data.vel_start;
    mp.VelEnd = req->mp_data.vel_end;
    mp.SFactor = req->mp_data.s_factor;
    mp.OverlapRate = req->mp_data.overlap_rate;
    mp.OverlapType = (MCL_OVERLAP_TYPE)req->mp_data.overlap_type;
    mp.IsCheckInPos = req->mp_data.is_check_inpos;

    switch (req->function)
    {
    case RobotMove::Request::PTP_AXIS:
    {
      FLT* axis = new FLT[req->axis.size()];
      for (size_t i=0;i<req->axis.size();i++)
        axis[i] = req->axis.at(i);

      res->result_code = HwLib::Instance().move_p2p_axis(req->cmd_id, mp, axis, req->mask);
      if (res->result_code != 0)
        res->message = HwLib::Instance().get_last_err_msg();

      delete[] axis;
      break;
    }
    case RobotMove::Request::LIN_POSE:
    {
      FLT* pose = new FLT[req->pose.size()];
      for (size_t i=0;i<req->pose.size();i++)
        pose[i] = req->pose.at(i);

      res->result_code = HwLib::Instance().move_lin_pose(req->cmd_id, mp, pose, req->mask);
      if (res->result_code != 0)
        res->message = HwLib::Instance().get_last_err_msg();

      delete[] pose;
      break;
    }

    default:
      ROS_PRINT("ERROR: RobotMove Type=%d is not implemented", req->function);
      res->message = "RobotMove Type not implemented";
      res->result_code = ROB_ERR_NOT_IMPLEMENTED;
      break;
    }
    return res->result_code;
  }

  short Manager_Service::jog_pose(const JogPose::Request::SharedPtr req, JogPose::Response::SharedPtr res)
  {
    TRY(pRobot->jog_pose(req->pose_id, req->jog_dir));
    res->ok = true;
    return 0;
  }

}
