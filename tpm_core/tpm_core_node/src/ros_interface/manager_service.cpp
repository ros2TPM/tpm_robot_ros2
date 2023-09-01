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

  enum eAxisParam
  {
    e_home_offset,
  };
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
    services_.push_back(node->create_service<MailBox>( "/op/axis_operation",
      std::bind(&Manager_Service::axis_operation, this, _1, _2)));

    services_.push_back(node->create_service<MailBox>( "/op/set_axis_param",
      std::bind(&Manager_Service::set_axis_param, this, _1, _2)));

    services_.push_back(node->create_service<MailBox>( "/op/set_robot_param",
      std::bind(&Manager_Service::set_robot_param, this, _1, _2)));
  
    //======== ROBC API=======
    services_.push_back(node->create_service<MailBox>(
      "/rob/stop",
      std::bind(&Manager_Service::robStop, this, _1, _2)));

    services_.push_back(node->create_service<MovePTP>(
      "/rob/movePTP",
      std::bind(&Manager_Service::robMovePTP, this, _1, _2)));

    services_.push_back(node->create_service<MailBox>(
      "/rob/getAxis",
      std::bind(&Manager_Service::robGetAxis, this, _1, _2)));

    services_.push_back(node->create_service<MailBox>(
      "/rob/getBuffDepth",
      std::bind(&Manager_Service::robGetBuffDepth, this, _1, _2)));
    
  }
  
  short Manager_Service::axis_operation(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive recv;
    recv.Update(req->buffer.size(), req->buffer.data());

    char funcType;
    signed char axisId;
    recv >>funcType >> axisId;
    
    TRY(pRobot->do_action(funcType, axisId));
    
    return 0;
  }
  
  short Manager_Service::set_axis_param(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive recv;
    recv.Update(req->buffer.size(), req->buffer.data());

    char paramType;
    signed char axisId;
    recv >> paramType >> axisId;

    ROS_PRINT("paramType=%d (axisId=%d)", paramType,  axisId);

    switch (paramType)
    {
    case e_home_offset:
      recv >> Config::home_offsets[axisId];
      break;
    
    default:
      ROS_PRINT("Error: set_axis_param does not implement paramType:%d", paramType);
      break;
    }
    return 0;
  }

  short Manager_Service::set_robot_param(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive recv;
    recv.Update(req->buffer.size(), req->buffer.data());

    char paramType;
    double value;
    recv >> paramType >> value;

    ROS_PRINT("rob_paramType=%d (value=%.2f)", paramType, value);

    switch (paramType)
    {
      case e_move_speed:
        Robot::set_vel_ratio(value/100);
        break;

      case e_jog_dist:
        Robot::set_jog_dist(value);
      break;
    
    default:
      ROS_PRINT("Error: set_robot_param does not implement paramType:%d", paramType);
      break;
    }
    return 0;
  }

  short Manager_Service::robStop(
    const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive recv;
    recv.Update(req->buffer.size(), req->buffer.data());

    UINT8 stopType;
    recv >> stopType;

    res->result_code = HwLib::Instance().stop(stopType);
    return res->result_code;
  }

  short Manager_Service::robMovePTP(
    const MovePTP::Request::SharedPtr req, MovePTP::Response::SharedPtr res)
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

    FLT* pos = new FLT[req->pos.size()];
    for (size_t i=0;i<req->pos.size();i++)
      pos[i] = req->pos.at(i);

    res->result_code = HwLib::Instance().move_p2p_axis(req->cmd_id, mp, pos, req->mask);
    return res->result_code;
  }

  short Manager_Service::robGetAxis(
    const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive send;

    FLT axis[6];

    res->result_code = HwLib::Instance().get_axis(axis);

    for (size_t i = 0; i < 6; i++)
    {
      double pos = axis[i];
      send << pos;
    }

    res->buffer.resize(send.GetSize());
    memcpy(res->buffer.data(), send.GetData(), send.GetSize());

    return res->result_code;
  }

  short Manager_Service::robGetBuffDepth(
    const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res)
  {
    CArchive send;
    INT32 buffDepth;

    res->result_code = HwLib::Instance().get_buffer_depth(&buffDepth);

    send << buffDepth;
    res->buffer.resize(send.GetSize());
    memcpy(res->buffer.data(), send.GetData(), send.GetSize());

    return res->result_code;
  }

}
