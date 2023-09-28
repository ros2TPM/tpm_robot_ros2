#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tpm_msgs/srv/jog_pose.hpp"
#include "tpm_msgs/srv/axis_operation.hpp"
#include "tpm_msgs/srv/robot_operation.hpp"
#include "tpm_msgs/srv/robot_move.hpp"

using namespace tpm_msgs::srv;

namespace tpm_core
{
  class Manager_Service
  {
  public:
    Manager_Service(rclcpp::Node::SharedPtr node);
    virtual ~Manager_Service() = default;

  private:
    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
    
    short axis_operation (const AxisOperation::Request::SharedPtr req, AxisOperation::Response::SharedPtr res);
    short robot_operation(const RobotOperation::Request::SharedPtr req, RobotOperation::Response::SharedPtr res);
    short jog_pose(const JogPose::Request::SharedPtr req, JogPose::Response::SharedPtr res);
    
    short rob_move(const RobotMove::Request::SharedPtr req, RobotMove::Response::SharedPtr res);

  };
}