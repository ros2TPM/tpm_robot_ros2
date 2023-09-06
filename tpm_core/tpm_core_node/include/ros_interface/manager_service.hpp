#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tpm_core_msgs/srv/mail_box.hpp"
#include "tpm_core_msgs/srv/move_ptp.hpp"
#include "tpm_core_msgs/srv/axis_operation.hpp"

using namespace tpm_core_msgs::srv;

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
    short set_axis_param (const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res);
    short set_robot_param(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res);

    short robStop   (const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res);

    //todo: rename: rob_move
    short robMovePTP(const MovePTP::Request::SharedPtr req, MovePTP::Response::SharedPtr res);

    //todo: remove these. (should do this by topic)
    short robGetAxis(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res);
    short robGetBuffDepth(const MailBox::Request::SharedPtr req, MailBox::Response::SharedPtr res);

  };
}