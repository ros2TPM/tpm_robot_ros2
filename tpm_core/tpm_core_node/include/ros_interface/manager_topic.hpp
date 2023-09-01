#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tpm_core_msgs/msg/robot_status.hpp"

namespace tpm_core
{
  class Manager_Topic
  {
  public:
    Manager_Topic(rclcpp::Node::SharedPtr node) ;
    virtual ~Manager_Topic() = default;

    void timerCallback_robotStatus();
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tpm_core_msgs::msg::RobotStatus>::SharedPtr publisher_;

    
    
  };
}