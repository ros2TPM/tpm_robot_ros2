#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tpm_msgs/msg/robot_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
    rclcpp::Publisher<tpm_msgs::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    
    
  };
}