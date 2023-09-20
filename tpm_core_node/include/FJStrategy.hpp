#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace tpm_core
{
  class FJStrategy
  {
  public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    virtual void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) = 0;
  };

  class FJStrategy_mclcPVT : public FJStrategy
  {
  public:
    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;
  };

  class FJStrategy_robcAxisPTP : public FJStrategy
  {
  public:
    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;
  };
}
