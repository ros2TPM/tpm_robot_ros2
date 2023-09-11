#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace tpm_core
{
  class Manager_Action
  {
  public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    Manager_Action(rclcpp::Node::SharedPtr node);
    virtual ~Manager_Action() = default;

  private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr fjt_action_server_;

    rclcpp_action::GoalResponse handle_fjt_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_fjt_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_fjt_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

    void execute_ftj(const std::shared_ptr<GoalHandleFJT> goal_handle);
  };
}