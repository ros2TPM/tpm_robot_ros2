#include "manager_action.hpp"
#include "def_macro.h"
#include "hwLib.hpp"

namespace tpm_core
{
  Manager_Action::Manager_Action(rclcpp::Node::SharedPtr node)
  {
    using namespace std::placeholders;
    
    fjt_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node, "/ar3_arm_controller/follow_joint_trajectory",
      std::bind(&Manager_Action::handle_fjt_goal, this, _1, _2),
      std::bind(&Manager_Action::handle_fjt_cancel, this, _1),
      std::bind(&Manager_Action::handle_fjt_accepted, this, _1)
    );

    fj_ = new FJStrategy_mclcPVT();
    // fj_ = new FJStrategy_robcAxisPTP();
  }

  rclcpp_action::GoalResponse Manager_Action::handle_fjt_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    int pointSize = goal->trajectory.points.size();

    ROS_PRINT("Handle goal: %d", pointSize);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse Manager_Action::handle_fjt_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    ROS_PRINT("Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void Manager_Action::handle_fjt_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    std::thread{[this, goal_handle]() {fj_->Execute(goal_handle);}}.detach();
  }

}
