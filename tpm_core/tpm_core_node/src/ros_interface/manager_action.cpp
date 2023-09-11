#include "manager_action.hpp"
#include "def_macro.h"
#include "hwLib.hpp"
#include "global_config.hpp" // for RobotSpec

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
    std::thread{[this, goal_handle]() {execute_ftj(goal_handle);}}.detach();
  }

  void Manager_Action::execute_ftj(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    auto traj_points = goal_handle->get_goal()->trajectory.points;
    size_t pointNum = traj_points.size();
    double maxVel = 0;

    // get max velocity
    MCL_MPDATA mpData;
    memset(&mpData, 0, sizeof(mpData));
    for(size_t i = 0; i < pointNum; i++)
    {
      auto point = goal_handle->get_goal()->trajectory.points.at(i);
      for (size_t j = 0; j < point.positions.size(); j++)
      {
        if (maxVel < abs(point.velocities.at(j)))
          maxVel = abs(point.velocities.at(j));
      }
    }

    if (RobotSpec::robot_type == 100) // delta_pris
      maxVel *= 1000;
    else
      maxVel *= 180 / M_PI;

    mpData.Coord = MCL_COORD_ABS;
    mpData.Feed  = maxVel;
    mpData.Accel = maxVel * 10;
    mpData.Decel = maxVel * 10;

    FLT positions[MAX_AXIS_PER_ROBOT];
    auto point = traj_points.back();
    for (size_t i = 0; i < point.positions.size(); i++)
      positions[i] = point.positions[i] * 180 / M_PI;

    auto rc = HwLib::Instance().move_p2p_axis(0, mpData, positions, 0xFF);
    if (rc != 0)
      ROS_PRINT("move_p2p_axis failed. result:%d", rc);

    // wait move done
    INT32 buff = 0;
    do
    {
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      if (goal_handle->is_canceling())
      {
        result->error_code = -1;
        result->error_string = "has cancel";
        goal_handle->canceled(result);
        ROS_PRINT("Goal Canceled");

        HwLib::Instance().stop(0);
        return;
      }

      HwLib::Instance().get_buffer_depth(&buff);
    } while (buff > 0);
    

    if(rclcpp::ok())
    {
      result->error_code = result->SUCCESSFUL;
      result->error_string = "Goal reached, success!";
      goal_handle->succeed(result);
    }

  }

}
