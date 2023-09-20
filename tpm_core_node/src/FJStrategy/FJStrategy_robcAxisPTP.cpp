#include "FJStrategy.hpp"
#include "def_macro.h"
#include "hwLib.hpp"
#include "global_config.hpp" // for RobotSpec

namespace tpm_core
{
  void FJStrategy_robcAxisPTP::Execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    ROS_PRINT("start move robcPTP...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    double maxVel = 0;

    // get max velocity
    MCL_MPDATA mpData;
    memset(&mpData, 0, sizeof(mpData));
    for(size_t i = 0; i < trjPointSize; i++)
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
    auto point = goal->trajectory.points.back();
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