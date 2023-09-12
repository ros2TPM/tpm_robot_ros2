#include "FJStrategy.hpp"
#include "def_macro.h"
#include "hwLib.hpp"

namespace tpm_core
{
  void FJStrategy_mclcPVT::Execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    ROS_PRINT("start move mclcPVT...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    size_t trjIdx = 0;

    size_t axisNum = MIN(MAX_AXIS_PER_ROBOT, goal->trajectory.points[0].positions.size());
    MCL_PVT_POINT pvtPoints[axisNum][trjPointSize];

    while(rclcpp::ok() && trjIdx < trjPointSize)
    {
      auto point = goal->trajectory.points[trjIdx];

      for (size_t i = 0; i < axisNum; i++)
      {
        double pos = point.positions[i]  * 180 / M_PI;
        double vel = point.velocities[i] * 180 / M_PI;

        pvtPoints[i][trjIdx].pos = pos;
        pvtPoints[i][trjIdx].vel = vel;
        pvtPoints[i][trjIdx].timeMs = (point.time_from_start.sec * 1e3f) + (point.time_from_start.nanosec / 1e6f);
      }

      trjIdx++;
    }

    for (size_t i = 0; i < axisNum; i++)
    {
      auto err = HwLib::Instance().axis_move_pvt(i, 0, trjPointSize, pvtPoints[i], 1000);
      if(err != 0)
        ROS_PRINT("Add PVT cmd failed. ErrCode: %d", err);
      
    ROS_PRINT("tag");
    }

    ROS_PRINT("t1");
    bool isMoving;
    U32 buff;
    std::chrono::milliseconds sleepTime(1);
    do{
      isMoving = false;
      for (size_t i = 0; i < axisNum; i++)
      {
        HwLib::Instance().axis_get_buffer_depth(i, &buff);
    ROS_PRINT("t2");
        if (buff != 0)
        {
          isMoving = true;
          break;
        }
      }

      if (goal_handle->is_canceling())
      {
        result->error_code = -1;
        result->error_string = "has canceled";
        ROS_PRINT("Goal Canceled");

        for (size_t i = 0; i < axisNum; i++)
          HwLib::Instance().axis_stop(i, MCL_STOP_RAPID);

        return;
      }

      rclcpp::sleep_for(sleepTime);
    }while(rclcpp::ok() && isMoving);

    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
    ROS_PRINT("Goal Secceeded");
  }
}