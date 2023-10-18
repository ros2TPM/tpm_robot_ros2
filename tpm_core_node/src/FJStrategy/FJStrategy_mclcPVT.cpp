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

    size_t axisNum = MAX_AXIS_PER_ROBOT;
    if (MAX_AXIS_PER_ROBOT > goal->trajectory.points[0].positions.size())
      axisNum = goal->trajectory.points[0].positions.size();
    
    MCL_PVT_POINT** pvtPoints = new MCL_PVT_POINT*[axisNum];
    for (size_t i = 0; i < axisNum; i++)
      pvtPoints[i] = new MCL_PVT_POINT[trjPointSize];

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

        //ROS_PRINT("Axis(%d) Point(%ld): %.3f", i, trjIdx, pos);
      }

      trjIdx++;
    }
    //ROS_PRINT("ToltalTime: %.2f", pvtPoints[0][trjPointSize-1].timeMs);

    short err;
    for (size_t i = 0; i < axisNum; i++)
    {
      err = HwLib::Instance().set_pvt_data(i, trjPointSize, pvtPoints[i]);
      if(err != 0)
        ROS_PRINT("Set PVT data failed. ErrCode: %d", err);
    }
    err = HwLib::Instance().move_pvt(1000, 0xFF);
    if(err != 0)
      ROS_PRINT("Add PVT cmd failed. ErrCode: %d", err);

    // Delete pvtPoints
    for (size_t i = 0; i < axisNum; i++)
      delete[] pvtPoints[i];
    delete[] pvtPoints;

    bool isMoving;
    U32 buff;
    std::chrono::milliseconds sleepTime(10);
    do{
      rclcpp::sleep_for(sleepTime);
      
      isMoving = false;
      HwLib::Instance().get_buffer_depth(&buff);
      if (buff != 0)
        isMoving = true;

      if (goal_handle->is_canceling())
      {
        result->error_code = -1;
        result->error_string = "has canceled";
        ROS_PRINT("Goal Canceled");

        HwLib::Instance().stop(0);

        return;
      }
    }while(rclcpp::ok() && isMoving);

    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
    ROS_PRINT("Goal Secceeded");
  }
}
