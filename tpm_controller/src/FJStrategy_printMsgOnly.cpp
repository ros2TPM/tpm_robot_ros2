
#include "FJStrategy.h"

void FJStrategy_printMsgOnly::Execute(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(_node->get_logger(), "FJStrategy_SimplePrint::Execute...!!");


    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(_node->get_logger(), "start move...");

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    int trjIdx = 0;

    while (rclcpp::ok() && trjIdx < trjPointSize)
    {
        auto point = goal->trajectory.points.at(trjIdx);
        
        if (trjIdx > 0)
        {
            auto last_time = goal->trajectory.points.at(trjIdx-1).time_from_start;
            auto current_time = point.time_from_start;

            rclcpp::Time time1(last_time.sec, last_time.nanosec);
            rclcpp::Time time2(current_time.sec, current_time.nanosec);
            auto time_to_sleep = time2 - time1;

            int64_t duration = ((int64_t)time_to_sleep.seconds()) * 1e9 + time_to_sleep.nanoseconds();
            std::chrono::nanoseconds ns(duration);
            RCLCPP_INFO(_node->get_logger(), "Velocities: ");
            for (size_t i = 0; i < point.velocities.size(); i++)
                RCLCPP_INFO(_node->get_logger(), "%.03lf  ", point.velocities.at(i));
            
            RCLCPP_INFO(_node->get_logger(), "\nAccelerations: ");
            for (size_t i = 0; i < point.accelerations.size(); i++)
                RCLCPP_INFO(_node->get_logger(), "%.03lf  ", point.accelerations.at(i));
            
            RCLCPP_INFO(_node->get_logger(), "\nsleep for: %ld\n", ns.count());
            rclcpp::sleep_for(ns);
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id   = goal->trajectory.header.frame_id;
        feedback->header.stamp      = goal->trajectory.header.stamp;
        feedback->joint_names       = goal->trajectory.joint_names;

        for (size_t i = 0; i < point.positions.size(); i++)
        {
            _mJointStates[i] = point.positions.at(i);
        }
        
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling())
        {
            result->error_code = -1;
            result->error_string = "has cancel";
            goal_handle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Goal Canceled");
            return;
        }
        RCLCPP_INFO(_node->get_logger(), "Publish Feedback");
        trjIdx++;
    }

    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
    RCLCPP_INFO(_node->get_logger(), "Goal Secceeded");
}
