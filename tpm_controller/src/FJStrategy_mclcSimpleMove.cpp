#include "FJStrategy.h"
#include "MCLC/MCLC.h"
#include <thread>
#include <chrono>

#define AXIS_NUM 6 //todo: get axis len from trajectoryPoints?

FJStrategy_mclcSimpleMove::FJStrategy_mclcSimpleMove(rclcpp::Node::SharedPtr node)
    :FJStrategy(node)
{
    mclc_init();
}

void FJStrategy_mclcSimpleMove::Execute(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(_node->get_logger(), "start move mclcSimpleMove...");

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    size_t trjIdx = 0;

    while (rclcpp::ok() && trjIdx < trjPointSize)
    {
        auto point = goal->trajectory.points.at(trjIdx);
        
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id   = goal->trajectory.header.frame_id;
        feedback->header.stamp      = goal->trajectory.header.stamp;
        feedback->joint_names       = goal->trajectory.joint_names;

        double preVel = 0;
        for (size_t i = 0; i < point.positions.size(); i++)
        {
            double pos = point.positions.at(i);
            double vel = point.velocities.at(i);
            //double acc = point.accelerations.at(i);

            //====================================================
            if (trjIdx <= 0)
                continue;
            
            MCL_MPDATA mp=MCL_MPDATA();
            mp.IsVsVeOnly = true;
            mp.Decel = 10;
            mp.VelStart = abs(preVel);
            mp.VelEnd = abs(vel);
            mp.Coord = MCL_COORD_TYPE::MCL_COORD_ABS;

            //RCLCPP_INFO(_node->get_logger(), "point(%d) axis(%d)--> pos: %.3f, vel: %.3f, acc: %.3f",
            //    trjIdx, i, pos, vel, acc);

            MCL_ERR err = mclc_axis_move_pos(i, trjIdx, &mp, pos);
            while (err == MCL_ERR_BUF_FULL)
            {
                rclcpp::sleep_for(std::chrono::nanoseconds(1000000));
                err = mclc_axis_move_pos(i, trjIdx, &mp, pos);
            }
            
            if (err != MCL_ERR::MCL_OK)
                RCLCPP_ERROR(_node->get_logger(), "Add POS cmd failed. ErrCode: %d", err);
        
            //====================================================

            preVel = vel;
        }
        
        trjIdx++;
    }


    bool isMoving;
    std::chrono::nanoseconds sleepTime(1000000); // 1ms
    do{
        U32 buff;
        isMoving = false;
        for (size_t i = 0; i < AXIS_NUM; i++)
        {
            mclc_axis_get_buffer_depth(i, &buff);
            if (buff != 0)
            {
                isMoving = true;
                break;
            }
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id   = goal->trajectory.header.frame_id;
        feedback->header.stamp      = goal->trajectory.header.stamp;
        feedback->joint_names       = goal->trajectory.joint_names;
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling())
        {
            result->error_code = -1;
            result->error_string = "has cancel";
            goal_handle->canceled(result);
            RCLCPP_INFO(_node->get_logger(), "Goal Canceled");

            for (size_t i = 0; i < 6; i++)
                mclc_axis_stop(i, MCL_STOP_TYPE::MCL_STOP_RAPID);
            return;
        }

        rclcpp::sleep_for(sleepTime);
    }while (rclcpp::ok() && isMoving);

    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
    RCLCPP_INFO(_node->get_logger(), "Goal Secceeded");
}

std::vector<double> FJStrategy_mclcSimpleMove::GetMJointStates()
{
    _mJointStates.clear();
    for (size_t i=0;i<6;i++)
    {
        FLT pos;
        mclc_axis_get_trgPosCmd(i, &pos);
        _mJointStates.push_back(pos);
    }
    return _mJointStates;
}
