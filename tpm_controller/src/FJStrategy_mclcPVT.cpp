#include "FJStrategy.h"
#include "MCLC/MCLC.h"

#define AXIS_NUM 6 //todo: use vector instead of fixed len?

FJStrategy_mclcPVT::FJStrategy_mclcPVT(rclcpp::Node::SharedPtr node)
    :FJStrategy(node)
{
    mclc_init();
}

void FJStrategy_mclcPVT::Execute(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(_node->get_logger(), "start move mclcPVT...");

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    size_t trjIdx = 0;

    MCL_PVT_POINT* pvtPoints[AXIS_NUM];
    for (size_t i = 0; i < AXIS_NUM; i++)
        pvtPoints[i] = new MCL_PVT_POINT[trjPointSize];

    while (rclcpp::ok() && trjIdx < trjPointSize)
    {
        auto point = goal->trajectory.points.at(trjIdx);
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id   = goal->trajectory.header.frame_id;
        feedback->header.stamp      = goal->trajectory.header.stamp;
        feedback->joint_names       = goal->trajectory.joint_names;

        //double preVel = 0;
        for (size_t i = 0; i < point.positions.size(); i++)
        {
            double pos = point.positions.at(i);
            double vel = point.velocities.at(i);

            //=======================================================
                pvtPoints[i][trjIdx].pos = pos;
                pvtPoints[i][trjIdx].vel = vel;
                pvtPoints[i][trjIdx].timeMs = (point.time_from_start.sec * 1e3f) + (point.time_from_start.nanosec / 1e6f);
                //RCLCPP::INFO(_node->get_logger(), "pvtPoints(%d).timeMs = %f\n", trjIdx, pvtPoints[i][trjIdx].timeMs);
            //=======================================================
            
            //preVel = vel;
        }
        
        trjIdx++;
    }

    //=======================================================
    for (size_t i = 0; i < AXIS_NUM; i++)
    {
        auto err = mclc_axis_move_pvt(i, 0, trjPointSize, pvtPoints[i], 1000);
        if (err != MCL_ERR::MCL_OK)
            RCLCPP_ERROR(_node->get_logger(), "Add PVT cmd failed. ErrCode: %d", err);
        
        delete[] pvtPoints[i];
    }
    //=======================================================

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
        feedback->header.frame_id = goal->trajectory.header.frame_id;
        feedback->header.stamp = goal->trajectory.header.stamp;
        feedback->joint_names = goal->trajectory.joint_names;
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

std::vector<double> FJStrategy_mclcPVT::GetMJointStates()
{
    _mJointStates.clear();
    for (size_t i=0;i<6;i++)
    {
        double pos;
        mclc_axis_get_trgPosCmd(i, &pos);
        _mJointStates.push_back(pos);
    }
    return _mJointStates;
}
