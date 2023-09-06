#include "FJStrategy.h"
#include "archive.hpp"

#define AXIS_NUM 6 //todo: use vector instead of fixed len?

using namespace tpm_msgs::msg;
using namespace std::chrono_literals;

FJStrategy_mclcPVTClient::FJStrategy_mclcPVTClient(rclcpp::Node::SharedPtr node)
    :FJStrategy(node)
{
    _mJointStates.resize(AXIS_NUM, 0);
    cliDDACycle = _node->create_client<tpm_msgs::srv::MailBox>("/mclc/ddaCycle");
    cliGetAxisBuff = _node->create_client<tpm_msgs::srv::MailBox>("/mclc/axisGetBuffDepth");
    cliGetAxisPos = _node->create_client<tpm_msgs::srv::MailBox>("/mclc/axisGetTrgPosCmd");
    cliMovePVT = _node->create_client<tpm_msgs::srv::MovePVT>("/mclc/axisMovePvt");
    cliStop = _node->create_client<tpm_msgs::srv::MailBox>("/mclc/axisStop");
}

void FJStrategy_mclcPVTClient::Execute(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(_node->get_logger(), "start move mclcPVT...");

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointSize = goal->trajectory.points.size();
    size_t trjIdx = 0;
 
    PVTPoint* pvtPoints[AXIS_NUM];
    for (size_t i = 0; i < AXIS_NUM; i++)
        pvtPoints[i] = new PVTPoint[trjPointSize];

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
                pvtPoints[i][trjIdx].time_ms = (point.time_from_start.sec * 1e3f) + (point.time_from_start.nanosec / 1e6f);
                //RCLCPP::INFO(_node->get_logger(), "pvtPoints(%d).timeMs = %f\n", trjIdx, pvtPoints[i][trjIdx].timeMs);
            //=======================================================
            
            //preVel = vel;
        }
        
        trjIdx++;
    }

    //=======================================================
    auto request = std::make_shared<tpm_msgs::srv::MovePVT::Request>();
    for (size_t i = 0; i < AXIS_NUM; i++)
    {
        request->axis_id = i;
        request->cmd_id = 0;
        request->stop_dec = 1000;
        request->pvt_points.resize(trjPointSize);
        memcpy(request->pvt_points.data(), pvtPoints[i], sizeof(PVTPoint) * trjPointSize);

        auto moveRes = cliMovePVT->async_send_request(request);
        delete[] pvtPoints[i];
    }
    //=======================================================

    bool isMoving;
    std::chrono::nanoseconds sleepTime(1000000); // 1ms

    CArchive send;
    char axisId = 0;
    send << axisId;
    auto getBuffReq = std::make_shared<tpm_msgs::srv::MailBox::Request>();

    getBuffReq->buffer.resize(send.GetSize());
    memcpy(getBuffReq->buffer.data(), send.GetData(), send.GetSize());
    do{
        isMoving = false;

        get_buffer_done_ = false;
        auto getBuffRes = cliGetAxisBuff->async_send_request(
        getBuffReq, std::bind(&FJStrategy_mclcPVTClient::get_buffer_callback, this, std::placeholders::_1));
        
        while(get_buffer_done_ == false)
        rclcpp::sleep_for(sleepTime);

        if (buffer_ != 0)
            isMoving = true;


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

            auto stopReq = std::make_shared<tpm_msgs::srv::MailBox::Request>();
            char type = 0;

            for (char i = 0; i < AXIS_NUM; i++)
            {
                send.Clear();
                send << i << type;
                stopReq->buffer.resize(send.GetSize());
                memcpy(stopReq->buffer.data(), send.GetData(), send.GetSize());
                auto stopRes = cliStop->async_send_request(stopReq);
            }
            return;
        }

        rclcpp::sleep_for(sleepTime);
    }while(rclcpp::ok() && isMoving);
    
    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
}


void FJStrategy_mclcPVTClient::get_buffer_callback(
  rclcpp::Client<tpm_msgs::srv::MailBox>::SharedFuture future)
{
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready)
  {
    CArchive recv;
    recv.Update(future.get()->buffer.size(), future.get()->buffer.data());
    recv >> buffer_;

    //RCLCPP_INFO(_node->get_logger(), "get_buffer: %d", buffer_);
    get_buffer_done_ = true;
  }
}

void FJStrategy_mclcPVTClient::get_axis_callback(
  rclcpp::Client<tpm_msgs::srv::MailBox>::SharedFuture future)
{
  //RCLCPP_INFO(_node->get_logger(), "get_axis_callback");
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready)
  {
    CArchive recv;
    recv.Update(future.get()->buffer.size(), future.get()->buffer.data());
    recv >> axis_pos_;

    get_axis_done_ = true;
  }
}
