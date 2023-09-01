#include "FJStrategy.h"
#include "archive.hpp"

using namespace std::chrono_literals;

FJStrategy_robcAxisPTP::FJStrategy_robcAxisPTP(rclcpp::Node::SharedPtr node)
    :FJStrategy(node)
{
  cliDDACycle = node->create_client<tpm_core_msgs::srv::MailBox>("/rob/ddaCycle");
  cliGetAxis = node->create_client<tpm_core_msgs::srv::MailBox>("/rob/getAxis");
  cliMoveAxisPTP = node->create_client<tpm_core_msgs::srv::MovePTP>("/rob/movePTP");
  cliRobGetBuff = node->create_client<tpm_core_msgs::srv::MailBox>("/rob/getBuffDepth");
  cliRobStop = node->create_client<tpm_core_msgs::srv::MailBox>("/rob/stop");
}

void FJStrategy_robcAxisPTP::Execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  //execute_all(goal_handle);
  execute_last_point(goal_handle);
}

void FJStrategy_robcAxisPTP::execute_all(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  RCLCPP_INFO(_node->get_logger(), "start move robPTP...");

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto trjPointSize = goal->trajectory.points.size();
  size_t trjIdx = 0;

  float maxVel = 0, maxAcc = 0;
  while (rclcpp::ok() && trjIdx < trjPointSize)
  {
    auto point = goal->trajectory.points.at(trjIdx);
    auto request = std::make_shared<tpm_core_msgs::srv::MovePTP::Request>();
    for (size_t i = 0; i < point.positions.size(); i++)
    {
      float pos = point.positions.at(i) * 180 / M_PI;
      request->pos.push_back(pos);

      if (maxVel < abs(point.velocities.at(i)))
        maxVel = abs(point.velocities.at(i));
      if (maxAcc < abs(point.accelerations.at(i)))
        maxAcc = abs(point.accelerations.at(i));
    }
    trjIdx++;

    request->cmd_id = trjIdx;
    request->mask = 255;
    request->mp_data.feed  = maxVel / M_PI * 180;
    request->mp_data.accel = maxAcc / M_PI * 180;
    request->mp_data.decel = maxAcc / M_PI * 180;
    request->mp_data.coord = 0;

    RCLCPP_INFO(_node->get_logger(), "RobCMovePTP: traj[%ld] A0=%.2f", trjIdx, request->pos[0]);
    auto moveRes = cliMoveAxisPTP->async_send_request(request);
    //rclcpp::spin_until_future_complete(_node, moveRes);

  }

  bool isMoving;
  std::chrono::nanoseconds sleepTime(1000000); // 1ms

  CArchive recv, send;
  auto getBuffReq = std::make_shared<tpm_core_msgs::srv::MailBox::Request>();
  do{
    isMoving = false;

    get_buffer_done_ = false;
    auto getBuffRes = cliRobGetBuff->async_send_request(
      getBuffReq, std::bind(&FJStrategy_robcAxisPTP::get_buffer_callback, this, std::placeholders::_1));
    
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

        auto stopReq = std::make_shared<tpm_core_msgs::srv::MailBox::Request>();
        char type = 0;
        send.Clear();
        send << type;
        stopReq->buffer.resize(send.GetSize());
        memcpy(stopReq->buffer.data(), send.GetData(), send.GetSize());
        auto stopRes = cliRobStop->async_send_request(stopReq);
        //rclcpp::spin_until_future_complete(_node, stopRes);

        return;
    }

    rclcpp::sleep_for(sleepTime);
  }while(rclcpp::ok() && isMoving);
  
  result->error_code = 0;
  result->error_string = "";

  goal_handle->succeed(result);

}
void FJStrategy_robcAxisPTP::execute_last_point(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  RCLCPP_INFO(_node->get_logger(), "start move robPTP...");

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto trjPointSize = goal->trajectory.points.size();

  float maxVel = 0;
  for (size_t j = 0; j < trjPointSize; j++)
  {
    auto point = goal->trajectory.points.at(j);
    for (size_t i = 0; i < point.positions.size(); i++)
    {
      if (maxVel < abs(point.velocities.at(i)))
        maxVel = abs(point.velocities.at(i));
    }
  }
  maxVel *= 180 / M_PI;//rad 2 deg.

  auto point = goal->trajectory.points.back();
  auto request = std::make_shared<tpm_core_msgs::srv::MovePTP::Request>();
  request->cmd_id = trjPointSize;
  request->mask = 0xff;
  request->mp_data.feed  = maxVel;
  request->mp_data.accel = maxVel * 10;
  request->mp_data.decel = maxVel * 10;
  request->mp_data.coord = 0;//ABS
  RCLCPP_INFO(_node->get_logger(), 
      "vel:%.2f  acc:%.2f", 
      request->mp_data.feed, 
      request->mp_data.accel);

  for (size_t i = 0; i < point.positions.size(); i++)
  {
    float pos = point.positions.at(i) * 180 / M_PI;
    request->pos.push_back(pos);
  }
  RCLCPP_INFO(_node->get_logger(), "RobCMovePTP: traj[%ld] A0=%.2f", trjPointSize, request->pos[0]);
  auto moveRes = cliMoveAxisPTP->async_send_request(request);


  bool isMoving;
  std::chrono::milliseconds sleepTime(100); // 100ms

  CArchive recv, send;
  auto getBuffReq = std::make_shared<tpm_core_msgs::srv::MailBox::Request>();
  do{
    isMoving = false;

    get_buffer_done_ = false;
    auto getBuffRes = cliRobGetBuff->async_send_request(
      getBuffReq, std::bind(&FJStrategy_robcAxisPTP::get_buffer_callback, this, std::placeholders::_1));
    
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

        auto stopReq = std::make_shared<tpm_core_msgs::srv::MailBox::Request>();
        char type = 0;
        send.Clear();
        send << type;
        stopReq->buffer.resize(send.GetSize());
        memcpy(stopReq->buffer.data(), send.GetData(), send.GetSize());
        auto stopRes = cliRobStop->async_send_request(stopReq);
        //rclcpp::spin_until_future_complete(_node, stopRes);

        return;
    }

    rclcpp::sleep_for(sleepTime);
  }while(rclcpp::ok() && isMoving);
  
  result->error_code = 0;
  result->error_string = "";

  goal_handle->succeed(result);
}

void FJStrategy_robcAxisPTP::get_buffer_callback(
  rclcpp::Client<tpm_core_msgs::srv::MailBox>::SharedFuture future)
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

