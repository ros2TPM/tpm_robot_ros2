#pragma once

/* This class implements different strategy for FollowJointTrajectory action.
*/

#include <vector>
#include <memory>
#include <chrono>
#include <iostream>

// ros2 system
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// definition of Actions & Topics
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
//#include "sensor_msgs/msg/joint_state.hpp"
#include "tpm_msgs/srv/move_ptp.hpp"
#include "tpm_msgs/srv/move_pvt.hpp"
#include "tpm_msgs/srv/mail_box.hpp"

// user includes
//#include "MCLC/MCLC.h"
//#include "MCLC_Simulation.h"

class FJStrategy
{
public:

    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
    
    FJStrategy(rclcpp::Node::SharedPtr node)
    {
        this->_node = node;
    }

    virtual void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) = 0;
    virtual std::vector<double> GetMJointStates() {return _mJointStates;}

protected:
    rclcpp::Node::SharedPtr _node; //for log print
    std::vector<double> _mJointStates;
};

class FJStrategy_printMsgOnly : public FJStrategy
{
public:
    FJStrategy_printMsgOnly(rclcpp::Node::SharedPtr node)
        :FJStrategy(node)
    {}

    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;  
};

class FJStrategy_mclcPVT : public FJStrategy
{
public:
    FJStrategy_mclcPVT(rclcpp::Node::SharedPtr node);

    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;
    std::vector<double> GetMJointStates() override;
};

class FJStrategy_mclcSimpleMove : public FJStrategy
{
public:
    FJStrategy_mclcSimpleMove(rclcpp::Node::SharedPtr node);

    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;
    std::vector<double> GetMJointStates() override;
};

class FJStrategy_mclcPVTClient : public FJStrategy
{
public:
    FJStrategy_mclcPVTClient(rclcpp::Node::SharedPtr node);

    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;

private:
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliDDACycle;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliGetAxisBuff;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliGetAxisPos;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliStop;
    rclcpp::Client<tpm_msgs::srv::MovePVT>::SharedPtr cliMovePVT;

    double axis_pos_;
    unsigned int buffer_;
    bool get_buffer_done_;
    bool get_axis_done_;
    void get_buffer_callback(rclcpp::Client<tpm_msgs::srv::MailBox>::SharedFuture future);
    void get_axis_callback(rclcpp::Client<tpm_msgs::srv::MailBox>::SharedFuture future);
};

class FJStrategy_robcAxisPTP : public FJStrategy
{
public:
    FJStrategy_robcAxisPTP(rclcpp::Node::SharedPtr node);

    void Execute(const std::shared_ptr<GoalHandleFJT> goal_handle) override;

private:
    rclcpp::Client<tpm_msgs::srv::MovePTP>::SharedPtr cliMoveAxisPTP;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliRobGetBuff;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliRobStop;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliDDACycle;
    rclcpp::Client<tpm_msgs::srv::MailBox>::SharedPtr cliGetAxis;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subRobAxes;
    void Handle_RobAxesChanged(const std_msgs::msg::Float64MultiArray& msg);

    int buffer_;
    bool get_buffer_done_;
    void get_buffer_callback(rclcpp::Client<tpm_msgs::srv::MailBox>::SharedFuture future);

    void execute_all(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_last_point(const std::shared_ptr<GoalHandleFJT> goal_handle);
};



