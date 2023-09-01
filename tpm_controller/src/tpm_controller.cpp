#include <vector>
#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tpm_core_msgs/msg/robot_status.hpp"

#include "FJStrategy.h"

#define AXIS_NUM 6

class TPMController : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit TPMController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("tpm_controller", options)
    {
        using namespace std::placeholders;

        // create a server to receive follow_joint_trajectory
        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this, "/ar3_arm_controller/follow_joint_trajectory",
            std::bind(&TPMController::handle_goal, this, _1, _2),
            std::bind(&TPMController::handle_cancel, this, _1),
            std::bind(&TPMController::handle_accepted, this, _1)
        );

        // publish for update JointState
        mJointStates = {0,-1.571,1.571,0,1,0};
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // subscription for update robot status
        robot_status_subsriber_ = this->create_subscription<tpm_core_msgs::msg::RobotStatus>("robot_status", 1,
            std::bind(&TPMController::Handle_RobotStatusUpdated, this, _1));
    }

    void setFJStrategy(FJStrategy* fj)
    {
        _fj = fj;
    }
private:
    FJStrategy* _fj; //follow joint strategy
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<tpm_core_msgs::msg::RobotStatus>::SharedPtr robot_status_subsriber_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        int pointSize = goal->trajectory.points.size();

        RCLCPP_INFO(this->get_logger(), "Handle goal: %d", pointSize);
        
        for (int i = 0; i < pointSize; i++)
        {
            auto point = goal->trajectory.points.at(i);
            //std::cout << point.positions;
        }

        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        std::thread{[this, goal_handle]() {_fj->Execute(goal_handle);}}.detach();
    }

    void Handle_RobotStatusUpdated(const tpm_core_msgs::msg::RobotStatus& robStatus)
    {
        //RCLCPP_INFO(this->get_logger(), "Handle_RobotStatusUpdated");
        mJointStates.clear();

        size_t axisNum = robStatus.axes.size();
        for (size_t i = 0; i < axisNum; i++)
            mJointStates.push_back(robStatus.axes[i].deg * M_PI / 180);

        sensor_msgs::msg::JointState jointStates;
        jointStates.header.frame_id = "";

        double timeSec = this->now().seconds();
        int sec = timeSec;
        jointStates.header.stamp.sec = sec;
        jointStates.header.stamp.nanosec = (timeSec - sec) * 1e9;

        jointStates.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        jointStates.position = mJointStates;

        joint_state_publisher_->publish(jointStates);
    }

    std::vector<double> mJointStates;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TPMController>();

    //FJStrategy* fj = new FJStrategy_printMsgOnly(node);
    //FJStrategy* fj = new FJStrategy_mclcSimpleMove(node);
    //FJStrategy* fj = new FJStrategy_mclcPVT(node);
    //FJStrategy* fj = new FJStrategy_mclcPVTClient(node);
    FJStrategy* fj = new FJStrategy_robcAxisPTP(node);

    node->setFJStrategy(fj);

    rclcpp::spin(node);
    rclcpp::shutdown();

    delete fj;

    return 0;
}
