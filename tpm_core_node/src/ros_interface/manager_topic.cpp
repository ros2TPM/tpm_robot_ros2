#include "manager_topic.hpp"
#include "global_instance.hpp" //MAX_AXIS_NUM
#include "hwLib.hpp"
#include "tpm_msgs/msg/axis_status.hpp"
#include "myRobot.h"
#include "def_macro.h"
#include <chrono> //for timer

#include "robot_kinematics/robotKinematics.hpp"

using namespace sensor_msgs::msg;
using namespace tpm_msgs::msg;
using namespace std::chrono; //for timer period.


namespace tpm_core
{
Manager_Topic::Manager_Topic(rclcpp::Node::SharedPtr node)
{
    size_t queueSize=1; //we always want the latest value. so don't need queue.
    publisher_ = node->create_publisher<RobotStatus>("robot_status", queueSize);
    joint_states_publisher_ = node->create_publisher<JointState>("joint_states", queueSize);
    
    timer_ = node->create_wall_timer(
      500ms, std::bind(&Manager_Topic::timerCallback_robotStatus, this));

}

void Manager_Topic::timerCallback_robotStatus()
{
    auto message = RobotStatus();

    //---read data from RIDT ----
    auto RIDT = HwLib::Instance().ri_get_RIDT();
    if(RIDT == nullptr)
        return;

    for(int i=0; i< Global::MAX_AXIS_NUM; i++){
        message.axes[i].deg = RIDT->robc.axis[i];
        message.axes[i].mnet_encoder = RIDT->m1a[i].encPos;

        U32 ioSts = RIDT->m1a[i].ioSts;
        message.axes[i].is_servo_on = ioSts & (1 << 14);
        message.axes[i].is_org      = ioSts & (1 << 4); 
        message.axes[i].is_pel      = ioSts & (1 << 2); 
        message.axes[i].is_nel      = ioSts & (1 << 3); 
        message.axes[i].is_alm      = ioSts & (1 << 1); 
        
        message.axes[i].deg = RIDT->robc.axis[i];
        message.axes[i].mnet_encoder = RIDT->m1a[i].encPos;
    }
    for(int i=0; i < 6; i++)
        message.end_pose[i] = RIDT->robc.pose[i];

    message.buffer_depth = RIDT->robc.bufDepth;

    publisher_->publish(message);

    auto jointStatesMsg = JointState();
    jointStatesMsg.header.stamp = rclcpp::Clock{}.now();
    RobotKinematics::GetInstance()->GetJointStates(
        RIDT->robc.axis, 
        RIDT->robc.pose, 
        jointStatesMsg.name, 
        jointStatesMsg.position
        );
    joint_states_publisher_->publish(jointStatesMsg);
}

}//end namesapce