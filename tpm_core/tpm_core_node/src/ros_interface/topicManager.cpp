#include "topicManager.hpp"
#include "global_instance.hpp" //MAX_AXIS_NUM
#include "hwLib.hpp"
#include "tpm_core_msgs/msg/axis_status.hpp"
#include "myRobot.h"
#include "def_macro.h"
#include <chrono> //for timer

using namespace tpm_core_msgs::msg;
using namespace std::chrono; //for timer period.


namespace tpm_core
{
TopicManager::TopicManager(rclcpp::Node::SharedPtr node)
{
    size_t queueSize=1; //we always want the latest value. so don't need queue.
    publisher_ = node->create_publisher<RobotStatus>("robot_status", queueSize);
    
    timer_ = node->create_wall_timer(
      500ms, std::bind(&TopicManager::timerCallback_robotStatus, this));

}

void TopicManager::timerCallback_robotStatus()
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

    publisher_->publish(message);
}

}//end namesapce