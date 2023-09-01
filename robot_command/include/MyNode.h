#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

struct Joints;

class MyNode : public rclcpp::Node {

  typedef moveit::planning_interface::MoveGroupInterface MGI;
  typedef geometry_msgs::msg::Pose Pose;

public:
  MyNode() : Node("my_node") 
  {
    
    //thread_ = std::thread(&MyNode::infiniteLoop, this);
    terminalThread_ = std::thread(&MyNode::terminalInput, this);
  }

  ~MyNode() {
    if (thread_.joinable()) 
      thread_.join();
    
    if (terminalThread_.joinable()) 
      terminalThread_.join();
    
  }

private:
  std::shared_ptr<rclcpp::Node> node_; //the node ptr of itself.
  std::thread thread_;
  std::thread terminalThread_;
  char userCmd_;
  bool isNeedStop_;

  void terminalInput() ;

  void infiniteLoop() ;
  void doWork();//work in infiniteLoop.
  void setTarget_Pose(const Pose& pose);
  void setTarget_Joint(const Joints& jnt);
  bool plan_and_execute();

};
