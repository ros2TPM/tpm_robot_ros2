#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "manager_service.hpp"
#include "manager_topic.hpp"
#include "manager_param.hpp"
#include "hwLib.hpp"
#include "myRobot.h"
#include "global_instance.hpp"
#include "global_config.hpp"
#include "def_macro.h"
#include <vector>

using namespace tpm_core;

class TpmCoreNode : public rclcpp::Node
{
  public:
    TpmCoreNode(): Node("tpm_core_node"){}
    ~TpmCoreNode()
    {
      ROS_PRINT("==== start disconnect...");
      HwLib::Instance().disconnect();
    }
    void init()
    {
      // Note: can't use 'this->shared_from_this()' in constructor. so put in a init() function.
        ROS_PRINT("==== Load parameters...");
        param_ = std::make_shared<Manager_Param> (this->shared_from_this());
        
        ROS_PRINT("==== start connect...");
        auto& hwLib = HwLib::Instance();
        hwLib.connect();
        hwLib.init();

        auto vec_axisIP = hwLib.get_vec_axisIP();
        Robot::getInstance().create_axes(vec_axisIP);

        ROS_PRINT("==== Create service...");
        srv_ = std::make_shared<Manager_Service> (this->shared_from_this());
        
        ROS_PRINT("==== Create topic...");
        tpc_ = std::make_shared<Manager_Topic>   (this->shared_from_this()); 
    }
  private:
    std::shared_ptr<Manager_Service> srv_; 
    std::shared_ptr<Manager_Topic>   tpc_;
    std::shared_ptr<Manager_Param>   param_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto myNode = std::make_shared<TpmCoreNode>();
  myNode->init();
  
  ROS_PRINT("==== Initialize complete ! ready to spin...");
  rclcpp::spin(myNode);
  rclcpp::shutdown();
  return 0;
}
