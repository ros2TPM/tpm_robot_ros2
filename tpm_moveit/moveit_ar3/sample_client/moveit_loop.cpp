#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose.hpp"
#include <thread>
#include <vector>
#include <fstream>

using moveit::planning_interface::MoveGroupInterface;

struct Points
{
    std::vector<double> values;
};

void deg2Rad(Points& pt)
{
    for(auto& item : pt.values)
        item *= M_PI/180;
}
void print(Points& pt)
{
    std::cout<<"{";
    for(auto& item : pt.values)
        std::cout<<item<<", ";
    std::cout<<"}\n"; 
}
template <typename T>
void processVectorItems(std::vector<T>& v, void (*func)(T&)) {
    for (auto& item : v) {
        func(item);
    }
}


std::shared_ptr<MoveGroupInterface> move_group_interface;
class MyNode : public rclcpp::Node {

  typedef moveit::planning_interface::MoveGroupInterface MGI;
  typedef geometry_msgs::msg::Pose Pose;

public:
  MyNode() : Node("my_node") 
  {    
    terminal_thread_ = std::thread(&MyNode::terminalInput, this);
  }
  ~MyNode() 
  {
    if (thread_.joinable()) 
      thread_.join();
    if (terminal_thread_.joinable()) 
      terminal_thread_.join();
  }

private:
  std::shared_ptr<rclcpp::Node> node_; //the node ptr of itself.
  std::thread thread_;
  std::thread terminal_thread_;
  std::vector<Points> pointVector;
  
  char userCmd_;
  bool isNeedStop_;

  void terminalInput() ;
  void infiniteLoop() ;
  void doWork();
  bool plan_and_execute();

  std::vector<Points> ReadPoints_fromFile()
  {
    std::vector<Points> pointsVec;
    const std::string fileName= "myJoints.txt";

    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return pointsVec;
    }

    // line format: {-10, 90, ..., 180}  //represents a point with joint values (deg).
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        char ignoreChar;
        Points joint;

        if (ss >> ignoreChar && ignoreChar == '{') //start of this line.
        {
            double value;
            while (ss >> value) 
            {
                joint.values.push_back(value);
                if (ss >> ignoreChar && ignoreChar == ',')
                    continue;
                else
                    break;
            }
            if (ignoreChar == '}')  //end of this line
                pointsVec.push_back(joint);           
            else 
                std::cerr << "Invalid format in file: " << fileName << std::endl;
        } else {
            std::cerr << "Invalid format in file: " << fileName << std::endl;
        }
    }
    std::cout<<"\njoints in deg:\n";
    processVectorItems(pointsVec, print);
    std::cout<<"\n";

    // convert deg to rad
    processVectorItems(pointsVec, deg2Rad);

    return pointsVec;
  }
};


void MyNode::terminalInput() {
    
    while (rclcpp::ok()) {
        std::cout << "[r]:run, [p]:print points, [q]:quit. ";
        std::cin >> userCmd_;

        switch(userCmd_)
        {
            case 'r':
                thread_ = std::thread(&MyNode::infiniteLoop, this);
                break;

            case 'p':
                ReadPoints_fromFile();
                break;

            case 'q':
                RCLCPP_INFO(this->get_logger(), "Stopping the thread...");
                isNeedStop_ = true;

                if (thread_.joinable()) 
                    thread_.join();

                RCLCPP_INFO(this->get_logger(), "thread stopped!");
                rclcpp::shutdown();
                break;
        }
        userCmd_ = 0;
    }
}

void MyNode::infiniteLoop() 
{
    RCLCPP_INFO(this->get_logger(), "try to find robot...");   
    node_ = shared_from_this(); //initialize the node ptr. (can't do this in constructor)
    move_group_interface = std::make_shared<MoveGroupInterface>(node_, "ar3_arm");
    move_group_interface->setMaxVelocityScalingFactor(1);

    RCLCPP_INFO(this->get_logger(), "start infinite loop...");
    isNeedStop_ = false;
    pointVector = ReadPoints_fromFile();
    int pointNum = pointVector.size();

    while (rclcpp::ok() && !isNeedStop_) {       
       
      // go through each points
      for(int i=0; i< pointNum; i++)
      {
        RCLCPP_INFO(this->get_logger(), "\nRunning pose %d/%d...", i+1, pointNum);
        move_group_interface->setJointValueTarget(pointVector[i].values);
        plan_and_execute();
        
        //delay 1s
        //std::this_thread::sleep_for(std::chrono::seconds(5));   
      }       
    }

    
    RCLCPP_INFO(this->get_logger(), "exit loop.");
}

bool MyNode::plan_and_execute()
{   
    // Create a plan to that target
    auto const [success, plan] = []{
        MGI::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface->plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    // Execute the plan
    if(success) 
        move_group_interface->execute(plan);
    else 
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");

    return success;
}




int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
