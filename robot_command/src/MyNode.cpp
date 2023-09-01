#include "MyNode.h"
#include <moveit/move_group_interface/move_group_interface.h>

#include "PointsManager.h"
#include "SolutionChooser.h"

using moveit::planning_interface::MoveGroupInterface;


std::shared_ptr<rclcpp::Node> nodeLocal; //the node ptr of itself.
std::shared_ptr<MoveGroupInterface> move_group_interface;
PoseVector poseVector;
JointsVector jointVector;


void MyNode::terminalInput() {
    
    while (rclcpp::ok()) {
        std::cout << "r:run, f:readFile q:quit. ";
        std::cin >> userCmd_;

        switch(userCmd_)
        {
            case 'r':
                thread_ = std::thread(&MyNode::infiniteLoop, this);
                break;

            case 'f':
                PointsManager::GetJoints_fromFile();
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
    RCLCPP_INFO(this->get_logger(), "enter loop...");
    isNeedStop_ = false;
    nodeLocal   = node_ = shared_from_this(); //initialize the node ptr. (can't do this in constructor)
    //poseVector  = PointsManager::GetFixedPose1();
    //jointVector = PointsManager::GetFixedJoints1();
    jointVector = PointsManager::GetJoints_fromFile();
    

    RCLCPP_INFO(this->get_logger(), "try to find robot...");   
    move_group_interface = std::make_shared<MoveGroupInterface>(node_, "ar3_arm");
    move_group_interface->setMaxVelocityScalingFactor(1);

    while (rclcpp::ok() && !isNeedStop_) {
        RCLCPP_INFO(this->get_logger(), "do work...");        
        doWork();
        
        //delay 1s
        //std::this_thread::sleep_for(std::chrono::seconds(1));         
    }

    RCLCPP_INFO(this->get_logger(), "move to zero position...");
    Joints jntZeros;
    jntZeros.values = {0,0,0,0,0,0};
    setTarget_Joint(jntZeros);
    plan_and_execute();
    
    RCLCPP_INFO(this->get_logger(), "exit loop.");
}

void MyNode::setTarget_Pose(const Pose& pose)
{
    RCLCPP_INFO(this->get_logger(), 
        "(x,y,z) = (%.2f, %.2f, %.2f)",
        pose.position.x,
        pose.position.y,
        pose.position.z
        );
   
    move_group_interface->setPoseTarget(pose);    
}
void MyNode::setTarget_Joint(const Joints& jnt)
{
    /*RCLCPP_INFO(this->get_logger(), 
        "joint[0,1,2] = (%.2f, %.2f, %.2f)",
        jnt.values[0],
        jnt.values[1],
        jnt.values[2]
        );
        */
    move_group_interface->setJointValueTarget(jnt.values);
}
bool MyNode::plan_and_execute()
{   
    // Create a plan to that target pose
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

void MyNode::doWork() {
    //RCLCPP_INFO(this->get_logger(), "Running...");
      

    int num = jointVector.size();
    for(int i=0; i< num; i++)
    {
        RCLCPP_INFO(this->get_logger(), "Running pose %d/%d...", i+1, num);
        setTarget_Joint(jointVector[i]);

        plan_and_execute();
    }
}


