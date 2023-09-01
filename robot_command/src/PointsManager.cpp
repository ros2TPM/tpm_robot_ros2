#include "PointsManager.h"

#include "geometry_msgs/msg/quaternion.h"
#include <cmath> // For trigonometric functions
#include <fstream>
#include <iostream>

void deg2Rad(Joints& jnt)
{
    for(auto& item : jnt.values)
        item *= M_PI/180;
}
void print(Joints& jnt)
{
    std::cout<<"{";
    for(auto& item : jnt.values)
        std::cout<<item<<", ";
    std::cout<<"}\n"; 
}
template <typename T>
void processVectorItems(std::vector<T>& v, void (*func)(T&)) {
    for (auto& item : v) {
        func(item);
    }
}

PoseVector PointsManager::GetFixedPose1()
{
    PoseVector poseVector;
    Pose p;
    p.orientation = Euler_To_Quaternion(0,180,0);
    //p.orientation.w = 1.0;
    //p.orientation.x = 1.0;
    p.position.x = 0.28;
    p.position.y = 0.0;
    p.position.z = 0.5;

    poseVector.push_back(p);

    p.position.x = 0.3;
    poseVector.push_back(p);

    //p.position.x = 0.4;
    //poseVector.push_back(p);

    //p.position.x = 0.2;
    //poseVector.push_back(p);

    return poseVector;

}
JointsVector PointsManager::GetFixedJoints1()
{
    JointsVector jointsVec;

    Joints j;
    j.values = {0,-1.5,1.5,0,0,0};
    jointsVec.push_back(j);

    j.values[0]=0.5;
    jointsVec.push_back(j);

    return jointsVec;
}
JointsVector PointsManager::GetJoints_fromFile()
{
    JointsVector jointsVec;
    const std::string fileName= "myJoints.txt";

    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << fileName << std::endl;
        return jointsVec;
    }

    // line format: {-10, 90, ..., 180}  //represents a point with joint values (deg).
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        char ignoreChar;
        Joints joint;

        if (ss >> ignoreChar && ignoreChar == '{') //start of this line.
        {
            //std::cout<<"\nline "<<jointsVec.size()+1 <<": ";
            double value;
            while (ss >> value) 
            {
                //std::cout<<value<<", ";
                joint.values.push_back(value);
                if (ss >> ignoreChar && ignoreChar == ',')
                    continue;
                else
                    break;
            }
            if (ignoreChar == '}')  //end of this line
                jointsVec.push_back(joint);           
            else 
                std::cerr << "Invalid format in file: " << fileName << std::endl;
        } else {
            std::cerr << "Invalid format in file: " << fileName << std::endl;
        }
    }
    std::cout<<"joints in deg:\n";
    processVectorItems(jointsVec, print);
    processVectorItems(jointsVec, deg2Rad);
    std::cout<<"\njoints in rad:\n";
    processVectorItems(jointsVec, print);
    return jointsVec;
}
// Transfer {x, y, z, a, b, c} to geometry_msgs::msg::Pose
Quaternion PointsManager::Euler_To_Quaternion(double a, double b, double c)
{
    Quaternion q;

    // Convert Euler angles to quaternion for orientation
    double roll  = a/180 * M_PI;    // Rotation along x-axis
    double pitch = b/180 * M_PI;   // Rotation along y-axis
    double yaw   = c/180 * M_PI;     // Rotation along z-axis

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
