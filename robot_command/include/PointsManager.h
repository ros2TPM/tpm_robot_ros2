#include <vector>
#include "geometry_msgs/msg/pose.hpp"



struct Joints
{
    std::vector<double> values;
};


typedef geometry_msgs::msg::Pose Pose;
typedef geometry_msgs::msg::Quaternion Quaternion;
typedef std::vector<Pose> PoseVector;
typedef std::vector<Joints> JointsVector;

class PointsManager
{
public:
    

    static PoseVector GetFixedPose1();
    static JointsVector GetFixedJoints1();
    static JointsVector GetJoints_fromFile();

private:
    static Quaternion Euler_To_Quaternion(double a, double b, double c);
};