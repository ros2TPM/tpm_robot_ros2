#include <vector>

namespace tpm_core {
class Axis;
class Robot //this is a singleton class.
{
private:
    // private constructor, copy constructor, and assignment operator
    Robot(){};
    Robot(const Robot&);
    Robot& operator=(const Robot&);

    // forward declare a inner class for private functions.
    class Private;

public:
    static Robot& getInstance(){
        static Robot instance; //initialized only once.
        return instance;
    }
    ~Robot();// This will be automatically called when the program exits
    
    static double get_vel_ratio();

    void create_axes(std::vector<unsigned char> slaveIps);
    void clear_axes();

    short do_axis_action(char funcType, signed char axisId);
    short do_action(char funcType, float arg1);

    short jog_pose(char poseId, signed char dir);

    bool is_homing(){return isHoming;}

    std::vector<Axis*> axes;   

private:
    int axisNum;    
    bool isHoming; //true if any axis is homing.
};
}