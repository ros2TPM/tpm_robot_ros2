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
    static short set_vel_ratio(double value);
    static short set_jog_dist(double value);

    void create_axes(std::vector<unsigned char> slaveIps);
    void clear_axes();

    short do_action(char funcType, signed char axisId);

    bool is_homing(){return isHoming;}
    void get_pulse_per_deg(float* out_array, int len); 

    std::vector<Axis*> axes;   

private:
    int axisNum;    
    bool isHoming; //true if any axis is homing.
};
}