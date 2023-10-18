#pragma once

namespace tpm_core {
typedef struct _AxisConfig
{
    //note: all unit is deg/mm, not pulse
    bool home_dir;
    double home_offsets;  
    double max_jog_vel;
    double pulse_per_deg;
    int home_escape_offset;
}AxisConfig;

class Axis
{
public:
    

    Axis(){};
    ~Axis(){};

    void init(unsigned short slvIp, int id);
    short servo(bool isOn);
    short clear_alarm();
    short start_homing();
    short search_org();
    short set_as_offset();
    short set_as_zero();
    short mv_to_zero();
    short jog(bool isPos);
    
    bool is_homing(){return _isHoming;}

private:
    static double jog_dist; //unit:deg. -1 means infinite.
    static double vel_ratio;    //value: [0~100%]
    
    unsigned short _slvIp;
    int _id;
    bool _isHoming; //true: is performing homing action 
    AxisConfig _config;

    friend class Robot;
    
};

}//namespace