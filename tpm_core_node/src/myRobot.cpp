#include "myRobot.h"
#include "myAxis.h"
#include "def_macro.h"
#include "hwLib.hpp"
#include "global_instance.hpp"
#include <thread>
#include <math.h>

#include "tpm_msgs/srv/axis_operation.hpp"

namespace tpm_core {

class Robot::Private
{
public:    
    static short servo      (Robot& self, signed char axisId, bool isOn)
    {    
        if (axisId==-1) 
            for(int i=0; i<self.axisNum; i++)
                TRY(self.axes[i]->servo(isOn));
        
        else 
            TRY(self.axes[axisId]->servo(isOn));
        
        return 0;
    }
    static short home       (Robot& self, signed char axisId)
    {    
        if(axisId >= 0)
            TRY(self.axes[axisId]->start_homing());

        else{
            //todo
        }
        
        return 0;
    }
    
    static short clear_alm  (Robot& self, signed char axisId)
    {
        if (axisId==-1) 
            for(int i=0; i<self.axisNum; i++) 
                TRY(self.axes[i]->clear_alarm());
        else 
            TRY(self.axes[axisId]->clear_alarm());
        
        return 0;    
    }
    static short search_org (Robot& self, signed char axisId)
    {
        if (axisId==-1) 
            for(int i=0; i<self.axisNum; i++) 
                TRY(self.axes[i]->search_org());
        else 
            TRY(self.axes[axisId]->search_org());
        
        return 0; 
    }
    static short set_as_offset(Robot& self, signed char axisId)
    {
        if (axisId==-1) 
            for(int i=0; i<self.axisNum; i++) 
                TRY(self.axes[i]->set_as_offset());
        else 
            TRY(self.axes[axisId]->set_as_offset());
        
        return 0;    
    }
    static short set_as_zero(Robot& self, signed char axisId)
    {
        if (axisId==-1) 
            for(int i=0; i<self.axisNum; i++) 
                TRY(self.axes[i]->set_as_zero());
        else 
            TRY(self.axes[axisId]->set_as_zero());
        
        return 0; 
    }
    static short mv_to_zero (Robot& self, signed char axisId)
    {
        if(axisId >= 0)
            TRY(self.axes[axisId]->mv_to_zero());

        else{
            ROS_PRINT("Axis all mv_to_zero");
            MCL_MPDATA mp = MCL_MPDATA();
            double maxV = 20;
            mp.Feed = maxV * Axis::vel_ratio; //unit:deg
            mp.Accel = mp.Feed * 10;
            mp.Decel = mp.Feed * 10;
            mp.Coord = MCL_COORD_ABS;

            FLT pos[MAX_AXIS_PER_ROBOT] = {0};
            UINT8 mask = 0xff;

            ROS_PRINT("mp.Feed= %f, maxV=%f, ratio=%f", mp.Feed, maxV, Axis::vel_ratio);
            TRY(HwLib::Instance().move_p2p_axis(0, mp, pos, mask));            
        }
        return 0;
    } 
    static short jog        (Robot& self, signed char axisId, bool isPos)
    {
        if (axisId==-1) {
            ROS_PRINT("ERROR: can't do jogging for all axis.");
            return -1;
        }
        else 
            TRY(self.axes[axisId]->jog(isPos));
        
        return 0; 
    }
    static short stop       (Robot& self, signed char axisId)
    {
        ROS_PRINT("Axis All stop");

        if(self.is_homing())
            for(int i=0; i<self.axisNum; i++) 
                HwLib::Instance().mnet_m1a_sd_stop(i);

        else  
            HwLib::Instance().stop(0);//0:smooth
        return 0;
    }  
};

Robot::~Robot(){
    clear_axes();
}
void Robot::create_axes(std::vector<unsigned char> slaveIps)
{
    clear_axes();
    for(int i= 0; i< (int)slaveIps.size(); i++)
    {
        Axis* ax = new Axis();
        ax->init(slaveIps[i], i);
        axes.push_back(ax);
    }
    axisNum = (int)axes.size();
}
void Robot::clear_axes()
{
    for(int i= 0; i< (int)axes.size(); i++)
        delete axes[i];
    
    axes.clear();
    axisNum = 0;
}

void Robot::get_pulse_per_deg(float* out_array, int len)
{
    for(int i=0; i<std::min(len,axisNum) ; i++)
        out_array[i] = (float)axes[i]->_config.pulse_per_deg;
}

short Robot::do_action(char funcType, signed char axisId)
{
    if(axisId < -1 || axisId >= axisNum){
        ROS_PRINT("ERROR: axisId=%d. should be [0~%d] or -1", axisId, axisNum-1);
        return -1;
    }

    switch(funcType)
    {
      case(tpm_msgs::srv::AxisOperation::Request::SERVO_ON):
        return Private::servo(*this, axisId, true);
      case(tpm_msgs::srv::AxisOperation::Request::SERVO_OFF):
        return Private::servo(*this, axisId, false);
      case(tpm_msgs::srv::AxisOperation::Request::HOME):
        return Private::home(*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::CLEAR_ALM):
        return Private::clear_alm(*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::JOG_POS):
        return Private::jog(*this, axisId, true);
      case(tpm_msgs::srv::AxisOperation::Request::JOG_NEG):
        return Private::jog(*this, axisId, false);
      case(tpm_msgs::srv::AxisOperation::Request::STOP):
        return Private::stop(*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::SET_AS_ZERO):
        return Private::set_as_offset(*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::SET_AS_OFFSET):
        return Private::set_as_zero (*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::MV_TO_ZERO):
        return Private::mv_to_zero  (*this, axisId);
      case(tpm_msgs::srv::AxisOperation::Request::SEARCH_ORG):
        return Private::search_org  (*this, axisId);

      default:
        ROS_PRINT("ERROR: funcType=%d is not implemented", funcType);
        return -1;
    }
    return 0;
}

//==== static function ====
short Robot::set_vel_ratio(double ratio)
{
    Axis::vel_ratio = ratio;
    TRY(HwLib::Instance().feedrate(ratio));
    return 0;
}
short Robot::set_jog_dist(double value)
{
    Axis::jog_dist = value;
    return 0;
}
double Robot::get_vel_ratio()
{
    return Axis::vel_ratio;
}
}//namespace