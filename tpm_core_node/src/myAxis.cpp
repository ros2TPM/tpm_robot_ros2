#include "myAxis.h"
#include "global_instance.hpp"
#include "global_config.hpp"
#include "hwLib.hpp"
#include "def_macro.h"
#include <thread>

namespace tpm_core {

double Axis::jog_dist = -1; //unit:deg. -1 means infinite.
double Axis::vel_ratio = 0.5;//value: [0~1] means [0~100%]

void Axis::init(unsigned short slvIp, int id)
{
    _slvIp = slvIp;
    _id = id;
    _config.home_dir        = Config::home_dir[id];
    _config.home_offsets    = Config::home_offsets[id];
    _config.max_jog_vel     = Config::max_jog_speed[id];
    _config.pulse_per_deg   = Config::pulse_per_deg[id];
}
short Axis::servo(bool isOn)
{
    ROS_PRINT("Axis %d servo. isOn=%d", _id, isOn);
    TRY(HwLib::Instance().mnet_m1a_set_svon(_id, isOn));
    return 0;
}
short Axis::clear_alarm()
{
    ROS_PRINT("Axis %d clear_alarm", _id);
    TRY(HwLib::Instance().mnet_m1a_set_ralm(_id, true));
    return 0;
}

short Axis::start_homing()
{
    TRY(search_org());

    auto& hwLib = HwLib::Instance();
    // wait for search done.
    while(1) {
        ushort motionSts = 0;
        TRY(hwLib.mnet_m1a_motion_done(_slvIp, &motionSts));
        
        if (motionSts == 0){ //0: stopped. 
            ROS_PRINT("Axis %d search org done!", _id);
            hwLib.mnet_m1a_reset_all(_slvIp);
            hwLib.ri_enable_DDAMode(true);
            _isHoming = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    TRY(set_as_offset());
    TRY(mv_to_zero());

    //wait for mv_to_zero done.
    while(1)
    {
        INT32 buffDepth;
        hwLib.get_buffer_depth(&buffDepth);
        if(buffDepth == 0)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
short Axis::search_org()
{
    UINT8 dir     =_config.home_dir;
    UINT32 maxVel =_config.max_jog_vel 
                * _config.pulse_per_deg
                * vel_ratio
                * 0.2 ;

    const UINT32 strVel=0;
    const float Tacc=0.5;//sec
    const INT32 escapeORGOffset = 5000;//pulse
      
    ROS_PRINT("Axis %d search_org. dir=%d, maxV(Pulse)=%d", _id, dir, maxVel);
    TRY(HwLib::Instance().ri_enable_DDAMode(false));
    TRY(HwLib::Instance().mnet_m1a_home_search(
        _slvIp, dir, strVel, maxVel, Tacc,
        escapeORGOffset));

    _isHoming = true;
    return 0;
}
short Axis::set_as_offset()
{
    ROS_PRINT("Axis %d set_as_offset", _id);
    TRY(HwLib::Instance().set_axis_position(_id, _config.home_offsets));
    return 0;
}
short Axis::set_as_zero()
{
    ROS_PRINT("Axis %d set_as_zero", _id);
    TRY(HwLib::Instance().set_axis_position(_id, 0));
    return 0;
}
short Axis::mv_to_zero()
{
    ROS_PRINT("Axis %d mv_to_zero", _id);
    MCL_MPDATA mp = MCL_MPDATA();
    mp.Feed = _config.max_jog_vel * vel_ratio; //unit:deg
    mp.Accel = mp.Feed * 10;
    mp.Decel = mp.Feed * 10;
    mp.Coord = MCL_COORD_ABS;

    FLT pos[MAX_AXIS_PER_ROBOT] = {0};
    UINT8 mask = (UINT8)(1 << _id);

    ROS_PRINT("mp.Feed= %f, maxV=%f, ratio=%f", mp.Feed, _config.max_jog_vel, vel_ratio);

    TRY(HwLib::Instance().move_p2p_axis(0, mp, pos, mask));
    return 0;
}
short Axis::jog(bool isPos)
{
    ROS_PRINT("Axis %d jog. isPos=%d", _id, isPos);
    /*
    MCL_MPDATA mp = MCL_MPDATA();
    mp.Feed = _config.max_jog_vel * Axis::vel_ratio;//unit:deg
    mp.Accel = mp.Feed * 10;
    mp.Decel = mp.Feed * 10;
    mp.Coord = MCL_COORD_REL;

    UINT8 mask = (UINT8)(1 << _id);
    FLT pos[MAX_AXIS_PER_ROBOT] = {0};
    
    if(!isPos)
        pos[_id] *= -1;
    */
    MCL_DIR_TYPE dir = isPos? MCL_DIR_POS : MCL_DIR_NEG;
    FLT Dist = Axis::jog_dist==-1? 0: Axis::jog_dist; 
    FLT Vel = _config.max_jog_vel * Axis::vel_ratio;//unit:deg
    FLT Acc = Vel*10;

    //ROS_PRINT("pos=%f, feed=%f", pos[_id], mp.Feed);
    //TRY(HwLib::Instance().move_p2p_axis(0, mp, pos, mask));
    TRY(HwLib::Instance().jog_axis(_id, dir, Dist, Vel, Acc));
    return 0;
}

}//namespace