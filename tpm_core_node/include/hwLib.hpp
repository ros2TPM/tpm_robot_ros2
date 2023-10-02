#pragma once

#include "def_type.h"
#include <vector>
#include <stdio.h>
#include <string>

#ifdef ROB_REAL
  #include "RPiRobIF.h"
#else
  #include "RobC.h"
  #include "def_RIDT.h"
#endif

template<typename T>
T& get_global() {
  static T instance;
  return instance;
}

namespace tpm_core
{
  class HwLib 
  {
  protected:
    // hide constructor, copy constructor, and assignment operator
    HwLib(){};

    // forward declare a inner class for private functions.
    class Private;   
  public:
    
    static HwLib& Instance();
    ~HwLib(){};// This will be automatically called when the program exits
    HwLib(const HwLib&)=delete;
    HwLib& operator=(const HwLib&)=delete;

    // ==== RI =====
    
    virtual short connect() = 0;
    virtual short disconnect() = 0;
    virtual ST_RIDT_t* ri_get_RIDT() = 0;

    virtual short ri_enable_DDAMode(bool enable) {return 0;}
    std::vector<UINT8> get_vec_axisIP(){return vec_axisIP;}

    //===== ROBC =======
    short init();   
    virtual short dda_cycle() {return 0;}
    virtual short stop(UINT8 stopType) {return 0;}
    virtual short set_axis_position(U8 AxisId, FLT value) {return 0;}
    virtual short jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc) {return 0;}
    virtual short jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame){return 0;}
    virtual short move_p2p_axis(UINT16 cmdId, MCL_MPDATA& mpData, FLT* axis, UINT8 mask) {return 0;}
    virtual short move_lin_pose(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pose, UINT8 mask) {return 0;}
    virtual short get_axis(FLT* values) {return 0;}
    virtual short get_pose(FLT* values) {return 0;}
    virtual short get_buffer_depth(U32* buffDepth) {return 0;}
    virtual std::string get_last_err_msg() {return "";}

    virtual short hold  () {return 0;}
    virtual short resume() {return 0;}
    virtual short feedrate(double feedrate) {return 0;}

    virtual short set_pvt_data(U8 AxisId, U32 PointNum, MCL_PVT_POINT* PvtPoints) {return 0;}
    virtual short move_pvt(FLT StopDec, U8 Mask) {return 0;}

    //===== Mnet ====
    virtual int mnet_m1a_set_svon   (UINT16 ip, UINT16 on_off) {return 0;}
    virtual int mnet_m1a_set_ralm   (UINT16 ip, UINT16 on_off) {return 0;}
    virtual int mnet_m1a_home_search(UINT16 ip, UINT8 Dir, UINT32 StrVel, UINT32 MaxVel, float Tacc, INT32 ORGOffset) {return 0;}
    virtual int mnet_m1a_motion_done(UINT16 ip, UINT16 *MoSt) {return 0;}
    virtual int mnet_m1a_reset_all  (UINT16 ip) {return 0;}
    virtual int mnet_m1a_sd_stop    (UINT16 ip) {return 0;}

  protected:
    virtual short init_inner(ROB_KIN_TYPE type, FLT* a, FLT* alpha, FLT* d, FLT* theta, FLT* thetaShift, FLT* posLimit, FLT* negLimit, FLT* pulsePerDeg) {return 0;}
    std::vector<UINT8> vec_axisIP;
    ST_RIDT_t* RIDT = nullptr;
  };

  class HwLib_Real : public HwLib
  {
  public:
    // ==== RI ====
    short connect() override;
    short disconnect() override;
    short ri_enable_DDAMode(bool enable) override;
    ST_RIDT_t* ri_get_RIDT() override;

    // ===== Robc =====
    short stop(UINT8 stopType) override;
    short set_axis_position(U8 AxisId, FLT value) override;
    short jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc) override;
    short jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame) override;
    short move_p2p_axis(UINT16 cmdId, MCL_MPDATA& mpData, FLT* axis, UINT8 mask) override;
    short move_lin_pose(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pose, UINT8 mask) override;
    short get_axis(FLT* values) override;
    short get_buffer_depth(U32* buffDepth) override;
    std::string get_last_err_msg() override;

    short hold  () override;
    short resume() override;
    short feedrate(double feedrate) override;

    //===== Mnet =====
    int mnet_m1a_set_svon   (UINT16 ip, UINT16 on_off) override;
    int mnet_m1a_set_ralm   (UINT16 ip, UINT16 on_off) override;
    int mnet_m1a_home_search(UINT16 ip, UINT8 Dir, UINT32 StrVel, UINT32 MaxVel, float Tacc, INT32 ORGOffset) override;
    int mnet_m1a_reset_all  (UINT16 ip) override;
    int mnet_m1a_motion_done(UINT16 ip, UINT16 *MoSt) override;
    int mnet_m1a_sd_stop    (UINT16 ip) override;
  protected:
    short init_inner(ROB_KIN_TYPE type, FLT* a, FLT* alpha, FLT* d, FLT* theta, FLT* thetaShift, FLT* posLimit, FLT* negLimit, FLT* pulsePerDeg) override;
    
  private:
    short init_axis(UINT16 ip);
  };

  class HwLib_Sim : public HwLib
  {
    friend class HwLib; //for singleton base class to access the constructor.
  private:
    HwLib_Sim();
  public:
    ~HwLib_Sim(){};// This will be automatically called when the program exits
    HwLib_Sim(const HwLib_Sim&)=delete;
    HwLib_Sim& operator=(const HwLib_Sim&)=delete;

    // ==== RI ====
    short connect() override;
    short disconnect() override;
    ST_RIDT_t* ri_get_RIDT() override {return RIDT;}

    // ==== ROBC ====
    short dda_cycle() override;
    short stop(UINT8 stopType) override;
    short set_axis_position(U8 AxisId, FLT value) override;
    short jog_axis(U8 AxisId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc) override;
    short jog_pose(U8 PoseId, MCL_DIR_TYPE dir, FLT Dist, FLT Vel, FLT Acc, ROB_FRAME_TYPE frame) override;
    short move_p2p_axis(UINT16 cmdId, MCL_MPDATA& mpData, FLT* axis, UINT8 mask) override;
    short move_lin_pose(UINT16 cmdId, MCL_MPDATA& mpData, FLT* pose, UINT8 mask) override;
    short get_axis(FLT* values) override;
    short get_pose(FLT* values) override;
    short get_buffer_depth(U32* buffDepth) override;
    std::string get_last_err_msg() override;

    short hold  () override;
    short resume() override;
    short feedrate(double feedrate) override;

    short set_pvt_data(U8 AxisId, U32 PointNum, MCL_PVT_POINT* PvtPoints) override;
    short move_pvt(FLT StopDec, U8 Mask) override;

  protected:
    short init_inner(ROB_KIN_TYPE type, FLT* a, FLT* alpha, FLT* d, FLT* theta, FLT* thetaShift, FLT* posLimit, FLT* negLimit, FLT* pulsePerDeg) override;
  private:
    FLT axis_[6];
    void DDA_Cycle();
    
  };
}
