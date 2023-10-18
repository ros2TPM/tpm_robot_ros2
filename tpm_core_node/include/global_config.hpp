#pragma once

#include <vector>


namespace tpm_core{
    
class Config
{
public:
    static bool use_sim;
    static int alm_logic;   
    static int org_logic;   
    static int feedback_src;
    static int home_mode;
    static double max_xyz_jog_speed;
    static double max_abc_jog_speed;
    static std::vector<double> home_offsets;
    static std::vector<long int> home_dir;
    static std::vector<double> max_axes_jog_speed;

    static void declare_parameters();
    static void load_parameters();
};

class RobotSpec
{
public:
    static int robot_type;
    static std::vector<double> a;
    static std::vector<double> alpha;
    static std::vector<double> d;
    static std::vector<double> theta;
    static std::vector<double> theta_shift;
    static std::vector<double> pos_limit;
    static std::vector<double> neg_limit;
    static std::vector<double> pulse_per_unit;
};

}//end namespace