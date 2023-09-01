#pragma once

#include <vector>


namespace tpm_core{
    
class Config
{
public:
    static int alm_logic;   
    static int org_logic;   
    static int feedback_src;
    static int home_mode;
    static std::vector<double> home_offsets;
    static std::vector<long int> home_dir;
    static std::vector<double> max_jog_speed;
    static std::vector<double> pulse_per_deg;

    static void declare_parameters();
    static void load_parameters();
};

}//end namespace