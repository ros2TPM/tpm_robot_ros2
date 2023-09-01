#include "global_config.hpp"
#include "global_instance.hpp"
#include "def_macro.h"

namespace tpm_core{

int Config::alm_logic;   
int Config::org_logic;   
int Config::feedback_src;
int Config::home_mode;
std::vector<double> Config::home_offsets;
std::vector<long int> Config::home_dir;
std::vector<double> Config::max_jog_speed;
std::vector<double> Config::pulse_per_deg;


void Config::declare_parameters()
{
  

  // monitor parameter, 要從其他node改parameter需要研究一下
  // must keep handle, or is doesn't work
  //param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(Global::coreNode);
  //cb_handle_ = param_subscriber_->add_parameter_callback("home_offsets", handle_home_offset_changed);
}
void Config::load_parameters()
{
    

}

}//end namespace