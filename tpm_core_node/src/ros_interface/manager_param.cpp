#include "manager_param.hpp"
#include "global_config.hpp"
#include "def_macro.h"

namespace tpm_core
{
Manager_Param::Manager_Param(rclcpp::Node::SharedPtr node)
{
    ROS_PRINT("=== declare_parameters =====");
      std::vector<double> zeros = {0.0,0.0,0.0,0.0,0.0,0.0};
      std::vector<int> zeros_i = {0, 0, 0, 0, 0, 0};

      node->declare_parameter("jog_dist", -1);

      node->declare_parameter("alm_logic", 0);
      node->declare_parameter("org_logic", 0);
      node->declare_parameter("feedback_src", 0);
      node->declare_parameter("home_mode", 0);
      node->declare_parameter("home_offsets", zeros);
      node->declare_parameter("home_dir", zeros_i);
      node->declare_parameter("max_jog_speed", zeros);
      node->declare_parameter("pulse_per_deg", zeros);

      ROS_PRINT("===== load_parameters ====" );
        Config::alm_logic       = node->get_parameter("alm_logic").as_int();
        Config::org_logic       = node->get_parameter("org_logic").as_int();
        Config::feedback_src    = node->get_parameter("feedback_src").as_int();
        Config::home_mode       = node->get_parameter("home_mode").as_int();
        node->get_parameter("home_offsets", Config::home_offsets);
        node->get_parameter("home_dir", Config::home_dir);
        node->get_parameter("max_jog_speed", Config::max_jog_speed);
        node->get_parameter("pulse_per_deg", Config::pulse_per_deg);

        ROS_PRINT("alm_logic = %d ", Config::alm_logic);
        ROS_PRINT("org_logic = %d ", Config::org_logic);
        ROS_PRINT("feedback_src = %d", Config::feedback_src);
        ROS_PRINT("home_mode = %d", Config::home_mode);

        ROS_PRINT("home_offsets: ");
        for(size_t i = 0; i < Config::home_offsets.size(); i++)
            ROS_PRINT("%2f,", Config::home_offsets[i]);

        ROS_PRINT("max_jog_speed: ");
        for(size_t i = 0; i < Config::max_jog_speed.size(); i++)
            ROS_PRINT("%2f,", Config::max_jog_speed[i]);


    node->declare_parameter("robot_type", 40);
    node->declare_parameter("a", zeros);
    node->declare_parameter("alpha", zeros);
    node->declare_parameter("d", zeros);
    node->declare_parameter("theta", zeros);
    node->declare_parameter("theta_shift", zeros);
    node->declare_parameter("pos_limit", zeros);
    node->declare_parameter("neg_limit", zeros);
    node->declare_parameter("pulse_per_unit", zeros);

    node->get_parameter("robot_type", RobotSpec::robot_type);
    node->get_parameter("a", RobotSpec::a);
    node->get_parameter("alpha", RobotSpec::alpha);
    node->get_parameter("d", RobotSpec::d);
    node->get_parameter("theta", RobotSpec::theta);
    node->get_parameter("theta_shift", RobotSpec::theta_shift);
    node->get_parameter("pos_limit", RobotSpec::pos_limit);
    node->get_parameter("neg_limit", RobotSpec::neg_limit);
    node->get_parameter("pulse_per_unit", RobotSpec::pulse_per_unit);
}

}