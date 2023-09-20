#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace tpm_core
{
  class Manager_Param
  {
  public:
    Manager_Param(rclcpp::Node::SharedPtr node);
    virtual ~Manager_Param() = default;
    
  };
}