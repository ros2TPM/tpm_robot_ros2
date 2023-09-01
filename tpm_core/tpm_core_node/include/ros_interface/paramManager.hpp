#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace tpm_core
{
  class prmManager
  {
  public:
    prmManager(rclcpp::Node::SharedPtr node);
    virtual ~prmManager() = default;
    
  };
}