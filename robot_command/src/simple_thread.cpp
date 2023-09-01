#include "rclcpp/rclcpp.hpp"
#include "MyNode.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
