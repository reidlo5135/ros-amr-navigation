#include "amr_tb3_base_driver/base_driver_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr::tb3::base_driver::BaseDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
