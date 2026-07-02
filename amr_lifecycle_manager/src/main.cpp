/**
 * @file main.cpp
 * @brief Entry point for the AMR lifecycle manager node.
 */

#include "amr_lifecycle_manager/lifecycle_manager.hpp"

/// @brief Initialize ROS, spin the lifecycle manager node, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::lifecycle::manager::LifecycleManager> node = std::make_shared<amr::lifecycle::manager::LifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
