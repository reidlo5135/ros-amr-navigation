/**
 * @file main.cpp
 * @brief Entry point for the AMR recovery server node.
 */

#include "amr_recovery_server/recovery_server.hpp"

/// @brief Initialize ROS, spin the recovery server node, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::recovery::server::RecoveryServer> node = std::make_shared<amr::recovery::server::RecoveryServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
