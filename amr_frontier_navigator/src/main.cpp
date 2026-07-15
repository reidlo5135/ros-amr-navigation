/**
 * @file main.cpp
 * @brief Entry point for the AMR frontier navigator node.
 */

#include "amr_frontier_navigator/frontier_navigator.hpp"

/// @brief Initialize ROS, spin the frontier navigator lifecycle node, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr::frontier_navigation::FrontierNavigator>();
  // Lifecycle callbacks may join the bounded execution worker while action cancel/result
  // responses still need executor progress.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
