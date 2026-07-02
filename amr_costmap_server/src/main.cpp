/**
 * @file main.cpp
 * @brief Entry point for the AMR costmap server node.
 */

#include "amr_costmap_server/costmap_server.hpp"

/// @brief Initialize ROS, spin the costmap server node, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::costmap::server::CostmapServer> node = std::make_shared<amr::costmap::server::CostmapServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
