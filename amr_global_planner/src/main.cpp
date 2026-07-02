/**
 * @file main.cpp
 * @brief Entry point for the AMR global planner node.
 */

#include "amr_global_planner/planner_server.hpp"

/// @brief Initialize ROS, spin the global planner node, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::planner::global::PlannerServer> node = std::make_shared<amr::planner::global::PlannerServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
