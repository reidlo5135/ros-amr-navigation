/**
 * @file main.cpp
 * @brief Entry point for the AMR spatial segmenter node.
 */

#include "amr_spatial_segmenter/spatial_segmenter.hpp"

/// @brief Initialize ROS, spin the spatial segmenter lifecycle node, and shut ROS down.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr::spatial_segmenter::SpatialSegmenter>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
