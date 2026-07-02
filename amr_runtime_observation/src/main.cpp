/**
 * @file main.cpp
 * @brief Entry point for the AMR runtime observation node.
 */

#include "amr_runtime_observation/runtime_observation.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<amr::runtime::observation::RuntimeObservation>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
