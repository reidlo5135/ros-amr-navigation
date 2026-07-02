/**
 * @file main.cpp
 * @brief Entry point for the AMR behavior-tree navigator node.
 */

#include "amr_bt_navigator/bt_navigator.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::bt::navigator::Btnavigator> node = std::make_shared<amr::bt::navigator::Btnavigator>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
