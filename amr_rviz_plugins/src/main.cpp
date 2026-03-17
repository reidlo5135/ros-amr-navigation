#include "amr_rviz_plugins/goal_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_rviz_plugins::GoalBridge>());
  rclcpp::shutdown();
  return 0;
}
