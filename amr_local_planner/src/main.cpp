#include <memory>

#include "amr_local_planner/local_planner.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_local_planner::LocalPlanner>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
