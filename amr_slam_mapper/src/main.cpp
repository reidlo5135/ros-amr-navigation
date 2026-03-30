#include "amr_slam_mapper/slam_mapper.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::slam::mapper::SlamMapper> node =
    std::make_shared<amr::slam::mapper::SlamMapper>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
