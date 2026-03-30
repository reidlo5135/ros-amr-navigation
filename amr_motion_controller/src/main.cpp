#include "amr_motion_controller/motion_controller.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<amr::motion::controller::MotionController> node =
    std::make_shared<amr::motion::controller::MotionController>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
