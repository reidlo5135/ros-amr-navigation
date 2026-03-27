#include "amr_localization/localization.hpp"

#include <csignal>

namespace
{

rclcpp::executors::SingleThreadedExecutor * g_executor = nullptr;

void handle_signal(int)
{
  if (g_executor != nullptr) {
    g_executor->cancel();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_localization::Localization>();
  rclcpp::executors::SingleThreadedExecutor executor;
  g_executor = &executor;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  g_executor = nullptr;
  rclcpp::shutdown();
  return 0;
}
