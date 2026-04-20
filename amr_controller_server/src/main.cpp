#include "amr_controller_server/controller_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  amr::controller::server::ControllerServer controller_server;
  controller_server.spin();

  rclcpp::shutdown();
  return 0;
}
