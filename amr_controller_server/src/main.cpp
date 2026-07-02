/**
 * @file main.cpp
 * @brief Entry point for the AMR controller server process.
 */

#include "amr_controller_server/controller_server.hpp"

/// @brief Initialize ROS, spin the combined controller server, and shut ROS down.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  amr::controller::server::ControllerServer controller_server;
  controller_server.spin();

  rclcpp::shutdown();
  return 0;
}
