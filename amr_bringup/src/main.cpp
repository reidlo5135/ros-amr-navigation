#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "amr_bringup/base/base_driver_node.hpp"
#include "amr_bringup/lidar/lidar_driver_node.hpp"
#include "amr_bringup/robot_bringup.hpp"

namespace
{

std::shared_ptr<rclcpp::Node> create_hardware_node(
  const rclcpp::NodeOptions & selector_options,
  const rclcpp::NodeOptions & hardware_options)
{
  auto selector = std::make_shared<rclcpp::Node>(
    amr::bringup::RobotBringupSchema::robot_bringup_node_name(),
    "/amr",
    selector_options);

  std::string component;
  std::string robot_type{"turtlebot3"};
  std::string robot_model{"burger"};

  selector->get_parameter_or("bringup.component", component, std::string{});
  selector->get_parameter_or("robot.type", robot_type, std::string{"turtlebot3"});
  selector->get_parameter_or("robot.model", robot_model, std::string{"burger"});

  if (component.empty()) {
    RCLCPP_ERROR(
      selector->get_logger(),
      "Missing required parameter 'bringup.component'. Expected 'base_driver' or 'lidar_driver'.");
    throw std::invalid_argument("bringup.component is required");
  }

  if (!amr::bringup::RobotBringupSchema::is_supported_robot(robot_type, robot_model)) {
    RCLCPP_ERROR(
      selector->get_logger(),
      "Unsupported robot selection '%s'. No AMR-owned hardware implementation is registered.",
      amr::bringup::RobotBringupSchema::make_robot_key(robot_type, robot_model).c_str());
    throw std::invalid_argument("unsupported robot.type / robot.model");
  }

  if (component == "base_driver") {
    RCLCPP_INFO(
      selector->get_logger(),
      "Starting base_driver for robot '%s'",
      amr::bringup::RobotBringupSchema::make_robot_key(robot_type, robot_model).c_str());
    return std::make_shared<amr::tb3::base_driver::BaseDriverNode>(hardware_options);
  }

  if (component == "lidar_driver") {
    RCLCPP_INFO(
      selector->get_logger(),
      "Starting lidar_driver for robot '%s'",
      amr::bringup::RobotBringupSchema::make_robot_key(robot_type, robot_model).c_str());
    return std::make_shared<amr::tb3::lidar_driver::LidarDriverNode>(hardware_options);
  }

  RCLCPP_ERROR(
    selector->get_logger(),
    "Unsupported bringup.component '%s'. Expected 'base_driver' or 'lidar_driver'.",
    component.c_str());
  throw std::invalid_argument("unsupported bringup.component");
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions selector_options;
    selector_options.automatically_declare_parameters_from_overrides(true);

    rclcpp::NodeOptions hardware_options;
    auto node = create_hardware_node(selector_options, hardware_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("amr_bringup_hardware"),
      "Failed to start AMR hardware bringup entrypoint: %s",
      error.what());
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("amr_bringup_hardware"),
      "Failed to start AMR hardware bringup entrypoint with unknown error");
  }

  rclcpp::shutdown();
  return 1;
}
