#include "amr_bringup/robot_bringup.hpp"

#include <stdexcept>

namespace amr::bringup
{

const char * RobotBringupSchema::robot_bringup_node_name()
{
  return "robot_bringup";
}

const char * RobotBringupSchema::base_driver_node_name()
{
  return "base_driver";
}

const char * RobotBringupSchema::lidar_driver_node_name()
{
  return "lidar_driver";
}

const char * RobotBringupSchema::description_node_name()
{
  return "description";
}

const char * RobotBringupSchema::robot_bringup_namespace()
{
  return "/amr/robot_bringup";
}

const char * RobotBringupSchema::base_driver_namespace()
{
  return "/amr/base_driver";
}

const char * RobotBringupSchema::lidar_driver_namespace()
{
  return "/amr/lidar_driver";
}

const char * RobotBringupSchema::description_namespace()
{
  return "/amr/description";
}

bool RobotBringupSchema::is_supported_robot(
  const std::string & robot_type,
  const std::string & robot_model)
{
  return robot_type == "turtlebot3" && robot_model == "burger";
}

std::string RobotBringupSchema::make_robot_key(
  const std::string & robot_type,
  const std::string & robot_model)
{
  if (robot_type.empty()) {
    return robot_model;
  }
  if (robot_model.empty()) {
    return robot_type;
  }
  return robot_type + ":" + robot_model;
}

std::string RobotBringupSchema::default_urdf_path(
  const std::string & robot_type,
  const std::string & robot_model)
{
  if (robot_type == "turtlebot3" && robot_model == "burger") {
    return "urdf/turtlebot3_burger.urdf.xacro";
  }

  throw std::invalid_argument(
          "No AMR-owned URDF is registered for robot " +
          make_robot_key(robot_type, robot_model));
}

RobotBringupProfile RobotBringupSchema::make_default_turtlebot3_burger_profile()
{
  RobotBringupProfile profile;
  profile.robot_type = "turtlebot3";
  profile.robot_model = "burger";
  profile.robot_side_only = true;

  profile.topics.cmd_vel = "/cmd_vel";
  profile.topics.odom = "/odom";
  profile.topics.imu = "/imu";
  profile.topics.scan = "/scan";
  profile.topics.joint_states = "/joint_states";
  profile.topics.tf = "/tf";
  profile.topics.tf_static = "/tf_static";
  profile.topics.robot_description = "/robot_description";

  profile.frames.map = "map";
  profile.frames.odom = "odom";
  profile.frames.base_footprint = "base_footprint";
  profile.frames.base_link = "base_link";
  profile.frames.base_scan = "base_scan";
  profile.frames.imu_link = "imu_link";

  profile.base_driver.enabled = true;
  profile.base_driver.port = "/dev/ttyACM0";
  profile.base_driver.baudrate = 115200;
  profile.base_driver.publish_tf = true;
  profile.base_driver.odom_rate_hz = 30.0;
  profile.base_driver.imu_rate_hz = 30.0;
  profile.base_driver.joint_state_rate_hz = 30.0;
  profile.base_driver.cmd_vel_timeout_sec = 0.5;
  profile.base_driver.wheel_separation_m = 0.160;
  profile.base_driver.wheel_radius_m = 0.033;
  profile.base_driver.max_linear_velocity_mps = 0.22;
  profile.base_driver.max_angular_velocity_radps = 2.84;

  profile.lidar_driver.enabled = true;
  profile.lidar_driver.port = "/dev/ttyUSB0";
  profile.lidar_driver.baudrate = 230400;
  profile.lidar_driver.frame_id = profile.frames.base_scan;
  profile.lidar_driver.scan_topic = profile.topics.scan;
  profile.lidar_driver.sensor_model = "auto";
  profile.lidar_driver.inverted = false;
  profile.lidar_driver.angle_min_rad = 0.0;
  profile.lidar_driver.angle_max_rad = 6.28318530717958647692;
  profile.lidar_driver.range_min_m = 0.12;
  profile.lidar_driver.range_max_m = 3.5;
  profile.lidar_driver.scan_time_sec = 0.1;

  profile.description.publish_robot_description = true;
  profile.description.urdf_path = default_urdf_path(profile.robot_type, "burger");

  return profile;
}

}  // namespace amr::bringup
