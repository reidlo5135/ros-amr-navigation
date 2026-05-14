#ifndef AMR_BRINGUP__ROBOT_BRINGUP_HPP_
#define AMR_BRINGUP__ROBOT_BRINGUP_HPP_

#include <cstdint>
#include <string>

namespace amr::bringup
{

struct TopicNames
{
  std::string cmd_vel;
  std::string odom;
  std::string imu;
  std::string scan;
  std::string joint_states;
  std::string tf;
  std::string tf_static;
  std::string robot_description;
};

struct FrameNames
{
  std::string map;
  std::string odom;
  std::string base_footprint;
  std::string base_link;
  std::string base_scan;
  std::string imu_link;
};

struct BaseDriverProfile
{
  bool enabled;
  std::string port;
  std::int32_t baudrate;
  bool publish_tf;
  double odom_rate_hz;
  double imu_rate_hz;
  double joint_state_rate_hz;
  double cmd_vel_timeout_sec;
  double wheel_separation_m;
  double wheel_radius_m;
  double max_linear_velocity_mps;
  double max_angular_velocity_radps;
};

struct LidarDriverProfile
{
  bool enabled;
  std::string port;
  std::int32_t baudrate;
  std::string frame_id;
  std::string scan_topic;
  std::string sensor_model;
  bool inverted;
  double angle_min_rad;
  double angle_max_rad;
  double range_min_m;
  double range_max_m;
  double scan_time_sec;
};

struct DescriptionProfile
{
  bool publish_robot_description;
  bool use_amr_description;
};

struct RobotBringupProfile
{
  std::string robot_model;
  bool robot_side_only;
  TopicNames topics;
  FrameNames frames;
  BaseDriverProfile base_driver;
  LidarDriverProfile lidar_driver;
  DescriptionProfile description;
};

class RobotBringupSchema
{
public:
  static const char * robot_bringup_node_name();
  static const char * base_driver_node_name();
  static const char * lidar_driver_node_name();
  static const char * description_node_name();

  static const char * robot_bringup_namespace();
  static const char * base_driver_namespace();
  static const char * lidar_driver_namespace();
  static const char * description_namespace();

  static RobotBringupProfile make_default_turtlebot3_burger_profile();
};

}  // namespace amr::bringup

#endif  // AMR_BRINGUP__ROBOT_BRINGUP_HPP_
