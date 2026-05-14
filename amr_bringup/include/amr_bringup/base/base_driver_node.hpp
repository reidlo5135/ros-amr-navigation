#ifndef AMR_TB3_BASE_DRIVER__BASE_DRIVER_NODE_HPP_
#define AMR_TB3_BASE_DRIVER__BASE_DRIVER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "amr_bringup/base/base_types.hpp"
#include "amr_bringup/base/differential_drive_odometry.hpp"
#include "amr_bringup/base/opencr_control_table.hpp"
#include "amr_bringup/base/opencr_sdk_wrapper.hpp"

namespace amr::tb3::base_driver
{

class BaseDriverNode : public rclcpp::Node
{
public:
  explicit BaseDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BaseDriverNode() override;

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();
  void log_configuration() const;

  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);
  void handle_watchdog();
  void handle_connection_check();
  void poll_feedback();
  void handle_feedback(const BaseFeedback & feedback);
  void handle_emergency_stop(const std_msgs::msg::Bool::SharedPtr message);

  void publish_odometry(const BaseFeedback & feedback);
  void publish_joint_states(const BaseFeedback & feedback);
  void publish_imu(const BaseFeedback & feedback);
  void publish_tf(const BaseFeedback & feedback);

  BaseCommand clamp_command(double linear_x_mps, double angular_z_radps) const;
  bool send_velocity_command(const BaseCommand & command);
  void send_stop_command();
  bool connect_opencr(bool calibrate_imu);
  std::optional<BaseFeedback> read_opencr_feedback();

  OpenCRSdkWrapper opencr_;
  BaseState base_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string port_;
  int baudrate_;
  std::uint8_t opencr_id_{200U};
  double opencr_protocol_version_{2.0};
  std::string base_frame_;
  std::string body_frame_;
  std::string odom_frame_;
  std::string imu_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string joint_states_topic_;
  std::string emergency_stop_topic_;
  bool publish_tf_;
  double cmd_vel_timeout_sec_;
  double wheel_separation_m_;
  double wheel_radius_m_;
  double max_linear_velocity_mps_;
  double max_angular_velocity_radps_;
  double feedback_rate_hz_{20.0};
  double heartbeat_rate_hz_{10.0};
  double startup_calibration_wait_sec_{5.0};
  bool startup_calibrate_imu_{true};
  bool fake_feedback_mode_{false};
  bool emergency_stop_active_{false};

  rclcpp::Time last_cmd_vel_stamp_;
  bool has_cmd_vel_{false};
  bool stop_command_sent_{false};
  DifferentialDriveOdometry odometry_;
  std::uint8_t heartbeat_counter_{0U};
  bool first_feedback_read_logged_{false};
  bool raw_wheel_state_initialized_{false};
  std::int32_t last_left_position_ticks_{0};
  std::int32_t last_right_position_ticks_{0};
  double cumulative_left_position_rad_{0.0};
  double cumulative_right_position_rad_{0.0};
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_TB3_BASE_DRIVER__BASE_DRIVER_NODE_HPP_
