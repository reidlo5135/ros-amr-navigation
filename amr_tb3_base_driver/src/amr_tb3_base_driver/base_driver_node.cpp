#include "amr_tb3_base_driver/base_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace amr::tb3::base_driver
{

using namespace std::chrono_literals;

BaseDriverNode::BaseDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("tb3_base_driver", options),
  transport_("/dev/ttyACM0", 115200),
  base_state_(),
  port_("/dev/ttyACM0"),
  baudrate_(115200),
  base_frame_("base_footprint"),
  body_frame_("base_link"),
  odom_frame_("odom"),
  imu_frame_("imu_link"),
  cmd_vel_topic_("/cmd_vel"),
  odom_topic_("/odom"),
  imu_topic_("/imu"),
  joint_states_topic_("/joint_states"),
  publish_tf_(true),
  cmd_vel_timeout_sec_(0.5),
  wheel_separation_m_(0.160),
  wheel_radius_m_(0.033),
  max_linear_velocity_mps_(0.22),
  max_angular_velocity_radps_(2.84),
  last_cmd_vel_stamp_(0, 0, this->get_clock()->get_clock_type())
{
  this->declare_parameters();
  this->load_parameters();
  this->setup_interfaces();

  if (!this->transport_.open()) {
    RCLCPP_WARN(
      this->get_logger(),
      "TB3 base driver transport is not connected yet: %s",
      this->transport_.last_error().c_str());
  } else {
    this->base_state_.connected = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Opened OpenCR serial transport on %s @ %d",
      this->port_.c_str(),
      this->baudrate_);
  }
}

BaseDriverNode::~BaseDriverNode()
{
  this->send_stop_command();
}

void BaseDriverNode::declare_parameters()
{
  this->declare_parameter("serial.port", this->port_);
  this->declare_parameter("serial.baudrate", this->baudrate_);
  this->declare_parameter("frames.base_footprint", this->base_frame_);
  this->declare_parameter("frames.base_link", this->body_frame_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.imu", this->imu_frame_);
  this->declare_parameter("topics.cmd_vel", this->cmd_vel_topic_);
  this->declare_parameter("topics.odom", this->odom_topic_);
  this->declare_parameter("topics.imu", this->imu_topic_);
  this->declare_parameter("topics.joint_states", this->joint_states_topic_);
  this->declare_parameter("publish_tf", this->publish_tf_);
  this->declare_parameter("watchdog.cmd_vel_timeout_sec", this->cmd_vel_timeout_sec_);
  this->declare_parameter("kinematics.wheel_separation_m", this->wheel_separation_m_);
  this->declare_parameter("kinematics.wheel_radius_m", this->wheel_radius_m_);
  this->declare_parameter("kinematics.max_linear_velocity_mps", this->max_linear_velocity_mps_);
  this->declare_parameter(
    "kinematics.max_angular_velocity_radps", this->max_angular_velocity_radps_);
}

void BaseDriverNode::load_parameters()
{
  this->get_parameter("serial.port", this->port_);
  this->get_parameter("serial.baudrate", this->baudrate_);
  this->get_parameter("frames.base_footprint", this->base_frame_);
  this->get_parameter("frames.base_link", this->body_frame_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.imu", this->imu_frame_);
  this->get_parameter("topics.cmd_vel", this->cmd_vel_topic_);
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.imu", this->imu_topic_);
  this->get_parameter("topics.joint_states", this->joint_states_topic_);
  this->get_parameter("publish_tf", this->publish_tf_);
  this->get_parameter("watchdog.cmd_vel_timeout_sec", this->cmd_vel_timeout_sec_);
  this->get_parameter("kinematics.wheel_separation_m", this->wheel_separation_m_);
  this->get_parameter("kinematics.wheel_radius_m", this->wheel_radius_m_);
  this->get_parameter("kinematics.max_linear_velocity_mps", this->max_linear_velocity_mps_);
  this->get_parameter(
    "kinematics.max_angular_velocity_radps", this->max_angular_velocity_radps_);

  this->transport_.set_port(this->port_);
  this->transport_.set_baudrate(this->baudrate_);
}

void BaseDriverNode::setup_interfaces()
{
  this->cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    this->cmd_vel_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      this->handle_cmd_vel(message);
    });

  this->watchdog_timer_ = this->create_wall_timer(
    100ms,
    [this]() {
      this->handle_watchdog();
    });

  this->connection_timer_ = this->create_wall_timer(
    1s,
    [this]() {
      this->handle_connection_check();
    });
}

void BaseDriverNode::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }

  const BaseCommand command = this->clamp_command(message->linear.x, message->angular.z);
  this->last_cmd_vel_stamp_ = this->now();
  this->has_cmd_vel_ = true;
  this->stop_command_sent_ = false;

  (void)this->send_velocity_command(command);
}

void BaseDriverNode::handle_watchdog()
{
  if (!this->has_cmd_vel_) {
    return;
  }

  const rclcpp::Duration elapsed = this->now() - this->last_cmd_vel_stamp_;
  if (elapsed.seconds() < this->cmd_vel_timeout_sec_) {
    return;
  }

  if (!this->stop_command_sent_) {
    RCLCPP_WARN(
      this->get_logger(),
      "No /cmd_vel received for %.3f sec, sending stop command",
      this->cmd_vel_timeout_sec_);
    this->send_stop_command();
  }
}

void BaseDriverNode::handle_connection_check()
{
  if (this->transport_.is_open()) {
    if (!this->base_state_.connected) {
      this->base_state_.connected = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Reconnected OpenCR transport on %s @ %d",
        this->port_.c_str(),
        this->baudrate_);
    }
    return;
  }

  if (this->transport_.reconnect()) {
    this->base_state_.connected = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Connected OpenCR transport on %s @ %d",
      this->port_.c_str(),
      this->baudrate_);
    return;
  }

  this->base_state_.connected = false;
  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "OpenCR transport disconnected: %s",
    this->transport_.last_error().c_str());
}

BaseCommand BaseDriverNode::clamp_command(
  const double linear_x_mps,
  const double angular_z_radps) const
{
  BaseCommand command;
  command.type = CommandType::kVelocity;
  command.linear_x_mps = std::clamp(
    linear_x_mps,
    -this->max_linear_velocity_mps_,
    this->max_linear_velocity_mps_);
  command.angular_z_radps = std::clamp(
    angular_z_radps,
    -this->max_angular_velocity_radps_,
    this->max_angular_velocity_radps_);
  return command;
}

bool BaseDriverNode::send_velocity_command(const BaseCommand & command)
{
  const auto encoded_command = this->protocol_.encode_command(command);
  if (!encoded_command.has_value()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "OpenCR command path is not ready yet: %s",
      this->protocol_.last_error().c_str());
    return false;
  }

  if (!this->transport_.is_open()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "OpenCR transport is not connected; dropping velocity command");
    return false;
  }

  if (!this->transport_.write(*encoded_command, 50ms)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Failed to send velocity command: %s",
      this->transport_.last_error().c_str());
    return false;
  }

  this->base_state_.transport_sequence += 1U;
  return true;
}

void BaseDriverNode::send_stop_command()
{
  BaseCommand stop_command;
  stop_command.type = CommandType::kStop;
  stop_command.linear_x_mps = 0.0;
  stop_command.angular_z_radps = 0.0;
  (void)this->send_velocity_command(stop_command);
  this->stop_command_sent_ = true;
}

}  // namespace amr::tb3::base_driver
