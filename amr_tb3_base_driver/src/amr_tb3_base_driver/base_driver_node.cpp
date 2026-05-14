#include "amr_tb3_base_driver/base_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
  emergency_stop_topic_(""),
  publish_tf_(true),
  cmd_vel_timeout_sec_(0.5),
  wheel_separation_m_(0.160),
  wheel_radius_m_(0.033),
  max_linear_velocity_mps_(0.22),
  max_angular_velocity_radps_(2.84),
  last_cmd_vel_stamp_(0, 0, this->get_clock()->get_clock_type()),
  odometry_(this->wheel_separation_m_, this->wheel_radius_m_)
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
  this->declare_parameter("safety.emergency_stop_topic", this->emergency_stop_topic_);
  this->declare_parameter("publish_tf", this->publish_tf_);
  this->declare_parameter("watchdog.cmd_vel_timeout_sec", this->cmd_vel_timeout_sec_);
  this->declare_parameter("kinematics.wheel_separation_m", this->wheel_separation_m_);
  this->declare_parameter("kinematics.wheel_radius_m", this->wheel_radius_m_);
  this->declare_parameter("kinematics.max_linear_velocity_mps", this->max_linear_velocity_mps_);
  this->declare_parameter(
    "kinematics.max_angular_velocity_radps", this->max_angular_velocity_radps_);
  this->declare_parameter("fake_feedback_mode", this->fake_feedback_mode_);
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
  this->get_parameter("safety.emergency_stop_topic", this->emergency_stop_topic_);
  this->get_parameter("publish_tf", this->publish_tf_);
  this->get_parameter("watchdog.cmd_vel_timeout_sec", this->cmd_vel_timeout_sec_);
  this->get_parameter("kinematics.wheel_separation_m", this->wheel_separation_m_);
  this->get_parameter("kinematics.wheel_radius_m", this->wheel_radius_m_);
  this->get_parameter("kinematics.max_linear_velocity_mps", this->max_linear_velocity_mps_);
  this->get_parameter(
    "kinematics.max_angular_velocity_radps", this->max_angular_velocity_radps_);
  this->get_parameter("fake_feedback_mode", this->fake_feedback_mode_);

  this->transport_.set_port(this->port_);
  this->transport_.set_baudrate(this->baudrate_);
  this->odometry_.set_wheel_geometry(this->wheel_separation_m_, this->wheel_radius_m_);
}

void BaseDriverNode::setup_interfaces()
{
  this->cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    this->cmd_vel_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      this->handle_cmd_vel(message);
    });

  if (!this->emergency_stop_topic_.empty()) {
    this->emergency_stop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      this->emergency_stop_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        this->handle_emergency_stop(message);
      });
  }

  this->odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->odom_topic_,
    rclcpp::SystemDefaultsQoS());
  this->imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    this->imu_topic_,
    rclcpp::SensorDataQoS());
  this->joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    this->joint_states_topic_,
    rclcpp::SystemDefaultsQoS());
  this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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

  this->feedback_timer_ = this->create_wall_timer(
    20ms,
    [this]() {
      this->poll_feedback();
    });
}

void BaseDriverNode::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }
  if (this->emergency_stop_active_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Ignoring /cmd_vel while emergency stop is active");
    this->send_stop_command();
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
  this->has_cmd_vel_ = false;
  this->stop_command_sent_ = true;
  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "OpenCR transport disconnected: %s",
    this->transport_.last_error().c_str());
}

void BaseDriverNode::handle_feedback(const BaseFeedback & feedback)
{
  this->publish_odometry(feedback);
  this->publish_joint_states(feedback);
  this->publish_imu(feedback);
  this->publish_tf(feedback);
}

void BaseDriverNode::poll_feedback()
{
  if (!this->transport_.is_open()) {
    return;
  }

  std::uint8_t buffer[256] = {};
  const std::ptrdiff_t bytes_read = this->transport_.read(buffer, sizeof(buffer), 1ms);
  if (bytes_read < 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "OpenCR feedback read failed: %s",
      this->transport_.last_error().c_str());
    return;
  }
  if (bytes_read == 0) {
    return;
  }

  this->rx_buffer_.insert(this->rx_buffer_.end(), buffer, buffer + bytes_read);

  while (true) {
    const auto packet = this->protocol_.try_parse_packet(this->rx_buffer_);
    if (!packet.has_value()) {
      break;
    }

    const auto state = this->protocol_.decode_state(*packet);
    if (state.has_value()) {
      this->base_state_.connected = true;
      this->base_state_.protocol_ready = state->protocol_ready;
      this->base_state_.transport_sequence += 1U;
      this->base_state_.last_error = state->last_error;
    }

    auto feedback = this->protocol_.decode_feedback(*packet);
    if (feedback.has_value()) {
      if (feedback->stamp.count() == 0) {
        feedback->stamp = std::chrono::nanoseconds(this->now().nanoseconds());
      }
      this->handle_feedback(*feedback);
    }
  }
}

void BaseDriverNode::handle_emergency_stop(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message) {
    return;
  }

  const bool previous_state = this->emergency_stop_active_;
  this->emergency_stop_active_ = message->data;

  if (this->emergency_stop_active_) {
    this->has_cmd_vel_ = false;
    this->send_stop_command();
    RCLCPP_ERROR(this->get_logger(), "Emergency stop is active");
    return;
  }

  if (previous_state) {
    RCLCPP_INFO(this->get_logger(), "Emergency stop cleared");
  }
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

void BaseDriverNode::publish_odometry(const BaseFeedback & feedback)
{
  if (
    !feedback.left_wheel.position_valid || !feedback.right_wheel.position_valid ||
    !this->odom_publisher_)
  {
    return;
  }

  if (!this->odometry_.update_from_wheel_positions(
      feedback.left_wheel.position_rad,
      feedback.right_wheel.position_rad,
      feedback.stamp))
  {
    return;
  }

  const OdometryState & state = this->odometry_.state();
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(feedback.stamp.count(), this->get_clock()->get_clock_type());
  odom.header.frame_id = this->odom_frame_;
  odom.child_frame_id = this->base_frame_;
  odom.pose.pose.position.x = state.x_m;
  odom.pose.pose.position.y = state.y_m;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.z = std::sin(state.yaw_rad * 0.5);
  odom.pose.pose.orientation.w = std::cos(state.yaw_rad * 0.5);
  odom.twist.twist.linear.x = state.linear_velocity_mps;
  odom.twist.twist.angular.z = state.angular_velocity_radps;
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.01;
  odom.pose.covariance[35] = 0.02;
  odom.twist.covariance[0] = 0.02;
  odom.twist.covariance[7] = 0.02;
  odom.twist.covariance[35] = 0.04;
  this->odom_publisher_->publish(odom);
}

void BaseDriverNode::publish_joint_states(const BaseFeedback & feedback)
{
  if (!this->joint_states_publisher_) {
    return;
  }

  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = rclcpp::Time(
    feedback.stamp.count(), this->get_clock()->get_clock_type());
  joint_states.name = {"left_wheel_joint", "right_wheel_joint"};
  joint_states.position = {
    feedback.left_wheel.position_rad,
    feedback.right_wheel.position_rad,
  };
  joint_states.velocity = {
    feedback.left_wheel.velocity_radps,
    feedback.right_wheel.velocity_radps,
  };
  this->joint_states_publisher_->publish(joint_states);
}

void BaseDriverNode::publish_imu(const BaseFeedback & feedback)
{
  if (!this->imu_publisher_) {
    return;
  }

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = rclcpp::Time(feedback.stamp.count(), this->get_clock()->get_clock_type());
  imu.header.frame_id = this->imu_frame_;

  if (feedback.imu.orientation_valid) {
    imu.orientation.x = feedback.imu.orientation_xyzw[0];
    imu.orientation.y = feedback.imu.orientation_xyzw[1];
    imu.orientation.z = feedback.imu.orientation_xyzw[2];
    imu.orientation.w = feedback.imu.orientation_xyzw[3];
    imu.orientation_covariance[0] = 0.02;
    imu.orientation_covariance[4] = 0.02;
    imu.orientation_covariance[8] = 0.04;
  } else {
    imu.orientation_covariance[0] = -1.0;
  }

  imu.angular_velocity.x = feedback.imu.angular_velocity_xyz[0];
  imu.angular_velocity.y = feedback.imu.angular_velocity_xyz[1];
  imu.angular_velocity.z = feedback.imu.angular_velocity_xyz[2];
  imu.linear_acceleration.x = feedback.imu.linear_acceleration_xyz[0];
  imu.linear_acceleration.y = feedback.imu.linear_acceleration_xyz[1];
  imu.linear_acceleration.z = feedback.imu.linear_acceleration_xyz[2];
  imu.angular_velocity_covariance[0] = 0.02;
  imu.angular_velocity_covariance[4] = 0.02;
  imu.angular_velocity_covariance[8] = 0.04;
  imu.linear_acceleration_covariance[0] = 0.04;
  imu.linear_acceleration_covariance[4] = 0.04;
  imu.linear_acceleration_covariance[8] = 0.08;
  this->imu_publisher_->publish(imu);
}

void BaseDriverNode::publish_tf(const BaseFeedback & feedback)
{
  if (!this->publish_tf_ || !this->tf_broadcaster_) {
    return;
  }

  const OdometryState & state = this->odometry_.state();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = rclcpp::Time(
    feedback.stamp.count(), this->get_clock()->get_clock_type());
  transform.header.frame_id = this->odom_frame_;
  transform.child_frame_id = this->base_frame_;
  transform.transform.translation.x = state.x_m;
  transform.transform.translation.y = state.y_m;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.z = std::sin(state.yaw_rad * 0.5);
  transform.transform.rotation.w = std::cos(state.yaw_rad * 0.5);
  this->tf_broadcaster_->sendTransform(transform);
}

}  // namespace amr::tb3::base_driver
