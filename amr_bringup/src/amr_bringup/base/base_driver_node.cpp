#include "amr_bringup/base/base_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace amr::tb3::base_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr double kTickToRad = 0.001533981;
constexpr double kVelocityUnitToRadPerSec = 0.229 * (2.0 * 3.14159265358979323846 / 60.0);
constexpr std::int8_t kOpenCRNoMotorStatus = -1;

template<typename T>
bool load_cached_value(
  const OpenCRSdkWrapper & wrapper,
  const ControlItem & item,
  T & out_value)
{
  const auto value = wrapper.get_cached_data<T>(item.addr, item.length);
  if (!value.has_value()) {
    return false;
  }

  out_value = *value;
  return true;
}

}  // namespace

BaseDriverNode::BaseDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("base_driver", options),
  base_state_(),
  port_("/dev/ttyACM0"),
  baudrate_(1000000),
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
  this->log_configuration();
  (void)this->connect_opencr(this->startup_calibrate_imu_);
}

BaseDriverNode::~BaseDriverNode()
{
  this->send_stop_command();
}

void BaseDriverNode::declare_parameters()
{
  this->declare_parameter("serial.port", this->port_);
  this->declare_parameter("serial.baudrate", this->baudrate_);
  this->declare_parameter("opencr.id", static_cast<int>(this->opencr_id_));
  this->declare_parameter("opencr.protocol_version", this->opencr_protocol_version_);
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
  this->declare_parameter("poll.feedback_rate_hz", this->feedback_rate_hz_);
  this->declare_parameter("poll.heartbeat_rate_hz", this->heartbeat_rate_hz_);
  this->declare_parameter("startup.calibrate_imu", this->startup_calibrate_imu_);
  this->declare_parameter("startup.calibration_wait_sec", this->startup_calibration_wait_sec_);
  this->declare_parameter("fake_feedback_mode", this->fake_feedback_mode_);
}

void BaseDriverNode::load_parameters()
{
  int opencr_id = static_cast<int>(this->opencr_id_);

  this->get_parameter("serial.port", this->port_);
  this->get_parameter("serial.baudrate", this->baudrate_);
  this->get_parameter("opencr.id", opencr_id);
  this->get_parameter("opencr.protocol_version", this->opencr_protocol_version_);
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
  this->get_parameter("poll.feedback_rate_hz", this->feedback_rate_hz_);
  this->get_parameter("poll.heartbeat_rate_hz", this->heartbeat_rate_hz_);
  this->get_parameter("startup.calibrate_imu", this->startup_calibrate_imu_);
  this->get_parameter("startup.calibration_wait_sec", this->startup_calibration_wait_sec_);
  this->get_parameter("fake_feedback_mode", this->fake_feedback_mode_);

  if (opencr_id < 0 || opencr_id > 255) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid opencr.id=%d. Falling back to 200.",
      opencr_id);
    opencr_id = 200;
  }
  this->opencr_id_ = static_cast<std::uint8_t>(opencr_id);

  this->odometry_.set_wheel_geometry(this->wheel_separation_m_, this->wheel_radius_m_);

  OpenCRSdkWrapper::DeviceConfig device_config;
  device_config.port = this->port_;
  device_config.id = this->opencr_id_;
  device_config.baudrate = this->baudrate_;
  device_config.protocol_version = static_cast<float>(this->opencr_protocol_version_);
  this->opencr_.configure(device_config);
}

void BaseDriverNode::log_configuration() const
{
  RCLCPP_INFO(
    this->get_logger(),
    "Base config: port=%s baudrate=%d opencr.id=%u protocol=%.1f cmd_vel=%s odom=%s imu=%s "
    "joint_states=%s frames=[%s,%s,%s,%s] publish_tf=%s timeout=%.3f "
    "wheel_separation=%.3f wheel_radius=%.3f max_linear=%.3f max_angular=%.3f "
    "feedback_rate_hz=%.1f heartbeat_rate_hz=%.1f startup_calibrate_imu=%s "
    "startup_calibration_wait_sec=%.1f fake_feedback_mode=%s",
    this->port_.c_str(),
    this->baudrate_,
    static_cast<unsigned>(this->opencr_id_),
    this->opencr_protocol_version_,
    this->cmd_vel_topic_.c_str(),
    this->odom_topic_.c_str(),
    this->imu_topic_.c_str(),
    this->joint_states_topic_.c_str(),
    this->odom_frame_.c_str(),
    this->base_frame_.c_str(),
    this->body_frame_.c_str(),
    this->imu_frame_.c_str(),
    this->publish_tf_ ? "true" : "false",
    this->cmd_vel_timeout_sec_,
    this->wheel_separation_m_,
    this->wheel_radius_m_,
    this->max_linear_velocity_mps_,
    this->max_angular_velocity_radps_,
    this->feedback_rate_hz_,
    this->heartbeat_rate_hz_,
    this->startup_calibrate_imu_ ? "true" : "false",
    this->startup_calibration_wait_sec_,
    this->fake_feedback_mode_ ? "true" : "false");
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

  const auto feedback_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(this->feedback_rate_hz_, 1.0)));
  this->feedback_timer_ = this->create_wall_timer(
    std::max(10ms, feedback_period),
    [this]() {
      this->poll_feedback();
    });

  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(this->heartbeat_rate_hz_, 1.0)));
  this->heartbeat_timer_ = this->create_wall_timer(
    std::max(20ms, heartbeat_period),
    [this]() {
      if (!this->opencr_.is_open()) {
        return;
      }

      if (!this->opencr_.write_byte(kOpenCRControlTable.heartbeat.addr, this->heartbeat_counter_)) {
        this->base_state_.connected = false;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Failed to send OpenCR heartbeat: %s",
          this->opencr_.last_error().c_str());
        return;
      }

      ++this->heartbeat_counter_;
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
  if (this->opencr_.is_open() && this->base_state_.connected) {
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "OpenCR transport is not ready, attempting reconnect on %s @ %d",
    this->port_.c_str(),
    this->baudrate_);

  if (this->connect_opencr(false)) {
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
    this->opencr_.last_error().c_str());
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
  if (!this->opencr_.is_open()) {
    return;
  }

  const auto feedback = this->read_opencr_feedback();
  if (!feedback.has_value()) {
    this->base_state_.connected = false;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "OpenCR feedback polling failed: %s",
      this->opencr_.last_error().c_str());
    return;
  }

  if (!this->first_feedback_read_logged_) {
    this->first_feedback_read_logged_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Received first OpenCR feedback block from id=%u on %s",
      static_cast<unsigned>(this->opencr_id_),
      this->port_.c_str());
  }

  this->base_state_.connected = true;
  this->base_state_.protocol_ready = true;
  this->base_state_.transport_sequence += 1U;
  this->base_state_.last_error.clear();
  this->handle_feedback(*feedback);
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
  if (!this->opencr_.is_open()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "OpenCR transport is not connected; dropping velocity command");
    return false;
  }

  union VelocityBuffer
  {
    std::int32_t dword[6];
    std::uint8_t byte[24];
  } data{};

  data.dword[0] = command.type == CommandType::kStop ?
    0 : static_cast<std::int32_t>(command.linear_x_mps * 100.0);
  data.dword[1] = 0;
  data.dword[2] = 0;
  data.dword[3] = 0;
  data.dword[4] = 0;
  data.dword[5] = command.type == CommandType::kStop ?
    0 : static_cast<std::int32_t>(command.angular_z_radps * 100.0);

  const std::uint16_t start_addr = kOpenCRControlTable.cmd_velocity_linear_x.addr;
  const std::uint16_t block_length =
    (kOpenCRControlTable.cmd_velocity_angular_z.addr -
    kOpenCRControlTable.cmd_velocity_linear_x.addr) +
    kOpenCRControlTable.cmd_velocity_angular_z.length;

  if (!this->opencr_.write_register(start_addr, block_length, &data.byte[0])) {
    this->base_state_.connected = false;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Failed to send OpenCR velocity command: %s",
      this->opencr_.last_error().c_str());
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

bool BaseDriverNode::connect_opencr(const bool calibrate_imu)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Attempting to open OpenCR via DynamixelSDK on %s @ %d (id=%u, protocol=%.1f)",
    this->port_.c_str(),
    this->baudrate_,
    static_cast<unsigned>(this->opencr_id_),
    this->opencr_protocol_version_);

  if (!this->opencr_.reconnect()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open OpenCR SDK connection on %s @ %d: %s",
      this->port_.c_str(),
      this->baudrate_,
      this->opencr_.last_error().c_str());
    return false;
  }

  this->opencr_.init_read_memory(
    kOpenCRControlTable.millis.addr,
    (kOpenCRControlTable.profile_acceleration_right.addr - kOpenCRControlTable.millis.addr) +
    kOpenCRControlTable.profile_acceleration_right.length);

  if (!this->opencr_.is_connected_to_device()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "OpenCR port opened, but the controller did not respond on id=%u: %s",
      static_cast<unsigned>(this->opencr_id_),
      this->opencr_.last_error().c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Connected to OpenCR on %s @ %d (id=%u)",
    this->port_.c_str(),
    this->baudrate_,
    static_cast<unsigned>(this->opencr_id_));

  if (calibrate_imu && this->startup_calibrate_imu_) {
    if (!this->opencr_.write_byte(kOpenCRControlTable.imu_re_calibration.addr, 1U)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to trigger OpenCR gyro recalibration: %s",
        this->opencr_.last_error().c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::duration<double>(this->startup_calibration_wait_sec_));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  }

  if (!this->opencr_.refresh_read_memory()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Connected to OpenCR, but initial read block failed: %s",
      this->opencr_.last_error().c_str());
    return false;
  }

  std::int8_t device_status = 0;
  if (load_cached_value(this->opencr_, kOpenCRControlTable.device_status, device_status)) {
    if (device_status == kOpenCRNoMotorStatus) {
      RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and power rail");
    } else {
      RCLCPP_INFO(this->get_logger(), "OpenCR device_status=%d", static_cast<int>(device_status));
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Connected to OpenCR, but device_status could not be decoded from cached memory");
  }

  this->raw_wheel_state_initialized_ = false;
  this->cumulative_left_position_rad_ = 0.0;
  this->cumulative_right_position_rad_ = 0.0;
  this->heartbeat_counter_ = 0U;
  this->base_state_.connected = true;
  this->base_state_.protocol_ready = true;
  this->base_state_.last_error.clear();
  return true;
}

std::optional<BaseFeedback> BaseDriverNode::read_opencr_feedback()
{
  if (!this->opencr_.refresh_read_memory()) {
    return std::nullopt;
  }

  BaseFeedback feedback;
  feedback.stamp = std::chrono::nanoseconds(this->now().nanoseconds());

  std::int32_t left_position_ticks = 0;
  std::int32_t right_position_ticks = 0;
  std::int32_t left_velocity_units = 0;
  std::int32_t right_velocity_units = 0;
  float imu_orientation_w = 1.0F;
  float imu_orientation_x = 0.0F;
  float imu_orientation_y = 0.0F;
  float imu_orientation_z = 0.0F;
  float imu_angular_velocity_x = 0.0F;
  float imu_angular_velocity_y = 0.0F;
  float imu_angular_velocity_z = 0.0F;
  float imu_linear_acceleration_x = 0.0F;
  float imu_linear_acceleration_y = 0.0F;
  float imu_linear_acceleration_z = 0.0F;
  std::int32_t battery_voltage_raw = 0;
  std::int32_t battery_percentage_raw = 0;

  if (
    !load_cached_value(this->opencr_, kOpenCRControlTable.present_position_left, left_position_ticks) ||
    !load_cached_value(
      this->opencr_, kOpenCRControlTable.present_position_right, right_position_ticks) ||
    !load_cached_value(this->opencr_, kOpenCRControlTable.present_velocity_left, left_velocity_units) ||
    !load_cached_value(
      this->opencr_, kOpenCRControlTable.present_velocity_right, right_velocity_units))
  {
    this->base_state_.last_error = "OpenCR wheel feedback decode failed";
    return std::nullopt;
  }

  if (!this->raw_wheel_state_initialized_) {
    this->last_left_position_ticks_ = left_position_ticks;
    this->last_right_position_ticks_ = right_position_ticks;
    this->raw_wheel_state_initialized_ = true;
  } else {
    this->cumulative_left_position_rad_ +=
      static_cast<double>(left_position_ticks - this->last_left_position_ticks_) * kTickToRad;
    this->cumulative_right_position_rad_ +=
      static_cast<double>(right_position_ticks - this->last_right_position_ticks_) * kTickToRad;
    this->last_left_position_ticks_ = left_position_ticks;
    this->last_right_position_ticks_ = right_position_ticks;
  }

  feedback.left_wheel.position_valid = true;
  feedback.right_wheel.position_valid = true;
  feedback.left_wheel.velocity_valid = true;
  feedback.right_wheel.velocity_valid = true;
  feedback.left_wheel.position_rad = this->cumulative_left_position_rad_;
  feedback.right_wheel.position_rad = this->cumulative_right_position_rad_;
  feedback.left_wheel.velocity_radps =
    static_cast<double>(left_velocity_units) * kVelocityUnitToRadPerSec;
  feedback.right_wheel.velocity_radps =
    static_cast<double>(right_velocity_units) * kVelocityUnitToRadPerSec;

  const bool has_orientation =
    load_cached_value(this->opencr_, kOpenCRControlTable.imu_orientation_w, imu_orientation_w) &&
    load_cached_value(this->opencr_, kOpenCRControlTable.imu_orientation_x, imu_orientation_x) &&
    load_cached_value(this->opencr_, kOpenCRControlTable.imu_orientation_y, imu_orientation_y) &&
    load_cached_value(this->opencr_, kOpenCRControlTable.imu_orientation_z, imu_orientation_z);
  const bool has_angular_velocity =
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_angular_velocity_x, imu_angular_velocity_x) &&
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_angular_velocity_y, imu_angular_velocity_y) &&
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_angular_velocity_z, imu_angular_velocity_z);
  const bool has_linear_acceleration =
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_linear_acceleration_x, imu_linear_acceleration_x) &&
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_linear_acceleration_y, imu_linear_acceleration_y) &&
    load_cached_value(
      this->opencr_, kOpenCRControlTable.imu_linear_acceleration_z, imu_linear_acceleration_z);

  feedback.imu.orientation_valid = has_orientation;
  if (has_orientation) {
    feedback.imu.orientation_xyzw = {
      static_cast<double>(imu_orientation_x),
      static_cast<double>(imu_orientation_y),
      static_cast<double>(imu_orientation_z),
      static_cast<double>(imu_orientation_w),
    };
  }

  if (has_angular_velocity) {
    feedback.imu.angular_velocity_xyz = {
      static_cast<double>(imu_angular_velocity_x),
      static_cast<double>(imu_angular_velocity_y),
      static_cast<double>(imu_angular_velocity_z),
    };
  }

  if (has_linear_acceleration) {
    feedback.imu.linear_acceleration_xyz = {
      static_cast<double>(imu_linear_acceleration_x),
      static_cast<double>(imu_linear_acceleration_y),
      static_cast<double>(imu_linear_acceleration_z),
    };
  }

  if (
    load_cached_value(this->opencr_, kOpenCRControlTable.battery_voltage, battery_voltage_raw) &&
    load_cached_value(
      this->opencr_, kOpenCRControlTable.battery_percentage, battery_percentage_raw))
  {
    feedback.battery.valid = true;
    feedback.battery.voltage_v = 0.01 * static_cast<double>(battery_voltage_raw);
    feedback.battery.percentage = 0.01 * static_cast<double>(battery_percentage_raw);
  }

  return feedback;
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
