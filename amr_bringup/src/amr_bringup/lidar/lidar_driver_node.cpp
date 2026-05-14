#include "amr_bringup/lidar/lidar_driver_node.hpp"

#include <chrono>
#include <algorithm>
#include <utility>

namespace amr::tb3::lidar_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

LidarDriverNode::LidarDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lidar_driver", options),
  transport_("/dev/ttyUSB0", 230400),
  parser_(),
  scan_publisher_(),
  connection_timer_(),
  poll_timer_(),
  port_("/dev/ttyUSB0"),
  baudrate_(230400),
  frame_id_("base_scan"),
  scan_topic_("/scan"),
  sensor_model_("auto"),
  inverted_(false),
  angle_min_rad_(0.0),
  angle_max_rad_(kTwoPi),
  range_min_m_(0.12),
  range_max_m_(3.5),
  scan_time_sec_(0.1)
{
  this->declare_parameters();
  this->load_parameters();
  this->setup_interfaces();

  if (!this->transport_.open()) {
    RCLCPP_WARN(
      this->get_logger(),
      "TB3 LiDAR transport is not connected yet: %s",
      this->transport_.last_error().c_str());
  }
}

void LidarDriverNode::declare_parameters()
{
  this->declare_parameter("serial.port", this->port_);
  this->declare_parameter("serial.baudrate", this->baudrate_);
  this->declare_parameter("frame_id", this->frame_id_);
  this->declare_parameter("topic", this->scan_topic_);
  this->declare_parameter("sensor_model", this->sensor_model_);
  this->declare_parameter("inverted", this->inverted_);
  this->declare_parameter("angle_min_rad", this->angle_min_rad_);
  this->declare_parameter("angle_max_rad", this->angle_max_rad_);
  this->declare_parameter("range_min_m", this->range_min_m_);
  this->declare_parameter("range_max_m", this->range_max_m_);
  this->declare_parameter("scan_time_sec", this->scan_time_sec_);
  this->declare_parameter("fake_scan_mode", this->fake_scan_mode_);
}

void LidarDriverNode::load_parameters()
{
  this->get_parameter("serial.port", this->port_);
  this->get_parameter("serial.baudrate", this->baudrate_);
  this->get_parameter("frame_id", this->frame_id_);
  this->get_parameter("topic", this->scan_topic_);
  this->get_parameter("sensor_model", this->sensor_model_);
  this->get_parameter("inverted", this->inverted_);
  this->get_parameter("angle_min_rad", this->angle_min_rad_);
  this->get_parameter("angle_max_rad", this->angle_max_rad_);
  this->get_parameter("range_min_m", this->range_min_m_);
  this->get_parameter("range_max_m", this->range_max_m_);
  this->get_parameter("scan_time_sec", this->scan_time_sec_);
  this->get_parameter("fake_scan_mode", this->fake_scan_mode_);

  this->transport_.set_port(this->port_);
  this->transport_.set_baudrate(this->baudrate_);
  this->parser_ = make_lidar_parser(this->sensor_model_);
  this->scan_config_.frame_id = this->frame_id_;
  this->scan_config_.inverted = this->inverted_;
  this->scan_config_.angle_min_rad = static_cast<float>(this->angle_min_rad_);
  this->scan_config_.angle_max_rad = static_cast<float>(this->angle_max_rad_);
  this->scan_config_.range_min_m = static_cast<float>(this->range_min_m_);
  this->scan_config_.range_max_m = static_cast<float>(this->range_max_m_);
  this->scan_config_.scan_time_sec = static_cast<float>(this->scan_time_sec_);
}

void LidarDriverNode::setup_interfaces()
{
  this->scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    this->scan_topic_,
    rclcpp::SensorDataQoS());

  this->connection_timer_ = this->create_wall_timer(
    1s,
    [this]() {
      this->handle_connection_check();
    });

  this->poll_timer_ = this->create_wall_timer(
    20ms,
    [this]() {
      this->poll_lidar();
    });
}

void LidarDriverNode::handle_connection_check()
{
  if (this->transport_.is_open()) {
    return;
  }

  if (this->transport_.reconnect()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Connected LiDAR transport on %s @ %d using parser %s",
      this->port_.c_str(),
      this->baudrate_,
      this->parser_ ? this->parser_->name().c_str() : "none");
    return;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "LiDAR transport disconnected: %s",
    this->transport_.last_error().c_str());
}

void LidarDriverNode::poll_lidar()
{
  if (!this->scan_publisher_ || !this->transport_.is_open() || !this->parser_) {
    return;
  }

  std::uint8_t buffer[512] = {};
  const std::ptrdiff_t bytes_read = this->transport_.read(buffer, sizeof(buffer), 1ms);
  if (bytes_read < 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "LiDAR read failed: %s",
      this->transport_.last_error().c_str());
    return;
  }
  if (bytes_read == 0) {
    return;
  }

  this->rx_buffer_.insert(this->rx_buffer_.end(), buffer, buffer + bytes_read);
  const auto frame = this->parser_->parse(this->rx_buffer_);
  if (!frame.has_value()) {
    return;
  }

  LidarScanFrame stamped_frame = *frame;
  if (stamped_frame.stamp.count() == 0) {
    stamped_frame.stamp = std::chrono::nanoseconds(this->now().nanoseconds());
  }

  this->scan_publisher_->publish(this->assembler_.build_message(stamped_frame, this->scan_config_));
}

}  // namespace amr::tb3::lidar_driver
