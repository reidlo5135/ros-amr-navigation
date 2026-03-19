#include "amr_obstacle_detection/obstacle_detection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_obstacle_detection
{

namespace
{

constexpr int kUnknownCellValue = -1;

}  // namespace

ObstacleDetection::ObstacleDetection(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("obstacle_detection", options),
  scan_topic_(""),
  pose_topic_(""),
  map_topic_(""),
  report_topic_(""),
  max_distance_(1.8),
  forward_angle_deg_(100.0),
  minimum_points_(3),
  static_clearance_cells_(2),
  blocking_distance_(1.2),
  critical_distance_(0.25),
  high_distance_(0.7),
  medium_distance_(1.2),
  obstacle_threshold_(50),
  map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  has_scan_(false),
  has_pose_(false),
  has_map_(false)
{
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.pose", this->pose_topic_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.report", this->report_topic_);
  this->declare_parameter("detection.max_distance", this->max_distance_);
  this->declare_parameter("detection.forward_angle_deg", this->forward_angle_deg_);
  this->declare_parameter("detection.minimum_points", this->minimum_points_);
  this->declare_parameter(
    "detection.static_clearance_cells", this->static_clearance_cells_);
  this->declare_parameter("detection.blocking_distance", this->blocking_distance_);
  this->declare_parameter("detection.critical_distance", this->critical_distance_);
  this->declare_parameter("detection.high_distance", this->high_distance_);
  this->declare_parameter("detection.medium_distance", this->medium_distance_);
  this->declare_parameter("detection.obstacle_threshold", this->obstacle_threshold_);
}

ObstacleDetection::CallbackReturn ObstacleDetection::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.pose", this->pose_topic_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.report", this->report_topic_);
  this->get_parameter("detection.max_distance", this->max_distance_);
  this->get_parameter("detection.forward_angle_deg", this->forward_angle_deg_);
  this->get_parameter("detection.minimum_points", this->minimum_points_);
  this->get_parameter("detection.static_clearance_cells", this->static_clearance_cells_);
  this->get_parameter("detection.blocking_distance", this->blocking_distance_);
  this->get_parameter("detection.critical_distance", this->critical_distance_);
  this->get_parameter("detection.high_distance", this->high_distance_);
  this->get_parameter("detection.medium_distance", this->medium_distance_);
  this->get_parameter("detection.obstacle_threshold", this->obstacle_threshold_);

  if (
    this->scan_topic_.empty() || this->pose_topic_.empty() ||
    this->map_topic_.empty() || this->report_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Obstacle detection topics must not be empty: scan='%s' pose='%s' map='%s' report='%s'",
      this->scan_topic_.c_str(),
      this->pose_topic_.c_str(),
      this->map_topic_.c_str(),
      this->report_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      this->handle_map(message);
    });
  this->report_publisher_ = this->create_publisher<amr_msgs::msg::ObstacleReport>(
    this->report_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    this->get_logger(),
    "Configured obstacle detection with scan='%s', pose='%s', map='%s', report='%s'",
    this->scan_topic_.c_str(),
    this->pose_topic_.c_str(),
    this->map_topic_.c_str(),
    this->report_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

ObstacleDetection::CallbackReturn ObstacleDetection::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->report_publisher_) {
    this->report_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated obstacle detection");
  return CallbackReturn::SUCCESS;
}

ObstacleDetection::CallbackReturn ObstacleDetection::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->report_publisher_) {
    this->report_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

ObstacleDetection::CallbackReturn ObstacleDetection::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->scan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->map_subscription_.reset();
  this->report_publisher_.reset();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->has_scan_ = false;
  this->has_pose_ = false;
  this->has_map_ = false;
  return CallbackReturn::SUCCESS;
}

ObstacleDetection::CallbackReturn ObstacleDetection::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return this->on_cleanup(state);
}

void ObstacleDetection::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_scan_ = true;
  this->publish_report();
}

void ObstacleDetection::handle_current_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_pose_ = true;
}

void ObstacleDetection::handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  this->map_ = message;
  this->has_map_ = true;
}

amr_msgs::msg::ObstacleReport ObstacleDetection::build_obstacle_report() const
{
  amr_msgs::msg::ObstacleReport report;
  report.header.stamp = this->now();
  report.header.frame_id =
    this->current_pose_.header.frame_id.empty() ? std::string("map") : this->current_pose_.header.frame_id;
  report.source = "scan";

  if (!this->has_scan_ || !this->has_pose_ || !this->has_map_) {
    return report;
  }

  const double robot_yaw = this->quaternion_yaw(this->current_pose_.pose.orientation);
  const double half_angle = 0.5 * this->forward_angle_deg_ * M_PI / 180.0;
  double closest_distance = std::numeric_limits<double>::infinity();
  double closest_bearing = 0.0;
  geometry_msgs::msg::Point closest_point;
  bool closest_is_dynamic = false;
  int hit_count = 0;

  for (std::size_t index = 0; index < this->latest_scan_.ranges.size(); ++index) {
    const double range = this->latest_scan_.ranges[index];
    if (
      !std::isfinite(range) || range < this->latest_scan_.range_min ||
      range > std::min(this->max_distance_, static_cast<double>(this->latest_scan_.range_max)))
    {
      continue;
    }

    const double scan_bearing =
      this->latest_scan_.angle_min + static_cast<double>(index) * this->latest_scan_.angle_increment;
    if (std::abs(scan_bearing) > half_angle) {
      continue;
    }

    ++hit_count;

    geometry_msgs::msg::Point point;
    point.x = this->current_pose_.pose.position.x + (range * std::cos(robot_yaw + scan_bearing));
    point.y = this->current_pose_.pose.position.y + (range * std::sin(robot_yaw + scan_bearing));
    point.z = 0.0;

    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_grid(*this->map_, point.x, point.y, grid_x, grid_y)) {
      continue;
    }

    const bool static_match = this->has_static_obstacle_near(*this->map_, grid_x, grid_y);
    if (range < closest_distance) {
      closest_distance = range;
      closest_bearing = scan_bearing;
      closest_point = point;
      closest_is_dynamic = !static_match;
    }
  }

  if (hit_count < this->minimum_points_ || !std::isfinite(closest_distance)) {
    return report;
  }

  report.active = true;
  report.is_dynamic = closest_is_dynamic;
  report.blocks_path = closest_is_dynamic && closest_distance <= this->blocking_distance_;
  report.distance = closest_distance;
  report.bearing = closest_bearing;
  report.obstacle_point = closest_point;

  if (closest_distance <= this->critical_distance_) {
    report.severity = amr_msgs::msg::ObstacleReport::SEVERITY_CRITICAL;
  } else if (closest_distance <= this->high_distance_) {
    report.severity = amr_msgs::msg::ObstacleReport::SEVERITY_HIGH;
  } else if (closest_distance <= this->medium_distance_) {
    report.severity = amr_msgs::msg::ObstacleReport::SEVERITY_MEDIUM;
  } else {
    report.severity = amr_msgs::msg::ObstacleReport::SEVERITY_LOW;
  }

  return report;
}

void ObstacleDetection::publish_report()
{
  if (!this->report_publisher_ || !this->report_publisher_->is_activated()) {
    return;
  }

  const auto report = this->build_obstacle_report();
  this->report_publisher_->publish(report);
}

bool ObstacleDetection::world_to_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  const double x,
  const double y,
  int & grid_x,
  int & grid_y) const
{
  if (map.info.resolution <= 0.0F) {
    return false;
  }

  const double resolution = static_cast<double>(map.info.resolution);
  grid_x = static_cast<int>(std::floor((x - map.info.origin.position.x) / resolution));
  grid_y = static_cast<int>(std::floor((y - map.info.origin.position.y) / resolution));

  return
    grid_x >= 0 &&
    grid_x < static_cast<int>(map.info.width) &&
    grid_y >= 0 &&
    grid_y < static_cast<int>(map.info.height);
}

bool ObstacleDetection::has_static_obstacle_near(
  const nav_msgs::msg::OccupancyGrid & map,
  const int grid_x,
  const int grid_y) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  for (int dy = -this->static_clearance_cells_; dy <= this->static_clearance_cells_; ++dy) {
    for (int dx = -this->static_clearance_cells_; dx <= this->static_clearance_cells_; ++dx) {
      const int nx = grid_x + dx;
      const int ny = grid_y + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      const int8_t value = map.data[static_cast<std::size_t>((ny * width) + nx)];
      if (value != kUnknownCellValue && value >= this->obstacle_threshold_) {
        return true;
      }
    }
  }

  return false;
}

double ObstacleDetection::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp = 2.0 * (
    (orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp = 1.0 - 2.0 * (
    (orientation.y * orientation.y) + (orientation.z * orientation.z));
  return std::atan2(siny_cosp, cosy_cosp);
}

double ObstacleDetection::normalize_angle(const double angle) const
{
  double normalized = angle;
  while (normalized > M_PI) {
    normalized -= 2.0 * M_PI;
  }
  while (normalized < -M_PI) {
    normalized += 2.0 * M_PI;
  }
  return normalized;
}

}  // namespace amr_obstacle_detection
