#include "amr_costmap_server/costmap_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace amr_costmap_server
{

namespace
{

constexpr int kUnknownCellValue = -1;

}  // namespace

CostmapServer::CostmapServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("costmap_server", options),
  map_topic_(""),
  obstacle_report_topic_(""),
  global_costmap_topic_(""),
  local_costmap_topic_(""),
  obstacle_threshold_(50),
  global_inflation_radius_(0.20),
  global_inflation_cost_(80),
  local_dynamic_inflation_radius_(0.30),
  local_dynamic_cost_(100),
  footprint_polygon_(),
  footprint_padding_(0.02),
  footprint_circumscribed_radius_(0.0),
  map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  has_map_(false),
  has_obstacle_report_(false)
{
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->declare_parameter("topics.global", this->global_costmap_topic_);
  this->declare_parameter("topics.local", this->local_costmap_topic_);
  this->declare_parameter("inflation.obstacle_threshold", this->obstacle_threshold_);
  this->declare_parameter("inflation.global.radius", this->global_inflation_radius_);
  this->declare_parameter("inflation.global.cost", this->global_inflation_cost_);
  this->declare_parameter("inflation.local.radius", this->local_dynamic_inflation_radius_);
  this->declare_parameter("inflation.local.cost", this->local_dynamic_cost_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_);
  this->declare_parameter("footprint.padding", this->footprint_padding_);
}

CostmapServer::CallbackReturn CostmapServer::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->get_parameter("topics.global", this->global_costmap_topic_);
  this->get_parameter("topics.local", this->local_costmap_topic_);
  this->get_parameter("inflation.obstacle_threshold", this->obstacle_threshold_);
  this->get_parameter("inflation.global.radius", this->global_inflation_radius_);
  this->get_parameter("inflation.global.cost", this->global_inflation_cost_);
  this->get_parameter("inflation.local.radius", this->local_dynamic_inflation_radius_);
  this->get_parameter("inflation.local.cost", this->local_dynamic_cost_);
  this->get_parameter("footprint.polygon", this->footprint_polygon_);
  this->get_parameter("footprint.padding", this->footprint_padding_);
  this->update_footprint_metrics();

  if (
    this->map_topic_.empty() || this->obstacle_report_topic_.empty() ||
    this->global_costmap_topic_.empty() || this->local_costmap_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Costmap topics must not be empty: map='%s' obstacle_report='%s' global='%s' local='%s'",
      this->map_topic_.c_str(),
      this->obstacle_report_topic_.c_str(),
      this->global_costmap_topic_.c_str(),
      this->local_costmap_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      this->handle_map(message);
    });
  this->obstacle_report_subscription_ = this->create_subscription<amr_msgs::msg::ObstacleReport>(
    this->obstacle_report_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::ObstacleReport::SharedPtr message) {
      this->handle_obstacle_report(message);
    });
  this->global_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->global_costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->local_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->local_costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(
    this->get_logger(),
    "Configured costmap server with map='%s', report='%s', global='%s', local='%s', footprint_radius=%.3f m padding=%.3f m",
    this->map_topic_.c_str(),
    this->obstacle_report_topic_.c_str(),
    this->global_costmap_topic_.c_str(),
    this->local_costmap_topic_.c_str(),
    this->footprint_circumscribed_radius_,
    this->footprint_padding_);
  return CallbackReturn::SUCCESS;
}

CostmapServer::CallbackReturn CostmapServer::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->global_costmap_publisher_) {
    this->global_costmap_publisher_->on_activate();
  }
  if (this->local_costmap_publisher_) {
    this->local_costmap_publisher_->on_activate();
  }
  this->publish_costmaps();
  RCLCPP_INFO(this->get_logger(), "Activated costmap server");
  return CallbackReturn::SUCCESS;
}

CostmapServer::CallbackReturn CostmapServer::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->global_costmap_publisher_) {
    this->global_costmap_publisher_->on_deactivate();
  }
  if (this->local_costmap_publisher_) {
    this->local_costmap_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CostmapServer::CallbackReturn CostmapServer::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->map_subscription_.reset();
  this->obstacle_report_subscription_.reset();
  this->global_costmap_publisher_.reset();
  this->local_costmap_publisher_.reset();
  this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->global_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->local_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->latest_obstacle_report_ = amr_msgs::msg::ObstacleReport();
  this->has_map_ = false;
  this->has_obstacle_report_ = false;
  return CallbackReturn::SUCCESS;
}

CostmapServer::CallbackReturn CostmapServer::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return this->on_cleanup(state);
}

void CostmapServer::handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  this->map_ = message;
  this->has_map_ = true;
  this->rebuild_costmaps();
  this->publish_costmaps();
}

void CostmapServer::handle_obstacle_report(
  const amr_msgs::msg::ObstacleReport::SharedPtr message)
{
  this->latest_obstacle_report_ = *message;
  this->has_obstacle_report_ = true;
  this->rebuild_costmaps();
  this->publish_costmaps();
}

void CostmapServer::rebuild_costmaps()
{
  if (!this->has_map_ || !this->map_) {
    return;
  }

  this->global_costmap_ = *this->map_;
  const int width = static_cast<int>(this->global_costmap_.info.width);
  const int height = static_cast<int>(this->global_costmap_.info.height);
  if (width <= 0 || height <= 0 || this->global_costmap_.data.empty()) {
    return;
  }

  const auto original = this->global_costmap_.data;
  const double global_effective_radius =
    this->global_inflation_radius_ + this->footprint_circumscribed_radius_ + this->footprint_padding_;
  const int global_radius_cells = std::max(
    0,
    static_cast<int>(std::ceil(
      global_effective_radius / this->global_costmap_.info.resolution)));

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      if (original[static_cast<std::size_t>(index)] < this->obstacle_threshold_) {
        continue;
      }

      for (int dy = -global_radius_cells; dy <= global_radius_cells; ++dy) {
        for (int dx = -global_radius_cells; dx <= global_radius_cells; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }

          const double distance = std::sqrt(static_cast<double>((dx * dx) + (dy * dy)));
          if (distance > static_cast<double>(global_radius_cells)) {
            continue;
          }

          const int neighbor_index = ny * width + nx;
          const int8_t neighbor_value = original[static_cast<std::size_t>(neighbor_index)];
          if (neighbor_value == kUnknownCellValue || neighbor_value >= this->obstacle_threshold_) {
            continue;
          }

          this->global_costmap_.data[static_cast<std::size_t>(neighbor_index)] =
            static_cast<int8_t>(std::max<int>(
            this->global_costmap_.data[static_cast<std::size_t>(neighbor_index)],
            this->global_inflation_cost_));
        }
      }
    }
  }

  this->local_costmap_ = this->global_costmap_;

  if (!this->has_obstacle_report_ || !this->latest_obstacle_report_.active ||
    !this->latest_obstacle_report_.is_dynamic)
  {
    return;
  }

  const double resolution = static_cast<double>(this->local_costmap_.info.resolution);
  const double local_effective_radius =
    this->local_dynamic_inflation_radius_ + this->footprint_circumscribed_radius_ +
    this->footprint_padding_;
  const int dynamic_radius_cells = std::max(
    1,
    static_cast<int>(std::ceil(local_effective_radius / resolution)));
  const int grid_x = static_cast<int>(std::floor(
      (this->latest_obstacle_report_.obstacle_point.x -
      this->local_costmap_.info.origin.position.x) / resolution));
  const int grid_y = static_cast<int>(std::floor(
      (this->latest_obstacle_report_.obstacle_point.y -
      this->local_costmap_.info.origin.position.y) / resolution));

  for (int dy = -dynamic_radius_cells; dy <= dynamic_radius_cells; ++dy) {
    for (int dx = -dynamic_radius_cells; dx <= dynamic_radius_cells; ++dx) {
      const int nx = grid_x + dx;
      const int ny = grid_y + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      const double distance = std::sqrt(static_cast<double>((dx * dx) + (dy * dy)));
      if (distance > static_cast<double>(dynamic_radius_cells)) {
        continue;
      }

      const int index = ny * width + nx;
      const int8_t current_value = this->local_costmap_.data[static_cast<std::size_t>(index)];
      if (current_value == kUnknownCellValue) {
        continue;
      }

      this->local_costmap_.data[static_cast<std::size_t>(index)] =
        static_cast<int8_t>(std::max<int>(current_value, this->local_dynamic_cost_));
    }
  }
}

void CostmapServer::publish_costmaps()
{
  if (!this->has_map_ || this->global_costmap_.data.empty() || this->local_costmap_.data.empty()) {
    return;
  }
  if (this->global_costmap_publisher_ && this->global_costmap_publisher_->is_activated()) {
    this->global_costmap_publisher_->publish(this->global_costmap_);
  }
  if (this->local_costmap_publisher_ && this->local_costmap_publisher_->is_activated()) {
    this->local_costmap_publisher_->publish(this->local_costmap_);
  }
}

void CostmapServer::update_footprint_metrics()
{
  this->footprint_circumscribed_radius_ = 0.0;

  if (this->footprint_polygon_.size() < 6U || (this->footprint_polygon_.size() % 2U) != 0U) {
    if (!this->footprint_polygon_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring footprint polygon: expected an even-length [x1, y1, ...] array with at least 3 points, got %zu entries",
        this->footprint_polygon_.size());
    }
    return;
  }

  for (std::size_t index = 0; index < this->footprint_polygon_.size(); index += 2U) {
    const double x = this->footprint_polygon_[index];
    const double y = this->footprint_polygon_[index + 1U];
    if (!std::isfinite(x) || !std::isfinite(y)) {
      RCLCPP_WARN(this->get_logger(), "Ignoring non-finite footprint vertex at index %zu", index / 2U);
      this->footprint_circumscribed_radius_ = 0.0;
      return;
    }

    this->footprint_circumscribed_radius_ = std::max(
      this->footprint_circumscribed_radius_,
      std::hypot(x, y));
  }
}

}  // namespace amr_costmap_server
