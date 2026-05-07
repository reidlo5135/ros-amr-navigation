#include "amr_costmap_server/costmap_server.hpp"

namespace amr::costmap::server
{

namespace
{

constexpr int kUnknownCellValue = -1;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

CostmapServer::CostmapServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("costmap_server", options),
  map_topic_(""),
  pose_topic_(""),
  scan_topic_(""),
  global_costmap_topic_(""),
  local_costmap_topic_(""),
  clear_costmap_service_name_("/amr/costmap_server/clear_costmap"),
  obstacle_threshold_(50),
  global_inflation_radius_(0.20),
  global_inflation_cost_(80),
  local_dynamic_inflation_radius_(0.30),
  local_dynamic_cost_(100),
  dynamic_max_distance_(1.8),
  dynamic_forward_angle_deg_(100.0),
  dynamic_static_clearance_cells_(4),
  publish_global_on_scan_(false),
  local_publish_min_period_ms_(100),
  local_window_enabled_(true),
  local_window_radius_(2.5),
  footprint_polygon_(),
  footprint_padding_(0.02),
  footprint_circumscribed_radius_(0.0),
  last_local_publish_time_(0, 0, RCL_ROS_TIME),
  map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  has_map_(false),
  has_pose_(false),
  has_scan_(false)
{
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.pose", this->pose_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.global", this->global_costmap_topic_);
  this->declare_parameter("topics.local", this->local_costmap_topic_);
  this->declare_parameter("services.clear_costmap", this->clear_costmap_service_name_);
  this->declare_parameter("inflation.obstacle_threshold", this->obstacle_threshold_);
  this->declare_parameter("inflation.global.radius", this->global_inflation_radius_);
  this->declare_parameter("inflation.global.cost", this->global_inflation_cost_);
  this->declare_parameter("inflation.local.radius", this->local_dynamic_inflation_radius_);
  this->declare_parameter("inflation.local.cost", this->local_dynamic_cost_);
  this->declare_parameter("dynamic.max_distance", this->dynamic_max_distance_);
  this->declare_parameter("dynamic.forward_angle_deg", this->dynamic_forward_angle_deg_);
  this->declare_parameter("dynamic.static_clearance_cells", this->dynamic_static_clearance_cells_);
  this->declare_parameter("publish.global_on_scan", this->publish_global_on_scan_);
  this->declare_parameter("publish.local_min_period_ms", this->local_publish_min_period_ms_);
  this->declare_parameter("local_window.enabled", this->local_window_enabled_);
  this->declare_parameter("local_window.radius", this->local_window_radius_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_);
  this->declare_parameter("footprint.padding", this->footprint_padding_);
}

CostmapServer::CallbackReturn CostmapServer::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.pose", this->pose_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.global", this->global_costmap_topic_);
  this->get_parameter("topics.local", this->local_costmap_topic_);
  this->get_parameter("services.clear_costmap", this->clear_costmap_service_name_);
  this->get_parameter("inflation.obstacle_threshold", this->obstacle_threshold_);
  this->get_parameter("inflation.global.radius", this->global_inflation_radius_);
  this->get_parameter("inflation.global.cost", this->global_inflation_cost_);
  this->get_parameter("inflation.local.radius", this->local_dynamic_inflation_radius_);
  this->get_parameter("inflation.local.cost", this->local_dynamic_cost_);
  this->get_parameter("dynamic.max_distance", this->dynamic_max_distance_);
  this->get_parameter("dynamic.forward_angle_deg", this->dynamic_forward_angle_deg_);
  this->get_parameter("dynamic.static_clearance_cells", this->dynamic_static_clearance_cells_);
  this->get_parameter("publish.global_on_scan", this->publish_global_on_scan_);
  this->get_parameter("publish.local_min_period_ms", this->local_publish_min_period_ms_);
  this->get_parameter("local_window.enabled", this->local_window_enabled_);
  this->get_parameter("local_window.radius", this->local_window_radius_);
  this->get_parameter("footprint.polygon", this->footprint_polygon_);
  this->get_parameter("footprint.padding", this->footprint_padding_);
  this->update_footprint_metrics();

  if (
    this->map_topic_.empty() || this->pose_topic_.empty() || this->scan_topic_.empty() ||
    this->global_costmap_topic_.empty() || this->local_costmap_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Costmap topics must not be empty: map='%s' pose='%s' scan='%s' global='%s' local='%s'",
      this->map_topic_.c_str(),
      this->pose_topic_.c_str(),
      this->scan_topic_.c_str(),
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
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });
  this->clear_costmap_service_ = this->create_service<amr_msgs::srv::ClearCostmap>(
    this->clear_costmap_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::ClearCostmap::Request> request,
      std::shared_ptr<amr_msgs::srv::ClearCostmap::Response> response)
    {
      this->handle_clear_costmap(request, response);
    });
  this->global_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->global_costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->local_costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->local_costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(
    this->get_logger(),
    "Configured costmap server with map='%s', pose='%s', scan='%s', global='%s', local='%s', clear='%s', footprint_radius=%.3f m padding=%.3f m, global_on_scan=%s, local_min_period_ms=%d, local_window=%s radius=%.2f m",
    this->map_topic_.c_str(),
    this->pose_topic_.c_str(),
    this->scan_topic_.c_str(),
    this->global_costmap_topic_.c_str(),
    this->local_costmap_topic_.c_str(),
    this->clear_costmap_service_name_.c_str(),
    this->footprint_circumscribed_radius_,
    this->footprint_padding_,
    this->publish_global_on_scan_ ? "true" : "false",
    this->local_publish_min_period_ms_,
    this->local_window_enabled_ ? "true" : "false",
    this->local_window_radius_);
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
  this->current_pose_subscription_.reset();
  this->scan_subscription_.reset();
  this->clear_costmap_service_.reset();
  this->global_costmap_publisher_.reset();
  this->local_costmap_publisher_.reset();
  this->map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->global_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->local_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->latest_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->has_map_ = false;
  this->has_pose_ = false;
  this->has_scan_ = false;
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
  this->rebuild_global_costmap();
  this->rebuild_local_costmap();
  this->publish_costmaps();
}

void CostmapServer::handle_current_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->latest_pose_ = *message;
  this->has_pose_ = true;
}

void CostmapServer::handle_scan(
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_scan_ = true;
  if (this->publish_global_on_scan_) {
    this->rebuild_global_costmap();
  }
  this->rebuild_local_costmap();
  if (this->publish_global_on_scan_) {
    this->publish_global_costmap();
  }
  if (this->should_publish_local_costmap()) {
    this->publish_local_costmap();
  }
}

void CostmapServer::handle_clear_costmap(
  const std::shared_ptr<amr_msgs::srv::ClearCostmap::Request> request,
  std::shared_ptr<amr_msgs::srv::ClearCostmap::Response> response)
{
  if (!this->has_map_ || !this->map_) {
    response->success = false;
    response->message = "Static map is not available yet.";
    return;
  }

  if (request->local_only) {
    this->latest_scan_ = sensor_msgs::msg::LaserScan();
    this->has_scan_ = false;
    this->local_costmap_ = this->global_costmap_;
    this->publish_local_costmap();
    response->success = true;
    response->message = "Cleared local dynamic obstacle layer.";
    return;
  }

  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->has_scan_ = false;
  this->rebuild_global_costmap();
  this->rebuild_local_costmap();
  this->publish_costmaps();
  response->success = true;
  response->message = "Rebuilt global and local costmaps from the static map.";
}

void CostmapServer::rebuild_global_costmap()
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
  const double footprint_effective_radius =
    this->footprint_circumscribed_radius_ + this->footprint_padding_;
  const double global_effective_radius =
    std::max(this->global_inflation_radius_, footprint_effective_radius);
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

}

void CostmapServer::rebuild_local_costmap()
{
  if (this->global_costmap_.data.empty()) {
    this->rebuild_global_costmap();
  }
  if (this->global_costmap_.data.empty()) {
    return;
  }

  const int global_width = static_cast<int>(this->global_costmap_.info.width);
  const int global_height = static_cast<int>(this->global_costmap_.info.height);
  const double resolution = static_cast<double>(this->global_costmap_.info.resolution);

  if (
    !this->local_window_enabled_ || !this->has_pose_ || resolution <= 0.0 ||
    global_width <= 0 || global_height <= 0)
  {
    this->local_costmap_ = this->global_costmap_;
  } else {
    int center_x = 0;
    int center_y = 0;
    if (!this->world_to_grid(
        this->latest_pose_.pose.position.x,
        this->latest_pose_.pose.position.y,
        center_x,
        center_y))
    {
      this->local_costmap_ = this->global_costmap_;
    } else {
      const int radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(this->local_window_radius_ / resolution)));
      const int local_width = (radius_cells * 2) + 1;
      const int local_height = (radius_cells * 2) + 1;
      const int start_x = center_x - radius_cells;
      const int start_y = center_y - radius_cells;

      this->local_costmap_ = nav_msgs::msg::OccupancyGrid();
      this->local_costmap_.header = this->global_costmap_.header;
      this->local_costmap_.header.stamp = this->now();
      this->local_costmap_.info = this->global_costmap_.info;
      this->local_costmap_.info.width = static_cast<uint32_t>(local_width);
      this->local_costmap_.info.height = static_cast<uint32_t>(local_height);
      this->local_costmap_.info.origin.position.x =
        this->global_costmap_.info.origin.position.x + (static_cast<double>(start_x) * resolution);
      this->local_costmap_.info.origin.position.y =
        this->global_costmap_.info.origin.position.y + (static_cast<double>(start_y) * resolution);
      this->local_costmap_.data.assign(
        static_cast<std::size_t>(local_width * local_height),
        static_cast<int8_t>(kUnknownCellValue));

      for (int y = 0; y < local_height; ++y) {
        const int source_y = start_y + y;
        if (source_y < 0 || source_y >= global_height) {
          continue;
        }
        for (int x = 0; x < local_width; ++x) {
          const int source_x = start_x + x;
          if (source_x < 0 || source_x >= global_width) {
            continue;
          }
          const int source_index = (source_y * global_width) + source_x;
          const int local_index = (y * local_width) + x;
          this->local_costmap_.data[static_cast<std::size_t>(local_index)] =
            this->global_costmap_.data[static_cast<std::size_t>(source_index)];
        }
      }
    }
  }

  const int width = static_cast<int>(this->local_costmap_.info.width);
  const int height = static_cast<int>(this->local_costmap_.info.height);

  if (!this->has_pose_ || !this->has_scan_)
  {
    return;
  }

  const double footprint_effective_radius =
    this->footprint_circumscribed_radius_ + this->footprint_padding_;
  const double local_effective_radius =
    std::max(this->local_dynamic_inflation_radius_, footprint_effective_radius);
  const int dynamic_radius_cells = std::max(
    1,
    static_cast<int>(std::ceil(local_effective_radius / resolution)));

  const double robot_yaw = std::atan2(
    2.0 * (
      this->latest_pose_.pose.orientation.w * this->latest_pose_.pose.orientation.z +
      this->latest_pose_.pose.orientation.x * this->latest_pose_.pose.orientation.y),
    1.0 - 2.0 * (
      this->latest_pose_.pose.orientation.y * this->latest_pose_.pose.orientation.y +
      this->latest_pose_.pose.orientation.z * this->latest_pose_.pose.orientation.z));
  const double half_angle = 0.5 * this->dynamic_forward_angle_deg_ * kPi / 180.0;

  for (std::size_t index = 0; index < this->latest_scan_.ranges.size(); ++index) {
    const double range = this->latest_scan_.ranges[index];
    if (
      !std::isfinite(range) || range < this->latest_scan_.range_min ||
      range > std::min(this->dynamic_max_distance_, static_cast<double>(this->latest_scan_.range_max)))
    {
      continue;
    }

    const double bearing =
      this->latest_scan_.angle_min + (static_cast<double>(index) * this->latest_scan_.angle_increment);
    if (std::abs(bearing) > half_angle) {
      continue;
    }

    const double world_x =
      this->latest_pose_.pose.position.x + (range * std::cos(robot_yaw + bearing));
    const double world_y =
      this->latest_pose_.pose.position.y + (range * std::sin(robot_yaw + bearing));

    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_costmap_grid(this->local_costmap_, world_x, world_y, grid_x, grid_y)) {
      continue;
    }

    int static_grid_x = 0;
    int static_grid_y = 0;
    if (
      !this->world_to_grid(world_x, world_y, static_grid_x, static_grid_y) ||
      this->has_static_obstacle_near(
        static_grid_x,
        static_grid_y,
        this->dynamic_static_clearance_cells_))
    {
      continue;
    }

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

        const int index_2d = ny * width + nx;
        const int8_t current_value = this->local_costmap_.data[static_cast<std::size_t>(index_2d)];
        if (current_value == kUnknownCellValue) {
          continue;
        }

        this->local_costmap_.data[static_cast<std::size_t>(index_2d)] =
          static_cast<int8_t>(std::max<int>(current_value, this->local_dynamic_cost_));
      }
    }
  }
}

bool CostmapServer::world_to_grid(double world_x, double world_y, int & grid_x, int & grid_y) const
{
  if (!this->map_) {
    return false;
  }

  const double resolution = static_cast<double>(this->map_->info.resolution);
  if (resolution <= 0.0) {
    return false;
  }

  grid_x = static_cast<int>(std::floor((world_x - this->map_->info.origin.position.x) / resolution));
  grid_y = static_cast<int>(std::floor((world_y - this->map_->info.origin.position.y) / resolution));

  return
    grid_x >= 0 && grid_x < static_cast<int>(this->map_->info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(this->map_->info.height);
}

bool CostmapServer::world_to_costmap_grid(
  const nav_msgs::msg::OccupancyGrid & costmap,
  double world_x,
  double world_y,
  int & grid_x,
  int & grid_y) const
{
  const double resolution = static_cast<double>(costmap.info.resolution);
  if (resolution <= 0.0) {
    return false;
  }

  grid_x = static_cast<int>(
    std::floor((world_x - costmap.info.origin.position.x) / resolution));
  grid_y = static_cast<int>(
    std::floor((world_y - costmap.info.origin.position.y) / resolution));

  return
    grid_x >= 0 && grid_x < static_cast<int>(costmap.info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(costmap.info.height);
}

bool CostmapServer::has_static_obstacle_near(int grid_x, int grid_y, int clearance_cells) const
{
  if (!this->map_ || this->map_->data.empty()) {
    return false;
  }

  const int width = static_cast<int>(this->map_->info.width);
  const int height = static_cast<int>(this->map_->info.height);
  for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
    for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
      const int nx = grid_x + dx;
      const int ny = grid_y + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      const int index = ny * width + nx;
      const int8_t cell_value = this->map_->data[static_cast<std::size_t>(index)];
      if (cell_value >= this->obstacle_threshold_) {
        return true;
      }
    }
  }

  return false;
}

void CostmapServer::publish_costmaps()
{
  if (!this->has_map_ || this->global_costmap_.data.empty() || this->local_costmap_.data.empty()) {
    return;
  }
  this->publish_global_costmap();
  this->publish_local_costmap();
}

void CostmapServer::publish_global_costmap()
{
  if (
    !this->has_map_ || this->global_costmap_.data.empty() ||
    !this->global_costmap_publisher_ || !this->global_costmap_publisher_->is_activated())
  {
    return;
  }
  this->global_costmap_publisher_->publish(this->global_costmap_);
}

void CostmapServer::publish_local_costmap()
{
  if (
    !this->has_map_ || this->local_costmap_.data.empty() ||
    !this->local_costmap_publisher_ || !this->local_costmap_publisher_->is_activated())
  {
    return;
  }
  this->local_costmap_publisher_->publish(this->local_costmap_);
  this->last_local_publish_time_ = this->now();
}

bool CostmapServer::should_publish_local_costmap()
{
  if (this->local_publish_min_period_ms_ <= 0) {
    return true;
  }
  const rclcpp::Time now = this->now();
  if (this->last_local_publish_time_.nanoseconds() == 0) {
    return true;
  }
  const double elapsed_ms = (now - this->last_local_publish_time_).seconds() * 1000.0;
  return elapsed_ms >= static_cast<double>(this->local_publish_min_period_ms_);
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

}  // namespace amr::costmap::server
