#include "amr_global_planner/planner_server.hpp"

namespace amr::planner::global
{

namespace
{

constexpr int kUnknownCellValue = -1;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)),
    1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z))));
}

}  // namespace

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("global_planner", options),
  costmap_topic_(""),
  computed_plan_topic_(""),
  plan_segment_service_name_("/amr/global_planner/plan_segment"),
  plan_route_service_name_("/amr/global_planner/plan_route"),
  obstacle_threshold_(50),
  connectivity_(8),
  allow_unknown_(false),
  simplify_path_(true),
  prevent_corner_cutting_(true),
  turn_penalty_(0.5),
  start_row_hold_penalty_(1.25),
  goal_row_align_distance_cells_(6),
  goal_row_align_penalty_(1.75),
  nearest_free_search_radius_cells_(4)
{
  this->declare_parameter("topics.costmap", this->costmap_topic_);
  this->declare_parameter("topics.plan", this->computed_plan_topic_);
  this->declare_parameter("services.segment", this->plan_segment_service_name_);
  this->declare_parameter("services.route", this->plan_route_service_name_);
  this->declare_parameter("planner.obstacle_threshold", this->obstacle_threshold_);
  this->declare_parameter("planner.connectivity", this->connectivity_);
  this->declare_parameter("planner.allow_unknown", this->allow_unknown_);
  this->declare_parameter("planner.simplify_path", this->simplify_path_);
  this->declare_parameter(
    "planner.prevent_corner_cutting", this->prevent_corner_cutting_);
  this->declare_parameter("planner.turn_penalty", this->turn_penalty_);
  this->declare_parameter("planner.start_row_hold_penalty", this->start_row_hold_penalty_);
  this->declare_parameter(
    "planner.goal_row_align_distance_cells", this->goal_row_align_distance_cells_);
  this->declare_parameter("planner.goal_row_align_penalty", this->goal_row_align_penalty_);
  this->declare_parameter("planner.nearest_free_search_radius_cells", this->nearest_free_search_radius_cells_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_param_);
}

PlannerServer::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.costmap", this->costmap_topic_);
  this->get_parameter("topics.plan", this->computed_plan_topic_);
  this->get_parameter("services.segment", this->plan_segment_service_name_);
  this->get_parameter("services.route", this->plan_route_service_name_);
  this->get_parameter("planner.obstacle_threshold", this->obstacle_threshold_);
  this->get_parameter("planner.connectivity", this->connectivity_);
  this->get_parameter("planner.allow_unknown", this->allow_unknown_);
  this->get_parameter("planner.simplify_path", this->simplify_path_);
  this->get_parameter(
    "planner.prevent_corner_cutting", this->prevent_corner_cutting_);
  this->get_parameter("planner.turn_penalty", this->turn_penalty_);
  this->get_parameter("planner.start_row_hold_penalty", this->start_row_hold_penalty_);
  this->get_parameter(
    "planner.goal_row_align_distance_cells", this->goal_row_align_distance_cells_);
  this->get_parameter("planner.goal_row_align_penalty", this->goal_row_align_penalty_);
  this->get_parameter(
    "planner.nearest_free_search_radius_cells", this->nearest_free_search_radius_cells_);
  this->get_parameter("footprint.polygon", this->footprint_polygon_param_);

  if (
    this->costmap_topic_.empty() || this->computed_plan_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Global planner topics must not be empty: costmap='%s' plan='%s'",
      this->costmap_topic_.c_str(),
      this->computed_plan_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  const auto connectivity =
    this->connectivity_ == 4 ?
    AStarConnectivity::Four :
    AStarConnectivity::Eight;
  this->a_star_planner_ = std::make_unique<AStarPlanner>(
    this->obstacle_threshold_,
    this->allow_unknown_,
    connectivity,
    this->turn_penalty_,
    this->prevent_corner_cutting_,
    this->start_row_hold_penalty_,
    this->goal_row_align_distance_cells_,
    this->goal_row_align_penalty_);
  this->footprint_polygon_ = amr::geometry::make_footprint_polygon(this->footprint_polygon_param_);

  this->global_costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->planned_path_ = nav_msgs::msg::Path();

  this->costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
      this->costmap_subscription_cb(map);
    });
  this->computed_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
    this->computed_plan_topic_, rclcpp::SystemDefaultsQoS());

  this->plan_segment_service_ = this->create_service<amr_msgs::srv::PlanSegment>(
    this->plan_segment_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
      std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response) {
      this->handle_plan_segment(request, response);
    });
  this->plan_route_service_ = this->create_service<amr_msgs::srv::PlanRoute>(
    this->plan_route_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::PlanRoute::Request> request,
      std::shared_ptr<amr_msgs::srv::PlanRoute::Response> response) {
      this->handle_plan_route(request, response);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured global planner with costmap '%s', plan '%s' and services '%s'/'%s'",
    this->costmap_topic_.c_str(),
    this->computed_plan_topic_.c_str(),
    this->plan_segment_service_name_.c_str(),
    this->plan_route_service_name_.c_str());
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->computed_plan_publisher_) {
    this->computed_plan_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated global planner");
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->computed_plan_publisher_) {
    this->computed_plan_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->a_star_planner_.reset();
  this->global_costmap_.reset();
  this->planned_path_ = nav_msgs::msg::Path();
  this->costmap_subscription_.reset();
  this->computed_plan_publisher_.reset();
  this->plan_segment_service_.reset();
  this->plan_route_service_.reset();
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->a_star_planner_.reset();
  this->global_costmap_.reset();
  this->planned_path_ = nav_msgs::msg::Path();
  this->costmap_subscription_.reset();
  this->computed_plan_publisher_.reset();
  this->plan_segment_service_.reset();
  this->plan_route_service_.reset();
  return CallbackReturn::SUCCESS;
}

void PlannerServer::handle_plan_segment(
  const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response)
{
  response->success = false;
  response->plan = nav_msgs::msg::Path();
  response->message.clear();

  response->success = this->compute_plan_between_poses(
    request->start, request->goal, response->plan, response->message);

  if (response->success) {
    this->planned_path_ = response->plan;
    if (this->computed_plan_publisher_ && this->computed_plan_publisher_->is_activated()) {
      this->computed_plan_publisher_->publish(this->planned_path_);
    }
  }
}

void PlannerServer::handle_plan_route(
  const std::shared_ptr<amr_msgs::srv::PlanRoute::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanRoute::Response> response)
{
  response->success = false;
  response->plans.clear();
  response->message.clear();

  auto current = request->start;
  for (std::size_t index = 0; index < request->waypoints.size(); ++index) {
    nav_msgs::msg::Path segment_path;
    std::string segment_message;
    const auto & waypoint = request->waypoints[index];
    const bool success = this->compute_plan_between_poses(
      current, waypoint, segment_path, segment_message);
    if (!success) {
      response->message =
        "Failed to compute segment " + std::to_string(index) + ": " + segment_message;
      response->plans.clear();
      return;
    }

    response->plans.push_back(segment_path);
    current = waypoint;
  }

  response->success = true;
  response->message = "Generated A* plans for all route segments.";
  this->planned_path_ = this->merge_paths(response->plans);
  if (this->computed_plan_publisher_ && this->computed_plan_publisher_->is_activated()) {
    this->computed_plan_publisher_->publish(this->planned_path_);
  }
}

bool PlannerServer::compute_plan_between_poses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path,
  std::string & message) const
{
  path = nav_msgs::msg::Path();
  message.clear();

  if (!this->a_star_planner_) {
    message = "A* planner is not configured";
    return false;
  }

  if (
    !this->global_costmap_ ||
    this->global_costmap_->info.width == 0 ||
    this->global_costmap_->info.height == 0 ||
    this->global_costmap_->data.empty())
  {
    message = "Global costmap is not available";
    return false;
  }

  GridCell start_cell{};
  GridCell goal_cell{};
  if (!this->world_to_grid(start, start_cell)) {
    message = "Start pose is outside map bounds";
    return false;
  }

  if (!this->world_to_grid(goal, goal_cell)) {
    message = "Goal pose is outside map bounds";
    return false;
  }

  const int width = static_cast<int>(this->global_costmap_->info.width);
  const int height = static_cast<int>(this->global_costmap_->info.height);
  const double start_yaw = yaw_from_quaternion(start.pose.orientation);
  const double goal_yaw = yaw_from_quaternion(goal.pose.orientation);
  if (
    !this->find_nearest_free_cell(
      this->global_costmap_->data, width, height, start_cell,
      this->nearest_free_search_radius_cells_, start_yaw) ||
    !this->find_nearest_free_cell(
      this->global_costmap_->data, width, height, goal_cell,
      this->nearest_free_search_radius_cells_, goal_yaw))
  {
    message = "Start or goal cell is occupied in inflated global costmap";
    return false;
  }

  this->a_star_planner_->set_collision_model(
    this->footprint_polygon_,
    this->global_costmap_->info.resolution,
    this->global_costmap_->info.origin.position.x,
    this->global_costmap_->info.origin.position.y);

  const auto result = this->a_star_planner_->plan(
    this->global_costmap_->data,
    width,
    height,
    start_cell,
    goal_cell,
    start_yaw,
    goal_yaw);

  message = result.message;
  if (!result.success) {
    return false;
  }

  path = this->create_path_message(result.path);
  return true;
}

bool PlannerServer::world_to_grid(
  const geometry_msgs::msg::PoseStamped & pose,
  GridCell & cell) const
{
  if (!this->global_costmap_ || this->global_costmap_->info.resolution <= 0.0F) {
    return false;
  }

  const auto & map_info = this->global_costmap_->info;
  const double resolution = static_cast<double>(map_info.resolution);
  const double origin_x = map_info.origin.position.x;
  const double origin_y = map_info.origin.position.y;

  cell.x = static_cast<int>(std::floor((pose.pose.position.x - origin_x) / resolution));
  cell.y = static_cast<int>(std::floor((pose.pose.position.y - origin_y) / resolution));
  return
    cell.x >= 0 &&
    cell.x < static_cast<int>(map_info.width) &&
    cell.y >= 0 &&
    cell.y < static_cast<int>(map_info.height);
}

geometry_msgs::msg::PoseStamped PlannerServer::grid_to_world(const GridCell & cell) const
{
  geometry_msgs::msg::PoseStamped pose;

  if (!this->global_costmap_) {
    return pose;
  }

  const auto & map_header = this->global_costmap_->header;
  const auto & map_info = this->global_costmap_->info;
  const double resolution = static_cast<double>(map_info.resolution);

  pose.header = map_header;
  pose.pose.position.x = map_info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * resolution;
  pose.pose.position.y = map_info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool PlannerServer::is_occupied_cell(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  const GridCell & cell) const
{
  if (cell.x < 0 || cell.x >= width || cell.y < 0 || cell.y >= height) {
    return true;
  }

  const int value = occupancy_grid[static_cast<std::size_t>((cell.y * width) + cell.x)];
  if (value == kUnknownCellValue) {
    return !this->allow_unknown_;
  }
  return value >= this->obstacle_threshold_;
}

bool PlannerServer::find_nearest_free_cell(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  GridCell & cell,
  const int max_radius,
  const double yaw) const
{
  if (!this->is_cell_collision(occupancy_grid, width, height, cell, yaw)) {
    return true;
  }

  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const GridCell candidate{cell.x + dx, cell.y + dy};
        if (!this->is_cell_collision(occupancy_grid, width, height, candidate, yaw)) {
          cell = candidate;
          return true;
        }
      }
    }
  }

  return false;
}

bool PlannerServer::is_cell_collision(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  const GridCell & cell,
  const double yaw) const
{
  if (this->footprint_polygon_.empty() || !this->global_costmap_) {
    return this->is_occupied_cell(occupancy_grid, width, height, cell);
  }

  const auto pose = this->grid_to_world(cell);
  return amr::geometry::footprint_pose_collides(
    occupancy_grid,
    width,
    height,
    this->global_costmap_->info.resolution,
    this->global_costmap_->info.origin.position.x,
    this->global_costmap_->info.origin.position.y,
    this->footprint_polygon_,
    pose.pose.position.x,
    pose.pose.position.y,
    yaw,
    this->obstacle_threshold_,
    this->allow_unknown_);
}

std::vector<GridCell> PlannerServer::simplify_grid_path(const std::vector<GridCell> & grid_path) const
{
  if (!this->simplify_path_ || grid_path.size() <= 2U) {
    return grid_path;
  }

  std::vector<GridCell> simplified_path;
  simplified_path.reserve(grid_path.size());
  simplified_path.push_back(grid_path.front());

  for (std::size_t index = 1; index + 1 < grid_path.size(); ++index) {
    const auto & previous = grid_path[index - 1];
    const auto & current = grid_path[index];
    const auto & next = grid_path[index + 1];

    const int previous_dx = current.x - previous.x;
    const int previous_dy = current.y - previous.y;
    const int next_dx = next.x - current.x;
    const int next_dy = next.y - current.y;

    if (previous_dx != next_dx || previous_dy != next_dy) {
      simplified_path.push_back(current);
    }
  }

  simplified_path.push_back(grid_path.back());
  return simplified_path;
}

nav_msgs::msg::Path PlannerServer::create_path_message(
  const std::vector<GridCell> & grid_path) const
{
  nav_msgs::msg::Path path;
  if (!this->global_costmap_) {
    return path;
  }

  path.header = this->global_costmap_->header;
  path.header.stamp = this->now();
  path.poses.reserve(grid_path.size());
  for (const auto & cell : grid_path) {
    path.poses.push_back(this->grid_to_world(cell));
  }

  return path;
}

nav_msgs::msg::Path PlannerServer::merge_paths(const std::vector<nav_msgs::msg::Path> & paths) const
{
  nav_msgs::msg::Path merged;
  if (paths.empty()) {
    return merged;
  }

  merged.header = paths.front().header;
  merged.header.stamp = this->now();

  for (std::size_t path_index = 0; path_index < paths.size(); ++path_index) {
    const auto & path = paths[path_index];
    for (std::size_t pose_index = 0; pose_index < path.poses.size(); ++pose_index) {
      if (path_index > 0 && pose_index == 0U && !merged.poses.empty()) {
        continue;
      }
      merged.poses.push_back(path.poses[pose_index]);
    }
  }

  return merged;
}

void PlannerServer::costmap_subscription_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  this->global_costmap_ = map;
}

}  // namespace amr::planner::global
