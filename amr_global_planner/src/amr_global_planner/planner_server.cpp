#include "amr_global_planner/planner_server.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace amr_global_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("global_planner", options),
  map_topic_("/amr/map_server/map"),
  computed_plan_topic_("/amr/global_planner/plan"),
  plan_segment_service_name_("/amr/global_planner/plan_segment"),
  plan_route_service_name_("/amr/global_planner/plan_route"),
  obstacle_threshold_(50),
  connectivity_(8),
  allow_unknown_(false),
  simplify_path_(true),
  prevent_corner_cutting_(true),
  turn_penalty_(0.5)
{
  this->declare_parameter("map_topic", this->map_topic_);
  this->declare_parameter("computed_plan_topic", this->computed_plan_topic_);
  this->declare_parameter("plan_segment_service", this->plan_segment_service_name_);
  this->declare_parameter("plan_route_service", this->plan_route_service_name_);
  this->declare_parameter("obstacle_threshold", this->obstacle_threshold_);
  this->declare_parameter("connectivity", this->connectivity_);
  this->declare_parameter("allow_unknown", this->allow_unknown_);
  this->declare_parameter("simplify_path", this->simplify_path_);
  this->declare_parameter("prevent_corner_cutting", this->prevent_corner_cutting_);
  this->declare_parameter("turn_penalty", this->turn_penalty_);
}

PlannerServer::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("map_topic", this->map_topic_);
  this->get_parameter("computed_plan_topic", this->computed_plan_topic_);
  this->get_parameter("plan_segment_service", this->plan_segment_service_name_);
  this->get_parameter("plan_route_service", this->plan_route_service_name_);
  this->get_parameter("obstacle_threshold", this->obstacle_threshold_);
  this->get_parameter("connectivity", this->connectivity_);
  this->get_parameter("allow_unknown", this->allow_unknown_);
  this->get_parameter("simplify_path", this->simplify_path_);
  this->get_parameter("prevent_corner_cutting", this->prevent_corner_cutting_);
  this->get_parameter("turn_penalty", this->turn_penalty_);

  const auto connectivity =
    this->connectivity_ == 4 ?
    planner::AStarConnectivity::Four :
    planner::AStarConnectivity::Eight;
  this->a_star_planner_ = std::make_unique<planner::AStarPlanner>(
    this->obstacle_threshold_,
    this->allow_unknown_,
    connectivity,
    this->turn_penalty_,
    this->prevent_corner_cutting_);

  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->planned_path_ = nav_msgs::msg::Path();

  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
      this->map_subscription_cb(map);
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
    "Configured global planner with map topic '%s' and services '%s'/'%s'",
    this->map_topic_.c_str(),
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
  this->map_occupancy_grid_.reset();
  this->planned_path_ = nav_msgs::msg::Path();
  this->map_subscription_.reset();
  this->computed_plan_publisher_.reset();
  this->plan_segment_service_.reset();
  this->plan_route_service_.reset();
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->a_star_planner_.reset();
  this->map_occupancy_grid_.reset();
  this->planned_path_ = nav_msgs::msg::Path();
  this->map_subscription_.reset();
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
    !this->map_occupancy_grid_ ||
    this->map_occupancy_grid_->info.width == 0 ||
    this->map_occupancy_grid_->info.height == 0 ||
    this->map_occupancy_grid_->data.empty())
  {
    message = "Map is not available";
    return false;
  }

  planner::GridCell start_cell{};
  planner::GridCell goal_cell{};
  if (!this->world_to_grid(start, start_cell)) {
    message = "Start pose is outside map bounds";
    return false;
  }

  if (!this->world_to_grid(goal, goal_cell)) {
    message = "Goal pose is outside map bounds";
    return false;
  }

  const auto result = this->a_star_planner_->plan(
    this->map_occupancy_grid_->data,
    static_cast<int>(this->map_occupancy_grid_->info.width),
    static_cast<int>(this->map_occupancy_grid_->info.height),
    start_cell,
    goal_cell);

  message = result.message;
  if (!result.success) {
    return false;
  }

  path = this->create_path_message(result.path);
  return true;
}

bool PlannerServer::world_to_grid(
  const geometry_msgs::msg::PoseStamped & pose,
  planner::GridCell & cell) const
{
  if (!this->map_occupancy_grid_ || this->map_occupancy_grid_->info.resolution <= 0.0F) {
    return false;
  }

  const auto & map_info = this->map_occupancy_grid_->info;
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

geometry_msgs::msg::PoseStamped PlannerServer::grid_to_world(const planner::GridCell & cell) const
{
  geometry_msgs::msg::PoseStamped pose;

  if (!this->map_occupancy_grid_) {
    return pose;
  }

  const auto & map_header = this->map_occupancy_grid_->header;
  const auto & map_info = this->map_occupancy_grid_->info;
  const double resolution = static_cast<double>(map_info.resolution);

  pose.header = map_header;
  pose.pose.position.x = map_info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * resolution;
  pose.pose.position.y = map_info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

std::vector<planner::GridCell> PlannerServer::simplify_grid_path(
  const std::vector<planner::GridCell> & grid_path) const
{
  if (!this->simplify_path_ || grid_path.size() <= 2U) {
    return grid_path;
  }

  std::vector<planner::GridCell> simplified_path;
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
  const std::vector<planner::GridCell> & grid_path) const
{
  nav_msgs::msg::Path path;
  if (!this->map_occupancy_grid_) {
    return path;
  }

  const auto simplified_grid_path = this->simplify_grid_path(grid_path);
  path.header = this->map_occupancy_grid_->header;
  path.header.stamp = this->now();
  path.poses.reserve(simplified_grid_path.size());
  for (const auto & cell : simplified_grid_path) {
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

void PlannerServer::map_subscription_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  this->map_occupancy_grid_ = map;
}

}  // namespace amr_global_planner
