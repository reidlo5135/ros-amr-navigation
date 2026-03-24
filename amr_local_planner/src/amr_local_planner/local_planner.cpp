#include "amr_local_planner/local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace amr_local_planner
{

namespace
{

constexpr int kUnknownCellValue = -1;

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell & other) const
  {
    return this->x == other.x && this->y == other.y;
  }
};

struct AStarNode
{
  double g_cost;
  double h_cost;
  int parent_index;
  bool opened;
  bool closed;
};

struct OpenSetEntry
{
  int index;
  double f_cost;
};

struct OpenSetEntryCompare
{
  bool operator()(const OpenSetEntry & lhs, const OpenSetEntry & rhs) const
  {
    return lhs.f_cost > rhs.f_cost;
  }
};

bool is_within_bounds(const GridCell & cell, int width, int height)
{
  return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
}

int to_index(const GridCell & cell, int width)
{
  return cell.y * width + cell.x;
}

bool is_occupied(
  const std::vector<int8_t> & occupancy_grid,
  int width,
  const GridCell & cell,
  int obstacle_threshold,
  bool allow_unknown)
{
  const int cell_value = occupancy_grid[static_cast<std::size_t>(to_index(cell, width))];
  if (cell_value == kUnknownCellValue) {
    return !allow_unknown;
  }
  return cell_value >= obstacle_threshold;
}

bool is_diagonal_move_blocked(
  const std::vector<int8_t> & occupancy_grid,
  int width,
  int height,
  const GridCell & current,
  const GridCell & next,
  int obstacle_threshold,
  bool allow_unknown,
  bool prevent_corner_cutting)
{
  if (!prevent_corner_cutting) {
    return false;
  }

  if (current.x == next.x || current.y == next.y) {
    return false;
  }

  const GridCell horizontal{next.x, current.y};
  const GridCell vertical{current.x, next.y};
  if (!is_within_bounds(horizontal, width, height) || !is_within_bounds(vertical, width, height)) {
    return true;
  }

  return
    is_occupied(occupancy_grid, width, horizontal, obstacle_threshold, allow_unknown) ||
    is_occupied(occupancy_grid, width, vertical, obstacle_threshold, allow_unknown);
}

double heuristic(const GridCell & from, const GridCell & to, int connectivity)
{
  const double dx = std::abs(from.x - to.x);
  const double dy = std::abs(from.y - to.y);
  if (connectivity == 8) {
    const double min_delta = std::min(dx, dy);
    const double max_delta = std::max(dx, dy);
    return (min_delta * std::sqrt(2.0)) + (max_delta - min_delta);
  }
  return dx + dy;
}

double turn_penalty(
  const GridCell & previous,
  const GridCell & current,
  const GridCell & next,
  double penalty)
{
  const int previous_dx = current.x - previous.x;
  const int previous_dy = current.y - previous.y;
  const int next_dx = next.x - current.x;
  const int next_dy = next.y - current.y;
  if (previous_dx == next_dx && previous_dy == next_dy) {
    return 0.0;
  }
  return penalty;
}

std::vector<GridCell> get_neighbors(const GridCell & cell, int connectivity)
{
  std::vector<GridCell> neighbors{
    {cell.x + 1, cell.y},
    {cell.x - 1, cell.y},
    {cell.x, cell.y + 1},
    {cell.x, cell.y - 1}
  };

  if (connectivity == 8) {
    neighbors.push_back({cell.x + 1, cell.y + 1});
    neighbors.push_back({cell.x + 1, cell.y - 1});
    neighbors.push_back({cell.x - 1, cell.y + 1});
    neighbors.push_back({cell.x - 1, cell.y - 1});
  }

  return neighbors;
}

double grid_path_length(const std::vector<GridCell> & path)
{
  if (path.size() < 2U) {
    return 0.0;
  }

  double total_length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const double dx = static_cast<double>(path[index].x - path[index - 1U].x);
    const double dy = static_cast<double>(path[index].y - path[index - 1U].y);
    total_length += std::sqrt((dx * dx) + (dy * dy));
  }

  return total_length;
}

bool plan_on_grid(
  const std::vector<int8_t> & occupancy_grid,
  int width,
  int height,
  const GridCell & start,
  const GridCell & goal,
  int obstacle_threshold,
  bool allow_unknown,
  int connectivity,
  bool prevent_corner_cutting,
  double penalty,
  std::vector<GridCell> & path)
{
  path.clear();

  if (
    width <= 0 || height <= 0 ||
    occupancy_grid.size() != static_cast<std::size_t>(width * height))
  {
    return false;
  }
  if (!is_within_bounds(start, width, height) || !is_within_bounds(goal, width, height)) {
    return false;
  }
  if (
    is_occupied(occupancy_grid, width, start, obstacle_threshold, allow_unknown) ||
    is_occupied(occupancy_grid, width, goal, obstacle_threshold, allow_unknown))
  {
    return false;
  }

  std::vector<AStarNode> nodes(
    static_cast<std::size_t>(width * height),
    {std::numeric_limits<double>::infinity(), 0.0, -1, false, false});
  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, OpenSetEntryCompare> open_set;

  const int start_index = to_index(start, width);
  const int goal_index = to_index(goal, width);
  nodes[static_cast<std::size_t>(start_index)].g_cost = 0.0;
  nodes[static_cast<std::size_t>(start_index)].h_cost = heuristic(start, goal, connectivity);
  nodes[static_cast<std::size_t>(start_index)].opened = true;
  open_set.push(
    {
      start_index,
      nodes[static_cast<std::size_t>(start_index)].g_cost +
      nodes[static_cast<std::size_t>(start_index)].h_cost
    });

  while (!open_set.empty()) {
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    AStarNode & current_node = nodes[static_cast<std::size_t>(current_entry.index)];
    if (current_node.closed) {
      continue;
    }

    current_node.closed = true;
    if (current_entry.index == goal_index) {
      int path_index = goal_index;
      while (path_index >= 0) {
        path.push_back({path_index % width, path_index / width});
        path_index = nodes[static_cast<std::size_t>(path_index)].parent_index;
      }
      std::reverse(path.begin(), path.end());
      return true;
    }

    const GridCell current_cell{current_entry.index % width, current_entry.index / width};
    for (const auto & neighbor : get_neighbors(current_cell, connectivity)) {
      if (
        !is_within_bounds(neighbor, width, height) ||
        is_occupied(occupancy_grid, width, neighbor, obstacle_threshold, allow_unknown) ||
        is_diagonal_move_blocked(
          occupancy_grid,
          width,
          height,
          current_cell,
          neighbor,
          obstacle_threshold,
          allow_unknown,
          prevent_corner_cutting))
      {
        continue;
      }

      const int neighbor_index = to_index(neighbor, width);
      AStarNode & neighbor_node = nodes[static_cast<std::size_t>(neighbor_index)];
      if (neighbor_node.closed) {
        continue;
      }

      const bool diagonal = neighbor.x != current_cell.x && neighbor.y != current_cell.y;
      double tentative_g_cost = current_node.g_cost + (diagonal ? std::sqrt(2.0) : 1.0);
      if (current_node.parent_index >= 0) {
        const GridCell previous{
          current_node.parent_index % width,
          current_node.parent_index / width};
        tentative_g_cost += turn_penalty(previous, current_cell, neighbor, penalty);
      }

      if (!neighbor_node.opened || tentative_g_cost < neighbor_node.g_cost) {
        neighbor_node.g_cost = tentative_g_cost;
        neighbor_node.h_cost = heuristic(neighbor, goal, connectivity);
        neighbor_node.parent_index = current_entry.index;
        neighbor_node.opened = true;
        open_set.push(
          {
            neighbor_index,
            neighbor_node.g_cost + neighbor_node.h_cost
          });
      }
    }
  }

  return false;
}

}  // namespace

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner", options),
  command_topic_(""),
  current_pose_topic_(""),
  map_topic_(""),
  obstacle_report_topic_(""),
  local_plan_topic_(""),
  local_escape_service_name_("/amr/local_planner/plan_local_escape"),
  publish_period_ms_(100),
  lookahead_distance_(0.8),
  goal_tolerance_(0.15),
  obstacle_threshold_(50),
  connectivity_(8),
  allow_unknown_(false),
  prevent_corner_cutting_(true),
  turn_penalty_(0.5),
  dynamic_obstacle_enabled_(true),
  dynamic_obstacle_replan_lookahead_distance_(1.4),
  dynamic_obstacle_escape_forward_distance_(1.2),
  dynamic_obstacle_escape_lateral_distance_(0.55),
  nearest_free_search_radius_cells_(4),
  last_command_id_(0U),
  last_progress_index_(0U),
  map_occupancy_grid_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  has_command_(false),
  has_current_pose_(false),
  has_map_(false),
  has_obstacle_report_(false)
{
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.costmap", this->map_topic_);
  this->declare_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->declare_parameter("topics.plan", this->local_plan_topic_);
  this->declare_parameter("services.local_escape", this->local_escape_service_name_);
  this->declare_parameter("planner.publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("planner.lookahead_distance", this->lookahead_distance_);
  this->declare_parameter("planner.goal_tolerance", this->goal_tolerance_);
  this->declare_parameter("planner.obstacle_threshold", this->obstacle_threshold_);
  this->declare_parameter("planner.connectivity", this->connectivity_);
  this->declare_parameter("planner.allow_unknown", this->allow_unknown_);
  this->declare_parameter(
    "planner.prevent_corner_cutting", this->prevent_corner_cutting_);
  this->declare_parameter("planner.turn_penalty", this->turn_penalty_);
  this->declare_parameter("planner.nearest_free_search_radius_cells", this->nearest_free_search_radius_cells_);
  this->declare_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
  this->declare_parameter(
    "dynamic_obstacle.replan_lookahead_distance", this->dynamic_obstacle_replan_lookahead_distance_);
  this->declare_parameter(
    "dynamic_obstacle.escape_forward_distance", this->dynamic_obstacle_escape_forward_distance_);
  this->declare_parameter(
    "dynamic_obstacle.escape_lateral_distance", this->dynamic_obstacle_escape_lateral_distance_);
}

LocalPlanner::CallbackReturn LocalPlanner::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.costmap", this->map_topic_);
  this->get_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->get_parameter("topics.plan", this->local_plan_topic_);
  this->get_parameter("services.local_escape", this->local_escape_service_name_);
  this->get_parameter("planner.publish_period_ms", this->publish_period_ms_);
  this->get_parameter("planner.lookahead_distance", this->lookahead_distance_);
  this->get_parameter("planner.goal_tolerance", this->goal_tolerance_);
  this->get_parameter("planner.obstacle_threshold", this->obstacle_threshold_);
  this->get_parameter("planner.connectivity", this->connectivity_);
  this->get_parameter("planner.allow_unknown", this->allow_unknown_);
  this->get_parameter(
    "planner.prevent_corner_cutting", this->prevent_corner_cutting_);
  this->get_parameter("planner.turn_penalty", this->turn_penalty_);
  this->get_parameter(
    "planner.nearest_free_search_radius_cells", this->nearest_free_search_radius_cells_);
  this->get_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
  this->get_parameter(
    "dynamic_obstacle.replan_lookahead_distance", this->dynamic_obstacle_replan_lookahead_distance_);
  this->get_parameter(
    "dynamic_obstacle.escape_forward_distance", this->dynamic_obstacle_escape_forward_distance_);
  this->get_parameter(
    "dynamic_obstacle.escape_lateral_distance", this->dynamic_obstacle_escape_lateral_distance_);

  if (
    this->command_topic_.empty() || this->current_pose_topic_.empty() ||
    this->map_topic_.empty() || this->obstacle_report_topic_.empty() ||
    this->local_plan_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Local planner topics must not be empty: command='%s' pose='%s' costmap='%s' obstacle_report='%s' local_plan='%s'",
      this->command_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->map_topic_.c_str(),
      this->obstacle_report_topic_.c_str(),
      this->local_plan_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
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
  this->local_escape_service_ = this->create_service<amr_msgs::srv::PlanLocalEscape>(
    this->local_escape_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
      std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response)
    {
      this->handle_plan_local_escape(request, response);
    });
  this->local_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
    this->local_plan_topic_, rclcpp::SystemDefaultsQoS());
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(this->publish_period_ms_),
    [this]() { this->publish_local_plan(); });
  this->timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(),
    "Configured local planner with command='%s', pose='%s', costmap='%s', obstacle_report='%s', plan='%s', lookahead=%.2f",
    this->command_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->map_topic_.c_str(),
    this->obstacle_report_topic_.c_str(),
    this->local_plan_topic_.c_str(),
    this->lookahead_distance_);

  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->local_plan_publisher_->on_activate();
  this->timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->timer_) {
    this->timer_->cancel();
  }
  if (this->local_plan_publisher_) {
    this->local_plan_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->map_subscription_.reset();
  this->obstacle_report_subscription_.reset();
  this->local_escape_service_.reset();
  this->local_plan_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->inflated_map_ = nav_msgs::msg::OccupancyGrid();
  this->working_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->latest_obstacle_report_ = amr_msgs::msg::ObstacleReport();
  this->last_command_id_ = 0U;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  this->has_map_ = false;
  this->has_obstacle_report_ = false;
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->map_subscription_.reset();
  this->obstacle_report_subscription_.reset();
  this->local_escape_service_.reset();
  this->local_plan_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->inflated_map_ = nav_msgs::msg::OccupancyGrid();
  this->working_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->latest_obstacle_report_ = amr_msgs::msg::ObstacleReport();
  this->last_command_id_ = 0U;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  this->has_map_ = false;
  this->has_obstacle_report_ = false;
  return CallbackReturn::SUCCESS;
}

void LocalPlanner::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  if (message->command_id != this->last_command_id_) {
    this->last_progress_index_ = 0U;
    this->last_command_id_ = message->command_id;
  }
  this->latest_command_ = *message;
  this->has_command_ = true;
  RCLCPP_INFO(
    this->get_logger(), "Received command %u for route '%s' and updating local plan",
    message->command_id, message->route_id.c_str());
  this->publish_local_plan();
}

void LocalPlanner::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void LocalPlanner::handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  this->map_occupancy_grid_ = message;
  this->has_map_ = true;
  this->inflated_map_ = *message;
  this->working_costmap_ = *message;

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "Updated local costmap for local planner: size=%u x %u resolution=%.3f",
    message->info.width,
    message->info.height,
    message->info.resolution);
}

void LocalPlanner::handle_obstacle_report(
  const amr_msgs::msg::ObstacleReport::SharedPtr message)
{
  this->latest_obstacle_report_ = *message;
  this->has_obstacle_report_ = true;
}

void LocalPlanner::handle_plan_local_escape(
  const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response)
{
  if (!response) {
    return;
  }

  response->success = false;
  response->plan = nav_msgs::msg::Path();

  if (!this->has_map_ || this->inflated_map_.data.empty()) {
    response->message = "Local planner map is not available yet.";
    return;
  }

  if (request->source_plan.poses.empty()) {
    response->message = "Source plan is empty.";
    return;
  }

  const auto closest_index =
    this->find_closest_pose_index(request->source_plan, request->current_pose, 0U);
  const auto escape_plan = this->build_inflated_local_plan(
    request->source_plan,
    request->current_pose,
    closest_index,
    std::max(this->lookahead_distance_, this->dynamic_obstacle_replan_lookahead_distance_));

  if (escape_plan.poses.size() < 2U) {
    response->message = "Local escape planner could not build a valid recovery path.";
    return;
  }

  response->success = true;
  response->plan = escape_plan;
  response->message = "Local escape recovery plan is ready.";
}

void LocalPlanner::publish_local_plan()
{
  if (
    !this->local_plan_publisher_ || !this->local_plan_publisher_->is_activated() ||
    !this->has_command_ || !this->has_current_pose_)
  {
    return;
  }

  const auto local_plan = this->build_local_plan(this->latest_command_, this->current_pose_);
  this->local_plan_publisher_->publish(local_plan);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Publishing local plan for command %u from progress index %zu with %zu poses",
    this->latest_command_.command_id,
    this->last_progress_index_,
    local_plan.poses.size());
}

nav_msgs::msg::Path LocalPlanner::build_local_plan(
  const amr_msgs::msg::MotionCommand & command,
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto source_plan = this->build_source_plan(command);
  nav_msgs::msg::Path local_plan;
  local_plan.header = source_plan.header;
  if (local_plan.header.frame_id.empty()) {
    local_plan.header.frame_id = current_pose.header.frame_id;
  }
  local_plan.header.stamp = this->now();

  if (source_plan.poses.empty()) {
    return local_plan;
  }

  const auto & goal_pose = source_plan.poses.back();
  if (this->pose_distance(current_pose, goal_pose) <= this->goal_tolerance_) {
    this->last_progress_index_ = source_plan.poses.size() - 1U;
    return local_plan;
  }

  const auto closest_index =
    this->find_closest_pose_index(source_plan, current_pose, this->last_progress_index_);
  this->last_progress_index_ = closest_index;
  const bool obstacle_active =
    this->dynamic_obstacle_enabled_ && this->has_obstacle_report_ &&
    this->latest_obstacle_report_.active &&
    this->latest_obstacle_report_.is_dynamic &&
    this->latest_obstacle_report_.blocks_path;
  const double replan_lookahead_distance = obstacle_active ?
    std::max(this->lookahead_distance_, this->dynamic_obstacle_replan_lookahead_distance_) :
    this->lookahead_distance_;

  if (this->has_map_ && !this->inflated_map_.data.empty()) {
    local_plan = this->build_inflated_local_plan(
      source_plan,
      current_pose,
      closest_index,
      replan_lookahead_distance);
    if (!local_plan.poses.empty()) {
      return local_plan;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Inflated local replanning failed; falling back to sliced local plan");
  }

  return this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    replan_lookahead_distance);
}

nav_msgs::msg::Path LocalPlanner::build_inflated_local_plan(
  const nav_msgs::msg::Path & source_plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::size_t closest_index,
  const double lookahead_distance)
{
  const auto sliced_plan = this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    lookahead_distance);
  if (sliced_plan.poses.size() < 2U || !this->has_map_) {
    return sliced_plan;
  }

  if (
    !this->dynamic_obstacle_enabled_ || !this->has_obstacle_report_ ||
    !this->latest_obstacle_report_.active || !this->latest_obstacle_report_.is_dynamic ||
    !this->latest_obstacle_report_.blocks_path)
  {
    this->working_costmap_ = this->inflated_map_;
    return sliced_plan;
  }

  this->working_costmap_ = this->inflated_map_;
  const auto & working_map = this->working_costmap_;

  const int width = static_cast<int>(working_map.info.width);
  const int height = static_cast<int>(working_map.info.height);
  int start_x = 0;
  int start_y = 0;
  if (!this->world_to_grid(current_pose.pose.position, start_x, start_y)) {
    return sliced_plan;
  }
  if (!this->find_nearest_free_cell(
      working_map.data, width, height, start_x, start_y,
      this->nearest_free_search_radius_cells_))
  {
    return sliced_plan;
  }

  auto plan_segment = [&](const GridCell & segment_start,
      const GridCell & segment_goal,
      std::vector<GridCell> & grid_path) {
      return plan_on_grid(
        working_map.data,
        width,
        height,
        segment_start,
        segment_goal,
        this->obstacle_threshold_,
        this->allow_unknown_,
        this->connectivity_,
        this->prevent_corner_cutting_,
        this->turn_penalty_,
        grid_path);
    };

  auto build_grid_plan_to_pose = [&](const geometry_msgs::msg::PoseStamped & goal_pose,
      std::vector<GridCell> & grid_path) {
      int goal_x = 0;
      int goal_y = 0;
      if (!this->world_to_grid(goal_pose.pose.position, goal_x, goal_y)) {
        return false;
      }
      if (!this->find_nearest_free_cell(
          working_map.data, width, height, goal_x, goal_y,
          this->nearest_free_search_radius_cells_))
      {
        return false;
      }
      return plan_segment({start_x, start_y}, {goal_x, goal_y}, grid_path);
    };

  std::vector<GridCell> best_grid_path;
  if (
    this->latest_obstacle_report_.active &&
    this->latest_obstacle_report_.is_dynamic &&
    this->latest_obstacle_report_.blocks_path)
  {
    const auto & rejoin_pose = sliced_plan.poses.back();
    int rejoin_x = 0;
    int rejoin_y = 0;
    if (
      this->world_to_grid(rejoin_pose.pose.position, rejoin_x, rejoin_y) &&
      this->find_nearest_free_cell(
        working_map.data, width, height, rejoin_x, rejoin_y,
        this->nearest_free_search_radius_cells_))
    {
      const double current_yaw = std::atan2(
        2.0 * (
          current_pose.pose.orientation.w * current_pose.pose.orientation.z +
          current_pose.pose.orientation.x * current_pose.pose.orientation.y),
        1.0 - 2.0 * (
          current_pose.pose.orientation.y * current_pose.pose.orientation.y +
          current_pose.pose.orientation.z * current_pose.pose.orientation.z));
      double path_heading = current_yaw;
      if (sliced_plan.poses.size() >= 2U) {
        const auto & heading_target = sliced_plan.poses[1U];
        const double heading_dx =
          heading_target.pose.position.x - current_pose.pose.position.x;
        const double heading_dy =
          heading_target.pose.position.y - current_pose.pose.position.y;
        if ((heading_dx * heading_dx) + (heading_dy * heading_dy) > 1e-6) {
          path_heading = std::atan2(heading_dy, heading_dx);
        }
      } else {
        const double heading_dx =
          rejoin_pose.pose.position.x - current_pose.pose.position.x;
        const double heading_dy =
          rejoin_pose.pose.position.y - current_pose.pose.position.y;
        if ((heading_dx * heading_dx) + (heading_dy * heading_dy) > 1e-6) {
          path_heading = std::atan2(heading_dy, heading_dx);
        }
      }
      const double forward_distance = std::max(
        this->dynamic_obstacle_escape_forward_distance_,
        this->latest_obstacle_report_.distance + 0.35);
      const double obstacle_heading = current_yaw + this->latest_obstacle_report_.bearing;
      const double path_forward_x = std::cos(path_heading);
      const double path_forward_y = std::sin(path_heading);
      const double obstacle_vector_x = std::cos(obstacle_heading);
      const double obstacle_vector_y = std::sin(obstacle_heading);
      const double obstacle_side =
        (path_forward_x * obstacle_vector_y) - (path_forward_y * obstacle_vector_x);
      const double preferred_sign = obstacle_side >= 0.0 ? -1.0 : 1.0;
      const std::vector<double> escape_signs{preferred_sign, -preferred_sign};
      double best_score = std::numeric_limits<double>::max();

      for (const double sign : escape_signs) {
        geometry_msgs::msg::PoseStamped escape_pose;
        escape_pose.header = current_pose.header;
        escape_pose.pose.orientation = current_pose.pose.orientation;
        escape_pose.pose.position.x =
          current_pose.pose.position.x +
          (std::cos(path_heading) * forward_distance) -
          (std::sin(path_heading) * sign * this->dynamic_obstacle_escape_lateral_distance_);
        escape_pose.pose.position.y =
          current_pose.pose.position.y +
          (std::sin(path_heading) * forward_distance) +
          (std::cos(path_heading) * sign * this->dynamic_obstacle_escape_lateral_distance_);
        escape_pose.pose.position.z = 0.0;

        int escape_x = 0;
        int escape_y = 0;
        if (!this->world_to_grid(escape_pose.pose.position, escape_x, escape_y)) {
          continue;
        }
        if (!this->find_nearest_free_cell(
            working_map.data, width, height, escape_x, escape_y,
            this->nearest_free_search_radius_cells_))
        {
          continue;
        }

        std::vector<GridCell> escape_path;
        if (!plan_segment({start_x, start_y}, {escape_x, escape_y}, escape_path) || escape_path.empty()) {
          continue;
        }

        std::vector<GridCell> rejoin_path;
        if (!plan_segment({escape_x, escape_y}, {rejoin_x, rejoin_y}, rejoin_path) || rejoin_path.empty()) {
          continue;
        }

        std::vector<GridCell> combined_path = escape_path;
        combined_path.insert(combined_path.end(), rejoin_path.begin() + 1, rejoin_path.end());
        const double score = grid_path_length(combined_path);
        if (score < best_score) {
          best_score = score;
          best_grid_path = std::move(combined_path);
        }
      }
    }
  }

  if (best_grid_path.empty() && !build_grid_plan_to_pose(sliced_plan.poses.back(), best_grid_path)) {
    return sliced_plan;
  }
  if (best_grid_path.empty()) {
    return sliced_plan;
  }

  nav_msgs::msg::Path local_plan;
  local_plan.header = sliced_plan.header;
  local_plan.header.stamp = this->now();
  local_plan.poses.push_back(current_pose);
  for (std::size_t index = 1; index < best_grid_path.size(); ++index) {
    local_plan.poses.push_back(
      this->grid_to_pose(
        best_grid_path[index].x,
        best_grid_path[index].y,
        local_plan.header.frame_id));
  }

  if (local_plan.poses.size() == 1U) {
    local_plan.poses.push_back(sliced_plan.poses.back());
  }

  return local_plan;
}

nav_msgs::msg::Path LocalPlanner::build_sliced_local_plan(
  const nav_msgs::msg::Path & source_plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::size_t closest_index) const
{
  return this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    this->lookahead_distance_);
}

nav_msgs::msg::Path LocalPlanner::build_sliced_local_plan_with_lookahead(
  const nav_msgs::msg::Path & source_plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::size_t closest_index,
  const double lookahead_distance) const
{
  nav_msgs::msg::Path local_plan;
  local_plan.header = source_plan.header;
  if (local_plan.header.frame_id.empty()) {
    local_plan.header.frame_id = current_pose.header.frame_id;
  }
  local_plan.header.stamp = this->now();
  local_plan.poses.push_back(current_pose);

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped segment_start = current_pose;
  for (std::size_t index = closest_index; index < source_plan.poses.size(); ++index) {
    const auto & target_pose = source_plan.poses[index];
    const double segment_distance = this->pose_distance(segment_start, target_pose);

    if (segment_distance <= 1e-6) {
      segment_start = target_pose;
      continue;
    }

    if (accumulated_distance + segment_distance >= lookahead_distance) {
      const double remaining_distance = lookahead_distance - accumulated_distance;
      const double interpolation_ratio =
        std::clamp(remaining_distance / segment_distance, 0.0, 1.0);
      local_plan.poses.push_back(
        this->interpolate_pose(segment_start, target_pose, interpolation_ratio));
      return local_plan;
    }

    local_plan.poses.push_back(target_pose);
    accumulated_distance += segment_distance;
    segment_start = target_pose;
  }

  return local_plan;
}

nav_msgs::msg::Path LocalPlanner::build_source_plan(const amr_msgs::msg::MotionCommand & command) const
{
  nav_msgs::msg::Path source_plan = command.plan;
  if (source_plan.header.frame_id.empty()) {
    source_plan.header = command.header;
  }
  if (source_plan.header.stamp.sec == 0 && source_plan.header.stamp.nanosec == 0U) {
    source_plan.header.stamp = this->now();
  }
  if (source_plan.poses.empty()) {
    source_plan.poses.push_back(command.goal_pose);
  }
  return source_plan;
}

std::size_t LocalPlanner::find_closest_pose_index(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::size_t start_index) const
{
  if (plan.poses.empty()) {
    return 0U;
  }

  const auto search_start = std::min(start_index, plan.poses.size() - 1U);
  std::size_t closest_index = search_start;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t index = search_start; index < plan.poses.size(); ++index) {
    const double distance = this->pose_distance(current_pose, plan.poses[index]);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = index;
    }
  }

  return closest_index;
}

bool LocalPlanner::world_to_grid(
  const geometry_msgs::msg::Point & point,
  int & grid_x,
  int & grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  const auto & info = this->map_occupancy_grid_->info;
  grid_x = static_cast<int>(std::floor((point.x - info.origin.position.x) / info.resolution));
  grid_y = static_cast<int>(std::floor((point.y - info.origin.position.y) / info.resolution));

  return
    grid_x >= 0 && grid_x < static_cast<int>(info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(info.height);
}

geometry_msgs::msg::PoseStamped LocalPlanner::grid_to_pose(
  const int grid_x,
  const int grid_y,
  const std::string & frame_id) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = this->now();
  pose.pose.position.x =
    this->inflated_map_.info.origin.position.x +
    ((static_cast<double>(grid_x) + 0.5) * this->inflated_map_.info.resolution);
  pose.pose.position.y =
    this->inflated_map_.info.origin.position.y +
    ((static_cast<double>(grid_y) + 0.5) * this->inflated_map_.info.resolution);
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool LocalPlanner::is_occupied_cell(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  const int grid_x,
  const int grid_y) const
{
  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
    return true;
  }

  const int value = occupancy_grid[static_cast<std::size_t>(grid_y * width + grid_x)];
  if (value == kUnknownCellValue) {
    return !this->allow_unknown_;
  }
  return value >= this->obstacle_threshold_;
}

bool LocalPlanner::find_nearest_free_cell(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  int & grid_x,
  int & grid_y,
  const int max_radius) const
{
  if (!this->is_occupied_cell(occupancy_grid, width, height, grid_x, grid_y)) {
    return true;
  }

  const int original_x = grid_x;
  const int original_y = grid_y;
  for (int radius = 1; radius <= max_radius; ++radius) {
    bool found_candidate = false;
    int best_x = grid_x;
    int best_y = grid_y;
    double best_distance_squared = std::numeric_limits<double>::max();
    int best_axis_offset = std::numeric_limits<int>::max();
    int best_total_offset = std::numeric_limits<int>::max();

    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }

        const int candidate_x = original_x + dx;
        const int candidate_y = original_y + dy;
        if (this->is_occupied_cell(
            occupancy_grid, width, height, candidate_x, candidate_y))
        {
          continue;
        }

        const int offset_x = candidate_x - original_x;
        const int offset_y = candidate_y - original_y;
        const double distance_squared =
          static_cast<double>((offset_x * offset_x) + (offset_y * offset_y));
        const int axis_offset = std::min(std::abs(offset_x), std::abs(offset_y));
        const int total_offset = std::abs(offset_x) + std::abs(offset_y);

        if (
          !found_candidate ||
          distance_squared < best_distance_squared ||
          (distance_squared == best_distance_squared && axis_offset < best_axis_offset) ||
          (
            distance_squared == best_distance_squared &&
            axis_offset == best_axis_offset &&
            total_offset < best_total_offset))
        {
          found_candidate = true;
          best_x = candidate_x;
          best_y = candidate_y;
          best_distance_squared = distance_squared;
          best_axis_offset = axis_offset;
          best_total_offset = total_offset;
        }
      }
    }

    if (found_candidate) {
      grid_x = best_x;
      grid_y = best_y;
      return true;
    }
  }

  return false;
}

geometry_msgs::msg::PoseStamped LocalPlanner::interpolate_pose(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const double ratio) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = goal.header;
  pose.header.stamp = this->now();
  pose.pose.position.x = start.pose.position.x + ((goal.pose.position.x - start.pose.position.x) * ratio);
  pose.pose.position.y = start.pose.position.y + ((goal.pose.position.y - start.pose.position.y) * ratio);
  pose.pose.position.z = start.pose.position.z + ((goal.pose.position.z - start.pose.position.z) * ratio);
  pose.pose.orientation = ratio < 1.0 ? start.pose.orientation : goal.pose.orientation;
  return pose;
}

double LocalPlanner::pose_distance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr_local_planner
