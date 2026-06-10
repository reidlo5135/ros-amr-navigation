#include "amr_controller_server/controller_server.hpp"


namespace amr::planner::local
{

namespace
{

constexpr int kUnknownCellValue = -1;

const char *local_plan_decision_label(const uint8_t decision)
{
  switch (decision)
  {
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
      return "ok";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
      return "goal_proximity_blocked";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
      return "global_replan_required";
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return "hard_blocked";
    default:
      return "unknown";
  }
}

const char *bool_label(const bool value)
{
  return value ? "true" : "false";
}

int throttle_ms_from_sec(const double seconds)
{
  return static_cast<int>(std::max(0.1, seconds) * 1000.0);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  return std::atan2(
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)),
    1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)));
}

double normalize_angle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi)
  {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi)
  {
    angle += 2.0 * kPi;
  }
  return angle;
}

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell &other) const
  {
    return this->x == other.x &&this->y == other.y;
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
  bool operator()(const OpenSetEntry &lhs, const OpenSetEntry &rhs) const
  {
    return lhs.f_cost > rhs.f_cost;
  }
};

bool is_within_bounds(const GridCell &cell, int width, int height)
{
  return cell.x >= 0 &&cell.x < width &&cell.y >= 0 &&cell.y < height;
}

int to_index(const GridCell &cell, int width)
{
  return cell.y * width + cell.x;
}

bool is_occupied(
  const std::vector<int8_t> &occupancy_grid,
  int width,
  int height,
  const GridCell &cell,
  int obstacle_threshold,
  bool allow_unknown,
  const amr::geometry::FootprintPolygon &footprint_polygon,
  double resolution,
  double origin_x,
  double origin_y,
  double yaw)
{
  if (!footprint_polygon.empty() &&resolution > 0.0 &&height > 0)
  {
    const double pose_x = origin_x + (static_cast<double>(cell.x) + 0.5) * resolution;
    const double pose_y = origin_y + (static_cast<double>(cell.y) + 0.5) * resolution;
    return amr::geometry::footprint_pose_collides(
      occupancy_grid,
      width,
      height,
      resolution,
      origin_x,
      origin_y,
      footprint_polygon,
      pose_x,
      pose_y,
      yaw,
      obstacle_threshold,
      allow_unknown);
  }

  const int cell_value = occupancy_grid[static_cast<std::size_t>(to_index(cell, width))];
  if (cell_value == kUnknownCellValue)
  {
    return !allow_unknown;
  }
  return cell_value >= obstacle_threshold;
}

bool is_diagonal_move_blocked(
  const std::vector<int8_t> &occupancy_grid,
  int width,
  int height,
  const GridCell &current,
  const GridCell &next,
  int obstacle_threshold,
  bool allow_unknown,
  bool prevent_corner_cutting,
  const amr::geometry::FootprintPolygon &footprint_polygon,
  double resolution,
  double origin_x,
  double origin_y)
{
  if (!prevent_corner_cutting)
  {
    return false;
  }

  if (current.x == next.x || current.y == next.y)
  {
    return false;
  }

  const GridCell horizontal{next.x, current.y};
  const GridCell vertical{current.x, next.y};
  if (!is_within_bounds(horizontal, width, height) || !is_within_bounds(vertical, width, height))
  {
    return true;
  }

  return
    is_occupied(
    occupancy_grid, width, height, horizontal, obstacle_threshold, allow_unknown,
    footprint_polygon, resolution, origin_x, origin_y, 0.0) ||
    is_occupied(
    occupancy_grid, width, height, vertical, obstacle_threshold, allow_unknown,
    footprint_polygon, resolution, origin_x, origin_y, 0.0);
}

double heuristic(const GridCell &from, const GridCell &to, int connectivity)
{
  const double dx = std::abs(from.x - to.x);
  const double dy = std::abs(from.y - to.y);
  if (connectivity == 8)
  {
    const double min_delta = std::min(dx, dy);
    const double max_delta = std::max(dx, dy);
    return (min_delta * std::sqrt(2.0)) + (max_delta - min_delta);
  }
  return dx + dy;
}

double turn_penalty(
  const GridCell &previous,
  const GridCell &current,
  const GridCell &next,
  double penalty)
{
  const int previous_dx = current.x - previous.x;
  const int previous_dy = current.y - previous.y;
  const int next_dx = next.x - current.x;
  const int next_dy = next.y - current.y;
  if (previous_dx == next_dx &&previous_dy == next_dy)
  {
    return 0.0;
  }
  return penalty;
}

std::vector<GridCell> get_neighbors(const GridCell &cell, int connectivity)
{
  std::vector<GridCell> neighbors{
    {cell.x + 1, cell.y},
    {cell.x - 1, cell.y},
    {cell.x, cell.y + 1},
    {cell.x, cell.y - 1}
  };

  if (connectivity == 8)
  {
    neighbors.push_back({cell.x + 1, cell.y + 1});
    neighbors.push_back({cell.x + 1, cell.y - 1});
    neighbors.push_back({cell.x - 1, cell.y + 1});
    neighbors.push_back({cell.x - 1, cell.y - 1});
  }

  return neighbors;
}

double grid_path_length(const std::vector<GridCell> &path)
{
  if (path.size() < 2U)
  {
    return 0.0;
  }

  double total_length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index)
  {
    const double dx = static_cast<double>(path[index].x - path[index - 1U].x);
    const double dy = static_cast<double>(path[index].y - path[index - 1U].y);
    total_length += std::sqrt((dx * dx) + (dy * dy));
  }

  return total_length;
}

bool plan_on_grid(
  const std::vector<int8_t> &occupancy_grid,
  int width,
  int height,
  const GridCell &start,
  const GridCell &goal,
  int obstacle_threshold,
  bool allow_unknown,
  int connectivity,
  bool prevent_corner_cutting,
  double penalty,
  const amr::geometry::FootprintPolygon &footprint_polygon,
  double resolution,
  double origin_x,
  double origin_y,
  std::vector<GridCell> &path)
{
  path.clear();

  if (
    width <= 0 || height <= 0 ||
    occupancy_grid.size() != static_cast<std::size_t>(width * height))
  {
    return false;
  }
  if (!is_within_bounds(start, width, height) || !is_within_bounds(goal, width, height))
  {
    return false;
  }
  if (
    is_occupied(
      occupancy_grid, width, height, start, obstacle_threshold, allow_unknown,
      footprint_polygon, resolution, origin_x, origin_y, 0.0) ||
    is_occupied(
      occupancy_grid, width, height, goal, obstacle_threshold, allow_unknown,
      footprint_polygon, resolution, origin_x, origin_y, 0.0))
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

  while (!open_set.empty())
  {
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    AStarNode &current_node = nodes[static_cast<std::size_t>(current_entry.index)];
    if (current_node.closed)
    {
      continue;
    }

    current_node.closed = true;
    if (current_entry.index == goal_index)
    {
      int path_index = goal_index;
      while (path_index >= 0)
      {
        path.push_back({path_index % width, path_index / width});
        path_index = nodes[static_cast<std::size_t>(path_index)].parent_index;
      }
      std::reverse(path.begin(), path.end());
      return true;
    }

    const GridCell current_cell{current_entry.index % width, current_entry.index / width};
    for (const GridCell &neighbor : get_neighbors(current_cell, connectivity))
    {
      if (
        !is_within_bounds(neighbor, width, height) ||
        is_occupied(
          occupancy_grid,
          width,
          height,
          neighbor,
          obstacle_threshold,
          allow_unknown,
          footprint_polygon,
          resolution,
          origin_x,
          origin_y,
          std::atan2(
            static_cast<double>(neighbor.y - current_cell.y),
            static_cast<double>(neighbor.x - current_cell.x))) ||
        is_diagonal_move_blocked(
          occupancy_grid,
          width,
          height,
          current_cell,
          neighbor,
          obstacle_threshold,
          allow_unknown,
          prevent_corner_cutting,
          footprint_polygon,
          resolution,
          origin_x,
          origin_y))
      {
        continue;
      }

      const int neighbor_index = to_index(neighbor, width);
      AStarNode &neighbor_node = nodes[static_cast<std::size_t>(neighbor_index)];
      if (neighbor_node.closed)
      {
        continue;
      }

      const bool diagonal = neighbor.x != current_cell.x &&neighbor.y != current_cell.y;
      double tentative_g_cost = current_node.g_cost + (diagonal ? std::sqrt(2.0) : 1.0);
      if (current_node.parent_index >= 0)
      {
        const GridCell previous{
          current_node.parent_index % width,
          current_node.parent_index / width};
        tentative_g_cost += turn_penalty(previous, current_cell, neighbor, penalty);
      }

      if (!neighbor_node.opened || tentative_g_cost < neighbor_node.g_cost)
      {
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

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("local_planner", options),
  command_topic_(""),
  current_pose_topic_(""),
  map_topic_(""),
  local_plan_topic_(""),
  local_plan_status_topic_(""),
  local_escape_service_name_("/amr/local_planner/plan_local_escape"),
  publish_period_ms_(100),
  lookahead_distance_(0.8),
  goal_tolerance_(0.15),
  obstacle_threshold_(50),
  connectivity_(8),
  allow_unknown_(false),
  prevent_corner_cutting_(true),
  turn_penalty_(0.5),
  path_refiner_enabled_(true),
  path_refiner_prune_distance_(0.03),
  path_refiner_interpolate_distance_(0.15),
  path_refiner_heading_assignment_enabled_(true),
  path_refiner_preserve_goal_orientation_(true),
  path_refiner_line_of_sight_simplification_enabled_(true),
  path_refiner_line_of_sight_sample_distance_(0.05),
  path_refiner_line_of_sight_max_skip_(8),
  path_refiner_collinear_pruning_enabled_(true),
  path_refiner_collinear_angle_threshold_(0.08),
  path_refiner_collinear_lateral_deviation_threshold_(0.015),
  path_refiner_corner_smoothing_enabled_(true),
  path_refiner_corner_smoothing_max_offset_(0.06),
  path_refiner_corner_smoothing_angle_threshold_(0.50),
  path_refiner_corner_smoothing_samples_(2),
  path_refiner_smoothing_max_length_ratio_(1.08),
  path_refiner_smoothing_max_pose_deviation_(0.04),
  path_refiner_collision_check_enabled_(true),
  path_refiner_collision_sample_distance_(0.05),
  dynamic_obstacle_enabled_(true),
  structured_logging_enabled_(true),
  state_log_throttle_sec_(1.0),
  dynamic_obstacle_replan_lookahead_distance_(1.4),
  dynamic_obstacle_escape_forward_distance_(1.2),
  dynamic_obstacle_escape_lateral_distance_(0.55),
  dynamic_obstacle_goal_proximity_disable_distance_(0.45),
  dynamic_obstacle_corridor_relax_distance_(0.75),
  dynamic_obstacle_recovery_confirm_cycles_(3),
  dynamic_obstacle_goal_proximity_confirm_cycles_(2),
  dynamic_obstacle_corridor_confirm_cycles_(5),
  dynamic_obstacle_clear_confirm_cycles_(2),
  nearest_free_search_radius_cells_(4),
  last_command_id_(0U),
  dynamic_blocked_decision_(amr_msgs::msg::LocalPlanStatus::DECISION_OK),
  dynamic_blocked_streak_(0),
  dynamic_clear_streak_(0),
  last_progress_index_(0U),
  map_occupancy_grid_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  has_command_(false),
  has_current_pose_(false),
  has_map_(false)
{
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.costmap", this->map_topic_);
  this->declare_parameter("topics.plan", this->local_plan_topic_);
  this->declare_parameter("topics.status", this->local_plan_status_topic_);
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
  this->declare_parameter("path_refiner.enabled", this->path_refiner_enabled_);
  this->declare_parameter("path_refiner.prune_distance", this->path_refiner_prune_distance_);
  this->declare_parameter("path_refiner.interpolate_distance", this->path_refiner_interpolate_distance_);
  this->declare_parameter(
    "path_refiner.heading_assignment_enabled", this->path_refiner_heading_assignment_enabled_);
  this->declare_parameter(
    "path_refiner.preserve_goal_orientation", this->path_refiner_preserve_goal_orientation_);
  this->declare_parameter(
    "path_refiner.line_of_sight_simplification_enabled",
    this->path_refiner_line_of_sight_simplification_enabled_);
  this->declare_parameter(
    "path_refiner.line_of_sight_sample_distance",
    this->path_refiner_line_of_sight_sample_distance_);
  this->declare_parameter(
    "path_refiner.line_of_sight_max_skip", this->path_refiner_line_of_sight_max_skip_);
  this->declare_parameter(
    "path_refiner.collinear_pruning_enabled", this->path_refiner_collinear_pruning_enabled_);
  this->declare_parameter(
    "path_refiner.collinear_angle_threshold", this->path_refiner_collinear_angle_threshold_);
  this->declare_parameter(
    "path_refiner.collinear_lateral_deviation_threshold",
    this->path_refiner_collinear_lateral_deviation_threshold_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_enabled", this->path_refiner_corner_smoothing_enabled_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_max_offset", this->path_refiner_corner_smoothing_max_offset_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_angle_threshold",
    this->path_refiner_corner_smoothing_angle_threshold_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_samples", this->path_refiner_corner_smoothing_samples_);
  this->declare_parameter(
    "path_refiner.smoothing_max_length_ratio", this->path_refiner_smoothing_max_length_ratio_);
  this->declare_parameter(
    "path_refiner.smoothing_max_pose_deviation", this->path_refiner_smoothing_max_pose_deviation_);
  this->declare_parameter(
    "path_refiner.collision_check_enabled", this->path_refiner_collision_check_enabled_);
  this->declare_parameter(
    "path_refiner.collision_sample_distance", this->path_refiner_collision_sample_distance_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_param_);
  this->declare_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
  this->declare_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->declare_parameter("logging.state_log_throttle_sec", this->state_log_throttle_sec_);
  this->declare_parameter(
    "dynamic_obstacle.replan_lookahead_distance", this->dynamic_obstacle_replan_lookahead_distance_);
  this->declare_parameter(
    "dynamic_obstacle.escape_forward_distance", this->dynamic_obstacle_escape_forward_distance_);
  this->declare_parameter(
    "dynamic_obstacle.escape_lateral_distance", this->dynamic_obstacle_escape_lateral_distance_);
  this->declare_parameter(
    "dynamic_obstacle.goal_proximity_disable_distance",
    this->dynamic_obstacle_goal_proximity_disable_distance_);
  this->declare_parameter(
    "dynamic_obstacle.corridor_relax_distance",
    this->dynamic_obstacle_corridor_relax_distance_);
  this->declare_parameter(
    "dynamic_obstacle.recovery_confirm_cycles",
    this->dynamic_obstacle_recovery_confirm_cycles_);
  this->declare_parameter(
    "dynamic_obstacle.goal_proximity_confirm_cycles",
    this->dynamic_obstacle_goal_proximity_confirm_cycles_);
  this->declare_parameter(
    "dynamic_obstacle.corridor_confirm_cycles",
    this->dynamic_obstacle_corridor_confirm_cycles_);
  this->declare_parameter(
    "dynamic_obstacle.clear_confirm_cycles",
    this->dynamic_obstacle_clear_confirm_cycles_);
}

LocalPlanner::CallbackReturn LocalPlanner::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.costmap", this->map_topic_);
  this->get_parameter("topics.plan", this->local_plan_topic_);
  this->get_parameter("topics.status", this->local_plan_status_topic_);
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
  this->get_parameter("path_refiner.enabled", this->path_refiner_enabled_);
  this->get_parameter("path_refiner.prune_distance", this->path_refiner_prune_distance_);
  this->get_parameter("path_refiner.interpolate_distance", this->path_refiner_interpolate_distance_);
  this->get_parameter(
    "path_refiner.heading_assignment_enabled", this->path_refiner_heading_assignment_enabled_);
  this->get_parameter(
    "path_refiner.preserve_goal_orientation", this->path_refiner_preserve_goal_orientation_);
  this->get_parameter(
    "path_refiner.line_of_sight_simplification_enabled",
    this->path_refiner_line_of_sight_simplification_enabled_);
  this->get_parameter(
    "path_refiner.line_of_sight_sample_distance",
    this->path_refiner_line_of_sight_sample_distance_);
  this->get_parameter(
    "path_refiner.line_of_sight_max_skip", this->path_refiner_line_of_sight_max_skip_);
  this->get_parameter(
    "path_refiner.collinear_pruning_enabled", this->path_refiner_collinear_pruning_enabled_);
  this->get_parameter(
    "path_refiner.collinear_angle_threshold", this->path_refiner_collinear_angle_threshold_);
  this->get_parameter(
    "path_refiner.collinear_lateral_deviation_threshold",
    this->path_refiner_collinear_lateral_deviation_threshold_);
  this->get_parameter(
    "path_refiner.corner_smoothing_enabled", this->path_refiner_corner_smoothing_enabled_);
  this->get_parameter(
    "path_refiner.corner_smoothing_max_offset", this->path_refiner_corner_smoothing_max_offset_);
  this->get_parameter(
    "path_refiner.corner_smoothing_angle_threshold",
    this->path_refiner_corner_smoothing_angle_threshold_);
  this->get_parameter(
    "path_refiner.corner_smoothing_samples", this->path_refiner_corner_smoothing_samples_);
  this->get_parameter(
    "path_refiner.smoothing_max_length_ratio", this->path_refiner_smoothing_max_length_ratio_);
  this->get_parameter(
    "path_refiner.smoothing_max_pose_deviation", this->path_refiner_smoothing_max_pose_deviation_);
  this->get_parameter(
    "path_refiner.collision_check_enabled", this->path_refiner_collision_check_enabled_);
  this->get_parameter(
    "path_refiner.collision_sample_distance", this->path_refiner_collision_sample_distance_);
  this->get_parameter("footprint.polygon", this->footprint_polygon_param_);
  this->get_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->get_parameter("logging.state_log_throttle_sec", this->state_log_throttle_sec_);
  this->get_parameter(
    "dynamic_obstacle.replan_lookahead_distance", this->dynamic_obstacle_replan_lookahead_distance_);
  this->get_parameter(
    "dynamic_obstacle.escape_forward_distance", this->dynamic_obstacle_escape_forward_distance_);
  this->get_parameter(
    "dynamic_obstacle.escape_lateral_distance", this->dynamic_obstacle_escape_lateral_distance_);
  this->get_parameter(
    "dynamic_obstacle.goal_proximity_disable_distance",
    this->dynamic_obstacle_goal_proximity_disable_distance_);
  this->get_parameter(
    "dynamic_obstacle.corridor_relax_distance",
    this->dynamic_obstacle_corridor_relax_distance_);
  this->get_parameter(
    "dynamic_obstacle.recovery_confirm_cycles",
    this->dynamic_obstacle_recovery_confirm_cycles_);
  this->get_parameter(
    "dynamic_obstacle.goal_proximity_confirm_cycles",
    this->dynamic_obstacle_goal_proximity_confirm_cycles_);
  this->get_parameter(
    "dynamic_obstacle.corridor_confirm_cycles",
    this->dynamic_obstacle_corridor_confirm_cycles_);
  this->get_parameter(
    "dynamic_obstacle.clear_confirm_cycles",
    this->dynamic_obstacle_clear_confirm_cycles_);

  if (
    this->command_topic_.empty() || this->current_pose_topic_.empty() ||
    this->map_topic_.empty() || this->local_plan_topic_.empty() ||
    this->local_plan_status_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Local planner topics must not be empty: command='%s' pose='%s' costmap='%s' local_plan='%s' local_plan_status='%s'",
      this->command_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->map_topic_.c_str(),
      this->local_plan_topic_.c_str(),
      this->local_plan_status_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->footprint_polygon_ = amr::geometry::make_footprint_polygon(this->footprint_polygon_param_);

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
  this->local_plan_status_publisher_ = this->create_publisher<amr_msgs::msg::LocalPlanStatus>(
    this->local_plan_status_topic_, rclcpp::SystemDefaultsQoS());
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(this->publish_period_ms_),
    [this]() { this->publish_local_plan(); });
  this->timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(),
    "Configured local planner with command='%s', pose='%s', costmap='%s', plan='%s', status='%s', lookahead=%.2f",
    this->command_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->map_topic_.c_str(),
    this->local_plan_topic_.c_str(),
    this->local_plan_status_topic_.c_str(),
    this->lookahead_distance_);

  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->local_plan_publisher_->on_activate();
  this->local_plan_status_publisher_->on_activate();
  this->timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->timer_)
  {
    this->timer_->cancel();
  }
  if (this->local_plan_publisher_)
  {
    this->local_plan_publisher_->on_deactivate();
  }
  if (this->local_plan_status_publisher_)
  {
    this->local_plan_status_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->map_subscription_.reset();
  this->local_escape_service_.reset();
  this->local_plan_publisher_.reset();
  this->local_plan_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->inflated_map_ = nav_msgs::msg::OccupancyGrid();
  this->working_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->last_command_id_ = 0U;
  this->dynamic_blocked_decision_ = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
  this->dynamic_blocked_streak_ = 0;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  this->has_map_ = false;
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->map_subscription_.reset();
  this->local_escape_service_.reset();
  this->local_plan_publisher_.reset();
  this->local_plan_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->inflated_map_ = nav_msgs::msg::OccupancyGrid();
  this->working_costmap_ = nav_msgs::msg::OccupancyGrid();
  this->last_command_id_ = 0U;
  this->dynamic_blocked_decision_ = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
  this->dynamic_blocked_streak_ = 0;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  this->has_map_ = false;
  return CallbackReturn::SUCCESS;
}

void LocalPlanner::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  if (message->command_id != this->last_command_id_)
  {
    this->last_progress_index_ = 0U;
    this->last_command_id_ = message->command_id;
    this->reset_dynamic_blocked_state();
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

void LocalPlanner::handle_plan_local_escape(
  const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response)
{
  if (!response)
  {
    return;
  }

  response->success = false;
  response->plan = nav_msgs::msg::Path();

  if (!this->has_map_ || this->inflated_map_.data.empty())
  {
    response->message = "local_escape_map_unavailable";
    return;
  }

  if (request->source_plan.poses.empty())
  {
    response->message = "local_escape_empty_source_plan";
    return;
  }

  const std::size_t closest_index =
    this->find_closest_pose_index(request->source_plan, request->current_pose, 0U);
  const nav_msgs::msg::Path escape_plan = this->build_inflated_local_plan(
    request->source_plan,
    request->current_pose,
    closest_index,
    std::max(this->lookahead_distance_, this->dynamic_obstacle_replan_lookahead_distance_));

  if (escape_plan.poses.size() < 2U)
  {
    response->message = "local_escape_no_valid_path";
    return;
  }

  response->success = true;
  response->plan = this->refine_local_plan(escape_plan);
  response->message = "local_escape_plan_ready";
}

void LocalPlanner::publish_local_plan()
{
  if (
    !this->local_plan_publisher_ || !this->local_plan_publisher_->is_activated() ||
    !this->local_plan_status_publisher_ || !this->local_plan_status_publisher_->is_activated() ||
    !this->has_command_ || !this->has_current_pose_)
  {
    return;
  }

  LocalPlanBuildResult build_result = this->build_local_plan(this->latest_command_, this->current_pose_);
  LocalPathQualityMetrics path_quality;
  if (build_result.local_plan_valid)
  {
    build_result.plan = this->refine_local_plan(build_result.plan, &path_quality);
  }
  this->local_plan_publisher_->publish(build_result.plan);

  amr_msgs::msg::LocalPlanStatus status;
  status.header.stamp = this->now();
  status.header.frame_id =
    build_result.plan.header.frame_id.empty() ? this->current_pose_.header.frame_id :
    build_result.plan.header.frame_id;
  status.command_id = this->latest_command_.command_id;
  status.active = this->has_command_;
  status.local_plan_valid = build_result.local_plan_valid;
  status.recovery_required = build_result.recovery_required;
  status.decision = build_result.decision;
  status.has_blocked_pose = build_result.has_blocked_pose;
  status.blocked_pose = build_result.blocked_pose;
  status.blocked_distance = build_result.blocked_distance;
  this->local_plan_status_publisher_->publish(status);

  if (this->structured_logging_enabled_)
  {
    if (build_result.local_plan_valid)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        throttle_ms_from_sec(this->state_log_throttle_sec_),
        "AMR_LOG schema=v1 component=controller event=local_path_quality node=local_planner goal_id=%u raw_path_points=%zu simplified_path_points=%zu refined_path_points=%zu path_length_m=%.3f path_curvature_score=%.3f lateral_error_m=%.3f line_of_sight_simplified=%s collinear_pruned_count=%d collision_check_passed=%s",
        this->latest_command_.command_id,
        path_quality.raw_path_points,
        path_quality.simplified_path_points,
        path_quality.refined_path_points,
        path_quality.path_length_m,
        path_quality.path_curvature_score,
        path_quality.lateral_error_m,
        bool_label(path_quality.line_of_sight_simplified),
        path_quality.collinear_pruned_count,
        bool_label(path_quality.collision_check_passed));
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      throttle_ms_from_sec(this->state_log_throttle_sec_),
      "AMR_LOG schema=v1 component=controller event=local_blocked_state node=local_planner goal_id=%u target_idx=%zu path_points=%zu decision=%s recovery=%s blocked=%s blocked_distance_m=%.3f blocked_streak=%d clear_streak=%d",
      this->latest_command_.command_id,
      this->last_progress_index_,
      build_result.plan.poses.size(),
      local_plan_decision_label(build_result.decision),
      bool_label(build_result.recovery_required),
      bool_label(build_result.has_blocked_pose),
      build_result.blocked_distance,
      this->dynamic_blocked_streak_,
      this->dynamic_clear_streak_);
  }
}

LocalPlanner::LocalPlanBuildResult LocalPlanner::build_local_plan(
  const amr_msgs::msg::MotionCommand &command,
  const geometry_msgs::msg::PoseStamped &current_pose)
{
  LocalPlanBuildResult result;
  const nav_msgs::msg::Path source_plan = this->build_source_plan(command);
  result.plan.header = source_plan.header;
  if (result.plan.header.frame_id.empty())
  {
    result.plan.header.frame_id = current_pose.header.frame_id;
  }
  result.plan.header.stamp = this->now();

  if (source_plan.poses.empty())
  {
    this->reset_dynamic_blocked_state();
    result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED;
    return result;
  }

  const geometry_msgs::msg::PoseStamped &goal_pose = source_plan.poses.back();
  const double goal_distance = this->pose_distance(current_pose, goal_pose);
  if (this->pose_distance(current_pose, goal_pose) <= this->goal_tolerance_)
  {
    this->reset_dynamic_blocked_state();
    this->last_progress_index_ = source_plan.poses.size() - 1U;
    result.plan.poses.push_back(goal_pose);
    result.local_plan_valid = true;
    result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
    return result;
  }

  const std::size_t closest_index =
    this->find_closest_pose_index(source_plan, current_pose, this->last_progress_index_);
  this->last_progress_index_ = closest_index;
  geometry_msgs::msg::PoseStamped blocked_pose;
  const nav_msgs::msg::Path sliced_plan = this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    std::max(this->lookahead_distance_, this->dynamic_obstacle_replan_lookahead_distance_));
  const bool obstacle_active =
    this->dynamic_obstacle_enabled_ &&
    this->find_first_blocked_pose_on_plan(sliced_plan, blocked_pose);
  const double replan_lookahead_distance = obstacle_active ?
    std::max(this->lookahead_distance_, this->dynamic_obstacle_replan_lookahead_distance_) :
    this->lookahead_distance_;

  if (this->has_map_ &&!this->inflated_map_.data.empty())
  {
    result.plan = this->build_inflated_local_plan(
      source_plan,
      current_pose,
      closest_index,
      replan_lookahead_distance);
    if (!result.plan.poses.empty())
    {
      result.local_plan_valid = true;
      geometry_msgs::msg::PoseStamped final_blocked_pose;
      if (this->find_first_blocked_pose_on_plan(result.plan, final_blocked_pose))
      {
        const uint8_t blocked_decision =
          goal_distance <= this->dynamic_obstacle_goal_proximity_disable_distance_ ?
          amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED :
          amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED;
        result.decision = blocked_decision;
        result.has_blocked_pose = true;
        result.blocked_pose = final_blocked_pose;
        result.blocked_distance = this->pose_distance(current_pose, final_blocked_pose);
        result.recovery_required =
          this->confirm_dynamic_recovery_decision(blocked_decision, result.blocked_distance);
        if (!result.recovery_required)
        {
          result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
        }
      }
      else if (obstacle_active)
      {
        this->confirm_dynamic_clear();
        result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
        result.has_blocked_pose = true;
        result.blocked_pose = blocked_pose;
        result.blocked_distance = this->pose_distance(current_pose, blocked_pose);
      }
      else
      {
        this->confirm_dynamic_clear();
        result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
      }
      return result;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Inflated local replanning failed; falling back to sliced local plan");
  }

  if (!obstacle_active)
  {
    this->confirm_dynamic_clear();
    result.plan = this->build_sliced_local_plan_with_lookahead(
      source_plan,
      current_pose,
      closest_index,
      replan_lookahead_distance);
    result.local_plan_valid = !result.plan.poses.empty();
    result.decision = result.local_plan_valid ?
      amr_msgs::msg::LocalPlanStatus::DECISION_OK :
      amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED;
    return result;
  }

  result.plan = sliced_plan;
  result.local_plan_valid = !result.plan.poses.empty();
  const uint8_t blocked_decision =
    goal_distance <= this->dynamic_obstacle_goal_proximity_disable_distance_ ?
    amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED :
    amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED;
  result.decision = blocked_decision;
  result.has_blocked_pose = true;
  result.blocked_pose = blocked_pose;
  result.blocked_distance = this->pose_distance(current_pose, blocked_pose);
  result.recovery_required =
    this->confirm_dynamic_recovery_decision(blocked_decision, result.blocked_distance);
  if (!result.recovery_required)
  {
    result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
  }
  return result;
}

nav_msgs::msg::Path LocalPlanner::build_inflated_local_plan(
  const nav_msgs::msg::Path &source_plan,
  const geometry_msgs::msg::PoseStamped &current_pose,
  const std::size_t closest_index,
  const double lookahead_distance)
{
  const nav_msgs::msg::Path sliced_plan = this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    lookahead_distance);
  if (sliced_plan.poses.size() < 2U || !this->has_map_)
  {
    return sliced_plan;
  }

  geometry_msgs::msg::PoseStamped blocked_pose;
  const bool blocked =
    this->dynamic_obstacle_enabled_ &&
    this->find_first_blocked_pose_on_plan(sliced_plan, blocked_pose);

  if (!blocked)
  {
    this->working_costmap_ = this->inflated_map_;
    return sliced_plan;
  }

  const double goal_distance = this->pose_distance(current_pose, sliced_plan.poses.back());
  if (goal_distance <= this->dynamic_obstacle_goal_proximity_disable_distance_)
  {
    this->working_costmap_ = this->inflated_map_;
    return sliced_plan;
  }

  this->working_costmap_ = this->inflated_map_;
  const nav_msgs::msg::OccupancyGrid &working_map = this->working_costmap_;

  const int width = static_cast<int>(working_map.info.width);
  const int height = static_cast<int>(working_map.info.height);
  int start_x = 0;
  int start_y = 0;
  if (!this->world_to_grid(current_pose.pose.position, start_x, start_y))
  {
    return sliced_plan;
  }
  if (!this->find_nearest_free_cell(
      working_map.data, width, height, start_x, start_y,
      this->nearest_free_search_radius_cells_,
      yaw_from_quaternion(current_pose.pose.orientation)))
  {
    return sliced_plan;
  }

  const std::function<bool(const GridCell &, const GridCell &, std::vector<GridCell> &)> plan_segment =
    [&](const GridCell &segment_start,
      const GridCell &segment_goal,
      std::vector<GridCell> &grid_path) -> bool
    {
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
        this->footprint_polygon_,
        working_map.info.resolution,
        working_map.info.origin.position.x,
        working_map.info.origin.position.y,
        grid_path);
    };

  const std::function<bool(const geometry_msgs::msg::PoseStamped &, std::vector<GridCell> &)> build_grid_plan_to_pose =
    [&](const geometry_msgs::msg::PoseStamped &goal_pose,
      std::vector<GridCell> &grid_path) -> bool
    {
      int goal_x = 0;
      int goal_y = 0;
      if (!this->world_to_grid(goal_pose.pose.position, goal_x, goal_y))
      {
        return false;
      }
      if (!this->find_nearest_free_cell(
          working_map.data, width, height, goal_x, goal_y,
          this->nearest_free_search_radius_cells_,
          yaw_from_quaternion(goal_pose.pose.orientation)))
      {
        return false;
      }
      return plan_segment({start_x, start_y}, {goal_x, goal_y}, grid_path);
    };

  std::vector<GridCell> best_grid_path;
  if (blocked)
  {
    const geometry_msgs::msg::PoseStamped &rejoin_pose = sliced_plan.poses.back();
    int rejoin_x = 0;
    int rejoin_y = 0;
    if (
      this->world_to_grid(rejoin_pose.pose.position, rejoin_x, rejoin_y) &&
      this->find_nearest_free_cell(
        working_map.data, width, height, rejoin_x, rejoin_y,
        this->nearest_free_search_radius_cells_,
        yaw_from_quaternion(rejoin_pose.pose.orientation)))
    {
      const double current_yaw = std::atan2(
        2.0 * (
          current_pose.pose.orientation.w * current_pose.pose.orientation.z +
          current_pose.pose.orientation.x * current_pose.pose.orientation.y),
        1.0 - 2.0 * (
          current_pose.pose.orientation.y * current_pose.pose.orientation.y +
          current_pose.pose.orientation.z * current_pose.pose.orientation.z));
      const double goal_heading_dx =
        rejoin_pose.pose.position.x - current_pose.pose.position.x;
      const double goal_heading_dy =
        rejoin_pose.pose.position.y - current_pose.pose.position.y;
      double path_heading = current_yaw;
      if ((goal_heading_dx * goal_heading_dx) + (goal_heading_dy * goal_heading_dy) > 1e-6)
      {
        path_heading = std::atan2(goal_heading_dy, goal_heading_dx);
      }
      else if (sliced_plan.poses.size() >= 2U)
      {
        const geometry_msgs::msg::PoseStamped &heading_target = sliced_plan.poses[1U];
        const double heading_dx =
          heading_target.pose.position.x - current_pose.pose.position.x;
        const double heading_dy =
          heading_target.pose.position.y - current_pose.pose.position.y;
        if ((heading_dx * heading_dx) + (heading_dy * heading_dy) > 1e-6)
        {
          path_heading = std::atan2(heading_dy, heading_dx);
        }
      }
      const double forward_distance = std::max(
        this->dynamic_obstacle_escape_forward_distance_,
        this->pose_distance(current_pose, blocked_pose) + 0.15);
      const double blocked_dx = blocked_pose.pose.position.x - current_pose.pose.position.x;
      const double blocked_dy = blocked_pose.pose.position.y - current_pose.pose.position.y;
      const double blocked_side =
        (std::cos(path_heading) * blocked_dy) - (std::sin(path_heading) * blocked_dx);
      double preferred_sign = blocked_side >= 0.0 ? -1.0 : 1.0;
      const double left_occupancy = this->sample_lateral_occupancy(
        current_pose.pose.position.x, current_pose.pose.position.y, path_heading, 1.0);
      const double right_occupancy = this->sample_lateral_occupancy(
        current_pose.pose.position.x, current_pose.pose.position.y, path_heading, -1.0);
      if (std::abs(left_occupancy - right_occupancy) > 0.25)
      {
        preferred_sign = left_occupancy <= right_occupancy ? 1.0 : -1.0;
      }
      const std::vector<double> escape_signs{preferred_sign, -preferred_sign};
      double best_score = std::numeric_limits<double>::max();

      for (const double sign : escape_signs)
      {
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
        if (!this->world_to_grid(escape_pose.pose.position, escape_x, escape_y))
        {
          continue;
        }
        if (!this->find_nearest_free_cell(
            working_map.data, width, height, escape_x, escape_y,
            this->nearest_free_search_radius_cells_,
            path_heading))
        {
          continue;
        }

        std::vector<GridCell> escape_path;
        if (!plan_segment({start_x, start_y}, {escape_x, escape_y}, escape_path) || escape_path.empty())
        {
          continue;
        }

        std::vector<GridCell> rejoin_path;
        if (!plan_segment({escape_x, escape_y}, {rejoin_x, rejoin_y}, rejoin_path) || rejoin_path.empty())
        {
          continue;
        }

        std::vector<GridCell> combined_path = escape_path;
        combined_path.insert(combined_path.end(), rejoin_path.begin() + 1, rejoin_path.end());
        const double escape_goal_dx =
          escape_pose.pose.position.x - rejoin_pose.pose.position.x;
        const double escape_goal_dy =
          escape_pose.pose.position.y - rejoin_pose.pose.position.y;
        const double lateral_alignment_penalty =
          std::abs(
          (-std::sin(path_heading) * escape_goal_dx) +
          (std::cos(path_heading) * escape_goal_dy));
        const double score =
          grid_path_length(combined_path) + (lateral_alignment_penalty * 2.0);
        if (score < best_score)
        {
          best_score = score;
          best_grid_path = std::move(combined_path);
        }
      }
    }
  }

  if (best_grid_path.empty() &&!build_grid_plan_to_pose(sliced_plan.poses.back(), best_grid_path))
  {
    return sliced_plan;
  }
  if (best_grid_path.empty())
  {
    return sliced_plan;
  }

  nav_msgs::msg::Path local_plan;
  local_plan.header = sliced_plan.header;
  local_plan.header.stamp = this->now();
  local_plan.poses.push_back(current_pose);
  for (std::size_t index = 1; index < best_grid_path.size(); ++index)
  {
    local_plan.poses.push_back(
      this->grid_to_pose(
        best_grid_path[index].x,
        best_grid_path[index].y,
        local_plan.header.frame_id));
  }

  if (local_plan.poses.size() == 1U)
  {
    local_plan.poses.push_back(sliced_plan.poses.back());
  }

  return local_plan;
}

bool LocalPlanner::find_first_blocked_pose_on_plan(
  const nav_msgs::msg::Path &plan,
  geometry_msgs::msg::PoseStamped &blocked_pose) const
{
  if (!this->has_map_ || this->inflated_map_.data.empty() || plan.poses.size() < 2U)
  {
    return false;
  }

  for (std::size_t index = 1; index < plan.poses.size(); ++index)
  {
    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_grid(plan.poses[index].pose.position, grid_x, grid_y))
    {
      continue;
    }
    if (this->is_grid_pose_collision(
        this->inflated_map_.data,
        static_cast<int>(this->inflated_map_.info.width),
        static_cast<int>(this->inflated_map_.info.height),
        grid_x,
        grid_y,
        yaw_from_quaternion(plan.poses[index].pose.orientation)))
    {
      blocked_pose = plan.poses[index];
      return true;
    }
  }

  return false;
}

void LocalPlanner::reset_dynamic_blocked_state()
{
  this->dynamic_blocked_decision_ = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
  this->dynamic_blocked_streak_ = 0;
  this->dynamic_clear_streak_ = 0;
}

bool LocalPlanner::confirm_dynamic_recovery_decision(uint8_t decision, double blocked_distance)
{
  if (decision == amr_msgs::msg::LocalPlanStatus::DECISION_OK)
  {
    this->reset_dynamic_blocked_state();
    return false;
  }

  if (decision != this->dynamic_blocked_decision_)
  {
    this->dynamic_blocked_decision_ = decision;
    this->dynamic_blocked_streak_ = 1;
  }
  else
  {
    this->dynamic_blocked_streak_ += 1;
  }

  const int required_cycles = this->required_dynamic_recovery_cycles(decision, blocked_distance);
  return this->dynamic_blocked_streak_ >= required_cycles;
}

bool LocalPlanner::confirm_dynamic_clear()
{
  if (this->dynamic_blocked_decision_ == amr_msgs::msg::LocalPlanStatus::DECISION_OK)
  {
    this->dynamic_clear_streak_ = 0;
    return true;
  }

  this->dynamic_clear_streak_ += 1;
  if (this->dynamic_clear_streak_ < std::max(1, this->dynamic_obstacle_clear_confirm_cycles_))
  {
    return false;
  }

  this->reset_dynamic_blocked_state();
  return true;
}

int LocalPlanner::required_dynamic_recovery_cycles(uint8_t decision, double blocked_distance) const
{
  if (decision == amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED)
  {
    return std::max(1, this->dynamic_obstacle_goal_proximity_confirm_cycles_);
  }

  if (
    blocked_distance >= this->dynamic_obstacle_corridor_relax_distance_ &&
    this->dynamic_obstacle_corridor_confirm_cycles_ > this->dynamic_obstacle_recovery_confirm_cycles_)
  {
    return std::max(1, this->dynamic_obstacle_corridor_confirm_cycles_);
  }

  return std::max(1, this->dynamic_obstacle_recovery_confirm_cycles_);
}

double LocalPlanner::sample_lateral_occupancy(
  double origin_x,
  double origin_y,
  double heading,
  double lateral_sign) const
{
  if (!this->has_map_ || this->inflated_map_.data.empty())
  {
    return 0.0;
  }

  const nav_msgs::msg::OccupancyGrid &map = this->inflated_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  double score = 0.0;

  for (double forward = 0.10; forward <= this->dynamic_obstacle_escape_forward_distance_; forward += 0.10)
  {
    for (double lateral = 0.05; lateral <= this->dynamic_obstacle_escape_lateral_distance_; lateral += 0.05)
    {
      geometry_msgs::msg::Point sample;
      sample.x =
        origin_x +
        (std::cos(heading) * forward) -
        (std::sin(heading) * lateral_sign * lateral);
      sample.y =
        origin_y +
        (std::sin(heading) * forward) +
        (std::cos(heading) * lateral_sign * lateral);
      int grid_x = 0;
      int grid_y = 0;
      if (!this->world_to_grid(sample, grid_x, grid_y))
      {
        score += 5.0;
        continue;
      }
      const int index = (grid_y * width) + grid_x;
      if (index < 0 || index >= width * height)
      {
        score += 5.0;
        continue;
      }
      const int8_t cell_value = map.data[static_cast<std::size_t>(index)];
      if (cell_value == kUnknownCellValue)
      {
        score += 1.0;
      }
      else
      {
        score += static_cast<double>(std::max(0, static_cast<int>(cell_value))) / 100.0;
      }
    }
  }

  return score;
}

nav_msgs::msg::Path LocalPlanner::build_sliced_local_plan(
  const nav_msgs::msg::Path &source_plan,
  const geometry_msgs::msg::PoseStamped &current_pose,
  const std::size_t closest_index) const
{
  return this->build_sliced_local_plan_with_lookahead(
    source_plan,
    current_pose,
    closest_index,
    this->lookahead_distance_);
}

nav_msgs::msg::Path LocalPlanner::build_sliced_local_plan_with_lookahead(
  const nav_msgs::msg::Path &source_plan,
  const geometry_msgs::msg::PoseStamped &current_pose,
  const std::size_t closest_index,
  const double lookahead_distance) const
{
  nav_msgs::msg::Path local_plan;
  local_plan.header = source_plan.header;
  if (local_plan.header.frame_id.empty())
  {
    local_plan.header.frame_id = current_pose.header.frame_id;
  }
  local_plan.header.stamp = this->now();
  const geometry_msgs::msg::PoseStamped &source_head = source_plan.poses[closest_index];
  const bool source_head_close =
    this->pose_distance(current_pose, source_head) <=
    std::max(0.03, this->path_refiner_smoothing_max_pose_deviation_);
  if (source_head_close)
  {
    local_plan.poses.push_back(source_head);
  }
  else
  {
    local_plan.poses.push_back(current_pose);
  }

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped segment_start = local_plan.poses.back();
  for (std::size_t index = closest_index; index < source_plan.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &target_pose = source_plan.poses[index];
    const double segment_distance = this->pose_distance(segment_start, target_pose);

    if (segment_distance <= 1e-6)
    {
      segment_start = target_pose;
      continue;
    }

    if (accumulated_distance + segment_distance >= lookahead_distance)
    {
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

nav_msgs::msg::Path LocalPlanner::refine_local_plan(
  const nav_msgs::msg::Path &plan,
  LocalPathQualityMetrics *quality_metrics) const
{
  if (quality_metrics != nullptr)
  {
    quality_metrics->raw_path_points = plan.poses.size();
    quality_metrics->simplified_path_points = plan.poses.size();
    quality_metrics->refined_path_points = plan.poses.size();
    quality_metrics->path_length_m = this->estimate_path_length(plan);
    quality_metrics->path_curvature_score = this->estimate_path_curvature_score(plan);
    quality_metrics->lateral_error_m = this->estimate_path_lateral_error(plan);
    quality_metrics->line_of_sight_simplified = false;
    quality_metrics->collinear_pruned_count = 0;
    quality_metrics->collision_check_passed =
      !this->path_refiner_collision_check_enabled_ || this->is_path_collision_free(plan);
  }

  if (!this->path_refiner_enabled_ || plan.poses.size() < 2U)
  {
    return plan;
  }

  bool line_of_sight_simplified = false;
  nav_msgs::msg::Path simplified_plan = this->simplify_path_line_of_sight(
    plan,
    line_of_sight_simplified);
  int collinear_pruned_count = 0;
  simplified_plan = this->prune_collinear_path(simplified_plan, collinear_pruned_count);
  if (quality_metrics != nullptr)
  {
    quality_metrics->line_of_sight_simplified = line_of_sight_simplified;
    quality_metrics->collinear_pruned_count = collinear_pruned_count;
    quality_metrics->simplified_path_points = simplified_plan.poses.size();
  }

  nav_msgs::msg::Path refined_plan = this->prune_and_interpolate_path(simplified_plan);
  if (this->path_refiner_heading_assignment_enabled_)
  {
    this->assign_path_headings(refined_plan);
  }

  if (this->path_refiner_corner_smoothing_enabled_)
  {
    nav_msgs::msg::Path smoothed_plan = this->apply_path_smoother(refined_plan);
    if (this->path_refiner_heading_assignment_enabled_)
    {
      this->assign_path_headings(smoothed_plan);
    }

    if (this->is_smoothed_path_acceptable(refined_plan, smoothed_plan))
    {
      if (quality_metrics != nullptr)
      {
        quality_metrics->refined_path_points = smoothed_plan.poses.size();
        quality_metrics->path_length_m = this->estimate_path_length(smoothed_plan);
        quality_metrics->path_curvature_score = this->estimate_path_curvature_score(smoothed_plan);
        quality_metrics->lateral_error_m = this->estimate_path_lateral_error(smoothed_plan);
        quality_metrics->collision_check_passed =
          !this->path_refiner_collision_check_enabled_ || this->is_path_collision_free(smoothed_plan);
      }
      return smoothed_plan;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Path refiner rejected smoothed path because the acceptance checks failed");
  }

  if (quality_metrics != nullptr)
  {
    quality_metrics->refined_path_points = refined_plan.poses.size();
    quality_metrics->path_length_m = this->estimate_path_length(refined_plan);
    quality_metrics->path_curvature_score = this->estimate_path_curvature_score(refined_plan);
    quality_metrics->lateral_error_m = this->estimate_path_lateral_error(refined_plan);
    quality_metrics->collision_check_passed =
      !this->path_refiner_collision_check_enabled_ || this->is_path_collision_free(refined_plan);
  }
  return refined_plan;
}

nav_msgs::msg::Path LocalPlanner::simplify_path_line_of_sight(
  const nav_msgs::msg::Path &plan,
  bool &line_of_sight_simplified) const
{
  line_of_sight_simplified = false;
  if (
    !this->path_refiner_line_of_sight_simplification_enabled_ ||
    plan.poses.size() < 3U ||
    !this->has_map_ || !this->map_occupancy_grid_ || this->map_occupancy_grid_->data.empty())
  {
    return plan;
  }

  nav_msgs::msg::Path simplified_plan;
  simplified_plan.header = plan.header;
  simplified_plan.header.stamp = this->now();
  simplified_plan.poses.reserve(plan.poses.size());
  simplified_plan.poses.push_back(plan.poses.front());

  const std::size_t max_skip = static_cast<std::size_t>(
    std::max(1, this->path_refiner_line_of_sight_max_skip_));
  const double sample_distance = std::max(this->path_refiner_line_of_sight_sample_distance_, 1e-3);
  std::size_t current_index = 0U;
  while (current_index + 1U < plan.poses.size())
  {
    std::size_t selected_index = current_index + 1U;
    const std::size_t max_candidate_index = std::min(
      plan.poses.size() - 1U,
      current_index + max_skip);
    for (std::size_t candidate_index = max_candidate_index; candidate_index > current_index + 1U; --candidate_index)
    {
      if (this->is_path_segment_collision_free(
          plan.poses[current_index],
          plan.poses[candidate_index],
          sample_distance))
      {
        selected_index = candidate_index;
        line_of_sight_simplified = true;
        break;
      }
    }

    simplified_plan.poses.push_back(plan.poses[selected_index]);
    current_index = selected_index;
  }

  return simplified_plan;
}

nav_msgs::msg::Path LocalPlanner::prune_collinear_path(
  const nav_msgs::msg::Path &plan,
  int &collinear_pruned_count) const
{
  collinear_pruned_count = 0;
  if (!this->path_refiner_collinear_pruning_enabled_ || plan.poses.size() < 3U)
  {
    return plan;
  }

  nav_msgs::msg::Path pruned_plan;
  pruned_plan.header = plan.header;
  pruned_plan.header.stamp = this->now();
  pruned_plan.poses.reserve(plan.poses.size());
  pruned_plan.poses.push_back(plan.poses.front());

  const double angle_threshold = std::max(0.0, this->path_refiner_collinear_angle_threshold_);
  const double lateral_threshold = std::max(
    0.0,
    this->path_refiner_collinear_lateral_deviation_threshold_);
  const double sample_distance = std::max(this->path_refiner_line_of_sight_sample_distance_, 1e-3);

  for (std::size_t index = 1U; index + 1U < plan.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &previous_pose = pruned_plan.poses.back();
    const geometry_msgs::msg::PoseStamped &current_pose = plan.poses[index];
    const geometry_msgs::msg::PoseStamped &next_pose = plan.poses[index + 1U];
    const double prev_dx = current_pose.pose.position.x - previous_pose.pose.position.x;
    const double prev_dy = current_pose.pose.position.y - previous_pose.pose.position.y;
    const double next_dx = next_pose.pose.position.x - current_pose.pose.position.x;
    const double next_dy = next_pose.pose.position.y - current_pose.pose.position.y;
    const double prev_distance = std::sqrt((prev_dx * prev_dx) + (prev_dy * prev_dy));
    const double next_distance = std::sqrt((next_dx * next_dx) + (next_dy * next_dy));
    if (prev_distance <= 1e-6 || next_distance <= 1e-6)
    {
      ++collinear_pruned_count;
      continue;
    }

    const double dot = std::clamp(
      ((prev_dx / prev_distance) * (next_dx / next_distance)) +
      ((prev_dy / prev_distance) * (next_dy / next_distance)),
      -1.0,
      1.0);
    const double angle = std::acos(dot);
    const double line_dx = next_pose.pose.position.x - previous_pose.pose.position.x;
    const double line_dy = next_pose.pose.position.y - previous_pose.pose.position.y;
    const double line_length = std::sqrt((line_dx * line_dx) + (line_dy * line_dy));
    double lateral_deviation = 0.0;
    if (line_length > 1e-6)
    {
      const double point_dx = current_pose.pose.position.x - previous_pose.pose.position.x;
      const double point_dy = current_pose.pose.position.y - previous_pose.pose.position.y;
      lateral_deviation = std::abs((point_dx * line_dy) - (point_dy * line_dx)) / line_length;
    }

    const bool near_collinear = angle <= angle_threshold && lateral_deviation <= lateral_threshold;
    const bool safe_to_skip =
      near_collinear &&
      this->is_path_segment_collision_free(previous_pose, next_pose, sample_distance);
    if (safe_to_skip)
    {
      ++collinear_pruned_count;
      continue;
    }

    pruned_plan.poses.push_back(current_pose);
  }

  pruned_plan.poses.push_back(plan.poses.back());
  if (this->path_refiner_collision_check_enabled_ && !this->is_path_collision_free(pruned_plan))
  {
    collinear_pruned_count = 0;
    return plan;
  }

  return pruned_plan;
}

nav_msgs::msg::Path LocalPlanner::prune_and_interpolate_path(const nav_msgs::msg::Path &plan) const
{
  nav_msgs::msg::Path refined_plan;
  refined_plan.header = plan.header;
  refined_plan.header.stamp = this->now();
  refined_plan.poses.reserve(plan.poses.size());
  refined_plan.poses.push_back(plan.poses.front());

  for (std::size_t index = 1U; index < plan.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &target_pose = plan.poses[index];
    const bool final_pose = index == plan.poses.size() - 1U;
    const geometry_msgs::msg::PoseStamped last_pose = refined_plan.poses.back();
    const double segment_distance = this->pose_distance(last_pose, target_pose);

    if (!final_pose &&segment_distance < this->path_refiner_prune_distance_)
    {
      continue;
    }

    if (
      this->path_refiner_interpolate_distance_ > 1e-6 &&
      segment_distance > this->path_refiner_interpolate_distance_)
    {
      const int segment_count = static_cast<int>(
        std::floor(segment_distance / this->path_refiner_interpolate_distance_));
      for (int segment_index = 1; segment_index <= segment_count; ++segment_index)
      {
        const double ratio = std::clamp(
          (static_cast<double>(segment_index) * this->path_refiner_interpolate_distance_) /
          segment_distance,
          0.0,
          1.0);
        if (ratio >= 1.0)
        {
          continue;
        }
        refined_plan.poses.push_back(this->interpolate_pose(last_pose, target_pose, ratio));
      }
    }

    refined_plan.poses.push_back(target_pose);
  }

  return refined_plan;
}

nav_msgs::msg::Path LocalPlanner::apply_path_smoother(const nav_msgs::msg::Path &plan) const
{
  return this->smooth_path_corners(plan);
}

nav_msgs::msg::Path LocalPlanner::smooth_path_corners(const nav_msgs::msg::Path &plan) const
{
  if (plan.poses.size() < 3U)
  {
    return plan;
  }

  nav_msgs::msg::Path smoothed_plan;
  smoothed_plan.header = plan.header;
  smoothed_plan.header.stamp = this->now();
  smoothed_plan.poses.reserve(plan.poses.size() * 2U);
  smoothed_plan.poses.push_back(plan.poses.front());

  const int sample_count = std::max(1, this->path_refiner_corner_smoothing_samples_);
  for (std::size_t index = 1U; index + 1U < plan.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &previous_pose = plan.poses[index - 1U];
    const geometry_msgs::msg::PoseStamped &corner_pose = plan.poses[index];
    const geometry_msgs::msg::PoseStamped &next_pose = plan.poses[index + 1U];
    const double in_dx = corner_pose.pose.position.x - previous_pose.pose.position.x;
    const double in_dy = corner_pose.pose.position.y - previous_pose.pose.position.y;
    const double out_dx = next_pose.pose.position.x - corner_pose.pose.position.x;
    const double out_dy = next_pose.pose.position.y - corner_pose.pose.position.y;
    const double in_distance = std::sqrt((in_dx * in_dx) + (in_dy * in_dy));
    const double out_distance = std::sqrt((out_dx * out_dx) + (out_dy * out_dy));

    if (in_distance <= 1e-6 || out_distance <= 1e-6)
    {
      smoothed_plan.poses.push_back(corner_pose);
      continue;
    }

    const double in_unit_x = in_dx / in_distance;
    const double in_unit_y = in_dy / in_distance;
    const double out_unit_x = out_dx / out_distance;
    const double out_unit_y = out_dy / out_distance;
    const double dot = std::clamp((in_unit_x * out_unit_x) + (in_unit_y * out_unit_y), -1.0, 1.0);
    const double angle = std::acos(dot);
    if (angle <= this->path_refiner_corner_smoothing_angle_threshold_)
    {
      smoothed_plan.poses.push_back(corner_pose);
      continue;
    }

    const double offset = std::min(
      this->path_refiner_corner_smoothing_max_offset_,
      std::min(in_distance, out_distance) * 0.45);
    if (offset <= this->path_refiner_prune_distance_)
    {
      smoothed_plan.poses.push_back(corner_pose);
      continue;
    }

    geometry_msgs::msg::PoseStamped entry_pose = corner_pose;
    entry_pose.header.stamp = this->now();
    entry_pose.pose.position.x = corner_pose.pose.position.x - (in_unit_x * offset);
    entry_pose.pose.position.y = corner_pose.pose.position.y - (in_unit_y * offset);

    geometry_msgs::msg::PoseStamped exit_pose = corner_pose;
    exit_pose.header.stamp = this->now();
    exit_pose.pose.position.x = corner_pose.pose.position.x + (out_unit_x * offset);
    exit_pose.pose.position.y = corner_pose.pose.position.y + (out_unit_y * offset);

    if (this->pose_distance(smoothed_plan.poses.back(), entry_pose) >= this->path_refiner_prune_distance_)
    {
      smoothed_plan.poses.push_back(entry_pose);
    }

    for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count + 1);
      const double inverse_ratio = 1.0 - ratio;
      geometry_msgs::msg::PoseStamped sample_pose = corner_pose;
      sample_pose.header.stamp = this->now();
      sample_pose.pose.position.x =
        (inverse_ratio * inverse_ratio * entry_pose.pose.position.x) +
        (2.0 * inverse_ratio * ratio * corner_pose.pose.position.x) +
        (ratio * ratio * exit_pose.pose.position.x);
      sample_pose.pose.position.y =
        (inverse_ratio * inverse_ratio * entry_pose.pose.position.y) +
        (2.0 * inverse_ratio * ratio * corner_pose.pose.position.y) +
        (ratio * ratio * exit_pose.pose.position.y);
      smoothed_plan.poses.push_back(sample_pose);
    }

    smoothed_plan.poses.push_back(exit_pose);
  }

  smoothed_plan.poses.push_back(plan.poses.back());
  return smoothed_plan;
}

bool LocalPlanner::is_smoothed_path_acceptable(
  const nav_msgs::msg::Path &base_plan,
  const nav_msgs::msg::Path &smoothed_plan) const
{
  if (smoothed_plan.poses.size() < 2U)
  {
    return false;
  }

  if (
    this->path_refiner_collision_check_enabled_ &&
    !this->is_path_collision_free(smoothed_plan))
  {
    return false;
  }

  const double base_length = this->estimate_path_length(base_plan);
  const double smoothed_length = this->estimate_path_length(smoothed_plan);
  if (
    base_length > 1e-6 &&
    smoothed_length > (base_length * std::max(1.0, this->path_refiner_smoothing_max_length_ratio_)))
  {
    return false;
  }

  const double base_curvature = this->estimate_path_curvature_score(base_plan);
  const double base_lateral_error = this->estimate_path_lateral_error(base_plan);
  if (
    base_curvature <= this->path_refiner_collinear_angle_threshold_ &&
    base_lateral_error <= this->path_refiner_collinear_lateral_deviation_threshold_)
  {
    const double smoothed_lateral_error = this->estimate_path_lateral_error(smoothed_plan);
    if (
      smoothed_lateral_error >
      base_lateral_error + std::max(0.01, this->path_refiner_collinear_lateral_deviation_threshold_))
    {
      return false;
    }
  }

  if (this->path_refiner_smoothing_max_pose_deviation_ <= 1e-6)
  {
    return true;
  }

  for (const auto &pose : smoothed_plan.poses)
  {
    if (
      this->estimate_pose_distance_to_path(pose, base_plan) >
      this->path_refiner_smoothing_max_pose_deviation_)
    {
      return false;
    }
  }

  return true;
}

double LocalPlanner::estimate_path_length(const nav_msgs::msg::Path &plan) const
{
  double total_distance = 0.0;
  if (plan.poses.size() < 2U)
  {
    return total_distance;
  }

  for (std::size_t index = 1U; index < plan.poses.size(); ++index)
  {
    total_distance += this->pose_distance(plan.poses[index - 1U], plan.poses[index]);
  }

  return total_distance;
}

double LocalPlanner::estimate_pose_distance_to_path(
  const geometry_msgs::msg::PoseStamped &pose,
  const nav_msgs::msg::Path &path) const
{
  if (path.poses.empty())
  {
    return std::numeric_limits<double>::infinity();
  }

  const auto &point = pose.pose.position;
  double min_distance = std::numeric_limits<double>::infinity();

  const auto update_distance_from_point =
    [&point, &min_distance](const geometry_msgs::msg::Point &candidate_point)
    {
      const double dx = candidate_point.x - point.x;
      const double dy = candidate_point.y - point.y;
      min_distance = std::min(min_distance, std::sqrt((dx * dx) + (dy * dy)));
    };

  update_distance_from_point(path.poses.front().pose.position);

  for (std::size_t index = 1U; index < path.poses.size(); ++index)
  {
    const auto &segment_start = path.poses[index - 1U].pose.position;
    const auto &segment_end = path.poses[index].pose.position;
    const double dx = segment_end.x - segment_start.x;
    const double dy = segment_end.y - segment_start.y;
    const double segment_length_sq = (dx * dx) + (dy * dy);

    if (segment_length_sq <= 1e-8)
    {
      update_distance_from_point(segment_end);
      continue;
    }

    const double projection = std::clamp(
      (((point.x - segment_start.x) * dx) + ((point.y - segment_start.y) * dy)) /
      segment_length_sq,
      0.0,
      1.0);
    geometry_msgs::msg::Point projected_point;
    projected_point.x = segment_start.x + (projection * dx);
    projected_point.y = segment_start.y + (projection * dy);
    update_distance_from_point(projected_point);
  }

  return min_distance;
}

double LocalPlanner::estimate_path_curvature_score(const nav_msgs::msg::Path &path) const
{
  if (path.poses.size() < 2U)
  {
    return 0.0;
  }

  const geometry_msgs::msg::Point &start = path.poses.front().pose.position;
  const geometry_msgs::msg::Point &end = path.poses.back().pose.position;
  const double baseline_dx = end.x - start.x;
  const double baseline_dy = end.y - start.y;
  if ((baseline_dx * baseline_dx) + (baseline_dy * baseline_dy) <= 1e-8)
  {
    return 0.0;
  }

  const double baseline_heading = std::atan2(baseline_dy, baseline_dx);
  double max_heading_delta = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index)
  {
    const geometry_msgs::msg::Point &previous = path.poses[index - 1U].pose.position;
    const geometry_msgs::msg::Point &current = path.poses[index].pose.position;
    const double segment_dx = current.x - previous.x;
    const double segment_dy = current.y - previous.y;
    if ((segment_dx * segment_dx) + (segment_dy * segment_dy) <= 1e-8)
    {
      continue;
    }

    const double segment_heading = std::atan2(segment_dy, segment_dx);
    max_heading_delta = std::max(
      max_heading_delta,
      std::abs(normalize_angle(segment_heading - baseline_heading)));
  }

  return max_heading_delta;
}

double LocalPlanner::estimate_path_lateral_error(const nav_msgs::msg::Path &path) const
{
  if (path.poses.size() < 3U)
  {
    return 0.0;
  }

  const geometry_msgs::msg::Point &start = path.poses.front().pose.position;
  const geometry_msgs::msg::Point &end = path.poses.back().pose.position;
  const double line_dx = end.x - start.x;
  const double line_dy = end.y - start.y;
  const double line_length = std::sqrt((line_dx * line_dx) + (line_dy * line_dy));
  if (line_length <= 1e-6)
  {
    return 0.0;
  }

  double max_lateral_error = 0.0;
  for (std::size_t index = 1U; index + 1U < path.poses.size(); ++index)
  {
    const geometry_msgs::msg::Point &point = path.poses[index].pose.position;
    const double point_dx = point.x - start.x;
    const double point_dy = point.y - start.y;
    const double lateral_error = std::abs((point_dx * line_dy) - (point_dy * line_dx)) / line_length;
    max_lateral_error = std::max(max_lateral_error, lateral_error);
  }

  return max_lateral_error;
}

bool LocalPlanner::is_path_segment_collision_free(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  const double sample_distance) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_ || this->map_occupancy_grid_->data.empty())
  {
    return false;
  }

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double segment_distance = std::sqrt((dx * dx) + (dy * dy));
  if (segment_distance <= 1e-6)
  {
    return this->is_pose_collision_free(start);
  }

  const double segment_heading = std::atan2(dy, dx);
  geometry_msgs::msg::PoseStamped start_sample = start;
  geometry_msgs::msg::PoseStamped goal_sample = goal;
  start_sample.pose.orientation = this->yaw_to_quaternion(segment_heading);
  goal_sample.pose.orientation = this->yaw_to_quaternion(segment_heading);
  if (!this->is_pose_collision_free(start_sample) || !this->is_pose_collision_free(goal_sample))
  {
    return false;
  }

  const double effective_sample_distance = std::max(sample_distance, 1e-3);
  const int sample_count = static_cast<int>(std::floor(segment_distance / effective_sample_distance));
  for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
  {
    const double ratio = std::clamp(
      (static_cast<double>(sample_index) * effective_sample_distance) / segment_distance,
      0.0,
      1.0);
    if (ratio >= 1.0)
    {
      continue;
    }

    geometry_msgs::msg::PoseStamped sample_pose = this->interpolate_pose(start_sample, goal_sample, ratio);
    sample_pose.pose.orientation = this->yaw_to_quaternion(segment_heading);
    if (!this->is_pose_collision_free(sample_pose))
    {
      return false;
    }
  }

  return true;
}

bool LocalPlanner::is_path_collision_free(const nav_msgs::msg::Path &plan) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_ || this->map_occupancy_grid_->data.empty())
  {
    return true;
  }

  const double sample_distance = std::max(this->path_refiner_collision_sample_distance_, 1e-3);
  for (std::size_t index = 0U; index < plan.poses.size(); ++index)
  {
    if (!this->is_pose_collision_free(plan.poses[index]))
    {
      return false;
    }

    if (index == 0U)
    {
      continue;
    }

    const geometry_msgs::msg::PoseStamped &previous_pose = plan.poses[index - 1U];
    const geometry_msgs::msg::PoseStamped &current_pose = plan.poses[index];
    const double segment_distance = this->pose_distance(previous_pose, current_pose);
    if (segment_distance <= sample_distance)
    {
      continue;
    }

    const int sample_count = static_cast<int>(std::floor(segment_distance / sample_distance));
    for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double ratio = std::clamp(
        (static_cast<double>(sample_index) * sample_distance) / segment_distance,
        0.0,
        1.0);
      if (ratio >= 1.0)
      {
        continue;
      }

      const geometry_msgs::msg::PoseStamped sample_pose =
        this->interpolate_pose(previous_pose, current_pose, ratio);
      if (!this->is_pose_collision_free(sample_pose))
      {
        return false;
      }
    }
  }

  return true;
}

bool LocalPlanner::is_pose_collision_free(const geometry_msgs::msg::PoseStamped &pose) const
{
  int grid_x = 0;
  int grid_y = 0;
  if (!this->world_to_grid(pose.pose.position, grid_x, grid_y))
  {
    return false;
  }

  return !this->is_grid_pose_collision(
    this->map_occupancy_grid_->data,
    static_cast<int>(this->map_occupancy_grid_->info.width),
    static_cast<int>(this->map_occupancy_grid_->info.height),
    grid_x,
    grid_y,
    yaw_from_quaternion(pose.pose.orientation));
}

void LocalPlanner::assign_path_headings(nav_msgs::msg::Path &plan) const
{
  if (plan.poses.size() < 2U)
  {
    return;
  }

  const geometry_msgs::msg::Quaternion goal_orientation = plan.poses.back().pose.orientation;
  for (std::size_t index = 0U; index + 1U < plan.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &current_pose = plan.poses[index];
    const geometry_msgs::msg::PoseStamped &next_pose = plan.poses[index + 1U];
    const double dx = next_pose.pose.position.x - current_pose.pose.position.x;
    const double dy = next_pose.pose.position.y - current_pose.pose.position.y;
    if ((dx * dx) + (dy * dy) <= 1e-8)
    {
      continue;
    }

    plan.poses[index].pose.orientation = this->yaw_to_quaternion(std::atan2(dy, dx));
  }

  if (this->path_refiner_preserve_goal_orientation_)
  {
    plan.poses.back().pose.orientation = goal_orientation;
  }
  else
  {
    plan.poses.back().pose.orientation = plan.poses[plan.poses.size() - 2U].pose.orientation;
  }
}

geometry_msgs::msg::Quaternion LocalPlanner::yaw_to_quaternion(const double yaw) const
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

nav_msgs::msg::Path LocalPlanner::build_source_plan(const amr_msgs::msg::MotionCommand &command) const
{
  nav_msgs::msg::Path source_plan = command.plan;
  if (source_plan.header.frame_id.empty())
  {
    source_plan.header = command.header;
  }
  if (source_plan.header.stamp.sec == 0 &&source_plan.header.stamp.nanosec == 0U)
  {
    source_plan.header.stamp = this->now();
  }
  if (source_plan.poses.empty())
  {
    source_plan.poses.push_back(command.goal_pose);
  }
  return source_plan;
}

std::size_t LocalPlanner::find_closest_pose_index(
  const nav_msgs::msg::Path &plan,
  const geometry_msgs::msg::PoseStamped &current_pose,
  const std::size_t start_index) const
{
  if (plan.poses.empty())
  {
    return 0U;
  }

  const std::size_t search_start = std::min(start_index, plan.poses.size() - 1U);
  std::size_t closest_index = search_start;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t index = search_start; index < plan.poses.size(); ++index)
  {
    const double distance = this->pose_distance(current_pose, plan.poses[index]);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_index = index;
    }
  }

  return closest_index;
}

bool LocalPlanner::world_to_grid(
  const geometry_msgs::msg::Point &point,
  int &grid_x,
  int &grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_)
  {
    return false;
  }

  const nav_msgs::msg::MapMetaData &info = this->map_occupancy_grid_->info;
  grid_x = static_cast<int>(std::floor((point.x - info.origin.position.x) / info.resolution));
  grid_y = static_cast<int>(std::floor((point.y - info.origin.position.y) / info.resolution));

  return
    grid_x >= 0 &&grid_x < static_cast<int>(info.width) &&
    grid_y >= 0 &&grid_y < static_cast<int>(info.height);
}

geometry_msgs::msg::PoseStamped LocalPlanner::grid_to_pose(
  const int grid_x,
  const int grid_y,
  const std::string &frame_id) const
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
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const int grid_x,
  const int grid_y) const
{
  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
  {
    return true;
  }

  const int value = occupancy_grid[static_cast<std::size_t>(grid_y * width + grid_x)];
  if (value == kUnknownCellValue)
  {
    return !this->allow_unknown_;
  }
  return value >= this->obstacle_threshold_;
}

bool LocalPlanner::is_grid_pose_collision(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const int grid_x,
  const int grid_y,
  const double yaw) const
{
  if (this->footprint_polygon_.empty() || !this->has_map_ || !this->map_occupancy_grid_)
  {
    return this->is_occupied_cell(occupancy_grid, width, height, grid_x, grid_y);
  }

  const double resolution = this->map_occupancy_grid_->info.resolution;
  const double pose_x =
    this->map_occupancy_grid_->info.origin.position.x + ((static_cast<double>(grid_x) + 0.5) * resolution);
  const double pose_y =
    this->map_occupancy_grid_->info.origin.position.y + ((static_cast<double>(grid_y) + 0.5) * resolution);
  return amr::geometry::footprint_pose_collides(
    occupancy_grid,
    width,
    height,
    resolution,
    this->map_occupancy_grid_->info.origin.position.x,
    this->map_occupancy_grid_->info.origin.position.y,
    this->footprint_polygon_,
    pose_x,
    pose_y,
    yaw,
    this->obstacle_threshold_,
    this->allow_unknown_);
}

bool LocalPlanner::find_nearest_free_cell(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  int &grid_x,
  int &grid_y,
  const int max_radius,
  const double yaw) const
{
  if (!this->is_grid_pose_collision(occupancy_grid, width, height, grid_x, grid_y, yaw))
  {
    return true;
  }

  const int original_x = grid_x;
  const int original_y = grid_y;
  for (int radius = 1; radius <= max_radius; ++radius)
  {
    bool found_candidate = false;
    int best_x = grid_x;
    int best_y = grid_y;
    double best_distance_squared = std::numeric_limits<double>::max();
    int best_axis_offset = std::numeric_limits<int>::max();
    int best_total_offset = std::numeric_limits<int>::max();

    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dx = -radius; dx <= radius; ++dx)
      {
        if (std::max(std::abs(dx), std::abs(dy)) != radius)
        {
          continue;
        }

        const int candidate_x = original_x + dx;
        const int candidate_y = original_y + dy;
        if (this->is_grid_pose_collision(
            occupancy_grid, width, height, candidate_x, candidate_y, yaw))
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
          (distance_squared == best_distance_squared &&axis_offset < best_axis_offset) ||
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

    if (found_candidate)
    {
      grid_x = best_x;
      grid_y = best_y;
      return true;
    }
  }

  return false;
}

geometry_msgs::msg::PoseStamped LocalPlanner::interpolate_pose(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
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
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr::planner::local

namespace amr::motion::controller
{

namespace
{

const char *motion_mode_label(const uint8_t mode)
{
  switch (mode)
  {
    case amr_msgs::msg::MotionCommand::MODE_NAVIGATE:
      return "navigate";
    case amr_msgs::msg::MotionCommand::MODE_BACKUP:
      return "backup";
    case amr_msgs::msg::MotionCommand::MODE_SPIN:
      return "spin";
    case amr_msgs::msg::MotionCommand::MODE_WAIT:
      return "wait";
    default:
      return "unknown";
  }
}

const char *bool_label(const bool value)
{
  return value ? "true" : "false";
}

const char *frame_label(const std::string &frame)
{
  return frame.empty() ? "none" : frame.c_str();
}

const char *steering_hysteresis_label(const bool active)
{
  return active ? "active" : "suppressed";
}

int velocity_sign(const double value)
{
  constexpr double kSignEpsilon = 1e-4;
  if (value > kSignEpsilon)
  {
    return 1;
  }
  if (value < -kSignEpsilon)
  {
    return -1;
  }
  return 0;
}

int throttle_ms_from_sec(const double seconds)
{
  return static_cast<int>(std::max(0.1, seconds) * 1000.0);
}

}  // namespace

MotionController::MotionController(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("motion_controller", options),
  command_topic_(""),
  local_plan_topic_(""),
  current_pose_topic_(""),
  scan_topic_(""),
  status_topic_(""),
  cmd_vel_topic_(""),
  control_frequency_(10.0),
  linear_speed_(0.10),
  min_linear_speed_(0.06),
  tracking_lookahead_distance_(0.22),
  straight_tracking_lookahead_distance_(0.45),
  tracking_min_target_distance_(0.08),
  tracking_progress_rollback_window_(2U),
  tracking_target_hysteresis_distance_(0.08),
  tracking_target_reset_distance_(0.35),
  angular_gain_(2.0),
  max_angular_speed_(0.65),
  distance_tolerance_(0.10),
  goal_heading_tolerance_(0.12),
  goal_reach_heading_tolerance_(0.08),
  final_align_max_angular_speed_(0.35),
  final_align_heading_deadband_(0.06),
  final_align_settle_time_sec_(0.30),
  goal_checker_xy_tolerance_(0.10),
  goal_checker_xy_hysteresis_(0.04),
  goal_checker_yaw_tolerance_(0.7853981633974483),
  goal_checker_hold_time_sec_(0.0),
  goal_checker_respect_goal_yaw_(false),
  goal_checker_ignore_yaw_(false),
  rotate_in_place_threshold_(0.55),
  rotate_in_place_goal_distance_(0.16),
  tracking_heading_deadband_(0.06),
  tracking_heading_release_threshold_(0.11),
  straight_tracking_enabled_(true),
  straight_curvature_threshold_(0.12),
  straight_lateral_error_threshold_(0.035),
  straight_heading_deadband_(0.12),
  straight_heading_release_threshold_(0.18),
  straight_angular_gain_(0.75),
  straight_max_angular_speed_(0.14),
  straight_heading_filter_alpha_(0.20),
  rejoin_target_distance_threshold_(0.12),
  rejoin_context_timeout_sec_(2.5),
  rejoin_context_distance_m_(0.45),
  rejoin_target_jump_threshold_m_(0.28),
  rejoin_heading_gate_threshold_(0.28),
  rejoin_min_linear_scale_(0.25),
  heading_slowdown_threshold_(0.16),
  min_heading_motion_scale_(0.18),
  max_linear_accel_(0.08),
  max_angular_accel_(0.8),
  progress_required_movement_radius_(0.05),
  progress_time_allowance_sec_(2.0),
  status_command_settle_time_sec_(0.7),
  status_blocked_confirm_cycles_(3),
  status_blocked_clear_cycles_(3),
  status_stalled_confirm_cycles_(2),
  structured_logging_enabled_(true),
  tracking_state_log_throttle_sec_(1.0),
  cmd_quality_log_throttle_sec_(1.0),
  target_jump_warn_threshold_m_(0.28),
  safety_gate_enabled_(true),
  safety_gate_allow_rotate_in_place_(true),
  safety_gate_stop_distance_(3.0),
  safety_gate_forward_angle_deg_(25.0),
  safety_gate_rotate_heading_threshold_(0.20),
  safety_gate_min_points_(3),
  velocity_control_mode_(VelocityControlMode::PID),
  recovery_start_yaw_(0.0),
  goal_checker_holding_(false),
  final_align_holding_(false),
  goal_xy_latched_(false),
  has_command_(false),
  has_local_plan_(false),
  has_current_pose_(false),
  has_latest_scan_(false),
  has_progress_reference_(false),
  has_recovery_reference_(false),
  has_tracking_progress_index_(false),
  has_tracking_target_index_(false),
  has_tracking_target_pose_(false),
  tracking_target_from_plan_(false),
  steering_hysteresis_active_(false),
  has_heading_error_filter_(false),
  rejoin_context_active_(false),
  rejoin_context_pending_(false),
  blocked_latched_(false),
  blocked_streak_(0),
  blocked_clear_streak_(0),
  stalled_streak_(0),
  cmd_ang_sign_(0),
  last_cmd_ang_sign_(0),
  cmd_ang_flip_count_(0),
  output_ang_sign_(0),
  last_output_ang_sign_(0),
  output_ang_flip_count_(0),
  tracking_progress_index_(0U),
  tracking_target_index_(0U),
  tracking_nearest_index_(0U),
  tracking_candidate_index_(0U),
  heading_error_filtered_(0.0),
  tracking_selected_target_distance_(0.0),
  tracking_selection_reason_("not_selected")
{
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.plan", this->local_plan_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.status", this->status_topic_);
  this->declare_parameter("topics.velocity", this->cmd_vel_topic_);

  this->declare_parameter("control.frequency", this->control_frequency_);
  this->declare_parameter("control.linear_speed", this->linear_speed_);
  this->declare_parameter("control.min_linear_speed", this->min_linear_speed_);
  this->declare_parameter("control.tracking_lookahead_distance", this->tracking_lookahead_distance_);
  this->declare_parameter(
    "control.straight_tracking_lookahead_distance", this->straight_tracking_lookahead_distance_);
  this->declare_parameter("control.tracking_min_target_distance", this->tracking_min_target_distance_);
  this->declare_parameter(
    "control.tracking_progress_rollback_window",
    static_cast<int64_t>(this->tracking_progress_rollback_window_));
  this->declare_parameter(
    "control.tracking_target_hysteresis_distance", this->tracking_target_hysteresis_distance_);
  this->declare_parameter(
    "control.tracking_target_reset_distance", this->tracking_target_reset_distance_);
  this->declare_parameter("control.angular_gain", this->angular_gain_);
  this->declare_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->declare_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->declare_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->declare_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->declare_parameter(
    "control.final_align_max_angular_speed", this->final_align_max_angular_speed_);
  this->declare_parameter(
    "control.final_align_heading_deadband", this->final_align_heading_deadband_);
  this->declare_parameter(
    "control.final_align_settle_time_sec", this->final_align_settle_time_sec_);
  this->declare_parameter("goal_checker.xy_tolerance", this->goal_checker_xy_tolerance_);
  this->declare_parameter("goal_checker.xy_hysteresis", this->goal_checker_xy_hysteresis_);
  this->declare_parameter("goal_checker.yaw_tolerance", this->goal_checker_yaw_tolerance_);
  this->declare_parameter("goal_checker.hold_time_sec", this->goal_checker_hold_time_sec_);
  this->declare_parameter("goal_checker.respect_goal_yaw", this->goal_checker_respect_goal_yaw_);
  this->declare_parameter("goal_checker.ignore_yaw", this->goal_checker_ignore_yaw_);
  this->declare_parameter(
    "control.rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->declare_parameter(
    "control.rotate_in_place_goal_distance", this->rotate_in_place_goal_distance_);
  this->declare_parameter(
    "control.tracking_heading_deadband", this->tracking_heading_deadband_);
  this->declare_parameter(
    "control.tracking_heading_release_threshold", this->tracking_heading_release_threshold_);
  this->declare_parameter("control.straight_tracking_enabled", this->straight_tracking_enabled_);
  this->declare_parameter(
    "control.straight_curvature_threshold", this->straight_curvature_threshold_);
  this->declare_parameter(
    "control.straight_lateral_error_threshold", this->straight_lateral_error_threshold_);
  this->declare_parameter("control.straight_heading_deadband", this->straight_heading_deadband_);
  this->declare_parameter(
    "control.straight_heading_release_threshold", this->straight_heading_release_threshold_);
  this->declare_parameter("control.straight_angular_gain", this->straight_angular_gain_);
  this->declare_parameter("control.straight_max_angular_speed", this->straight_max_angular_speed_);
  this->declare_parameter(
    "control.straight_heading_filter_alpha", this->straight_heading_filter_alpha_);
  this->declare_parameter(
    "control.rejoin_target_distance_threshold", this->rejoin_target_distance_threshold_);
  this->declare_parameter(
    "control.rejoin_context_timeout_sec", this->rejoin_context_timeout_sec_);
  this->declare_parameter(
    "control.rejoin_context_distance_m", this->rejoin_context_distance_m_);
  this->declare_parameter(
    "control.rejoin_target_jump_threshold_m", this->rejoin_target_jump_threshold_m_);
  this->declare_parameter(
    "control.rejoin_heading_gate_threshold", this->rejoin_heading_gate_threshold_);
  this->declare_parameter(
    "control.rejoin_min_linear_scale", this->rejoin_min_linear_scale_);
  this->declare_parameter(
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->declare_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->declare_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->declare_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);
  this->declare_parameter(
    "status.command_settle_time_sec", this->status_command_settle_time_sec_);
  this->declare_parameter(
    "status.blocked_confirm_cycles", this->status_blocked_confirm_cycles_);
  this->declare_parameter(
    "status.blocked_clear_cycles", this->status_blocked_clear_cycles_);
  this->declare_parameter(
    "status.stalled_confirm_cycles", this->status_stalled_confirm_cycles_);
  this->declare_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->declare_parameter(
    "logging.tracking_state_throttle_sec", this->tracking_state_log_throttle_sec_);
  this->declare_parameter(
    "logging.cmd_quality_throttle_sec", this->cmd_quality_log_throttle_sec_);
  this->declare_parameter(
    "logging.target_jump_warn_threshold_m", this->target_jump_warn_threshold_m_);

  this->declare_parameter("safety_gate.enabled", this->safety_gate_enabled_);
  this->declare_parameter(
    "safety_gate.allow_rotate_in_place", this->safety_gate_allow_rotate_in_place_);
  this->declare_parameter(
    "safety_gate.stop_distance", this->safety_gate_stop_distance_);
  this->declare_parameter(
    "safety_gate.forward_angle_deg", this->safety_gate_forward_angle_deg_);
  this->declare_parameter(
    "safety_gate.rotate_heading_threshold", this->safety_gate_rotate_heading_threshold_);
  this->declare_parameter(
    "safety_gate.minimum_points", this->safety_gate_min_points_);

  this->declare_parameter("velocity_controller.mode", std::string("pid"));
  this->declare_parameter("velocity_controller.linear.kp", 0.35);
  this->declare_parameter("velocity_controller.linear.ki", 0.0);
  this->declare_parameter("velocity_controller.linear.kd", 0.04);
  this->declare_parameter("velocity_controller.linear.integral_limit", 0.20);
  this->declare_parameter("velocity_controller.angular.kp", 0.45);
  this->declare_parameter("velocity_controller.angular.ki", 0.0);
  this->declare_parameter("velocity_controller.angular.kd", 0.0);
  this->declare_parameter("velocity_controller.angular.integral_limit", 0.30);
  this->declare_parameter("velocity_controller.max_linear_accel", this->max_linear_accel_);
  this->declare_parameter("velocity_controller.max_angular_accel", this->max_angular_accel_);
}

MotionController::CallbackReturn MotionController::on_configure(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.plan", this->local_plan_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.status", this->status_topic_);
  this->get_parameter("topics.velocity", this->cmd_vel_topic_);

  this->get_parameter("control.frequency", this->control_frequency_);
  this->get_parameter("control.linear_speed", this->linear_speed_);
  this->get_parameter("control.min_linear_speed", this->min_linear_speed_);
  this->get_parameter("control.tracking_lookahead_distance", this->tracking_lookahead_distance_);
  this->get_parameter(
    "control.straight_tracking_lookahead_distance", this->straight_tracking_lookahead_distance_);
  this->get_parameter("control.tracking_min_target_distance", this->tracking_min_target_distance_);
  this->tracking_progress_rollback_window_ = static_cast<std::size_t>(
    this->get_parameter("control.tracking_progress_rollback_window").as_int());
  this->get_parameter(
    "control.tracking_target_hysteresis_distance", this->tracking_target_hysteresis_distance_);
  this->get_parameter(
    "control.tracking_target_reset_distance", this->tracking_target_reset_distance_);
  this->get_parameter("control.angular_gain", this->angular_gain_);
  this->get_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->get_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->get_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->get_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->get_parameter(
    "control.final_align_max_angular_speed", this->final_align_max_angular_speed_);
  this->get_parameter(
    "control.final_align_heading_deadband", this->final_align_heading_deadband_);
  this->get_parameter(
    "control.final_align_settle_time_sec", this->final_align_settle_time_sec_);
  this->get_parameter("goal_checker.xy_tolerance", this->goal_checker_xy_tolerance_);
  this->get_parameter("goal_checker.xy_hysteresis", this->goal_checker_xy_hysteresis_);
  this->get_parameter("goal_checker.yaw_tolerance", this->goal_checker_yaw_tolerance_);
  this->get_parameter("goal_checker.hold_time_sec", this->goal_checker_hold_time_sec_);
  this->get_parameter("goal_checker.respect_goal_yaw", this->goal_checker_respect_goal_yaw_);
  this->get_parameter("goal_checker.ignore_yaw", this->goal_checker_ignore_yaw_);
  this->get_parameter(
    "control.rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->get_parameter(
    "control.rotate_in_place_goal_distance", this->rotate_in_place_goal_distance_);
  this->get_parameter(
    "control.tracking_heading_deadband", this->tracking_heading_deadband_);
  this->get_parameter(
    "control.tracking_heading_release_threshold", this->tracking_heading_release_threshold_);
  this->get_parameter("control.straight_tracking_enabled", this->straight_tracking_enabled_);
  this->get_parameter(
    "control.straight_curvature_threshold", this->straight_curvature_threshold_);
  this->get_parameter(
    "control.straight_lateral_error_threshold", this->straight_lateral_error_threshold_);
  this->get_parameter("control.straight_heading_deadband", this->straight_heading_deadband_);
  this->get_parameter(
    "control.straight_heading_release_threshold", this->straight_heading_release_threshold_);
  this->get_parameter("control.straight_angular_gain", this->straight_angular_gain_);
  this->get_parameter("control.straight_max_angular_speed", this->straight_max_angular_speed_);
  this->get_parameter(
    "control.straight_heading_filter_alpha", this->straight_heading_filter_alpha_);
  this->get_parameter(
    "control.rejoin_target_distance_threshold", this->rejoin_target_distance_threshold_);
  this->get_parameter(
    "control.rejoin_context_timeout_sec", this->rejoin_context_timeout_sec_);
  this->get_parameter(
    "control.rejoin_context_distance_m", this->rejoin_context_distance_m_);
  this->get_parameter(
    "control.rejoin_target_jump_threshold_m", this->rejoin_target_jump_threshold_m_);
  this->get_parameter(
    "control.rejoin_heading_gate_threshold", this->rejoin_heading_gate_threshold_);
  this->get_parameter(
    "control.rejoin_min_linear_scale", this->rejoin_min_linear_scale_);
  this->get_parameter(
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->get_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->get_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->get_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);
  this->get_parameter(
    "status.command_settle_time_sec", this->status_command_settle_time_sec_);
  this->get_parameter(
    "status.blocked_confirm_cycles", this->status_blocked_confirm_cycles_);
  this->get_parameter(
    "status.blocked_clear_cycles", this->status_blocked_clear_cycles_);
  this->get_parameter(
    "status.stalled_confirm_cycles", this->status_stalled_confirm_cycles_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->get_parameter(
    "logging.tracking_state_throttle_sec", this->tracking_state_log_throttle_sec_);
  this->get_parameter(
    "logging.cmd_quality_throttle_sec", this->cmd_quality_log_throttle_sec_);
  this->get_parameter(
    "logging.target_jump_warn_threshold_m", this->target_jump_warn_threshold_m_);

  this->get_parameter("safety_gate.enabled", this->safety_gate_enabled_);
  this->get_parameter(
    "safety_gate.allow_rotate_in_place", this->safety_gate_allow_rotate_in_place_);
  this->get_parameter(
    "safety_gate.stop_distance", this->safety_gate_stop_distance_);
  this->get_parameter(
    "safety_gate.forward_angle_deg", this->safety_gate_forward_angle_deg_);
  this->get_parameter(
    "safety_gate.rotate_heading_threshold", this->safety_gate_rotate_heading_threshold_);
  this->get_parameter(
    "safety_gate.minimum_points", this->safety_gate_min_points_);

  this->velocity_control_mode_ = this->parse_velocity_control_mode(
    this->get_parameter("velocity_controller.mode").as_string());
  this->linear_controller_config_.kp =
    this->get_parameter("velocity_controller.linear.kp").as_double();
  this->linear_controller_config_.ki =
    this->get_parameter("velocity_controller.linear.ki").as_double();
  this->linear_controller_config_.kd =
    this->get_parameter("velocity_controller.linear.kd").as_double();
  this->linear_controller_config_.integral_limit =
    this->get_parameter("velocity_controller.linear.integral_limit").as_double();
  this->angular_controller_config_.kp =
    this->get_parameter("velocity_controller.angular.kp").as_double();
  this->angular_controller_config_.ki =
    this->get_parameter("velocity_controller.angular.ki").as_double();
  this->angular_controller_config_.kd =
    this->get_parameter("velocity_controller.angular.kd").as_double();
  this->angular_controller_config_.integral_limit =
    this->get_parameter("velocity_controller.angular.integral_limit").as_double();
  this->max_linear_accel_ =
    this->get_parameter("velocity_controller.max_linear_accel").as_double();
  this->max_angular_accel_ =
    this->get_parameter("velocity_controller.max_angular_accel").as_double();

  if (
    this->command_topic_.empty() || this->local_plan_topic_.empty() ||
    this->current_pose_topic_.empty() || this->scan_topic_.empty() ||
    this->status_topic_.empty() ||
    this->cmd_vel_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Motion controller topics must not be empty: command='%s' local_plan='%s' pose='%s' scan='%s' status='%s' cmd_vel='%s'",
      this->command_topic_.c_str(),
      this->local_plan_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->scan_topic_.c_str(),
      this->status_topic_.c_str(),
      this->cmd_vel_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->reset_velocity_controller_state();
  this->reset_goal_checker_state();

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
    });
  this->local_plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    this->local_plan_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Path::SharedPtr message) {
      this->handle_local_plan(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });
  this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    this->cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  this->motion_status_publisher_ = this->create_publisher<amr_msgs::msg::MotionStatus>(
    this->status_topic_, rclcpp::SystemDefaultsQoS());

  const std::chrono::milliseconds control_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(this->control_frequency_, 1.0)));
  this->timer_ = this->create_wall_timer(
    control_period,
    [this]() { this->publish_control(); });
  this->timer_->cancel();
  const std::string velocity_control_mode = this->get_parameter("velocity_controller.mode").as_string();

  RCLCPP_INFO(
    this->get_logger(),
    "Configured motion controller with command='%s', plan='%s', pose='%s', cmd_vel='%s', mode='%s'",
    this->command_topic_.c_str(),
    this->local_plan_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->cmd_vel_topic_.c_str(),
    velocity_control_mode.c_str());

  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_activate(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->cmd_vel_publisher_->on_activate();
  this->motion_status_publisher_->on_activate();
  this->timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated motion controller");
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_deactivate(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->timer_)
  {
    this->timer_->cancel();
  }
  this->publish_zero_twist();
  if (this->cmd_vel_publisher_)
  {
    this->cmd_vel_publisher_->on_deactivate();
  }
  if (this->motion_status_publisher_)
  {
    this->motion_status_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated motion controller");
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_cleanup(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->publish_zero_twist();
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->scan_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->tracking_target_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->has_tracking_progress_index_ = false;
  this->has_tracking_target_index_ = false;
  this->has_tracking_target_pose_ = false;
  this->tracking_progress_index_ = 0U;
  this->tracking_target_index_ = 0U;
  this->tracking_nearest_index_ = 0U;
  this->tracking_candidate_index_ = 0U;
  this->tracking_selected_target_distance_ = 0.0;
  this->tracking_selection_reason_ = "reset";
  this->tracking_target_from_plan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
  this->reset_status_semantics_state();
  this->reset_rejoin_context_state();
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_shutdown(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->publish_zero_twist();
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->scan_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->tracking_target_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->has_tracking_progress_index_ = false;
  this->has_tracking_target_index_ = false;
  this->has_tracking_target_pose_ = false;
  this->tracking_progress_index_ = 0U;
  this->tracking_target_index_ = 0U;
  this->tracking_nearest_index_ = 0U;
  this->tracking_candidate_index_ = 0U;
  this->tracking_selected_target_distance_ = 0.0;
  this->tracking_selection_reason_ = "reset";
  this->tracking_target_from_plan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
  this->reset_status_semantics_state();
  this->reset_rejoin_context_state();
  return CallbackReturn::SUCCESS;
}

void MotionController::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  const bool recovery_rejoin_pending =
    message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&
    this->rejoin_context_pending_;
  this->latest_command_ = *message;
  this->has_command_ = true;
  if (message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE)
  {
    // Drop the previous command's plan immediately. A stale empty plan can otherwise
    // make the new command look invalid before the local planner republishes.
    this->latest_local_plan_ = nav_msgs::msg::Path();
    this->has_local_plan_ = false;
    this->has_tracking_progress_index_ = false;
    this->has_tracking_target_index_ = false;
    this->has_tracking_target_pose_ = false;
    this->tracking_progress_index_ = 0U;
    this->tracking_target_index_ = 0U;
    this->tracking_nearest_index_ = 0U;
    this->tracking_candidate_index_ = 0U;
    this->tracking_selected_target_distance_ = 0.0;
    this->tracking_selection_reason_ = "new_command";
    this->tracking_target_from_plan_ = false;
    if (recovery_rejoin_pending &&this->has_current_pose_)
    {
      this->activate_rejoin_context();
    }
    else if (!recovery_rejoin_pending)
    {
      this->reset_rejoin_context_state();
    }
  }
  else
  {
    this->reset_rejoin_context_state();
    this->rejoin_context_pending_ = true;
  }
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
  this->reset_status_semantics_state();
  this->has_recovery_reference_ = false;
  this->recovery_start_time_ = this->now();
  this->recovery_start_yaw_ = 0.0;
  this->latest_command_time_ = this->now();
  if (this->structured_logging_enabled_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=controller event=motion_command node=motion_controller goal_id=%u mode=%s route_id=%s target_x=%.3f target_y=%.3f recovery=%s recovery_type=%s",
      message->command_id,
      motion_mode_label(message->mode),
      message->route_id.empty() ? "none" : message->route_id.c_str(),
      message->goal_pose.pose.position.x,
      message->goal_pose.pose.position.y,
      bool_label(message->mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE),
      motion_mode_label(message->mode));
  }
}

void MotionController::handle_local_plan(const nav_msgs::msg::Path::SharedPtr message)
{
  this->latest_local_plan_ = *message;
  this->has_local_plan_ = true;
  if (message->poses.empty())
  {
    this->has_tracking_progress_index_ = false;
    this->has_tracking_target_index_ = false;
    this->has_tracking_target_pose_ = false;
    this->tracking_progress_index_ = 0U;
    this->tracking_target_index_ = 0U;
    this->tracking_nearest_index_ = 0U;
    this->tracking_candidate_index_ = 0U;
    this->tracking_selected_target_distance_ = 0.0;
    this->tracking_selection_reason_ = "local_plan_empty";
    this->tracking_target_from_plan_ = false;
  }
  else
  {
    this->tracking_progress_index_ = std::min(
      this->tracking_progress_index_,
      message->poses.size() - 1U);
    this->tracking_target_index_ = std::min(
      this->tracking_target_index_,
      message->poses.size() - 1U);
    this->tracking_selection_reason_ = "plan_updated";
  }
  if (this->structured_logging_enabled_)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
      "AMR_LOG schema=v1 component=controller event=tracking_target_selected node=motion_controller path_points=%zu target_idx=%zu target_x=%.3f target_y=%.3f result=plan_updated",
      message->poses.size(),
      this->tracking_target_index_,
      this->has_tracking_target_pose_ ? this->tracking_target_pose_.pose.position.x : 0.0,
      this->has_tracking_target_pose_ ? this->tracking_target_pose_.pose.position.y : 0.0);
  }
}

void MotionController::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void MotionController::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_latest_scan_ = true;
}

void MotionController::publish_control()
{
  if (
    !this->cmd_vel_publisher_ || !this->cmd_vel_publisher_->is_activated() ||
    !this->motion_status_publisher_ || !this->motion_status_publisher_->is_activated())
  {
    return;
  }

  geometry_msgs::msg::Twist desired_twist;
  geometry_msgs::msg::Twist output_twist;
  double debug_remaining_distance = 0.0;
  std::string debug_goal_state = "idle";
  std::string debug_rejoin_state = "inactive";
  bool debug_has_tracking_target = false;
  std::size_t debug_tracking_target_index = 0U;
  double debug_tracking_target_x = 0.0;
  double debug_tracking_target_y = 0.0;
  double debug_tracking_target_distance = 0.0;
  double debug_target_jump_m = 0.0;
  double debug_tracking_lookahead_distance = this->tracking_lookahead_distance_;
  double debug_target_dx = 0.0;
  double debug_target_dy = 0.0;
  double debug_current_yaw = 0.0;
  double debug_target_heading = 0.0;
  double debug_goal_yaw = 0.0;
  bool debug_command_align_heading_at_goal = false;
  bool debug_final_heading_required = false;
  bool debug_xy_reached = false;
  bool debug_yaw_reached = true;
  bool debug_straight_segment = false;
  bool debug_rejoin_context_active = false;
  double debug_path_curvature_score = 0.0;
  double debug_lateral_error_m = 0.0;
  double debug_heading_error_raw = 0.0;
  double debug_heading_error_filtered = 0.0;
  double debug_steering_heading_error = 0.0;
  bool debug_steering_deadband_active = false;
  const char *debug_steering_hysteresis_state = steering_hysteresis_label(false);
  std::string debug_pose_frame;
  std::string debug_plan_frame;
  std::string debug_target_frame;
  amr_msgs::msg::MotionStatus status;
  status.header.stamp = this->now();
  status.header.frame_id =
    this->current_pose_.header.frame_id.empty() ? "map" : this->current_pose_.header.frame_id;
  status.current_pose = this->current_pose_;
  status.command_id = this->has_command_ ? this->latest_command_.command_id : 0U;
  status.mode = this->has_command_ ? this->latest_command_.mode :
    amr_msgs::msg::MotionCommand::MODE_NAVIGATE;
  status.active = false;
  status.command_completed = false;
  status.goal_reached = false;
  status.obstacle_detected = false;
  status.blocked = false;
  status.stalled = false;
  status.local_plan_valid = false;
  status.costmap_blocked = false;
  status.safety_gate_blocked = false;
  status.has_blocked_pose = false;
  status.blocked_pose = geometry_msgs::msg::PoseStamped();
  status.remaining_distance = 0.0;
  status.heading_error = 0.0;

  const bool has_navigation_inputs =
    this->has_command_ &&this->has_current_pose_ &&
    (this->latest_command_.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE || this->has_local_plan_);

  if (has_navigation_inputs)
  {
    status.active = true;
    const double current_yaw = this->quaternion_yaw(this->current_pose_.pose.orientation);
    debug_current_yaw = current_yaw;

    if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE)
    {
      if (this->rejoin_context_pending_ &&this->has_current_pose_)
      {
        this->activate_rejoin_context();
      }
      this->update_rejoin_context_state();
      const bool had_tracking_target = this->has_tracking_target_pose_;
      const std::size_t previous_tracking_target_index = this->tracking_target_index_;
      const geometry_msgs::msg::PoseStamped previous_tracking_target = this->tracking_target_pose_;
      geometry_msgs::msg::PoseStamped tracking_target = this->select_tracking_target(
        this->tracking_lookahead_distance_, true);
      bool target_jump_activated_rejoin = false;
      debug_has_tracking_target = this->has_tracking_target_pose_;
      debug_tracking_target_index = this->tracking_target_index_;
      debug_tracking_target_x = tracking_target.pose.position.x;
      debug_tracking_target_y = tracking_target.pose.position.y;
      debug_target_jump_m = had_tracking_target ?
        this->pose_distance(previous_tracking_target, tracking_target) : 0.0;
      if (
        had_tracking_target &&
        debug_target_jump_m >= std::max(0.0, this->rejoin_target_jump_threshold_m_))
      {
        this->activate_rejoin_context();
        target_jump_activated_rejoin = true;
      }
      this->update_rejoin_context_state();
      const double local_plan_remaining_distance =
        this->estimate_remaining_distance(this->latest_local_plan_);
      double tracking_target_distance = this->pose_distance(this->current_pose_, tracking_target);
      debug_tracking_target_distance = tracking_target_distance;
      const GoalCheckResult goal_check = this->check_goal(
        this->current_pose_, this->latest_command_.goal_pose, current_yaw);
      const double goal_distance = goal_check.distance_error;
      debug_remaining_distance = std::max(local_plan_remaining_distance, goal_distance);
      const bool distance_reached = goal_check.distance_reached;
      const bool align_heading_at_goal = goal_check.align_heading;
      const double goal_yaw = goal_check.target_yaw;
      debug_command_align_heading_at_goal = this->latest_command_.align_heading_at_goal;
      debug_final_heading_required = align_heading_at_goal;
      debug_xy_reached = distance_reached;
      debug_yaw_reached = goal_check.heading_reached;
      debug_goal_yaw = goal_yaw;
      double target_dx =
        tracking_target.pose.position.x - this->current_pose_.pose.position.x;
      double target_dy =
        tracking_target.pose.position.y - this->current_pose_.pose.position.y;
      double target_heading = current_yaw;
      if ((target_dx * target_dx) + (target_dy * target_dy) > 1e-6)
      {
        target_heading = std::atan2(target_dy, target_dx);
      }
      if (align_heading_at_goal &&distance_reached)
      {
        target_heading = goal_yaw;
      }
      double heading_error = this->normalize_angle(target_heading - current_yaw);
      double abs_heading_error = std::abs(heading_error);
      const bool final_heading_phase = align_heading_at_goal &&distance_reached;
      const bool final_align_phase =
        align_heading_at_goal &&
        (distance_reached || goal_distance <= this->rotate_in_place_goal_distance_);
      const bool command_settling =
        this->latest_command_time_.nanoseconds() > 0 &&
        (this->now() - this->latest_command_time_).seconds() <
        this->status_command_settle_time_sec_;
      const bool rejoin_context_active = this->rejoin_context_active_;
      debug_rejoin_context_active = rejoin_context_active;
      StraightSegmentAssessment straight_assessment = this->assess_straight_segment(
        this->tracking_nearest_index_,
        this->straight_tracking_lookahead_distance_);
      bool straight_segment =
        this->straight_tracking_enabled_ &&
        straight_assessment.straight_segment &&
        !rejoin_context_active &&
        !final_align_phase &&
        !distance_reached &&
        goal_distance > this->rotate_in_place_goal_distance_;
      if (straight_segment)
      {
        debug_tracking_lookahead_distance = std::max(
          this->tracking_lookahead_distance_,
          this->straight_tracking_lookahead_distance_);
        if (debug_tracking_lookahead_distance > this->tracking_lookahead_distance_ + 1e-6)
        {
          tracking_target = this->select_tracking_target(debug_tracking_lookahead_distance, false);
          debug_has_tracking_target = this->has_tracking_target_pose_;
          debug_tracking_target_index = this->tracking_target_index_;
          debug_tracking_target_x = tracking_target.pose.position.x;
          debug_tracking_target_y = tracking_target.pose.position.y;
          tracking_target_distance = this->pose_distance(this->current_pose_, tracking_target);
          debug_tracking_target_distance = tracking_target_distance;
          target_dx = tracking_target.pose.position.x - this->current_pose_.pose.position.x;
          target_dy = tracking_target.pose.position.y - this->current_pose_.pose.position.y;
          target_heading = current_yaw;
          if ((target_dx * target_dx) + (target_dy * target_dy) > 1e-6)
          {
            target_heading = std::atan2(target_dy, target_dx);
          }
          heading_error = this->normalize_angle(target_heading - current_yaw);
          abs_heading_error = std::abs(heading_error);
        }
      }
      else
      {
        debug_tracking_lookahead_distance = this->tracking_lookahead_distance_;
      }
      debug_straight_segment = straight_segment;
      debug_path_curvature_score = straight_assessment.path_curvature_score;
      debug_lateral_error_m = straight_assessment.lateral_error_m;
      debug_target_dx = target_dx;
      debug_target_dy = target_dy;
      debug_target_heading = target_heading;
      debug_heading_error_raw = heading_error;
      debug_target_jump_m = had_tracking_target ?
        this->pose_distance(previous_tracking_target, tracking_target) : 0.0;
      if (
        this->structured_logging_enabled_ &&
        had_tracking_target &&
        debug_target_jump_m >= std::min(
        std::max(0.0, this->target_jump_warn_threshold_m_),
        std::max(0.0, this->rejoin_target_jump_threshold_m_)))
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
          "AMR_LOG schema=v1 component=controller event=target_jump_detected node=motion_controller goal_id=%u previous_idx=%zu nearest_idx=%zu candidate_idx=%zu selected_idx=%zu target_idx=%zu target_x=%.3f target_y=%.3f target_jump_m=%.3f threshold_m=%.3f rejoin_context_active=%s rejoin_activated=%s selection_reason=%s",
          this->latest_command_.command_id,
          previous_tracking_target_index,
          this->tracking_nearest_index_,
          this->tracking_candidate_index_,
          this->tracking_target_index_,
          this->tracking_target_index_,
          debug_tracking_target_x,
          debug_tracking_target_y,
          debug_target_jump_m,
          std::min(
            std::max(0.0, this->target_jump_warn_threshold_m_),
            std::max(0.0, this->rejoin_target_jump_threshold_m_)),
          bool_label(this->rejoin_context_active_),
          bool_label(target_jump_activated_rejoin),
          this->tracking_selection_reason_.c_str());
      }
      const bool rejoin_phase =
        rejoin_context_active &&
        !final_align_phase &&
        goal_distance > this->rotate_in_place_goal_distance_;
      const bool rejoin_settling = rejoin_phase &&command_settling;
      const bool rejoin_linear_gate =
        rejoin_phase &&
        tracking_target_distance >= std::max(0.0, this->rejoin_target_distance_threshold_);
      const bool heading_settled =
        !align_heading_at_goal ||
        abs_heading_error <= this->final_align_heading_deadband_;
      if (!final_heading_phase || !heading_settled)
      {
        this->final_align_holding_ = false;
        this->final_align_hold_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      }
      else if (!this->final_align_holding_)
      {
        this->final_align_hold_start_time_ = this->now();
        this->final_align_holding_ = true;
      }
      const bool final_align_stable =
        !final_heading_phase ||
        (heading_settled &&(
        this->final_align_settle_time_sec_ <= 1e-6 ||
        (
          this->final_align_holding_ &&
          (this->now() - this->final_align_hold_start_time_).seconds() >=
          this->final_align_settle_time_sec_)));
      const bool aligning_in_place = final_heading_phase &&!final_align_stable;

      double heading_error_for_control = heading_error;
      if (!final_align_phase &&straight_segment)
      {
        const double filter_alpha = this->clamp(this->straight_heading_filter_alpha_, 0.0, 1.0);
        if (!this->has_heading_error_filter_)
        {
          this->heading_error_filtered_ = heading_error;
          this->has_heading_error_filter_ = true;
        }
        else
        {
          this->heading_error_filtered_ = this->normalize_angle(
            (filter_alpha * heading_error) +
            ((1.0 - filter_alpha) * this->heading_error_filtered_));
        }
        heading_error_for_control = this->heading_error_filtered_;
      }
      else
      {
        this->has_heading_error_filter_ = false;
        this->heading_error_filtered_ = heading_error;
      }

      const double control_abs_heading_error = std::abs(heading_error_for_control);
      const double straight_release_threshold = std::max(
        std::max(0.0, this->straight_heading_release_threshold_),
        std::max(0.0, this->straight_heading_deadband_));
      const bool straight_control_limited =
        straight_segment &&
        control_abs_heading_error <= straight_release_threshold;
      const double active_heading_deadband = straight_control_limited ?
        std::max(0.0, this->straight_heading_deadband_) :
        std::max(0.0, this->tracking_heading_deadband_);
      const double active_heading_release_threshold = std::max(
        straight_control_limited ?
        std::max(0.0, this->straight_heading_release_threshold_) :
        std::max(0.0, this->tracking_heading_release_threshold_),
        active_heading_deadband);
      if (final_align_phase)
      {
        this->steering_hysteresis_active_ = true;
      }
      else if (this->steering_hysteresis_active_)
      {
        if (control_abs_heading_error <= active_heading_deadband)
        {
          this->steering_hysteresis_active_ = false;
        }
      }
      else if (control_abs_heading_error >= active_heading_release_threshold)
      {
        this->steering_hysteresis_active_ = true;
      }

      const bool suppress_small_heading_correction =
        !final_align_phase &&!this->steering_hysteresis_active_;
      const bool suppress_final_align_correction =
        final_heading_phase &&heading_settled;
      const double steering_heading_error =
        (suppress_small_heading_correction || suppress_final_align_correction) ?
        0.0 : heading_error_for_control;
      const double steering_abs_heading_error = std::abs(steering_heading_error);
      debug_heading_error_filtered = heading_error_for_control;
      debug_steering_heading_error = steering_heading_error;
      debug_steering_deadband_active =
        suppress_small_heading_correction || suppress_final_align_correction;
      debug_steering_hysteresis_state = steering_hysteresis_label(this->steering_hysteresis_active_);
      const std::string &pose_frame = this->current_pose_.header.frame_id;
      const std::string &plan_frame = this->latest_local_plan_.header.frame_id;
      const std::string &target_frame = tracking_target.header.frame_id;
      debug_pose_frame = pose_frame;
      debug_plan_frame = plan_frame;
      debug_target_frame = target_frame;
      const bool pose_plan_mismatch =
        !pose_frame.empty() &&!plan_frame.empty() &&pose_frame != plan_frame;
      const bool pose_target_mismatch =
        !pose_frame.empty() &&!target_frame.empty() &&pose_frame != target_frame;
      const bool plan_target_mismatch =
        !plan_frame.empty() &&!target_frame.empty() &&plan_frame != target_frame;
      if (
        this->structured_logging_enabled_ &&
        (pose_plan_mismatch || pose_target_mismatch || plan_target_mismatch))
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
          "AMR_LOG schema=v1 component=controller event=tracking_frame_mismatch node=motion_controller goal_id=%u pose_frame=%s plan_frame=%s target_frame=%s nearest_idx=%zu candidate_idx=%zu selected_idx=%zu selection_reason=%s",
          this->latest_command_.command_id,
          frame_label(pose_frame),
          frame_label(plan_frame),
          frame_label(target_frame),
          this->tracking_nearest_index_,
          this->tracking_candidate_index_,
          this->tracking_target_index_,
          this->tracking_selection_reason_.c_str());
      }
      double angular_speed_limit = final_align_phase ?
        std::min(this->max_angular_speed_, this->final_align_max_angular_speed_) :
        (straight_control_limited ?
        std::min(this->max_angular_speed_, std::max(0.0, this->straight_max_angular_speed_)) :
        this->max_angular_speed_);
      if (rejoin_settling)
      {
        angular_speed_limit = std::min(
          angular_speed_limit,
          std::max(0.05, std::min(this->straight_max_angular_speed_, this->max_angular_speed_ * 0.5)));
      }
      const double angular_gain_used = straight_control_limited ?
        std::max(0.0, this->straight_angular_gain_) :
        this->angular_gain_;
      const bool non_error_hold_phase = final_align_phase;

      const bool safety_gate_blocked = this->is_safety_gate_triggered();
      const bool blocked_candidate =
        safety_gate_blocked &&
        !command_settling &&
        !non_error_hold_phase;
      status.local_plan_valid = this->has_local_plan_ &&!this->latest_local_plan_.poses.empty();
      status.costmap_blocked = false;
      status.safety_gate_blocked = safety_gate_blocked;
      status.obstacle_detected = safety_gate_blocked;
      status.blocked = this->update_blocked_state(blocked_candidate);
      status.has_blocked_pose = false;
      status.blocked_pose = geometry_msgs::msg::PoseStamped();
      status.goal_reached = align_heading_at_goal ?
        (distance_reached &&final_align_stable) :
        goal_check.goal_reached;
      status.remaining_distance = goal_distance;
      status.heading_error = heading_error;
      if (status.goal_reached)
      {
        debug_goal_state = "reached";
      }
      else if (final_heading_phase)
      {
        debug_goal_state = final_align_stable ? "final_heading_settled" : "final_heading_align";
      }
      else if (distance_reached)
      {
        debug_goal_state = "xy_reached";
      }
      else if (goal_distance <= this->rotate_in_place_goal_distance_)
      {
        debug_goal_state = "goal_approach";
      }
      else
      {
        debug_goal_state = "tracking";
      }
      debug_rejoin_state = rejoin_phase ? "rejoin" : "tracking";

      if (!this->has_progress_reference_)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
        this->has_progress_reference_ = true;
      }
      else if (
        this->pose_distance(this->current_pose_, this->progress_reference_pose_) >=
        this->progress_required_movement_radius_)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      }
      else if (final_heading_phase)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      }
      else if (distance_reached &&final_align_stable)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      }
      else if (
        !status.blocked &&
        !command_settling &&
        !non_error_hold_phase &&
        (this->now() - this->progress_reference_time_).seconds() >=
        this->progress_time_allowance_sec_)
      {
        status.stalled = this->update_stalled_state(true);
      }
      else
      {
        status.stalled = this->update_stalled_state(false);
      }

      if (status.goal_reached)
      {
        status.command_completed = true;
        this->has_command_ = false;
        this->has_local_plan_ = false;
        this->current_twist_ = geometry_msgs::msg::Twist();
        this->reset_velocity_controller_state();
        this->reset_progress_checker_state();
        this->reset_goal_checker_state();
        this->reset_status_semantics_state();
        if (this->structured_logging_enabled_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "AMR_LOG schema=v1 component=controller event=goal_state node=motion_controller goal_id=%u phase=reached xy_reached=true yaw_reached=%s align_heading_at_goal=%s respect_goal_yaw=%s ignore_yaw=%s final_heading_required=%s dist_goal_m=%.3f goal_yaw_rad=%.3f current_yaw_rad=%.3f heading_err_rad=%.3f cmd_lin=%.3f cmd_ang=%.3f result=success",
            status.command_id,
            bool_label(goal_check.heading_reached),
            bool_label(this->latest_command_.align_heading_at_goal),
            bool_label(this->goal_checker_respect_goal_yaw_),
            bool_label(this->goal_checker_ignore_yaw_),
            bool_label(align_heading_at_goal),
            goal_distance,
            goal_yaw,
            current_yaw,
            heading_error,
            0.0,
            0.0);
        }
      }
      else if (distance_reached &&(!align_heading_at_goal || final_align_stable))
      {
        desired_twist = geometry_msgs::msg::Twist();
      }
      else if (status.blocked || status.stalled)
      {
        desired_twist = geometry_msgs::msg::Twist();
        if (
          status.blocked &&
          this->safety_gate_allow_rotate_in_place_ &&
          abs_heading_error > this->safety_gate_rotate_heading_threshold_)
        {
          const double safety_angular_speed_limit = final_align_phase ?
            std::min(this->max_angular_speed_, this->final_align_max_angular_speed_) :
            this->max_angular_speed_;
          desired_twist.angular.z = this->clamp(
            this->angular_gain_ * heading_error,
            -safety_angular_speed_limit,
            safety_angular_speed_limit);
        }
        if (this->structured_logging_enabled_)
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
            "AMR_LOG schema=v1 component=controller event=local_blocked_state node=motion_controller goal_id=%u phase=%s blocked=%s stalled=%s safety_blocked=%s dist_goal_m=%.3f heading_err_rad=%.3f cmd_lin=%.3f cmd_ang=%.3f",
            status.command_id,
            debug_goal_state.c_str(),
            bool_label(status.blocked),
            bool_label(status.stalled),
            bool_label(status.safety_gate_blocked),
            goal_distance,
            heading_error,
            desired_twist.linear.x,
            desired_twist.angular.z);
        }
      }
      else
      {
        desired_twist.angular.z = this->clamp(
          angular_gain_used * steering_heading_error,
          -angular_speed_limit,
          angular_speed_limit);

        const bool rotate_in_place_only =
          aligning_in_place ||
          (align_heading_at_goal &&
          goal_distance <= this->rotate_in_place_goal_distance_ &&
          abs_heading_error > this->rotate_in_place_threshold_);
        if (!rotate_in_place_only)
        {
          const double base_linear_speed = std::max(
            0.0, std::min(this->linear_speed_, goal_distance));
          double scale = 1.0;
          if (steering_abs_heading_error > this->heading_slowdown_threshold_)
          {
            const double scale_window = std::max(
              3.14159265358979323846 - this->heading_slowdown_threshold_,
              1e-6);
            scale = 1.0 - (
              (steering_abs_heading_error - this->heading_slowdown_threshold_) /
              scale_window);
          }
          if (rejoin_linear_gate &&steering_abs_heading_error > this->rejoin_heading_gate_threshold_)
          {
            const double gate_window = std::max(
              3.14159265358979323846 - this->rejoin_heading_gate_threshold_,
              1e-6);
            const double rejoin_scale = 1.0 - (
              (steering_abs_heading_error - this->rejoin_heading_gate_threshold_) /
              gate_window);
            scale *= this->clamp(rejoin_scale, this->rejoin_min_linear_scale_, 1.0);
          }
          if (rejoin_settling)
          {
            scale *= this->clamp(this->rejoin_min_linear_scale_, 0.0, 1.0);
          }
          const double goal_approach_distance = std::max(
            std::max(0.0, this->rotate_in_place_goal_distance_) * 2.0,
            std::max(0.0, this->goal_checker_xy_tolerance_) +
            std::max(0.0, this->goal_checker_xy_hysteresis_));
          if (goal_approach_distance > 1e-6 &&goal_distance <= goal_approach_distance)
          {
            scale *= this->clamp(goal_distance / goal_approach_distance, 0.20, 1.0);
          }

          scale = this->clamp(scale, this->min_heading_motion_scale_, 1.0);
          desired_twist.linear.x = base_linear_speed * scale;
          if (goal_distance > goal_approach_distance)
          {
            desired_twist.linear.x = std::max(
              std::min(this->min_linear_speed_, base_linear_speed),
              desired_twist.linear.x);
          }
        }
      }
    }
    else
    {
      this->reset_status_semantics_state();
      this->ensure_recovery_reference_initialized();
      debug_goal_state = std::string("recovery_") + motion_mode_label(this->latest_command_.mode);
      debug_rejoin_state = "recovery";
      const double elapsed_sec = (this->now() - this->recovery_start_time_).seconds();

      if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_BACKUP)
      {
        const double traveled = this->pose_distance(this->current_pose_, this->recovery_reference_pose_);
        const double remaining = std::max(0.0, this->latest_command_.recovery_distance - traveled);
        status.remaining_distance = remaining;
        debug_remaining_distance = remaining;
        if (
          remaining <= this->distance_tolerance_ ||
          (this->latest_command_.recovery_duration > 0.0 &&
          elapsed_sec >= this->latest_command_.recovery_duration))
        {
          status.command_completed = true;
        }
        else
        {
          desired_twist.linear.x =
            -std::max(this->latest_command_.recovery_speed, this->min_linear_speed_);
        }
      }
      else if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_SPIN)
      {
        const double target_yaw = this->recovery_start_yaw_ + this->latest_command_.recovery_angle;
        const double heading_error = this->normalize_angle(target_yaw - current_yaw);
        status.heading_error = heading_error;
        status.remaining_distance = std::abs(heading_error);
        debug_remaining_distance = status.remaining_distance;
        if (std::abs(heading_error) <= this->goal_heading_tolerance_)
        {
          status.command_completed = true;
        }
        else
        {
          desired_twist.angular.z = this->clamp(
            this->angular_gain_ * heading_error,
            -this->max_angular_speed_,
            this->max_angular_speed_);
        }
      }
      else if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_WAIT)
      {
        const double remaining = std::max(0.0, this->latest_command_.recovery_duration - elapsed_sec);
        status.remaining_distance = remaining;
        debug_remaining_distance = remaining;
        status.command_completed = remaining <= 1e-3;
      }

      if (status.command_completed)
      {
        this->has_command_ = false;
        this->current_twist_ = geometry_msgs::msg::Twist();
        this->reset_velocity_controller_state();
        this->reset_progress_checker_state();
        this->reset_goal_checker_state();
        this->reset_status_semantics_state();
        if (this->structured_logging_enabled_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "AMR_LOG schema=v1 component=controller event=motion_command node=motion_controller goal_id=%u mode=%s recovery=true recovery_type=%s result=completed duration_sec=%.3f",
            status.command_id,
            motion_mode_label(status.mode),
            motion_mode_label(status.mode),
            elapsed_sec);
        }
      }
    }
  }
  else
  {
    this->reset_status_semantics_state();
    status.goal_reached = true;
  }

  this->cmd_ang_sign_ = velocity_sign(desired_twist.angular.z);
  if (this->cmd_ang_sign_ != 0)
  {
    if (this->last_cmd_ang_sign_ != 0 &&this->last_cmd_ang_sign_ != this->cmd_ang_sign_)
    {
      ++this->cmd_ang_flip_count_;
    }
    this->last_cmd_ang_sign_ = this->cmd_ang_sign_;
  }

  this->current_twist_ = this->apply_velocity_controller(this->current_twist_, desired_twist);
  output_twist = this->current_twist_;
  this->output_ang_sign_ = velocity_sign(output_twist.angular.z);
  if (this->output_ang_sign_ != 0)
  {
    if (
      this->last_output_ang_sign_ != 0 &&
      this->last_output_ang_sign_ != this->output_ang_sign_)
    {
      ++this->output_ang_flip_count_;
    }
    this->last_output_ang_sign_ = this->output_ang_sign_;
  }

  this->cmd_vel_publisher_->publish(output_twist);
  this->motion_status_publisher_->publish(status);

  if (status.active &&this->structured_logging_enabled_)
  {
    if (status.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&debug_has_tracking_target)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
        "AMR_LOG schema=v1 component=controller event=tracking_heading_debug node=motion_controller goal_id=%u pose_frame=%s plan_frame=%s target_frame=%s nearest_idx=%zu candidate_idx=%zu selected_idx=%zu target_dist_m=%.3f target_dx=%.3f target_dy=%.3f current_yaw_rad=%.3f target_heading_rad=%.3f heading_err_rad=%.3f steering_err_rad=%.3f rejoin_context_active=%s straight_segment=%s path_curvature_score=%.3f lateral_error_m=%.3f heading_error_raw_rad=%.3f heading_error_filtered_rad=%.3f steering_deadband_active=%s steering_hysteresis_state=%s cmd_ang_sign=%d cmd_ang_flip_count=%d output_ang_sign=%d output_ang_flip_count=%d selection_reason=%s",
        status.command_id,
        frame_label(debug_pose_frame),
        frame_label(debug_plan_frame),
        frame_label(debug_target_frame),
        this->tracking_nearest_index_,
        this->tracking_candidate_index_,
        this->tracking_target_index_,
        debug_tracking_target_distance,
        debug_target_dx,
        debug_target_dy,
        debug_current_yaw,
        debug_target_heading,
        debug_heading_error_raw,
        debug_steering_heading_error,
        bool_label(debug_rejoin_context_active),
        bool_label(debug_straight_segment),
        debug_path_curvature_score,
        debug_lateral_error_m,
        debug_heading_error_raw,
        debug_heading_error_filtered,
        bool_label(debug_steering_deadband_active),
        debug_steering_hysteresis_state,
        this->cmd_ang_sign_,
        this->cmd_ang_flip_count_,
        this->output_ang_sign_,
        this->output_ang_flip_count_,
        this->tracking_selection_reason_.c_str());
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
      "AMR_LOG schema=v1 component=controller event=tracking_state node=motion_controller goal_id=%u mode=%s phase=%s rejoin=%s rejoin_context_active=%s tracking=%s target_idx=%zu target_x=%.3f target_y=%.3f target_dist_m=%.3f target_jump_m=%.3f lookahead_m=%.3f path_points=%zu dist_goal_m=%.3f heading_err_rad=%.3f straight_segment=%s path_curvature_score=%.3f lateral_error_m=%.3f heading_error_raw_rad=%.3f heading_error_filtered_rad=%.3f steering_deadband_active=%s steering_hysteresis_state=%s blocked=%s safety_blocked=%s recovery=%s cmd_lin=%.3f cmd_ang=%.3f output_lin=%.3f output_ang=%.3f cmd_ang_sign=%d cmd_ang_flip_count=%d output_ang_sign=%d output_ang_flip_count=%d",
      status.command_id,
      motion_mode_label(status.mode),
      debug_goal_state.c_str(),
      bool_label(debug_rejoin_state == "rejoin"),
      bool_label(debug_rejoin_context_active),
      bool_label(debug_has_tracking_target),
      debug_tracking_target_index,
      debug_tracking_target_x,
      debug_tracking_target_y,
      debug_tracking_target_distance,
      debug_target_jump_m,
      debug_tracking_lookahead_distance,
      this->latest_local_plan_.poses.size(),
      debug_remaining_distance,
      status.heading_error,
      bool_label(debug_straight_segment),
      debug_path_curvature_score,
      debug_lateral_error_m,
      debug_heading_error_raw,
      debug_heading_error_filtered,
      bool_label(debug_steering_deadband_active),
      debug_steering_hysteresis_state,
      bool_label(status.blocked || status.stalled),
      bool_label(status.safety_gate_blocked),
      bool_label(status.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE),
      desired_twist.linear.x,
      desired_twist.angular.z,
      output_twist.linear.x,
      output_twist.angular.z,
      this->cmd_ang_sign_,
      this->cmd_ang_flip_count_,
      this->output_ang_sign_,
      this->output_ang_flip_count_);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      throttle_ms_from_sec(this->cmd_quality_log_throttle_sec_),
      "AMR_LOG schema=v1 component=controller event=cmd_quality node=motion_controller goal_id=%u phase=%s cmd_lin=%.3f cmd_ang=%.3f output_lin=%.3f output_ang=%.3f blocked=%s safety_blocked=%s last_cmd_age_sec=%.3f rejoin_context_active=%s straight_segment=%s steering_deadband_active=%s cmd_ang_sign=%d cmd_ang_flip_count=%d output_ang_sign=%d output_ang_flip_count=%d",
      status.command_id,
      debug_goal_state.c_str(),
      desired_twist.linear.x,
      desired_twist.angular.z,
      output_twist.linear.x,
      output_twist.angular.z,
      bool_label(status.blocked || status.stalled),
      bool_label(status.safety_gate_blocked),
      this->latest_command_time_.nanoseconds() > 0 ?
      (this->now() - this->latest_command_time_).seconds() : 0.0,
      bool_label(debug_rejoin_context_active),
      bool_label(debug_straight_segment),
      bool_label(debug_steering_deadband_active),
      this->cmd_ang_sign_,
      this->cmd_ang_flip_count_,
      this->output_ang_sign_,
      this->output_ang_flip_count_);

    if (debug_goal_state != "tracking" &&debug_goal_state != "idle")
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
        "AMR_LOG schema=v1 component=controller event=goal_state node=motion_controller goal_id=%u phase=%s xy_reached=%s yaw_reached=%s align_heading_at_goal=%s respect_goal_yaw=%s ignore_yaw=%s final_heading_required=%s dist_goal_m=%.3f goal_yaw_rad=%.3f current_yaw_rad=%.3f heading_err_rad=%.3f cmd_lin=%.3f cmd_ang=%.3f",
        status.command_id,
        debug_goal_state.c_str(),
        bool_label(debug_xy_reached),
        bool_label(debug_yaw_reached),
        bool_label(debug_command_align_heading_at_goal),
        bool_label(this->goal_checker_respect_goal_yaw_),
        bool_label(this->goal_checker_ignore_yaw_),
        bool_label(debug_final_heading_required),
        status.remaining_distance,
        debug_goal_yaw,
        debug_current_yaw,
        status.heading_error,
        desired_twist.linear.x,
        desired_twist.angular.z);
    }

    if (debug_rejoin_state == "rejoin")
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        throttle_ms_from_sec(this->tracking_state_log_throttle_sec_),
        "AMR_LOG schema=v1 component=controller event=rejoin_state node=motion_controller goal_id=%u phase=rejoin rejoin_context_active=%s target_idx=%zu target_x=%.3f target_y=%.3f target_jump_m=%.3f dist_goal_m=%.3f heading_err_rad=%.3f cmd_lin=%.3f cmd_ang=%.3f",
        status.command_id,
        bool_label(debug_rejoin_context_active),
        debug_tracking_target_index,
        debug_tracking_target_x,
        debug_tracking_target_y,
        debug_target_jump_m,
        status.remaining_distance,
        status.heading_error,
        desired_twist.linear.x,
        desired_twist.angular.z);
    }
  }
}

void MotionController::reset_velocity_controller_state()
{
  this->linear_controller_state_ = AxisControllerState{};
  this->angular_controller_state_ = AxisControllerState{};
}

void MotionController::reset_progress_checker_state()
{
  this->progress_reference_pose_ = geometry_msgs::msg::PoseStamped();
  this->recovery_reference_pose_ = geometry_msgs::msg::PoseStamped();
  this->progress_reference_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->recovery_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->recovery_start_yaw_ = 0.0;
  this->has_progress_reference_ = false;
  this->has_recovery_reference_ = false;
}

void MotionController::reset_goal_checker_state()
{
  this->goal_checker_hold_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->goal_checker_holding_ = false;
  this->final_align_hold_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->final_align_holding_ = false;
  this->goal_xy_latched_ = false;
}

void MotionController::reset_status_semantics_state()
{
  this->latest_command_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->blocked_latched_ = false;
  this->blocked_streak_ = 0;
  this->blocked_clear_streak_ = 0;
  this->stalled_streak_ = 0;
  this->reset_tracking_diagnostics_state();
}

void MotionController::reset_tracking_diagnostics_state()
{
  this->steering_hysteresis_active_ = false;
  this->has_heading_error_filter_ = false;
  this->heading_error_filtered_ = 0.0;
  this->cmd_ang_sign_ = 0;
  this->last_cmd_ang_sign_ = 0;
  this->cmd_ang_flip_count_ = 0;
  this->output_ang_sign_ = 0;
  this->last_output_ang_sign_ = 0;
  this->output_ang_flip_count_ = 0;
}

void MotionController::publish_zero_twist()
{
  this->current_twist_ = geometry_msgs::msg::Twist();
  if (!this->cmd_vel_publisher_ || !this->cmd_vel_publisher_->is_activated())
  {
    return;
  }

  this->cmd_vel_publisher_->publish(this->current_twist_);
}

void MotionController::ensure_recovery_reference_initialized()
{
  if (this->has_recovery_reference_)
  {
    return;
  }

  this->recovery_reference_pose_ = this->current_pose_;
  this->recovery_start_time_ = this->now();
  this->recovery_start_yaw_ = this->quaternion_yaw(this->current_pose_.pose.orientation);
  this->has_recovery_reference_ = true;
}

void MotionController::reset_rejoin_context_state()
{
  this->rejoin_context_active_ = false;
  this->rejoin_context_pending_ = false;
  this->rejoin_context_start_pose_ = geometry_msgs::msg::PoseStamped();
  this->rejoin_context_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
}

void MotionController::activate_rejoin_context()
{
  this->rejoin_context_active_ = true;
  this->rejoin_context_pending_ = false;
  this->rejoin_context_start_pose_ = this->current_pose_;
  this->rejoin_context_start_time_ = this->now();
}

void MotionController::update_rejoin_context_state()
{
  if (!this->rejoin_context_active_)
  {
    return;
  }

  const double timeout_sec = std::max(0.0, this->rejoin_context_timeout_sec_);
  const double distance_m = std::max(0.0, this->rejoin_context_distance_m_);
  const bool timed_out =
    timeout_sec > 1e-6 &&
    this->rejoin_context_start_time_.nanoseconds() > 0 &&
    (this->now() - this->rejoin_context_start_time_).seconds() >= timeout_sec;
  const bool distance_reached =
    distance_m > 1e-6 &&
    this->pose_distance(this->current_pose_, this->rejoin_context_start_pose_) >= distance_m;
  if (timed_out || distance_reached)
  {
    this->rejoin_context_active_ = false;
  }
}

MotionController::VelocityControlMode MotionController::parse_velocity_control_mode(
  const std::string &mode) const
{
  std::string normalized = mode;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });

  if (normalized == "p")
  {
    return VelocityControlMode::P;
  }
  if (normalized == "pi")
  {
    return VelocityControlMode::PI;
  }
  if (normalized == "pid")
  {
    return VelocityControlMode::PID;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "Unknown velocity_controller.mode '%s'; falling back to pid",
    mode.c_str());
  return VelocityControlMode::PID;
}

double MotionController::apply_axis_controller(
  const double current,
  const double target,
  AxisControllerState &state,
  const AxisControllerConfig &config,
  const double max_step,
  const double dt) const
{
  if (std::abs(target) <= 1e-6)
  {
    state.integral = 0.0;
    state.previous_error = 0.0;
    state.first_update = false;
    return 0.0;
  }

  const double error = target - current;
  double derivative = 0.0;

  if (
    this->velocity_control_mode_ == VelocityControlMode::PI ||
    this->velocity_control_mode_ == VelocityControlMode::PID)
  {
    state.integral = this->clamp(
      state.integral + (error * dt),
      -config.integral_limit,
      config.integral_limit);
  }
  else
  {
    state.integral = 0.0;
  }

  if (
    !state.first_update &&
    this->velocity_control_mode_ == VelocityControlMode::PID &&
    dt > 1e-6)
  {
    derivative = (error - state.previous_error) / dt;
  }

  double control_delta = config.kp * error;
  if (
    this->velocity_control_mode_ == VelocityControlMode::PI ||
    this->velocity_control_mode_ == VelocityControlMode::PID)
  {
    control_delta += config.ki * state.integral;
  }
  if (this->velocity_control_mode_ == VelocityControlMode::PID)
  {
    control_delta += config.kd * derivative;
  }

  control_delta = this->clamp(control_delta, -max_step, max_step);
  double next = current + control_delta;

  if (target >= current)
  {
    next = std::min(next, target);
  }
  else
  {
    next = std::max(next, target);
  }

  state.previous_error = error;
  state.first_update = false;
  return next;
}

geometry_msgs::msg::Twist MotionController::apply_velocity_controller(
  const geometry_msgs::msg::Twist &current,
  const geometry_msgs::msg::Twist &target)
{
  geometry_msgs::msg::Twist controlled = current;
  const double dt = 1.0 / std::max(this->control_frequency_, 1.0);
  const double max_linear_step = this->max_linear_accel_ * dt;
  const double max_angular_step = this->max_angular_accel_ * dt;

  controlled.linear.x = this->apply_axis_controller(
    current.linear.x,
    target.linear.x,
    this->linear_controller_state_,
    this->linear_controller_config_,
    max_linear_step,
    dt);
  controlled.angular.z = this->apply_axis_controller(
    current.angular.z,
    target.angular.z,
    this->angular_controller_state_,
    this->angular_controller_config_,
    max_angular_step,
    dt);
  return controlled;
}

double MotionController::estimate_remaining_distance(const nav_msgs::msg::Path &path) const
{
  double distance = 0.0;
  if (path.poses.size() < 2U)
  {
    return distance;
  }

  for (std::size_t index = 1; index < path.poses.size(); ++index)
  {
    const geometry_msgs::msg::Point &previous = path.poses[index - 1].pose.position;
    const geometry_msgs::msg::Point &current = path.poses[index].pose.position;
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    distance += std::sqrt((dx * dx) + (dy * dy));
  }

  return distance;
}

double MotionController::quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation) const
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MotionController::normalize_angle(double angle) const
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi)
  {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi)
  {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MotionController::clamp(
  const double value,
  const double min_value,
  const double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

MotionController::StraightSegmentAssessment MotionController::assess_straight_segment(
  const std::size_t start_index,
  const double lookahead_distance) const
{
  StraightSegmentAssessment assessment;
  const std::size_t plan_size = this->latest_local_plan_.poses.size();
  if (plan_size < 2U)
  {
    return assessment;
  }

  const std::size_t bounded_start = std::min(start_index, plan_size - 1U);
  const double target_window = std::max(0.0, lookahead_distance);
  std::size_t end_index = bounded_start;
  double accumulated_distance = 0.0;
  for (std::size_t index = bounded_start + 1U; index < plan_size; ++index)
  {
    accumulated_distance += this->pose_distance(
      this->latest_local_plan_.poses[index - 1U],
      this->latest_local_plan_.poses[index]);
    end_index = index;
    if (accumulated_distance >= target_window)
    {
      break;
    }
  }

  if (end_index == bounded_start && bounded_start + 1U < plan_size)
  {
    end_index = bounded_start + 1U;
  }

  assessment.start_index = bounded_start;
  assessment.end_index = end_index;
  if (end_index <= bounded_start)
  {
    return assessment;
  }

  const geometry_msgs::msg::Point &start = this->latest_local_plan_.poses[bounded_start].pose.position;
  const geometry_msgs::msg::Point &end = this->latest_local_plan_.poses[end_index].pose.position;
  const double line_dx = end.x - start.x;
  const double line_dy = end.y - start.y;
  const double line_length = std::sqrt((line_dx * line_dx) + (line_dy * line_dy));
  assessment.segment_length_m = line_length;
  if (line_length <= 1e-6)
  {
    return assessment;
  }

  const double baseline_heading = std::atan2(line_dy, line_dx);
  double max_lateral_error = 0.0;
  double max_heading_delta = 0.0;
  for (std::size_t index = bounded_start + 1U; index <= end_index; ++index)
  {
    const geometry_msgs::msg::Point &previous =
      this->latest_local_plan_.poses[index - 1U].pose.position;
    const geometry_msgs::msg::Point &current = this->latest_local_plan_.poses[index].pose.position;
    const double segment_dx = current.x - previous.x;
    const double segment_dy = current.y - previous.y;
    if ((segment_dx * segment_dx) + (segment_dy * segment_dy) > 1e-8)
    {
      const double segment_heading = std::atan2(segment_dy, segment_dx);
      max_heading_delta = std::max(
        max_heading_delta,
        std::abs(this->normalize_angle(segment_heading - baseline_heading)));
    }

    const double point_dx = current.x - start.x;
    const double point_dy = current.y - start.y;
    const double lateral_error = std::abs((point_dx * line_dy) - (point_dy * line_dx)) / line_length;
    max_lateral_error = std::max(max_lateral_error, lateral_error);
  }

  assessment.path_curvature_score = max_heading_delta;
  assessment.lateral_error_m = max_lateral_error;
  assessment.straight_segment =
    max_heading_delta <= std::max(0.0, this->straight_curvature_threshold_) &&
    max_lateral_error <= std::max(0.0, this->straight_lateral_error_threshold_);
  return assessment;
}

geometry_msgs::msg::PoseStamped MotionController::select_tracking_target(
  const double lookahead_distance,
  const bool allow_retained_target)
{
  const double min_target_distance = std::max(0.0, this->tracking_min_target_distance_);
  const double effective_lookahead_distance = std::max(0.0, lookahead_distance);
  auto select_pose = [this](
    const geometry_msgs::msg::PoseStamped &target,
    const std::size_t selected_index,
    const bool from_plan,
    const std::string &selection_reason) -> geometry_msgs::msg::PoseStamped
    {
      this->tracking_target_index_ = selected_index;
      this->has_tracking_target_index_ = from_plan;
      this->tracking_target_pose_ = target;
      this->has_tracking_target_pose_ = true;
      this->tracking_target_from_plan_ = from_plan;
      this->tracking_selected_target_distance_ = this->pose_distance(this->current_pose_, target);
      this->tracking_selection_reason_ = selection_reason;
      return target;
    };

  if (this->latest_local_plan_.poses.empty())
  {
    this->tracking_progress_index_ = 0U;
    this->tracking_nearest_index_ = 0U;
    this->tracking_candidate_index_ = 0U;
    this->has_tracking_progress_index_ = false;
    return select_pose(this->latest_command_.goal_pose, 0U, false, "local_plan_empty");
  }

  const std::size_t plan_size = this->latest_local_plan_.poses.size();
  const std::size_t previous_progress_index = this->has_tracking_progress_index_ ?
    std::min(this->tracking_progress_index_, plan_size - 1U) : 0U;
  const std::size_t rollback_window = std::min(
    this->tracking_progress_rollback_window_,
    previous_progress_index);
  const std::size_t search_start = previous_progress_index - rollback_window;

  std::size_t nearest_index = search_start;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = search_start; index < plan_size; ++index)
  {
    const double distance = this->pose_distance(this->current_pose_, this->latest_local_plan_.poses[index]);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  this->tracking_progress_index_ = nearest_index;
  this->tracking_nearest_index_ = nearest_index;
  this->has_tracking_progress_index_ = true;

  if (plan_size == 1U)
  {
    this->tracking_candidate_index_ = 0U;
    const double single_pose_distance = this->pose_distance(
      this->current_pose_, this->latest_local_plan_.poses.front());
    const double goal_distance = this->pose_distance(this->current_pose_, this->latest_command_.goal_pose);
    if (single_pose_distance < min_target_distance &&goal_distance >= min_target_distance)
    {
      return select_pose(
        this->latest_command_.goal_pose, 0U, false, "local_plan_too_short_goal_fallback");
    }
    if (single_pose_distance < min_target_distance)
    {
      return select_pose(
        this->latest_local_plan_.poses.front(), 0U, true, "local_plan_too_short_goal_proximity");
    }
    return select_pose(this->latest_local_plan_.poses.front(), 0U, true, "local_plan_single_pose");
  }

  std::size_t candidate_target_index = nearest_index;
  geometry_msgs::msg::PoseStamped candidate_target_pose = this->latest_local_plan_.poses[nearest_index];
  bool candidate_target_from_plan = true;
  std::string candidate_selection_reason = nearest_distance < min_target_distance ?
    "nearest_target_too_close" : "nearest";
  double accumulated_distance = 0.0;
  for (std::size_t index = nearest_index + 1U; index < plan_size; ++index)
  {
    const geometry_msgs::msg::PoseStamped &previous = this->latest_local_plan_.poses[index - 1U];
    const geometry_msgs::msg::PoseStamped &current = this->latest_local_plan_.poses[index];
    accumulated_distance += this->pose_distance(previous, current);
    const double target_distance = this->pose_distance(this->current_pose_, current);
    if (target_distance < min_target_distance)
    {
      continue;
    }
    candidate_target_index = index;
    candidate_target_pose = current;
    candidate_selection_reason = accumulated_distance >= effective_lookahead_distance ?
      "candidate_lookahead" : "candidate_min_distance";
    if (accumulated_distance >= effective_lookahead_distance)
    {
      break;
    }
  }
  if (candidate_target_index == nearest_index)
  {
    candidate_target_index = std::min(nearest_index + 1U, plan_size - 1U);
    candidate_target_pose = this->latest_local_plan_.poses[candidate_target_index];
    const double candidate_target_distance = this->pose_distance(this->current_pose_, candidate_target_pose);
    const double goal_distance = this->pose_distance(this->current_pose_, this->latest_command_.goal_pose);
    if (candidate_target_distance < min_target_distance &&goal_distance >= min_target_distance)
    {
      candidate_target_pose = this->latest_command_.goal_pose;
      candidate_target_from_plan = false;
      candidate_selection_reason = "candidate_goal_fallback_target_too_close";
    }
    else
    {
      candidate_selection_reason = candidate_target_distance < min_target_distance ?
        "candidate_next_pose_too_close" : "candidate_next_pose";
    }
  }

  if (this->has_tracking_target_index_ && candidate_target_from_plan)
  {
    const std::size_t previous_target_index = std::min(this->tracking_target_index_, plan_size - 1U);
    const std::size_t rollback_limit =
      previous_target_index > this->tracking_progress_rollback_window_ ?
      previous_target_index - this->tracking_progress_rollback_window_ : 0U;
    if (candidate_target_index < rollback_limit)
    {
      candidate_target_index = rollback_limit;
      candidate_target_pose = this->latest_local_plan_.poses[candidate_target_index];
      candidate_selection_reason = "candidate_rollback_limited";
    }
  }
  this->tracking_candidate_index_ = candidate_target_index;

  std::size_t selected_target_index = candidate_target_index;
  geometry_msgs::msg::PoseStamped selected_target_pose = candidate_target_pose;
  bool selected_target_from_plan = candidate_target_from_plan;
  std::string selected_selection_reason = candidate_selection_reason;
  if (allow_retained_target &&this->has_tracking_target_pose_)
  {
    std::size_t retained_target_index = nearest_index;
    double retained_plan_distance = std::numeric_limits<double>::max();
    for (std::size_t index = nearest_index; index < plan_size; ++index)
    {
      const double distance = this->pose_distance(
        this->tracking_target_pose_,
        this->latest_local_plan_.poses[index]);
      if (distance < retained_plan_distance)
      {
        retained_plan_distance = distance;
        retained_target_index = index;
      }
    }

    const double retained_target_distance = this->pose_distance(
      this->current_pose_,
      this->latest_local_plan_.poses[retained_target_index]);
    const double candidate_target_distance = this->pose_distance(
      this->current_pose_,
      candidate_target_pose);
    const bool retained_target_matches_plan =
      retained_plan_distance <= this->tracking_target_reset_distance_;
    const bool retained_target_is_forward = retained_target_index >= nearest_index;
    const bool retained_target_is_start_pose = retained_target_index == 0U;
    const bool retained_target_too_close = retained_target_distance < min_target_distance;
    const bool candidate_materially_better =
      candidate_target_distance + this->tracking_target_hysteresis_distance_ < retained_target_distance;

    if (
      retained_target_matches_plan &&retained_target_is_forward &&
      !retained_target_is_start_pose &&!retained_target_too_close &&
      !candidate_materially_better)
    {
      selected_target_index = retained_target_index;
      selected_target_pose = this->latest_local_plan_.poses[retained_target_index];
      selected_target_from_plan = true;
      selected_selection_reason = "retained_target";
    }
    else if (retained_target_too_close)
    {
      selected_selection_reason = "retained_target_too_close";
    }
    else if (retained_target_is_start_pose)
    {
      selected_selection_reason = "retained_idx0_current_pose";
    }
  }

  return select_pose(
    selected_target_pose,
    selected_target_index,
    selected_target_from_plan,
    selected_selection_reason);
}

MotionController::GoalCheckResult MotionController::check_goal(
  const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &goal_pose,
  const double current_yaw)
{
  GoalCheckResult result;
  result.distance_error = this->pose_distance(current_pose, goal_pose);
  const double xy_tolerance = std::max(0.0, this->goal_checker_xy_tolerance_);
  const double xy_release_tolerance =
    xy_tolerance + std::max(0.0, this->goal_checker_xy_hysteresis_);
  if (this->goal_xy_latched_)
  {
    this->goal_xy_latched_ = result.distance_error <= xy_release_tolerance;
  }
  else
  {
    this->goal_xy_latched_ = result.distance_error <= xy_tolerance;
  }
  result.distance_reached = this->goal_xy_latched_;
  result.align_heading =
    (this->goal_checker_respect_goal_yaw_ || this->latest_command_.align_heading_at_goal) &&
    !this->goal_checker_ignore_yaw_;
  result.target_yaw = this->quaternion_yaw(goal_pose.pose.orientation);
  result.heading_error = this->normalize_angle(result.target_yaw - current_yaw);
  const double heading_tolerance = result.align_heading ?
    this->goal_reach_heading_tolerance_ :
    this->goal_checker_yaw_tolerance_;
  result.heading_reached =
    !result.align_heading || std::abs(result.heading_error) <= heading_tolerance;

  const bool reached_now = result.distance_reached &&result.heading_reached;
  if (!reached_now)
  {
    this->goal_checker_hold_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    this->goal_checker_holding_ = false;
    if (!result.distance_reached)
    {
      this->final_align_hold_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      this->final_align_holding_ = false;
    }
    result.goal_reached = false;
    return result;
  }

  if (this->goal_checker_hold_time_sec_ <= 1e-6)
  {
    result.goal_reached = true;
    return result;
  }

  if (!this->goal_checker_holding_)
  {
    this->goal_checker_hold_start_time_ = this->now();
    this->goal_checker_holding_ = true;
  }

  result.goal_reached =
    (this->now() - this->goal_checker_hold_start_time_).seconds() >=
    this->goal_checker_hold_time_sec_;
  return result;
}

bool MotionController::is_safety_gate_triggered() const
{
  if (!this->safety_gate_enabled_ || !this->has_latest_scan_)
  {
    return false;
  }

  const double half_angle_rad =
    (this->safety_gate_forward_angle_deg_ * 3.14159265358979323846 / 180.0) * 0.5;
  int hit_count = 0;

  for (std::size_t index = 0; index < this->latest_scan_.ranges.size(); ++index)
  {
    const double angle =
      this->latest_scan_.angle_min +
      (static_cast<double>(index) * this->latest_scan_.angle_increment);
    if (std::abs(angle) > half_angle_rad)
    {
      continue;
    }

    const double range = this->latest_scan_.ranges[index];
    if (!std::isfinite(range))
    {
      continue;
    }
    if (
      range < this->latest_scan_.range_min ||
      range > this->latest_scan_.range_max)
    {
      continue;
    }
    if (range <= this->safety_gate_stop_distance_)
    {
      ++hit_count;
      if (hit_count >= this->safety_gate_min_points_)
      {
        return true;
      }
    }
  }

  return false;
}

bool MotionController::update_blocked_state(const bool blocked_candidate)
{
  if (blocked_candidate)
  {
    this->blocked_clear_streak_ = 0;
    this->blocked_streak_ += 1;
    if (this->blocked_streak_ >= std::max(1, this->status_blocked_confirm_cycles_))
    {
      this->blocked_latched_ = true;
    }
  }
  else
  {
    this->blocked_streak_ = 0;
    if (this->blocked_latched_)
    {
      this->blocked_clear_streak_ += 1;
      if (this->blocked_clear_streak_ >= std::max(1, this->status_blocked_clear_cycles_))
      {
        this->blocked_latched_ = false;
        this->blocked_clear_streak_ = 0;
      }
    }
  }

  return this->blocked_latched_;
}

bool MotionController::update_stalled_state(const bool stalled_candidate)
{
  if (!stalled_candidate)
  {
    this->stalled_streak_ = 0;
    return false;
  }

  this->stalled_streak_ += 1;
  return this->stalled_streak_ >= std::max(1, this->status_stalled_confirm_cycles_);
}

double MotionController::pose_distance(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr::motion::controller

namespace amr::controller::server
{

ControllerServer::ControllerServer()
: local_planner_(std::make_shared<amr::planner::local::LocalPlanner>()),
  motion_controller_(std::make_shared<amr::motion::controller::MotionController>())
{
}

void ControllerServer::spin()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->local_planner_->get_node_base_interface());
  executor.add_node(this->motion_controller_->get_node_base_interface());
  executor.spin();
}

}  // namespace amr::controller::server
