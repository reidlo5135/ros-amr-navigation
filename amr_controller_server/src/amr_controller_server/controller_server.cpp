#include "amr_controller_server/controller_server.hpp"


namespace amr::planner::local
{

namespace
{

constexpr int kUnknownCellValue = -1;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  return std::atan2(
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)),
    1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)));
}

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell &other) const
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
  bool operator()(const OpenSetEntry &lhs, const OpenSetEntry &rhs) const
  {
    return lhs.f_cost > rhs.f_cost;
  }
};

bool is_within_bounds(const GridCell &cell, int width, int height)
{
  return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
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
  if (!footprint_polygon.empty() && resolution > 0.0 && height > 0)
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
  if (previous_dx == next_dx && previous_dy == next_dy)
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

      const bool diagonal = neighbor.x != current_cell.x && neighbor.y != current_cell.y;
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
  path_refiner_corner_smoothing_enabled_(true),
  path_refiner_corner_smoothing_max_offset_(0.12),
  path_refiner_corner_smoothing_angle_threshold_(0.35),
  path_refiner_corner_smoothing_samples_(3),
  path_refiner_collision_check_enabled_(true),
  path_refiner_collision_sample_distance_(0.05),
  dynamic_obstacle_enabled_(true),
  dynamic_obstacle_replan_lookahead_distance_(1.4),
  dynamic_obstacle_escape_forward_distance_(1.2),
  dynamic_obstacle_escape_lateral_distance_(0.55),
  dynamic_obstacle_goal_proximity_disable_distance_(0.45),
  dynamic_obstacle_corridor_relax_distance_(0.75),
  dynamic_obstacle_recovery_confirm_cycles_(3),
  dynamic_obstacle_goal_proximity_confirm_cycles_(2),
  dynamic_obstacle_corridor_confirm_cycles_(5),
  nearest_free_search_radius_cells_(4),
  last_command_id_(0U),
  dynamic_blocked_decision_(amr_msgs::msg::LocalPlanStatus::DECISION_OK),
  dynamic_blocked_streak_(0),
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
    "path_refiner.corner_smoothing_enabled", this->path_refiner_corner_smoothing_enabled_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_max_offset", this->path_refiner_corner_smoothing_max_offset_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_angle_threshold",
    this->path_refiner_corner_smoothing_angle_threshold_);
  this->declare_parameter(
    "path_refiner.corner_smoothing_samples", this->path_refiner_corner_smoothing_samples_);
  this->declare_parameter(
    "path_refiner.collision_check_enabled", this->path_refiner_collision_check_enabled_);
  this->declare_parameter(
    "path_refiner.collision_sample_distance", this->path_refiner_collision_sample_distance_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_param_);
  this->declare_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
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
    "path_refiner.corner_smoothing_enabled", this->path_refiner_corner_smoothing_enabled_);
  this->get_parameter(
    "path_refiner.corner_smoothing_max_offset", this->path_refiner_corner_smoothing_max_offset_);
  this->get_parameter(
    "path_refiner.corner_smoothing_angle_threshold",
    this->path_refiner_corner_smoothing_angle_threshold_);
  this->get_parameter(
    "path_refiner.corner_smoothing_samples", this->path_refiner_corner_smoothing_samples_);
  this->get_parameter(
    "path_refiner.collision_check_enabled", this->path_refiner_collision_check_enabled_);
  this->get_parameter(
    "path_refiner.collision_sample_distance", this->path_refiner_collision_sample_distance_);
  this->get_parameter("footprint.polygon", this->footprint_polygon_param_);
  this->get_parameter("dynamic_obstacle.enabled", this->dynamic_obstacle_enabled_);
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
  if (build_result.local_plan_valid)
  {
    build_result.plan = this->refine_local_plan(build_result.plan);
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

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Publishing local plan for command %u from progress index %zu with %zu poses",
    this->latest_command_.command_id,
    this->last_progress_index_,
    build_result.plan.poses.size());
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

  if (this->has_map_ && !this->inflated_map_.data.empty())
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
        this->reset_dynamic_blocked_state();
        result.decision = amr_msgs::msg::LocalPlanStatus::DECISION_OK;
        result.has_blocked_pose = true;
        result.blocked_pose = blocked_pose;
        result.blocked_distance = this->pose_distance(current_pose, blocked_pose);
      }
      else
      {
        this->reset_dynamic_blocked_state();
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
    this->reset_dynamic_blocked_state();
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

  if (best_grid_path.empty() && !build_grid_plan_to_pose(sliced_plan.poses.back(), best_grid_path))
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
  local_plan.poses.push_back(current_pose);

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped segment_start = current_pose;
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

nav_msgs::msg::Path LocalPlanner::refine_local_plan(const nav_msgs::msg::Path &plan) const
{
  if (!this->path_refiner_enabled_ || plan.poses.size() < 2U)
  {
    return plan;
  }

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

    if (!final_pose && segment_distance < this->path_refiner_prune_distance_)
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

  if (this->path_refiner_heading_assignment_enabled_)
  {
    this->assign_path_headings(refined_plan);
  }

  if (this->path_refiner_corner_smoothing_enabled_)
  {
    nav_msgs::msg::Path smoothed_plan = this->smooth_path_corners(refined_plan);
    if (this->path_refiner_heading_assignment_enabled_)
    {
      this->assign_path_headings(smoothed_plan);
    }

    if (
      !this->path_refiner_collision_check_enabled_ ||
      this->is_path_collision_free(smoothed_plan))
    {
      return smoothed_plan;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Path refiner rejected corner-smoothed path because collision check failed");
  }

  return refined_plan;
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
  if (source_plan.header.stamp.sec == 0 && source_plan.header.stamp.nanosec == 0U)
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
    grid_x >= 0 && grid_x < static_cast<int>(info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(info.height);
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

MotionController::MotionController(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("motion_controller", options),
  command_topic_(""),
  local_plan_topic_(""),
  current_pose_topic_(""),
  scan_topic_(""),
  status_topic_(""),
  cmd_vel_topic_(""),
  control_frequency_(10.0),
  linear_speed_(0.07),
  min_linear_speed_(0.05),
  tracking_lookahead_distance_(0.25),
  angular_gain_(1.5),
  max_angular_speed_(0.8),
  distance_tolerance_(0.15),
  goal_heading_tolerance_(0.20),
  goal_reach_heading_tolerance_(0.35),
  final_align_max_angular_speed_(0.35),
  goal_checker_xy_tolerance_(0.15),
  goal_checker_yaw_tolerance_(0.7853981633974483),
  goal_checker_hold_time_sec_(0.0),
  goal_checker_respect_goal_yaw_(true),
  goal_checker_ignore_yaw_(false),
  rotate_in_place_threshold_(0.6),
  rotate_in_place_goal_distance_(0.35),
  tracking_heading_deadband_(0.05),
  heading_slowdown_threshold_(0.2),
  min_heading_motion_scale_(0.15),
  max_linear_accel_(0.08),
  max_angular_accel_(0.8),
  progress_required_movement_radius_(0.05),
  progress_time_allowance_sec_(2.0),
  safety_gate_enabled_(true),
  safety_gate_allow_rotate_in_place_(true),
  safety_gate_stop_distance_(3.0),
  safety_gate_forward_angle_deg_(25.0),
  safety_gate_rotate_heading_threshold_(0.20),
  safety_gate_min_points_(3),
  velocity_control_mode_(VelocityControlMode::PID),
  recovery_start_yaw_(0.0),
  goal_checker_holding_(false),
  has_command_(false),
  has_local_plan_(false),
  has_current_pose_(false),
  has_latest_scan_(false),
  has_progress_reference_(false),
  has_recovery_reference_(false)
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
  this->declare_parameter("control.angular_gain", this->angular_gain_);
  this->declare_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->declare_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->declare_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->declare_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->declare_parameter(
    "control.final_align_max_angular_speed", this->final_align_max_angular_speed_);
  this->declare_parameter("goal_checker.xy_tolerance", this->goal_checker_xy_tolerance_);
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
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->declare_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->declare_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->declare_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);

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
  this->declare_parameter("velocity_controller.angular.kd", 0.02);
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
  this->get_parameter("control.angular_gain", this->angular_gain_);
  this->get_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->get_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->get_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->get_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->get_parameter(
    "control.final_align_max_angular_speed", this->final_align_max_angular_speed_);
  this->get_parameter("goal_checker.xy_tolerance", this->goal_checker_xy_tolerance_);
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
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->get_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->get_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->get_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);

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
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
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
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
  return CallbackReturn::SUCCESS;
}

void MotionController::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  this->latest_command_ = *message;
  this->has_command_ = true;
  if (message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE)
  {
    // Drop the previous command's plan immediately. A stale empty plan can otherwise
    // make the new command look invalid before the local planner republishes.
    this->latest_local_plan_ = nav_msgs::msg::Path();
    this->has_local_plan_ = false;
  }
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->reset_goal_checker_state();
  this->has_recovery_reference_ = false;
  this->recovery_start_time_ = this->now();
  this->recovery_start_yaw_ = 0.0;
  RCLCPP_INFO(
    this->get_logger(),
    "Received motion command %u mode=%u with goal x=%.3f y=%.3f",
    message->command_id,
    message->mode,
    message->goal_pose.pose.position.x,
    message->goal_pose.pose.position.y);
}

void MotionController::handle_local_plan(const nav_msgs::msg::Path::SharedPtr message)
{
  this->latest_local_plan_ = *message;
  this->has_local_plan_ = true;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Updated local plan with %zu poses",
    message->poses.size());
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
    this->has_command_ && this->has_current_pose_ &&
    (this->latest_command_.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE || this->has_local_plan_);

  if (has_navigation_inputs)
  {
    status.active = true;
    const double current_yaw = this->quaternion_yaw(this->current_pose_.pose.orientation);

    if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE)
    {
      const geometry_msgs::msg::PoseStamped tracking_target = this->select_tracking_target();
      const double local_plan_remaining_distance =
        this->estimate_remaining_distance(this->latest_local_plan_);
      const GoalCheckResult goal_check = this->check_goal(
        this->current_pose_, this->latest_command_.goal_pose, current_yaw);
      const double goal_distance = goal_check.distance_error;
      debug_remaining_distance = std::max(local_plan_remaining_distance, goal_distance);
      const bool distance_reached = goal_check.distance_reached;
      const bool align_heading_at_goal = goal_check.align_heading;
      const double goal_yaw = goal_check.target_yaw;
      const double target_dx =
        tracking_target.pose.position.x - this->current_pose_.pose.position.x;
      const double target_dy =
        tracking_target.pose.position.y - this->current_pose_.pose.position.y;
      double target_heading = current_yaw;
      if ((target_dx * target_dx) + (target_dy * target_dy) > 1e-6)
      {
        target_heading = std::atan2(target_dy, target_dx);
      }
      if (align_heading_at_goal && distance_reached)
      {
        target_heading = goal_yaw;
      }
      const double heading_error = this->normalize_angle(target_heading - current_yaw);
      const double abs_heading_error = std::abs(heading_error);
      const bool heading_reached = goal_check.heading_reached;
      const bool aligning_in_place = align_heading_at_goal && distance_reached && !heading_reached;
      const bool final_align_phase =
        align_heading_at_goal &&
        (distance_reached || goal_distance <= this->rotate_in_place_goal_distance_);
      const bool suppress_small_heading_correction =
        !final_align_phase &&
        abs_heading_error <= this->tracking_heading_deadband_;
      const double steering_heading_error = suppress_small_heading_correction ? 0.0 : heading_error;
      const double steering_abs_heading_error = std::abs(steering_heading_error);
      const double angular_speed_limit = final_align_phase ?
        std::min(this->max_angular_speed_, this->final_align_max_angular_speed_) :
        this->max_angular_speed_;

      const bool safety_gate_blocked = this->is_safety_gate_triggered();
      status.local_plan_valid = this->has_local_plan_ && !this->latest_local_plan_.poses.empty();
      status.costmap_blocked = false;
      status.safety_gate_blocked = safety_gate_blocked;
      status.obstacle_detected = safety_gate_blocked;
      status.blocked = status.obstacle_detected;
      status.has_blocked_pose = false;
      status.blocked_pose = geometry_msgs::msg::PoseStamped();
      status.goal_reached = goal_check.goal_reached;
      status.remaining_distance = goal_distance;
      status.heading_error = heading_error;

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
      else if (aligning_in_place)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      }
      else if (distance_reached && heading_reached)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      }
      else if (
        !status.blocked &&
        (this->now() - this->progress_reference_time_).seconds() >=
        this->progress_time_allowance_sec_)
      {
        status.stalled = true;
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
        RCLCPP_INFO(
          this->get_logger(),
          "Goal reached for command %u",
          status.command_id);
      }
      else if (distance_reached && heading_reached)
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
          desired_twist.angular.z = this->clamp(
            this->angular_gain_ * heading_error,
            -angular_speed_limit,
            angular_speed_limit);
        }
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Navigation hold: blocked=%s stalled=%s",
          status.blocked ? "true" : "false",
          status.stalled ? "true" : "false");
      }
      else
      {
        desired_twist.angular.z = this->clamp(
          this->angular_gain_ * steering_heading_error,
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

          scale = this->clamp(scale, this->min_heading_motion_scale_, 1.0);
          desired_twist.linear.x = base_linear_speed * scale;
          if (goal_distance > this->rotate_in_place_goal_distance_)
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
      this->ensure_recovery_reference_initialized();
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
        RCLCPP_INFO(
          this->get_logger(),
          "Recovery command %u completed",
          status.command_id);
      }
    }
  }
  else
  {
    status.goal_reached = true;
  }

  this->current_twist_ = this->apply_velocity_controller(this->current_twist_, desired_twist);
  output_twist = this->current_twist_;

  this->cmd_vel_publisher_->publish(output_twist);
  this->motion_status_publisher_->publish(status);

  if (status.active)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Control cmd=%u target(v=%.3f,w=%.3f) output(v=%.3f,w=%.3f) remaining=%.3f heading=%.3f",
      status.command_id,
      desired_twist.linear.x,
      desired_twist.angular.z,
      output_twist.linear.x,
      output_twist.angular.z,
      debug_remaining_distance,
      status.heading_error);
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

geometry_msgs::msg::PoseStamped MotionController::select_tracking_target() const
{
  if (this->latest_local_plan_.poses.empty())
  {
    return this->latest_command_.goal_pose;
  }

  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < this->latest_local_plan_.poses.size(); ++index)
  {
    const double distance = this->pose_distance(this->current_pose_, this->latest_local_plan_.poses[index]);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  double accumulated_distance = 0.0;
  for (std::size_t index = nearest_index + 1U; index < this->latest_local_plan_.poses.size(); ++index)
  {
    const geometry_msgs::msg::PoseStamped &previous = this->latest_local_plan_.poses[index - 1U];
    const geometry_msgs::msg::PoseStamped &current = this->latest_local_plan_.poses[index];
    accumulated_distance += this->pose_distance(previous, current);
    if (accumulated_distance >= this->tracking_lookahead_distance_)
    {
      return current;
    }
  }

  return this->latest_local_plan_.poses.back();
}

MotionController::GoalCheckResult MotionController::check_goal(
  const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &goal_pose,
  const double current_yaw)
{
  GoalCheckResult result;
  result.distance_error = this->pose_distance(current_pose, goal_pose);
  result.distance_reached = result.distance_error <= this->goal_checker_xy_tolerance_;
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

  const bool reached_now = result.distance_reached && result.heading_reached;
  if (!reached_now)
  {
    this->reset_goal_checker_state();
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
