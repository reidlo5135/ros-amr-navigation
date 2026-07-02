/**
 * @file planner_server.cpp
 * @brief Implementation of the global planner lifecycle node.
 */

#include "amr_global_planner/planner_server.hpp"

namespace amr::planner::global
{

namespace
{

constexpr int kUnknownCellValue = -1;

/// @brief Extract planar yaw from a quaternion.
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  return std::atan2(
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)),
    1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z))));
}

/// @brief Escape whitespace in log values for structured logging.
std::string log_value(std::string value)
{
  if (value.empty()) {
    return "none";
  }
  for (char &character : value) {
    if (character == ' ' || character == '\t' || character == '\n' || character == '\r' || character == '=') {
      character = '_';
    }
  }
  return value;
}

/// @brief Estimate total path length for planning diagnostics.
double estimate_path_length(const nav_msgs::msg::Path &path)
{
  double length = 0.0;
  if (path.poses.size() < 2U) {
    return length;
  }
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto &previous = path.poses[index - 1U].pose.position;
    const auto &current = path.poses[index].pose.position;
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    length += std::sqrt((dx * dx) + (dy * dy));
  }
  return length;
}

}  // namespace

/// @copydoc PlannerServer::PlannerServer()
PlannerServer::PlannerServer(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("global_planner", options),
  costmap_topic_("/global_costmap"),
  computed_plan_topic_("/global_plan"),
  plan_segment_service_name_("/plan_segment"),
  plan_route_service_name_("/plan_route"),
  obstacle_threshold_(50),
  connectivity_(8),
  allow_unknown_(false),
  simplify_path_(true),
  prevent_corner_cutting_(true),
  turn_penalty_(0.5),
  start_row_hold_penalty_(1.25),
  goal_row_align_distance_cells_(6),
  goal_row_align_penalty_(1.75),
  nearest_free_search_radius_cells_(4),
  axis_aligned_straightening_enabled_(true),
  same_row_tolerance_cells_(1),
  same_column_tolerance_cells_(1),
  same_y_tolerance_m_(0.05),
  same_x_tolerance_m_(0.05),
  same_row_max_lateral_deviation_cells_(1),
  axis_aligned_interpolation_distance_(0.10),
  axis_aligned_require_line_of_sight_(true),
  same_row_straightening_enabled_(true),
  same_row_interpolation_distance_(0.10),
  same_row_require_line_of_sight_(true),
  structured_logging_enabled_(true)
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
  this->same_row_straightening_enabled_ = this->declare_parameter(
    "planner.same_row_straightening_enabled", this->same_row_straightening_enabled_);
  this->same_row_tolerance_cells_ = this->declare_parameter(
    "planner.same_row_tolerance_cells", this->same_row_tolerance_cells_);
  this->same_y_tolerance_m_ = this->declare_parameter(
    "planner.same_y_tolerance_m", this->same_y_tolerance_m_);
  this->same_row_max_lateral_deviation_cells_ = this->declare_parameter(
    "planner.same_row_max_lateral_deviation_cells", this->same_row_max_lateral_deviation_cells_);
  this->same_row_interpolation_distance_ = this->declare_parameter(
    "planner.same_row_interpolation_distance", this->same_row_interpolation_distance_);
  this->same_row_require_line_of_sight_ = this->declare_parameter(
    "planner.same_row_require_line_of_sight", this->same_row_require_line_of_sight_);
  this->axis_aligned_straightening_enabled_ = this->same_row_straightening_enabled_;
  this->same_column_tolerance_cells_ = this->same_row_tolerance_cells_;
  this->same_x_tolerance_m_ = this->same_y_tolerance_m_;
  this->axis_aligned_interpolation_distance_ = this->same_row_interpolation_distance_;
  this->axis_aligned_require_line_of_sight_ = this->same_row_require_line_of_sight_;
  this->declare_parameter(
    "planner.axis_aligned_straightening_enabled", this->axis_aligned_straightening_enabled_);
  this->declare_parameter("planner.same_column_tolerance_cells", this->same_column_tolerance_cells_);
  this->declare_parameter("planner.same_x_tolerance_m", this->same_x_tolerance_m_);
  this->declare_parameter(
    "planner.axis_aligned_interpolation_distance", this->axis_aligned_interpolation_distance_);
  this->declare_parameter(
    "planner.axis_aligned_require_line_of_sight", this->axis_aligned_require_line_of_sight_);
  this->declare_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->declare_parameter("footprint.polygon", this->footprint_polygon_param_);
}

/// @copydoc PlannerServer::on_configure()
PlannerServer::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State &state)
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
  this->get_parameter(
    "planner.same_row_straightening_enabled", this->same_row_straightening_enabled_);
  this->get_parameter("planner.same_row_tolerance_cells", this->same_row_tolerance_cells_);
  this->get_parameter("planner.same_y_tolerance_m", this->same_y_tolerance_m_);
  this->get_parameter(
    "planner.same_row_max_lateral_deviation_cells", this->same_row_max_lateral_deviation_cells_);
  this->get_parameter(
    "planner.same_row_interpolation_distance", this->same_row_interpolation_distance_);
  this->get_parameter(
    "planner.same_row_require_line_of_sight", this->same_row_require_line_of_sight_);
  this->get_parameter(
    "planner.axis_aligned_straightening_enabled", this->axis_aligned_straightening_enabled_);
  this->get_parameter("planner.same_column_tolerance_cells", this->same_column_tolerance_cells_);
  this->get_parameter("planner.same_x_tolerance_m", this->same_x_tolerance_m_);
  this->get_parameter(
    "planner.axis_aligned_interpolation_distance", this->axis_aligned_interpolation_distance_);
  this->get_parameter(
    "planner.axis_aligned_require_line_of_sight", this->axis_aligned_require_line_of_sight_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);
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

/// @copydoc PlannerServer::on_activate()
PlannerServer::CallbackReturn PlannerServer::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->computed_plan_publisher_) {
    this->computed_plan_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated global planner");
  return CallbackReturn::SUCCESS;
}

/// @copydoc PlannerServer::on_deactivate()
PlannerServer::CallbackReturn PlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->computed_plan_publisher_) {
    this->computed_plan_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

/// @copydoc PlannerServer::on_cleanup()
PlannerServer::CallbackReturn PlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
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

/// @copydoc PlannerServer::on_shutdown()
PlannerServer::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State &state)
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

/// @copydoc PlannerServer::handle_plan_segment()
void PlannerServer::handle_plan_segment(
  const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response)
{
  response->success = false;
  response->plan = nav_msgs::msg::Path();
  response->message.clear();

  const rclcpp::Time started_at = this->now();
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=global_planner event=plan_requested start_x=%.3f start_y=%.3f goal_x=%.3f goal_y=%.3f",
      request->start.pose.position.x,
      request->start.pose.position.y,
      request->goal.pose.position.x,
      request->goal.pose.position.y);
  }

  response->success = this->compute_plan_between_poses(
    request->start, request->goal, response->plan, response->message);

  if (response->success) {
    this->planned_path_ = response->plan;
    if (this->computed_plan_publisher_ &&this->computed_plan_publisher_->is_activated()) {
      this->computed_plan_publisher_->publish(this->planned_path_);
    }
    if (this->structured_logging_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "AMR_LOG schema=v1 component=global_planner event=plan_succeeded start_x=%.3f start_y=%.3f goal_x=%.3f goal_y=%.3f path_points=%zu path_length_m=%.3f duration_sec=%.3f",
        request->start.pose.position.x,
        request->start.pose.position.y,
        request->goal.pose.position.x,
        request->goal.pose.position.y,
        response->plan.poses.size(),
        estimate_path_length(response->plan),
        (this->now() - started_at).seconds());
    }
  } else if (this->structured_logging_enabled_) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=global_planner event=plan_failed start_x=%.3f start_y=%.3f goal_x=%.3f goal_y=%.3f duration_sec=%.3f reason=%s",
      request->start.pose.position.x,
      request->start.pose.position.y,
      request->goal.pose.position.x,
      request->goal.pose.position.y,
      (this->now() - started_at).seconds(),
      log_value(response->message).c_str());
  }
}

/// @copydoc PlannerServer::handle_plan_route()
void PlannerServer::handle_plan_route(
  const std::shared_ptr<amr_msgs::srv::PlanRoute::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanRoute::Response> response)
{
  response->success = false;
  response->plans.clear();
  response->message.clear();

  const rclcpp::Time started_at = this->now();
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=global_planner event=plan_requested route_segments=%zu start_x=%.3f start_y=%.3f",
      request->waypoints.size(),
      request->start.pose.position.x,
      request->start.pose.position.y);
  }

  auto current = request->start;
  for (std::size_t index = 0; index < request->waypoints.size(); ++index) {
    nav_msgs::msg::Path segment_path;
    std::string segment_message;
    const auto &waypoint = request->waypoints[index];
    const bool success = this->compute_plan_between_poses(
      current, waypoint, segment_path, segment_message);
    if (!success) {
      response->message =
        "Failed to compute segment " + std::to_string(index) + ": " + segment_message;
      response->plans.clear();
      if (this->structured_logging_enabled_) {
        RCLCPP_WARN(
          this->get_logger(),
          "AMR_LOG schema=v1 component=global_planner event=plan_failed route_segments=%zu segment_idx=%zu duration_sec=%.3f reason=%s",
          request->waypoints.size(),
          index,
          (this->now() - started_at).seconds(),
          log_value(response->message).c_str());
      }
      return;
    }

    response->plans.push_back(segment_path);
    current = waypoint;
  }

  response->success = true;
  response->message = "Generated A* plans for all route segments.";
  this->planned_path_ = this->merge_paths(response->plans);
  if (this->computed_plan_publisher_ &&this->computed_plan_publisher_->is_activated()) {
    this->computed_plan_publisher_->publish(this->planned_path_);
  }
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=global_planner event=plan_succeeded route_segments=%zu path_points=%zu path_length_m=%.3f duration_sec=%.3f",
      request->waypoints.size(),
      this->planned_path_.poses.size(),
      estimate_path_length(this->planned_path_),
      (this->now() - started_at).seconds());
  }
}

/// @copydoc PlannerServer::compute_plan_between_poses()
bool PlannerServer::compute_plan_between_poses(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav_msgs::msg::Path &path,
  std::string &message) const
{
  path = nav_msgs::msg::Path();
  message.clear();

  if (!this->a_star_planner_) {
    message = "A* planner is not configured";
    return false;
  }

  const auto expected_costmap_size = this->global_costmap_ ?
    static_cast<std::size_t>(this->global_costmap_->info.width) *
    static_cast<std::size_t>(this->global_costmap_->info.height) :
    0U;
  if (
    !this->global_costmap_ ||
    this->global_costmap_->info.width == 0 ||
    this->global_costmap_->info.height == 0 ||
    this->global_costmap_->info.resolution <= 0.0F ||
    this->global_costmap_->data.size() != expected_costmap_size)
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
  const GridCell requested_start_cell = start_cell;
  const GridCell requested_goal_cell = goal_cell;

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

  const auto straight_axis_name = [](const StraightAxis axis) {
      switch (axis) {
        case StraightAxis::Horizontal:
          return "horizontal";
        case StraightAxis::Vertical:
          return "vertical";
        case StraightAxis::None:
        default:
          return "none";
      }
    };
  const StraightAxis straight_axis = this->classify_axis_aligned_straight_candidate(
    start, goal, start_cell, goal_cell);
  const bool axis_aligned_candidate = straight_axis != StraightAxis::None;
  const bool same_row_candidate = straight_axis == StraightAxis::Horizontal;
  const bool straight_line_safe =
    axis_aligned_candidate && this->is_straight_line_collision_free(start, goal);
  const double y_delta_m = std::abs(goal.pose.position.y - start.pose.position.y);
  const double x_delta_m = std::abs(goal.pose.position.x - start.pose.position.x);
  const int row_delta = std::abs(goal_cell.y - start_cell.y);
  const int col_delta = std::abs(goal_cell.x - start_cell.x);
  std::string fallback_reason = "not_axis_aligned_candidate";
  if (!this->axis_aligned_straightening_enabled_) {
    fallback_reason = "straightening_disabled";
  } else if (axis_aligned_candidate && !this->axis_aligned_require_line_of_sight_) {
    fallback_reason = "line_of_sight_check_disabled";
  } else if (axis_aligned_candidate && !straight_line_safe) {
    fallback_reason = "line_of_sight_blocked";
  } else if (axis_aligned_candidate) {
    fallback_reason = "none";
  }

  if (this->axis_aligned_straightening_enabled_ && axis_aligned_candidate && straight_line_safe)
  {
    path = this->create_straight_path_message(start, goal);
    message = "Generated axis-aligned straight path";
    if (this->structured_logging_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "AMR_LOG schema=v1 component=global_planner event=plan_quality start_row=%d goal_row=%d row_delta=%d requested_start_row=%d requested_goal_row=%d start_y=%.3f goal_y=%.3f y_delta_m=%.3f start_col=%d goal_col=%d col_delta=%d requested_start_col=%d requested_goal_col=%d start_x=%.3f goal_x=%.3f x_delta_m=%.3f same_row_candidate=%s axis_aligned_candidate=true straight_axis=%s straight_line_safe=true straight_path_used=true fallback_reason=none raw_path_points=%zu final_path_points=%zu max_row_deviation=0 max_lateral_deviation_m=%.3f",
        start_cell.y,
        goal_cell.y,
        row_delta,
        requested_start_cell.y,
        requested_goal_cell.y,
        start.pose.position.y,
        goal.pose.position.y,
        y_delta_m,
        start_cell.x,
        goal_cell.x,
        col_delta,
        requested_start_cell.x,
        requested_goal_cell.x,
        start.pose.position.x,
        goal.pose.position.x,
        x_delta_m,
        same_row_candidate ? "true" : "false",
        straight_axis_name(straight_axis),
        path.poses.size(),
        path.poses.size(),
        this->estimate_path_lateral_deviation(path, start, goal));
    }
    return true;
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
  const int max_row_deviation = this->estimate_max_row_deviation(result.path, start_cell, goal_cell);
  const double max_lateral_deviation_m = this->estimate_path_lateral_deviation(path, start, goal);
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=global_planner event=plan_quality start_row=%d goal_row=%d row_delta=%d requested_start_row=%d requested_goal_row=%d start_y=%.3f goal_y=%.3f y_delta_m=%.3f start_col=%d goal_col=%d col_delta=%d requested_start_col=%d requested_goal_col=%d start_x=%.3f goal_x=%.3f x_delta_m=%.3f same_row_candidate=%s axis_aligned_candidate=%s straight_axis=%s straight_line_safe=%s straight_path_used=false fallback_reason=%s raw_path_points=%zu final_path_points=%zu max_row_deviation=%d max_lateral_deviation_m=%.3f",
      start_cell.y,
      goal_cell.y,
      row_delta,
      requested_start_cell.y,
      requested_goal_cell.y,
      start.pose.position.y,
      goal.pose.position.y,
      y_delta_m,
      start_cell.x,
      goal_cell.x,
      col_delta,
      requested_start_cell.x,
      requested_goal_cell.x,
      start.pose.position.x,
      goal.pose.position.x,
      x_delta_m,
      same_row_candidate ? "true" : "false",
      axis_aligned_candidate ? "true" : "false",
      straight_axis_name(straight_axis),
      straight_line_safe ? "true" : "false",
      fallback_reason.c_str(),
      result.path.size(),
      path.poses.size(),
      max_row_deviation,
      max_lateral_deviation_m);
  }
  return true;
}

/// @copydoc PlannerServer::world_to_grid()
bool PlannerServer::world_to_grid(
  const geometry_msgs::msg::PoseStamped &pose,
  GridCell &cell) const
{
  if (!this->global_costmap_ || this->global_costmap_->info.resolution <= 0.0F) {
    return false;
  }

  const auto &map_info = this->global_costmap_->info;
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

/// @copydoc PlannerServer::grid_to_world()
geometry_msgs::msg::PoseStamped PlannerServer::grid_to_world(const GridCell &cell) const
{
  geometry_msgs::msg::PoseStamped pose;

  if (!this->global_costmap_) {
    return pose;
  }

  const auto &map_header = this->global_costmap_->header;
  const auto &map_info = this->global_costmap_->info;
  const double resolution = static_cast<double>(map_info.resolution);

  pose.header = map_header;
  pose.pose.position.x = map_info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * resolution;
  pose.pose.position.y = map_info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

/// @copydoc PlannerServer::is_occupied_cell()
bool PlannerServer::is_occupied_cell(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const GridCell &cell) const
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

/// @copydoc PlannerServer::find_nearest_free_cell()
bool PlannerServer::find_nearest_free_cell(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  GridCell &cell,
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

/// @copydoc PlannerServer::is_cell_collision()
bool PlannerServer::is_cell_collision(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const GridCell &cell,
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

/// @copydoc PlannerServer::classify_axis_aligned_straight_candidate()
PlannerServer::StraightAxis PlannerServer::classify_axis_aligned_straight_candidate(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  const GridCell &start_cell,
  const GridCell &goal_cell) const
{
  if (!this->axis_aligned_straightening_enabled_) {
    return StraightAxis::None;
  }

  const int row_delta = std::abs(goal_cell.y - start_cell.y);
  const int col_delta = std::abs(goal_cell.x - start_cell.x);
  const double y_delta_m = std::abs(goal.pose.position.y - start.pose.position.y);
  const double x_delta_m = std::abs(goal.pose.position.x - start.pose.position.x);
  const bool horizontal_candidate =
    row_delta <= std::max(0, this->same_row_tolerance_cells_) ||
    y_delta_m <= std::max(0.0, this->same_y_tolerance_m_);
  const bool vertical_candidate =
    col_delta <= std::max(0, this->same_column_tolerance_cells_) ||
    x_delta_m <= std::max(0.0, this->same_x_tolerance_m_);

  if (horizontal_candidate &&vertical_candidate) {
    return x_delta_m >= y_delta_m ? StraightAxis::Horizontal : StraightAxis::Vertical;
  }
  if (horizontal_candidate) {
    return StraightAxis::Horizontal;
  }
  if (vertical_candidate) {
    return StraightAxis::Vertical;
  }

  return StraightAxis::None;
}

/// @copydoc PlannerServer::is_world_pose_collision_free()
bool PlannerServer::is_world_pose_collision_free(
  const geometry_msgs::msg::PoseStamped &pose,
  const double yaw) const
{
  if (!this->global_costmap_ || this->global_costmap_->data.empty()) {
    return false;
  }

  const int width = static_cast<int>(this->global_costmap_->info.width);
  const int height = static_cast<int>(this->global_costmap_->info.height);
  GridCell cell{};
  if (!this->world_to_grid(pose, cell)) {
    return false;
  }

  if (this->footprint_polygon_.empty()) {
    return !this->is_occupied_cell(this->global_costmap_->data, width, height, cell);
  }

  return !amr::geometry::footprint_pose_collides(
    this->global_costmap_->data,
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

/// @copydoc PlannerServer::is_straight_line_collision_free()
bool PlannerServer::is_straight_line_collision_free(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const
{
  if (!this->axis_aligned_require_line_of_sight_) {
    return false;
  }
  if (!this->global_costmap_ || this->global_costmap_->data.empty()) {
    return false;
  }

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::sqrt((dx * dx) + (dy * dy));
  const double yaw = distance > 1e-6 ? std::atan2(dy, dx) : yaw_from_quaternion(goal.pose.orientation);
  const double step = std::max(
    std::max(1e-3, this->axis_aligned_interpolation_distance_),
    static_cast<double>(this->global_costmap_->info.resolution));
  const int sample_count = std::max(1, static_cast<int>(std::ceil(distance / step)));

  for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
    const double ratio = sample_count > 0 ?
      static_cast<double>(sample_index) / static_cast<double>(sample_count) : 1.0;
    geometry_msgs::msg::PoseStamped sample_pose = start;
    sample_pose.pose.position.x = start.pose.position.x + (dx * ratio);
    sample_pose.pose.position.y = start.pose.position.y + (dy * ratio);
    sample_pose.pose.position.z = 0.0;
    sample_pose.pose.orientation.z = std::sin(yaw * 0.5);
    sample_pose.pose.orientation.w = std::cos(yaw * 0.5);
    if (!this->is_world_pose_collision_free(sample_pose, yaw)) {
      return false;
    }
  }

  return true;
}

/// @copydoc PlannerServer::create_straight_path_message()
nav_msgs::msg::Path PlannerServer::create_straight_path_message(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const
{
  nav_msgs::msg::Path path;
  if (!this->global_costmap_) {
    return path;
  }

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::sqrt((dx * dx) + (dy * dy));
  const double yaw = distance > 1e-6 ? std::atan2(dy, dx) : yaw_from_quaternion(goal.pose.orientation);
  const double step = std::max(1e-3, this->axis_aligned_interpolation_distance_);
  const int sample_count = std::max(1, static_cast<int>(std::ceil(distance / step)));

  path.header = this->global_costmap_->header;
  path.header.stamp = this->now();
  path.poses.reserve(static_cast<std::size_t>(sample_count + 1));
  for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
    const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count);
    geometry_msgs::msg::PoseStamped pose = start;
    pose.header = path.header;
    pose.pose.position.x = start.pose.position.x + (dx * ratio);
    pose.pose.position.y = start.pose.position.y + (dy * ratio);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    path.poses.push_back(pose);
  }

  return path;
}

/// @copydoc PlannerServer::estimate_max_row_deviation()
int PlannerServer::estimate_max_row_deviation(
  const std::vector<GridCell> &grid_path,
  const GridCell &start_cell,
  const GridCell &goal_cell) const
{
  if (grid_path.empty()) {
    return 0;
  }

  const int row_min = std::min(start_cell.y, goal_cell.y);
  const int row_max = std::max(start_cell.y, goal_cell.y);
  int max_deviation = 0;
  for (const auto &cell : grid_path) {
    if (cell.y < row_min) {
      max_deviation = std::max(max_deviation, row_min - cell.y);
    } else if (cell.y > row_max) {
      max_deviation = std::max(max_deviation, cell.y - row_max);
    }
  }
  return max_deviation;
}

/// @copydoc PlannerServer::estimate_path_lateral_deviation()
double PlannerServer::estimate_path_lateral_deviation(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) const
{
  const double line_dx = goal.pose.position.x - start.pose.position.x;
  const double line_dy = goal.pose.position.y - start.pose.position.y;
  const double line_length = std::sqrt((line_dx * line_dx) + (line_dy * line_dy));
  if (line_length <= 1e-6 || path.poses.empty()) {
    return 0.0;
  }

  double max_lateral_deviation = 0.0;
  for (const auto &pose : path.poses) {
    const double point_dx = pose.pose.position.x - start.pose.position.x;
    const double point_dy = pose.pose.position.y - start.pose.position.y;
    const double deviation = std::abs((point_dx * line_dy) - (point_dy * line_dx)) / line_length;
    max_lateral_deviation = std::max(max_lateral_deviation, deviation);
  }
  return max_lateral_deviation;
}

/// @copydoc PlannerServer::simplify_grid_path()
std::vector<GridCell> PlannerServer::simplify_grid_path(const std::vector<GridCell> &grid_path) const
{
  if (!this->simplify_path_ || grid_path.size() <= 2U) {
    return grid_path;
  }

  std::vector<GridCell> simplified_path;
  simplified_path.reserve(grid_path.size());
  simplified_path.push_back(grid_path.front());

  for (std::size_t index = 1; index + 1 < grid_path.size(); ++index) {
    const auto &previous = grid_path[index - 1];
    const auto &current = grid_path[index];
    const auto &next = grid_path[index + 1];

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

/// @copydoc PlannerServer::create_path_message()
nav_msgs::msg::Path PlannerServer::create_path_message(
  const std::vector<GridCell> &grid_path) const
{
  nav_msgs::msg::Path path;
  if (!this->global_costmap_) {
    return path;
  }

  path.header = this->global_costmap_->header;
  path.header.stamp = this->now();
  path.poses.reserve(grid_path.size());
  for (const auto &cell : grid_path) {
    path.poses.push_back(this->grid_to_world(cell));
  }

  return path;
}

/// @copydoc PlannerServer::merge_paths()
nav_msgs::msg::Path PlannerServer::merge_paths(const std::vector<nav_msgs::msg::Path> &paths) const
{
  nav_msgs::msg::Path merged;
  if (paths.empty()) {
    return merged;
  }

  merged.header = paths.front().header;
  merged.header.stamp = this->now();

  for (std::size_t path_index = 0; path_index < paths.size(); ++path_index) {
    const auto &path = paths[path_index];
    for (std::size_t pose_index = 0; pose_index < path.poses.size(); ++pose_index) {
      if (path_index > 0 &&pose_index == 0U &&!merged.poses.empty()) {
        continue;
      }
      merged.poses.push_back(path.poses[pose_index]);
    }
  }

  return merged;
}

/// @copydoc PlannerServer::costmap_subscription_cb()
void PlannerServer::costmap_subscription_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  const auto width = static_cast<std::size_t>(map->info.width);
  const auto height = static_cast<std::size_t>(map->info.height);
  const auto expected_size = width * height;
  if (
    width == 0U || height == 0U || map->info.resolution <= 0.0F ||
    map->data.size() != expected_size)
  {
    this->global_costmap_.reset();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Ignoring invalid global costmap: size=%zu x %zu resolution=%.6f data=%zu expected=%zu",
      width,
      height,
      static_cast<double>(map->info.resolution),
      map->data.size(),
      expected_size);
    return;
  }

  this->global_costmap_ = map;
}

}  // namespace amr::planner::global
