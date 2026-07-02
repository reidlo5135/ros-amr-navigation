#ifndef AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

/**
 * @file controller_server.hpp
 * @brief Local planner, motion controller, and combined controller server declarations.
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "amr_geometry/footprint.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/plan_local_escape.hpp"

namespace amr::planner::local
{

/// @brief Builds local plans from global plans and costmaps for motion tracking.
class LocalPlanner : public rclcpp_lifecycle::LifecycleNode
{
private:
  /// @brief Internal result bundle for one local-plan build cycle.
  struct LocalPlanBuildResult
  {
    /// @brief Local path selected for publication.
    nav_msgs::msg::Path plan;
    /// @brief True when the local plan is suitable for controller tracking.
    bool local_plan_valid{false};
    /// @brief True when recovery or replanning should be requested.
    bool recovery_required{false};
    /// @brief Local planner decision code published in status.
    uint8_t decision{amr_msgs::msg::LocalPlanStatus::DECISION_OK};
    /// @brief True when a blocked pose estimate is available.
    bool has_blocked_pose{false};
    /// @brief Pose on the local/global plan where blockage was detected.
    geometry_msgs::msg::PoseStamped blocked_pose;
    /// @brief Estimated distance from current pose to the blocked pose.
    double blocked_distance{0.0};
    /// @brief Number of source global path poses considered.
    std::size_t global_path_points{0U};
    std::size_t nearest_idx{0U};
    std::size_t candidate_idx{0U};
    std::size_t selected_idx{0U};
    std::size_t start_idx{0U};
    std::size_t end_idx{0U};
    double lookahead_m{0.0};
    double dist_goal_m{0.0};
    double source_path_curvature_score{0.0};
    double source_path_lateral_error_m{0.0};
    double source_current_lateral_error_m{0.0};
    bool xy_reached{false};
    bool fallback_used{false};
    bool degenerate{false};
    std::string reason{"none"};
    std::string pose_frame;
    std::string plan_frame;
    std::string source_straight_axis{"none"};
  };

  /// @brief Diagnostic quality metrics collected while refining a local path.
  struct LocalPathQualityMetrics
  {
    std::size_t raw_path_points{0U};
    std::size_t simplified_path_points{0U};
    std::size_t refined_path_points{0U};
    double path_length_m{0.0};
    double raw_path_curvature_score{0.0};
    double path_curvature_score{0.0};
    double raw_lateral_error_m{0.0};
    double lateral_error_m{0.0};
    double lateral_error_delta_m{0.0};
    bool line_of_sight_simplified{false};
    int collinear_pruned_count{0};
    bool corner_smoothing_applied{false};
    bool collision_check_passed{true};
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure subscriptions, publishers, service, TF, and planner parameters.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate lifecycle publishers and start periodic planning.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Deactivate lifecycle publishers and stop periodic planning.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release configured resources and cached planner state.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release planner resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Cache the latest motion command and reset command-scoped state.
  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  /// @brief Cache a new local costmap.
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  /// @brief Cache a new global plan.
  void handle_global_plan(const nav_msgs::msg::Path::SharedPtr message);
  /// @brief Build and return a local escape plan for recovery.
  void handle_plan_local_escape(
    const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response);
  /// @brief Update current robot pose from TF.
  bool update_current_pose_from_tf();
  /// @brief Build and publish local plan/status for the current command.
  void publish_local_plan();
  /// @brief Build a local plan from the current command and pose.
  LocalPlanBuildResult build_local_plan(
    const amr_msgs::msg::MotionCommand &command,
    const geometry_msgs::msg::PoseStamped &current_pose);
  /// @brief Build a local plan on the inflated costmap when direct slicing is blocked.
  nav_msgs::msg::Path build_inflated_local_plan(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index,
    double lookahead_distance);
  /// @brief Slice a local window from the source plan using the default lookahead.
  nav_msgs::msg::Path build_sliced_local_plan(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index) const;
  /// @brief Slice a local window from the source plan up to a lookahead distance.
  nav_msgs::msg::Path build_sliced_local_plan_with_lookahead(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index,
    double lookahead_distance,
    std::size_t *end_index = nullptr) const;
  /// @brief Build a fallback local path when the nominal slice is degenerate.
  nav_msgs::msg::Path build_fallback_local_plan(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t nearest_index,
    double lookahead_distance,
    std::size_t &selected_index) const;
  /// @brief Return true when a local path is too short or sparse to track.
  bool is_degenerate_local_path(
    const nav_msgs::msg::Path &plan,
    std::size_t global_path_points,
    bool xy_reached) const;
  /// @brief Select the source plan associated with a motion command.
  nav_msgs::msg::Path build_source_plan(const amr_msgs::msg::MotionCommand &command) const;
  /// @brief Find the source-plan pose nearest to the current robot pose.
  std::size_t find_closest_pose_index(
    const nav_msgs::msg::Path &plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t start_index) const;
  /// @brief Convert a world point into the working costmap grid.
  bool world_to_grid(const geometry_msgs::msg::Point &point, int &grid_x, int &grid_y) const;
  /// @brief Convert a grid cell into a pose in the requested frame.
  geometry_msgs::msg::PoseStamped grid_to_pose(
    int grid_x,
    int grid_y,
    const std::string &frame_id) const;
  /// @brief Return true when a grid cell is occupied or unknown is disallowed.
  bool is_occupied_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y) const;
  /// @brief Return true when the robot footprint collides at a grid cell pose.
  bool is_grid_pose_collision(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y,
    double yaw) const;
  /// @brief Search nearby cells for the nearest footprint-valid free cell.
  bool find_nearest_free_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int &grid_x,
    int &grid_y,
    int max_radius,
    double yaw) const;
  /// @brief Interpolate between two poses by ratio.
  geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    double ratio) const;
  /// @brief Compute 2D Euclidean distance between two poses.
  double pose_distance(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;
  /// @brief Locate the first pose on a plan that collides with the working costmap.
  bool find_first_blocked_pose_on_plan(
    const nav_msgs::msg::Path &plan,
    geometry_msgs::msg::PoseStamped &blocked_pose) const;
  /// @brief Reset hysteresis counters for dynamic obstacle decisions.
  void reset_dynamic_blocked_state();
  /// @brief Confirm a dynamic recovery decision through configured debounce cycles.
  bool confirm_dynamic_recovery_decision(uint8_t decision, double blocked_distance);
  /// @brief Confirm that dynamic blockage has cleared.
  bool confirm_dynamic_clear();
  /// @brief Resolve required confirmation cycles for a dynamic obstacle decision.
  int required_dynamic_recovery_cycles(uint8_t decision, double blocked_distance) const;
  /// @brief Sample occupancy laterally from a path heading for corridor diagnostics.
  double sample_lateral_occupancy(
    double origin_x,
    double origin_y,
    double heading,
    double lateral_sign) const;
  /// @brief Apply configured simplification, pruning, smoothing, and collision checks.
  nav_msgs::msg::Path refine_local_plan(
    const nav_msgs::msg::Path &plan,
    LocalPathQualityMetrics *quality_metrics = nullptr) const;
  /// @brief Simplify a path by skipping poses with clear line of sight.
  nav_msgs::msg::Path simplify_path_line_of_sight(
    const nav_msgs::msg::Path &plan,
    bool &line_of_sight_simplified) const;
  /// @brief Remove nearly collinear path poses.
  nav_msgs::msg::Path prune_collinear_path(
    const nav_msgs::msg::Path &plan,
    int &collinear_pruned_count) const;
  /// @brief Prune dense points and interpolate long path segments.
  nav_msgs::msg::Path prune_and_interpolate_path(const nav_msgs::msg::Path &plan) const;
  /// @brief Apply the configured smoothing pipeline to a path.
  nav_msgs::msg::Path apply_path_smoother(const nav_msgs::msg::Path &plan) const;
  /// @brief Add corner smoothing samples around sharp path turns.
  nav_msgs::msg::Path smooth_path_corners(const nav_msgs::msg::Path &plan) const;
  /// @brief Validate that a smoothed path remains close and safe enough.
  bool is_smoothed_path_acceptable(
    const nav_msgs::msg::Path &base_plan,
    const nav_msgs::msg::Path &smoothed_plan) const;
  /// @brief Estimate total 2D length of a path.
  double estimate_path_length(const nav_msgs::msg::Path &plan) const;
  /// @brief Estimate the shortest 2D distance from a pose to a path.
  double estimate_pose_distance_to_path(
    const geometry_msgs::msg::PoseStamped &pose,
    const nav_msgs::msg::Path &path) const;
  /// @brief Estimate maximum heading change along a path.
  double estimate_path_curvature_score(const nav_msgs::msg::Path &path) const;
  /// @brief Estimate lateral deviation of a path from its start-goal chord.
  double estimate_path_lateral_error(const nav_msgs::msg::Path &path) const;
  /// @brief Check sampled collision freedom between two path poses.
  bool is_path_segment_collision_free(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    double sample_distance) const;
  /// @brief Check collision freedom for all segments of a path.
  bool is_path_collision_free(const nav_msgs::msg::Path &plan) const;
  /// @brief Check collision freedom for a single pose footprint.
  bool is_pose_collision_free(const geometry_msgs::msg::PoseStamped &pose) const;
  /// @brief Assign heading orientations along a local path.
  void assign_path_headings(nav_msgs::msg::Path &plan) const;
  /// @brief Build a yaw-only quaternion.
  geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_subscription_;
  rclcpp::Service<amr_msgs::srv::PlanLocalEscape>::SharedPtr local_escape_service_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string command_topic_;
  std::string map_topic_;
  std::string global_plan_topic_;
  std::string local_plan_topic_;
  std::string local_plan_status_topic_;
  std::string local_escape_service_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_;
  int publish_period_ms_;
  double lookahead_distance_;
  double goal_tolerance_;
  int obstacle_threshold_;
  int connectivity_;
  bool allow_unknown_;
  bool prevent_corner_cutting_;
  double turn_penalty_;
  bool path_refiner_enabled_;
  double path_refiner_prune_distance_;
  double path_refiner_interpolate_distance_;
  bool path_refiner_heading_assignment_enabled_;
  bool path_refiner_preserve_goal_orientation_;
  bool path_refiner_line_of_sight_simplification_enabled_;
  double path_refiner_line_of_sight_sample_distance_;
  int path_refiner_line_of_sight_max_skip_;
  bool path_refiner_collinear_pruning_enabled_;
  double path_refiner_collinear_angle_threshold_;
  double path_refiner_collinear_lateral_deviation_threshold_;
  bool path_refiner_corner_smoothing_enabled_;
  double path_refiner_corner_smoothing_max_offset_;
  double path_refiner_corner_smoothing_angle_threshold_;
  int path_refiner_corner_smoothing_samples_;
  double path_refiner_smoothing_max_length_ratio_;
  double path_refiner_smoothing_max_pose_deviation_;
  bool path_refiner_collision_check_enabled_;
  double path_refiner_collision_sample_distance_;
  bool dynamic_obstacle_enabled_;
  bool local_path_guard_enabled_;
  int local_path_guard_min_points_;
  double local_path_guard_min_length_m_;
  int local_path_guard_fallback_min_points_;
  double local_path_guard_fallback_lookahead_m_;
  bool local_path_guard_allow_single_point_when_goal_reached_;
  double local_path_guard_degenerate_log_throttle_sec_;
  bool structured_logging_enabled_;
  double state_log_throttle_sec_;
  double dynamic_obstacle_replan_lookahead_distance_;
  double dynamic_obstacle_escape_forward_distance_;
  double dynamic_obstacle_escape_lateral_distance_;
  double dynamic_obstacle_goal_proximity_disable_distance_;
  double dynamic_obstacle_corridor_relax_distance_;
  int dynamic_obstacle_recovery_confirm_cycles_;
  int dynamic_obstacle_goal_proximity_confirm_cycles_;
  int dynamic_obstacle_corridor_confirm_cycles_;
  int dynamic_obstacle_clear_confirm_cycles_;
  int nearest_free_search_radius_cells_;
  std::vector<double> footprint_polygon_param_;
  amr::geometry::FootprintPolygon footprint_polygon_;
  uint32_t last_command_id_;
  uint8_t dynamic_blocked_decision_;
  int dynamic_blocked_streak_;
  int dynamic_clear_streak_;
  std::size_t last_progress_index_;
  amr_msgs::msg::MotionCommand latest_command_;
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path latest_global_plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  nav_msgs::msg::OccupancyGrid working_costmap_;
  bool has_command_;
  bool has_current_pose_;
  bool has_global_plan_;
  bool has_map_;

public:
  /// @brief Construct the local planner node and declare ROS parameters.
  explicit LocalPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the local planner node.
  virtual ~LocalPlanner() = default;
};

}  // namespace amr::planner::local

namespace amr::motion::controller
{

/// @brief Lifecycle pure-pursuit motion controller that publishes /cmd_vel and motion status.
class MotionController : public rclcpp_lifecycle::LifecycleNode
{
private:
  /// @brief Supported linear/angular velocity controller modes.
  enum class VelocityControlMode
  {
    /// @brief Proportional-only control.
    P,
    /// @brief Proportional-integral control.
    PI,
    /// @brief Proportional-integral-derivative control.
    PID
  };

  /// @brief PID-style axis controller gains and integral limit.
  struct AxisControllerConfig
  {
    /// @brief Proportional gain.
    double kp{0.0};
    /// @brief Integral gain.
    double ki{0.0};
    /// @brief Derivative gain.
    double kd{0.0};
    /// @brief Absolute integral clamp.
    double integral_limit{0.0};
  };

  /// @brief Runtime state for one velocity axis controller.
  struct AxisControllerState
  {
    /// @brief Integrated control error.
    double integral{0.0};
    /// @brief Previous cycle error used for derivative control.
    double previous_error{0.0};
    /// @brief True before the first update has initialized history.
    bool first_update{true};
  };

  /// @brief Result of checking current pose against goal tolerances.
  struct GoalCheckResult
  {
    /// @brief True when XY tolerance and hysteresis indicate goal distance reached.
    bool distance_reached{false};
    /// @brief True when required goal heading tolerance is reached.
    bool heading_reached{true};
    /// @brief True when all goal checker conditions are satisfied.
    bool goal_reached{false};
    /// @brief True when final heading alignment is required.
    bool align_heading{false};
    /// @brief Current XY distance to goal.
    double distance_error{0.0};
    /// @brief Current yaw error to goal heading.
    double heading_error{0.0};
    /// @brief Goal yaw in radians.
    double target_yaw{0.0};
  };

  /// @brief Straight-path assessment used to select straight tracking gains.
  struct StraightSegmentAssessment
  {
    /// @brief True when the assessed path window is straight enough.
    bool straight_segment{false};
    /// @brief Maximum heading delta over the assessed path window.
    double path_curvature_score{0.0};
    /// @brief Maximum lateral deviation over the assessed path window.
    double lateral_error_m{0.0};
    /// @brief Assessed segment length in meters.
    double segment_length_m{0.0};
    /// @brief Start index of the assessed path window.
    std::size_t start_index{0U};
    /// @brief End index of the assessed path window.
    std::size_t end_index{0U};
  };

  /// @brief Lifecycle callback return type alias.
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure subscriptions, publishers, TF, timers, and controller parameters.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate publishers and start the control timer.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Stop the control timer and deactivate publishers.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release configured resources and reset controller state.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Cache a new motion command and reset command-scoped state.
  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  /// @brief Cache a new local plan and update tracking-target bounds.
  void handle_local_plan(const nav_msgs::msg::Path::SharedPtr message);
  /// @brief Cache the latest laser scan for safety gate and obstacle regulation.
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  /// @brief Update current robot pose from TF.
  bool update_current_pose_from_tf();
  /// @brief Run one control cycle and publish command/status outputs.
  void publish_control();
  /// @brief Reset PID/acceleration limiter state.
  void reset_velocity_controller_state();
  /// @brief Reset progress checker reference state.
  void reset_progress_checker_state();
  /// @brief Reset goal checker latch and hold state.
  void reset_goal_checker_state();
  /// @brief Reset blocked/stalled status hysteresis state.
  void reset_status_semantics_state();
  /// @brief Publish a zero twist immediately when active.
  void publish_zero_twist();
  /// @brief Parse velocity controller mode from a parameter string.
  VelocityControlMode parse_velocity_control_mode(const std::string &mode) const;
  /// @brief Apply one configured velocity-axis controller update.
  double apply_axis_controller(
    double current,
    double target,
    AxisControllerState &state,
    const AxisControllerConfig &config,
    double max_step,
    double dt) const;
  /// @brief Apply linear and angular velocity controllers to a target twist.
  geometry_msgs::msg::Twist apply_velocity_controller(
    const geometry_msgs::msg::Twist &current,
    const geometry_msgs::msg::Twist &target);
  /// @brief Estimate path length remaining in the provided local path.
  double estimate_remaining_distance(const nav_msgs::msg::Path &path) const;
  /// @brief Extract yaw from a quaternion.
  double quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation) const;
  /// @brief Normalize an angle to [-pi, pi].
  double normalize_angle(double angle) const;
  /// @brief Clamp a value to the provided inclusive range.
  double clamp(double value, double min_value, double max_value) const;
  /// @brief Compute fixed or RPP velocity-scaled tracking lookahead.
  double compute_tracking_lookahead_distance() const;
  /// @brief Compute pure-pursuit curvature from current pose to target pose.
  double compute_pure_pursuit_curvature(
    const geometry_msgs::msg::PoseStamped &target_pose) const;
  /// @brief Apply RPP curvature-based linear speed regulation.
  double apply_curvature_speed_regulation(double linear_speed, double curvature) const;
  /// @brief Apply RPP goal-approach linear speed regulation.
  double apply_approach_speed_regulation(double linear_speed) const;
  /// @brief Estimate nearest valid obstacle distance in the forward scan cone.
  double estimate_forward_obstacle_distance() const;
  /// @brief Apply scan-based obstacle-proximity speed regulation.
  double apply_obstacle_speed_regulation(double linear_speed) const;
  /// @brief Predict scan-cone collision from the currently requested twist.
  bool is_projected_collision_detected(const geometry_msgs::msg::Twist &cmd) const;
  /// @brief Assess whether a local-plan window is straight enough for straight tracking.
  StraightSegmentAssessment assess_straight_segment(
    std::size_t start_index,
    double lookahead_distance) const;
  /// @brief Select a lookahead target while preserving tracking hysteresis.
  geometry_msgs::msg::PoseStamped select_tracking_target(
    double lookahead_distance,
    bool allow_retained_target);
  /// @brief Evaluate goal checker state and heading alignment requirements.
  GoalCheckResult check_goal(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose,
    double current_yaw);
  /// @brief Return true when scan points trigger the safety stop gate.
  bool is_safety_gate_triggered() const;
  /// @brief Update blocked hysteresis from a raw blocked candidate.
  bool update_blocked_state(bool blocked_candidate);
  /// @brief Update stalled hysteresis from a raw stalled candidate.
  bool update_stalled_state(bool stalled_candidate);
  /// @brief Initialize recovery reference pose/time for non-navigation commands.
  void ensure_recovery_reference_initialized();
  /// @brief Reset rejoin-context state used after target jumps or recovery.
  void reset_rejoin_context_state();
  /// @brief Activate rejoin context from the current pose.
  void activate_rejoin_context();
  /// @brief Expire rejoin context based on timeout or traveled distance.
  void update_rejoin_context_state();
  /// @brief Reset steering and command diagnostic counters.
  void reset_tracking_diagnostics_state();
  /// @brief Compute 2D Euclidean distance between two poses.
  double pose_distance(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string command_topic_;
  std::string local_plan_topic_;
  std::string scan_topic_;
  std::string status_topic_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_;
  double control_frequency_;
  double linear_speed_;
  double min_linear_speed_;
  double tracking_lookahead_distance_;
  double straight_tracking_lookahead_distance_;
  double tracking_min_target_distance_;
  std::size_t tracking_progress_rollback_window_;
  double tracking_target_hysteresis_distance_;
  double tracking_target_reset_distance_;
  double angular_gain_;
  double max_angular_speed_;
  double distance_tolerance_;
  double goal_heading_tolerance_;
  double goal_reach_heading_tolerance_;
  double final_align_max_angular_speed_;
  double final_align_heading_deadband_;
  double final_align_settle_time_sec_;
  double goal_checker_xy_tolerance_;
  double goal_checker_xy_hysteresis_;
  double goal_checker_yaw_tolerance_;
  double goal_checker_hold_time_sec_;
  bool goal_checker_respect_goal_yaw_;
  bool goal_checker_ignore_yaw_;
  double rotate_in_place_threshold_;
  double rotate_in_place_goal_distance_;
  double tracking_heading_deadband_;
  double tracking_heading_release_threshold_;
  bool straight_tracking_enabled_;
  double straight_curvature_threshold_;
  double straight_lateral_error_threshold_;
  double straight_heading_deadband_;
  double straight_heading_release_threshold_;
  double straight_angular_gain_;
  double straight_max_angular_speed_;
  double straight_heading_filter_alpha_;
  double rejoin_target_distance_threshold_;
  double rejoin_context_timeout_sec_;
  double rejoin_context_distance_m_;
  double rejoin_target_jump_threshold_m_;
  double rejoin_heading_gate_threshold_;
  double rejoin_min_linear_scale_;
  double heading_slowdown_threshold_;
  double min_heading_motion_scale_;
  bool regulated_pure_pursuit_enabled_;
  bool rpp_use_velocity_scaled_lookahead_;
  double rpp_min_tracking_lookahead_distance_;
  double rpp_max_tracking_lookahead_distance_;
  double rpp_lookahead_time_;
  bool rpp_use_curvature_speed_regulation_;
  double rpp_regulated_linear_scaling_min_radius_;
  double rpp_regulated_linear_scaling_min_speed_;
  bool rpp_use_approach_velocity_scaling_;
  double rpp_approach_velocity_scaling_distance_;
  double rpp_min_approach_linear_speed_;
  bool rpp_use_obstacle_velocity_scaling_;
  double rpp_obstacle_velocity_scaling_distance_;
  double rpp_obstacle_velocity_scaling_min_speed_;
  bool rpp_use_collision_projection_;
  double rpp_collision_projection_time_;
  double rpp_collision_projection_step_distance_;
  double max_linear_accel_;
  double max_angular_accel_;
  double progress_required_movement_radius_;
  double progress_time_allowance_sec_;
  double status_command_settle_time_sec_;
  int status_blocked_confirm_cycles_;
  int status_blocked_clear_cycles_;
  int status_stalled_confirm_cycles_;
  bool structured_logging_enabled_;
  double tracking_state_log_throttle_sec_;
  double cmd_quality_log_throttle_sec_;
  double target_jump_warn_threshold_m_;
  bool safety_gate_enabled_;
  bool safety_gate_allow_rotate_in_place_;
  double safety_gate_stop_distance_;
  double safety_gate_forward_angle_deg_;
  double safety_gate_rotate_heading_threshold_;
  int safety_gate_min_points_;
  VelocityControlMode velocity_control_mode_;
  AxisControllerConfig linear_controller_config_;
  AxisControllerConfig angular_controller_config_;
  AxisControllerState linear_controller_state_;
  AxisControllerState angular_controller_state_;
  geometry_msgs::msg::Twist current_twist_;
  amr_msgs::msg::MotionCommand latest_command_;
  nav_msgs::msg::Path latest_local_plan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped tracking_target_pose_;
  geometry_msgs::msg::PoseStamped progress_reference_pose_;
  geometry_msgs::msg::PoseStamped recovery_reference_pose_;
  geometry_msgs::msg::PoseStamped rejoin_context_start_pose_;
  sensor_msgs::msg::LaserScan latest_scan_;
  rclcpp::Time progress_reference_time_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time rejoin_context_start_time_;
  rclcpp::Time goal_checker_hold_start_time_;
  rclcpp::Time final_align_hold_start_time_;
  rclcpp::Time latest_command_time_;
  double recovery_start_yaw_;
  bool goal_checker_holding_;
  bool final_align_holding_;
  bool goal_xy_latched_;
  bool has_command_;
  bool has_local_plan_;
  bool has_current_pose_;
  bool has_latest_scan_;
  bool has_progress_reference_;
  bool has_recovery_reference_;
  bool has_tracking_progress_index_;
  bool has_tracking_target_index_;
  bool has_tracking_target_pose_;
  bool tracking_target_from_plan_;
  bool steering_hysteresis_active_;
  bool has_heading_error_filter_;
  bool rejoin_context_active_;
  bool rejoin_context_pending_;
  bool blocked_latched_;
  int blocked_streak_;
  int blocked_clear_streak_;
  int stalled_streak_;
  int cmd_ang_sign_;
  int last_cmd_ang_sign_;
  int cmd_ang_flip_count_;
  int output_ang_sign_;
  int last_output_ang_sign_;
  int output_ang_flip_count_;
  std::size_t tracking_progress_index_;
  std::size_t tracking_target_index_;
  std::size_t tracking_nearest_index_;
  std::size_t tracking_candidate_index_;
  double heading_error_filtered_;
  double tracking_selected_target_distance_;
  std::string tracking_selection_reason_;

public:
  /// @brief Construct the motion controller node and declare ROS parameters.
  explicit MotionController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the motion controller node.
  virtual ~MotionController() = default;
};

}  // namespace amr::motion::controller

namespace amr::controller::server
{

/// @brief Convenience owner that spins the local planner and motion controller together.
class ControllerServer
{
private:
  std::shared_ptr<amr::planner::local::LocalPlanner> local_planner_;
  std::shared_ptr<amr::motion::controller::MotionController> motion_controller_;

public:
  /// @brief Construct both lifecycle controller nodes.
  explicit ControllerServer();
  /// @brief Destroy the combined controller server.
  virtual ~ControllerServer() = default;

  /// @brief Spin both controller lifecycle nodes in a single-threaded executor.
  void spin();
};

}  // namespace amr::controller::server

#endif  // AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
