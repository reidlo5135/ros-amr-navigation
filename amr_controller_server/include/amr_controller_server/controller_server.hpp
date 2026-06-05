#ifndef AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

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

#include "amr_geometry/footprint.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/plan_local_escape.hpp"

namespace amr::planner::local
{

class LocalPlanner : public rclcpp_lifecycle::LifecycleNode
{
private:
  struct LocalPlanBuildResult
  {
    nav_msgs::msg::Path plan;
    bool local_plan_valid{false};
    bool recovery_required{false};
    uint8_t decision{amr_msgs::msg::LocalPlanStatus::DECISION_OK};
    bool has_blocked_pose{false};
    geometry_msgs::msg::PoseStamped blocked_pose;
    double blocked_distance{0.0};
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_plan_local_escape(
    const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response);
  void publish_local_plan();
  LocalPlanBuildResult build_local_plan(
    const amr_msgs::msg::MotionCommand &command,
    const geometry_msgs::msg::PoseStamped &current_pose);
  nav_msgs::msg::Path build_inflated_local_plan(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index,
    double lookahead_distance);
  nav_msgs::msg::Path build_sliced_local_plan(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index) const;
  nav_msgs::msg::Path build_sliced_local_plan_with_lookahead(
    const nav_msgs::msg::Path &source_plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t closest_index,
    double lookahead_distance) const;
  nav_msgs::msg::Path build_source_plan(const amr_msgs::msg::MotionCommand &command) const;
  std::size_t find_closest_pose_index(
    const nav_msgs::msg::Path &plan,
    const geometry_msgs::msg::PoseStamped &current_pose,
    std::size_t start_index) const;
  bool world_to_grid(const geometry_msgs::msg::Point &point, int &grid_x, int &grid_y) const;
  geometry_msgs::msg::PoseStamped grid_to_pose(
    int grid_x,
    int grid_y,
    const std::string &frame_id) const;
  bool is_occupied_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y) const;
  bool is_grid_pose_collision(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y,
    double yaw) const;
  bool find_nearest_free_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    int &grid_x,
    int &grid_y,
    int max_radius,
    double yaw) const;
  geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    double ratio) const;
  double pose_distance(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;
  bool find_first_blocked_pose_on_plan(
    const nav_msgs::msg::Path &plan,
    geometry_msgs::msg::PoseStamped &blocked_pose) const;
  void reset_dynamic_blocked_state();
  bool confirm_dynamic_recovery_decision(uint8_t decision, double blocked_distance);
  bool confirm_dynamic_clear();
  int required_dynamic_recovery_cycles(uint8_t decision, double blocked_distance) const;
  double sample_lateral_occupancy(
    double origin_x,
    double origin_y,
    double heading,
    double lateral_sign) const;
  nav_msgs::msg::Path refine_local_plan(const nav_msgs::msg::Path &plan) const;
  nav_msgs::msg::Path prune_and_interpolate_path(const nav_msgs::msg::Path &plan) const;
  nav_msgs::msg::Path apply_path_smoother(const nav_msgs::msg::Path &plan) const;
  nav_msgs::msg::Path smooth_path_corners(const nav_msgs::msg::Path &plan) const;
  bool is_smoothed_path_acceptable(
    const nav_msgs::msg::Path &base_plan,
    const nav_msgs::msg::Path &smoothed_plan) const;
  double estimate_path_length(const nav_msgs::msg::Path &plan) const;
  double estimate_pose_distance_to_path(
    const geometry_msgs::msg::PoseStamped &pose,
    const nav_msgs::msg::Path &path) const;
  bool is_path_collision_free(const nav_msgs::msg::Path &plan) const;
  bool is_pose_collision_free(const geometry_msgs::msg::PoseStamped &pose) const;
  void assign_path_headings(nav_msgs::msg::Path &plan) const;
  geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Service<amr_msgs::srv::PlanLocalEscape>::SharedPtr local_escape_service_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string current_pose_topic_;
  std::string map_topic_;
  std::string local_plan_topic_;
  std::string local_plan_status_topic_;
  std::string local_escape_service_name_;
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
  bool path_refiner_corner_smoothing_enabled_;
  double path_refiner_corner_smoothing_max_offset_;
  double path_refiner_corner_smoothing_angle_threshold_;
  int path_refiner_corner_smoothing_samples_;
  double path_refiner_smoothing_max_length_ratio_;
  double path_refiner_smoothing_max_pose_deviation_;
  bool path_refiner_collision_check_enabled_;
  double path_refiner_collision_sample_distance_;
  bool dynamic_obstacle_enabled_;
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
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  nav_msgs::msg::OccupancyGrid working_costmap_;
  bool has_command_;
  bool has_current_pose_;
  bool has_map_;

public:
  explicit LocalPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~LocalPlanner() = default;
};

}  // namespace amr::planner::local

namespace amr::motion::controller
{

class MotionController : public rclcpp_lifecycle::LifecycleNode
{
private:
  enum class VelocityControlMode
  {
    P,
    PI,
    PID
  };

  struct AxisControllerConfig
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double integral_limit{0.0};
  };

  struct AxisControllerState
  {
    double integral{0.0};
    double previous_error{0.0};
    bool first_update{true};
  };

  struct GoalCheckResult
  {
    bool distance_reached{false};
    bool heading_reached{true};
    bool goal_reached{false};
    bool align_heading{false};
    double distance_error{0.0};
    double heading_error{0.0};
    double target_yaw{0.0};
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  void handle_local_plan(const nav_msgs::msg::Path::SharedPtr message);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void publish_control();
  void reset_velocity_controller_state();
  void reset_progress_checker_state();
  void reset_goal_checker_state();
  void reset_status_semantics_state();
  void publish_zero_twist();
  VelocityControlMode parse_velocity_control_mode(const std::string &mode) const;
  double apply_axis_controller(
    double current,
    double target,
    AxisControllerState &state,
    const AxisControllerConfig &config,
    double max_step,
    double dt) const;
  geometry_msgs::msg::Twist apply_velocity_controller(
    const geometry_msgs::msg::Twist &current,
    const geometry_msgs::msg::Twist &target);
  double estimate_remaining_distance(const nav_msgs::msg::Path &path) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation) const;
  double normalize_angle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;
  geometry_msgs::msg::PoseStamped select_tracking_target();
  GoalCheckResult check_goal(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose,
    double current_yaw);
  bool is_safety_gate_triggered() const;
  bool update_blocked_state(bool blocked_candidate);
  bool update_stalled_state(bool stalled_candidate);
  void ensure_recovery_reference_initialized();
  double pose_distance(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string local_plan_topic_;
  std::string current_pose_topic_;
  std::string scan_topic_;
  std::string status_topic_;
  std::string cmd_vel_topic_;
  double control_frequency_;
  double linear_speed_;
  double min_linear_speed_;
  double tracking_lookahead_distance_;
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
  double goal_checker_yaw_tolerance_;
  double goal_checker_hold_time_sec_;
  bool goal_checker_respect_goal_yaw_;
  bool goal_checker_ignore_yaw_;
  double rotate_in_place_threshold_;
  double rotate_in_place_goal_distance_;
  double tracking_heading_deadband_;
  double rejoin_target_distance_threshold_;
  double rejoin_heading_gate_threshold_;
  double rejoin_min_linear_scale_;
  double heading_slowdown_threshold_;
  double min_heading_motion_scale_;
  double max_linear_accel_;
  double max_angular_accel_;
  double progress_required_movement_radius_;
  double progress_time_allowance_sec_;
  double status_command_settle_time_sec_;
  int status_blocked_confirm_cycles_;
  int status_blocked_clear_cycles_;
  int status_stalled_confirm_cycles_;
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
  sensor_msgs::msg::LaserScan latest_scan_;
  rclcpp::Time progress_reference_time_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time goal_checker_hold_start_time_;
  rclcpp::Time final_align_hold_start_time_;
  rclcpp::Time latest_command_time_;
  double recovery_start_yaw_;
  bool goal_checker_holding_;
  bool final_align_holding_;
  bool has_command_;
  bool has_local_plan_;
  bool has_current_pose_;
  bool has_latest_scan_;
  bool has_progress_reference_;
  bool has_recovery_reference_;
  bool has_tracking_progress_index_;
  bool has_tracking_target_index_;
  bool has_tracking_target_pose_;
  bool blocked_latched_;
  int blocked_streak_;
  int blocked_clear_streak_;
  int stalled_streak_;
  std::size_t tracking_progress_index_;
  std::size_t tracking_target_index_;

public:
  explicit MotionController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~MotionController() = default;
};

}  // namespace amr::motion::controller

namespace amr::controller::server
{

class ControllerServer
{
private:
  std::shared_ptr<amr::planner::local::LocalPlanner> local_planner_;
  std::shared_ptr<amr::motion::controller::MotionController> motion_controller_;

public:
  explicit ControllerServer();
  virtual ~ControllerServer() = default;

  void spin();
};

}  // namespace amr::controller::server

#endif  // AMR_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
