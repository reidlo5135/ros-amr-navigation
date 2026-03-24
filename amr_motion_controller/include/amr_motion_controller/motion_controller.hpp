#ifndef AMR_MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
#define AMR_MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_

#include <string>

#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_motion_controller
{

class MotionController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MotionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  void handle_local_plan(const nav_msgs::msg::Path::SharedPtr message);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void publish_control();
  void reset_velocity_controller_state();
  void reset_progress_checker_state();
  void publish_zero_twist();
  VelocityControlMode parse_velocity_control_mode(const std::string & mode) const;
  double apply_axis_controller(
    double current,
    double target,
    AxisControllerState & state,
    const AxisControllerConfig & config,
    double max_step,
    double dt) const;
  geometry_msgs::msg::Twist apply_velocity_controller(
    const geometry_msgs::msg::Twist & current,
    const geometry_msgs::msg::Twist & target);
  double estimate_remaining_distance(const nav_msgs::msg::Path & path) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalize_angle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;
  geometry_msgs::msg::PoseStamped select_tracking_target() const;
  bool is_safety_gate_triggered() const;
  void ensure_recovery_reference_initialized();
  double pose_distance(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

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
  double angular_gain_;
  double max_angular_speed_;
  double distance_tolerance_;
  double goal_heading_tolerance_;
  double rotate_in_place_threshold_;
  double rotate_in_place_goal_distance_;
  double heading_slowdown_threshold_;
  double min_heading_motion_scale_;
  double max_linear_accel_;
  double max_angular_accel_;
  double progress_required_movement_radius_;
  double progress_time_allowance_sec_;
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
  geometry_msgs::msg::PoseStamped progress_reference_pose_;
  geometry_msgs::msg::PoseStamped recovery_reference_pose_;
  sensor_msgs::msg::LaserScan latest_scan_;
  rclcpp::Time progress_reference_time_;
  rclcpp::Time recovery_start_time_;
  double recovery_start_yaw_;
  bool has_command_;
  bool has_local_plan_;
  bool has_current_pose_;
  bool has_latest_scan_;
  bool has_progress_reference_;
  bool has_recovery_reference_;
};

}  // namespace amr_motion_controller

#endif  // AMR_MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
