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

namespace amr_motion_controller
{

class MotionController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MotionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
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
  void publish_control();
  double estimate_remaining_distance(const nav_msgs::msg::Path & path) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalize_angle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;
  geometry_msgs::msg::PoseStamped select_tracking_target() const;
  double pose_distance(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string local_plan_topic_;
  std::string current_pose_topic_;
  std::string status_topic_;
  std::string cmd_vel_topic_;
  int publish_period_ms_;
  double nominal_linear_velocity_;
  double max_linear_velocity_;
  double min_linear_velocity_;
  double max_angular_velocity_;
  double heading_gain_;
  double rotate_in_place_threshold_;
  double goal_tolerance_;
  amr_msgs::msg::MotionCommand latest_command_;
  nav_msgs::msg::Path latest_local_plan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_command_;
  bool has_local_plan_;
  bool has_current_pose_;
};

}  // namespace amr_motion_controller

#endif  // AMR_MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
