#ifndef AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_
#define AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/srv/plan_recovery.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_recovery_server
{

class RecoveryServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RecoveryServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_plan_recovery(
    const std::shared_ptr<amr_msgs::srv::PlanRecovery::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanRecovery::Response> response);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  amr_msgs::msg::MotionCommand build_backup_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_spin_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_wait_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_arl_spin_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_arl_wait_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_probe_forward_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_probe_forward_long_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  amr_msgs::msg::MotionCommand build_arl_probe_auto_command(
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  static double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation);
  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  rclcpp::Service<amr_msgs::srv::PlanRecovery>::SharedPtr plan_recovery_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::string plan_recovery_service_name_;
  std::string scan_topic_;
  std::string default_node_id_;
  double wait_duration_sec_;
  double backup_distance_;
  double backup_speed_;
  double spin_angle_rad_;
  double arl_wait_duration_sec_;
  double arl_spin_angle_rad_;
  double arl_probe_distance_;
  double arl_probe_long_distance_;
  double arl_probe_speed_;
  double arl_probe_safety_margin_;
  double arl_probe_sector_half_width_rad_;
  double arl_probe_spin_heading_threshold_rad_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_latest_scan_;
};

}  // namespace amr_recovery_server

#endif  // AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_
