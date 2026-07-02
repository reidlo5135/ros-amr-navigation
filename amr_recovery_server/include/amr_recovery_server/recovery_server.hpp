#ifndef AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_
#define AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_

/**
 * @file recovery_server.hpp
 * @brief Lifecycle service node that generates backup, spin, and wait recovery commands.
 */

#include <cmath>
#include <memory>
#include <string>

#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/srv/plan_recovery.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace amr::recovery::server
{

/// @brief Provides recovery motion command plans for the navigation behavior tree.
class RecoveryServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct the recovery server node and declare ROS parameters.
  explicit RecoveryServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the recovery server node.
  virtual ~RecoveryServer() = default;

private:
  /// @brief Lifecycle callback return type alias.
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure recovery service resources.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate recovery planning.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Deactivate recovery planning.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release configured service resources.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Handle a recovery planning service request.
  void handle_plan_recovery(
    const std::shared_ptr<amr_msgs::srv::PlanRecovery::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanRecovery::Response> response);
  /// @brief Build a reverse recovery motion command from the current pose.
  amr_msgs::msg::MotionCommand build_backup_command(
    const geometry_msgs::msg::PoseStamped &current_pose) const;
  /// @brief Build an in-place spin recovery motion command from the current pose.
  amr_msgs::msg::MotionCommand build_spin_command(
    const geometry_msgs::msg::PoseStamped &current_pose) const;
  /// @brief Build a timed wait recovery motion command from the current pose.
  amr_msgs::msg::MotionCommand build_wait_command(
    const geometry_msgs::msg::PoseStamped &current_pose) const;
  /// @brief Extract yaw from a quaternion.
  static double quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation);
  /// @brief Build a yaw-only quaternion.
  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  rclcpp::Service<amr_msgs::srv::PlanRecovery>::SharedPtr plan_recovery_service_;
  std::string plan_recovery_service_name_;
  std::string default_node_id_;
  double wait_duration_sec_;
  double backup_distance_;
  double backup_speed_;
  double spin_angle_rad_;
  bool structured_logging_enabled_;
};

}  // namespace amr::recovery::server

#endif  // AMR_RECOVERY_SERVER__RECOVERY_SERVER_HPP_
