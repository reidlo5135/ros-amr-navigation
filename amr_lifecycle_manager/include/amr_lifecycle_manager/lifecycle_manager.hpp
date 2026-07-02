#ifndef AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

/**
 * @file lifecycle_manager.hpp
 * @brief Startup coordinator for AMR lifecycle-managed navigation nodes.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amr::lifecycle::manager
{

/// @brief Drives configured lifecycle nodes through configure and activate transitions.
class LifecycleManager : public rclcpp::Node
{
public:
  /// @brief Construct the lifecycle manager and optionally start autostart bringup.
  explicit LifecycleManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Request bringup-thread shutdown and destroy the manager.
  virtual ~LifecycleManager() override;

private:
  /// @brief Service clients associated with one managed lifecycle node.
  struct ManagedNode
  {
    /// @brief Fully qualified node name.
    std::string name;
    /// @brief Client used to query the lifecycle state.
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
    /// @brief Client used to request lifecycle transitions.
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
  };

  /// @brief Configure and activate each managed node in sequence.
  void run_bringup();
  /// @brief Wait until lifecycle service clients are available for a node.
  bool wait_for_service_clients(const ManagedNode &managed_node) const;
  /// @brief Request one lifecycle transition and wait for the service response.
  bool request_transition(
    const ManagedNode &managed_node,
    std::uint8_t transition_id,
    std::chrono::milliseconds timeout) const;
  /// @brief Poll a managed node until it reaches the requested lifecycle state.
  bool wait_for_state(
    const ManagedNode &managed_node,
    std::uint8_t target_state_id,
    std::chrono::milliseconds timeout) const;
  /// @brief Publish the configured initial pose after bringup when enabled.
  void publish_initial_pose();

  std::vector<std::string> managed_node_names_;
  std::vector<ManagedNode> managed_nodes_;
  bool autostart_;
  int service_timeout_ms_;
  int state_poll_interval_ms_;
  bool initial_pose_enabled_;
  std::string initial_pose_topic_;
  std::string initial_pose_frame_id_;
  double initial_pose_delay_sec_;
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_yaw_;
  double initial_pose_covariance_x_;
  double initial_pose_covariance_y_;
  double initial_pose_covariance_yaw_;
  bool structured_logging_enabled_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  std::thread bringup_thread_;
  std::atomic<bool> shutdown_requested_;
};

}  // namespace amr::lifecycle::manager

#endif  // AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
