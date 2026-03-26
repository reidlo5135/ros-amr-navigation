#ifndef AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LifecycleManager() override;

private:
  struct ManagedNode
  {
    std::string name;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
  };

  void run_bringup();
  bool wait_for_service_clients(const ManagedNode & managed_node) const;
  bool request_transition(
    const ManagedNode & managed_node,
    std::uint8_t transition_id,
    std::chrono::milliseconds timeout) const;
  bool wait_for_state(
    const ManagedNode & managed_node,
    std::uint8_t target_state_id,
    std::chrono::milliseconds timeout) const;
  void publish_initial_pose();

  std::vector<std::string> managed_node_names_;
  std::vector<ManagedNode> managed_nodes_;
  bool autostart_;
  int service_timeout_ms_;
  int state_poll_interval_ms_;
  std::string startup_localization_mode_;
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
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  std::thread bringup_thread_;
  std::atomic<bool> shutdown_requested_;
};

}  // namespace amr_lifecycle_manager

#endif  // AMR_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
