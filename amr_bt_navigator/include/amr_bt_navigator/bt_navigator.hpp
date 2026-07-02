#ifndef AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

/**
 * @file bt_navigator.hpp
 * @brief Lifecycle action server that coordinates planning, motion commands, and recovery.
 */

#include <algorithm>
#include <cmath>
#include <chrono>
#include <exception>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/clear_costmap.hpp"
#include "amr_msgs/srv/plan_local_escape.hpp"
#include "amr_msgs/srv/plan_recovery.hpp"
#include "amr_msgs/srv/plan_segment.hpp"

namespace amr::bt::navigator
{

/// @brief Navigation action server for single-goal and route execution.
class Btnavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct the navigator node and declare ROS parameters.
  explicit Btnavigator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the navigator node.
  virtual ~Btnavigator() = default;

private:
  /// @brief Lifecycle callback return type alias.
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  /// @brief NavigateToPose action alias.
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  /// @brief NavigateToPoses action alias.
  using NavigateToPoses = amr_msgs::action::NavigateToPoses;
  /// @brief Single-goal action handle alias.
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  /// @brief Multi-goal action handle alias.
  using GoalHandleNavigateToPoses = rclcpp_action::ServerGoalHandle<NavigateToPoses>;

  /// @brief Result of executing one goal pose.
  struct ExecutionResult
  {
    /// @brief True when execution completed successfully.
    bool success{false};
    /// @brief True when execution was canceled by the action client.
    bool canceled{false};
    /// @brief Human-readable result message.
    std::string message;
    /// @brief Action error code; 0 means no error.
    uint16_t error_code{9000U};  // UNKNOWN by default; 0 = NONE on success/cancel
  };

  /// @brief Configure action servers, clients, subscriptions, publishers, and TF.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate command publisher and action handling.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Deactivate command publisher and stop accepting active output.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release configured resources and cached status.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Validate and accept/reject a NavigateToPose goal request.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  /// @brief Validate and accept/reject a NavigateToPoses goal request.
  rclcpp_action::GoalResponse handle_goals(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToPoses::Goal> goal);
  /// @brief Handle cancellation for a NavigateToPose goal.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  /// @brief Handle cancellation for a NavigateToPoses route.
  rclcpp_action::CancelResponse handle_cancel_goals(
    const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle);
  /// @brief Launch execution for an accepted single-goal handle.
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  /// @brief Launch execution for an accepted multi-goal handle.
  void handle_accepted_goals(const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle);
  /// @brief Execute a NavigateToPose action goal.
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  /// @brief Execute a NavigateToPoses route goal.
  void execute_goals(const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle);
  /// @brief Execute one pose goal through plan, command dispatch, feedback, and recovery.
  ExecutionResult execute_goal_pose(
    const geometry_msgs::msg::PoseStamped &goal_pose,
    const std::string &route_id,
    bool align_heading_at_goal,
    const std::function<bool()> &is_cancel_requested,
    const std::function<void(
      const geometry_msgs::msg::PoseStamped &,
      const amr_msgs::msg::MotionStatus &,
      int32_t,
      const rclcpp::Duration &)> &publish_feedback);
  /// @brief Cache the latest motion controller status.
  void handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message);
  /// @brief Cache the latest local planner status.
  void handle_local_plan_status(const amr_msgs::msg::LocalPlanStatus::SharedPtr message);
  /// @brief Update current robot pose from TF.
  bool update_current_pose_from_tf();
  /// @brief Return a mutex-protected copy of the current pose.
  geometry_msgs::msg::PoseStamped get_current_pose_copy();
  /// @brief Return a mutex-protected copy of latest motion status.
  amr_msgs::msg::MotionStatus get_motion_status_copy() const;
  /// @brief Return a mutex-protected copy of latest local plan status.
  amr_msgs::msg::LocalPlanStatus get_local_plan_status_copy() const;
  /// @brief Check whether required planner/controller state is available.
  bool is_navigator_ready(std::string &error_message) const;
  /// @brief Wait for global planner service availability.
  bool wait_for_planner_service(
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Wait for recovery-related service availability.
  bool wait_for_recovery_services(
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Request a global plan segment from the planner service.
  bool request_global_plan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    nav_msgs::msg::Path &plan,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Request a recovery motion command from the recovery service.
  bool request_recovery_command(
    const std::string &behavior,
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose,
    amr_msgs::msg::MotionCommand &command,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Request a local escape path from the controller local planner.
  bool request_local_escape_plan(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const nav_msgs::msg::Path &source_plan,
    nav_msgs::msg::Path &escape_plan,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Request local costmap clearing.
  bool clear_local_costmap(
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Wait until a dispatched motion command completes or fails.
  bool wait_for_command_completion(
    uint32_t command_id,
    int timeout_ms,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested = {});
  /// @brief Return true when normal navigation has resumed after recovery.
  bool has_reacquired_navigation(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::MotionStatus &motion_status,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status) const;
  /// @brief Return true when the planner status owns the active recovery need.
  bool has_planner_owned_recovery(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status) const;
  /// @brief Decide whether local escape should be attempted for the current state.
  bool should_try_local_escape(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status,
    const nav_msgs::msg::Path &planned_path,
    bool local_escape_dispatched) const;
  /// @brief Select backup, spin, wait, or local policy based on attempt index and state.
  std::string select_recovery_behavior(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status,
    int attempt_index) const;
  /// @brief Decide whether to resend the existing navigation plan after recovery.
  bool should_redispatch_existing_plan_after_recovery(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status,
    const std::string &executed_behavior) const;
  /// @brief Return timeout budget for a recovery behavior.
  int recovery_behavior_timeout_ms(const std::string &behavior) const;
  /// @brief Build a user-facing local escape failure description.
  std::string describe_local_escape_failure(const std::string &planner_message) const;
  /// @brief Build a user-facing recovery policy description.
  std::string describe_recovery_policy(
    const amr_msgs::msg::MotionCommand &active_command,
    const amr_msgs::msg::LocalPlanStatus &local_plan_status,
    const std::string &behavior,
    bool redispatch_existing_plan) const;
  /// @brief Return true when an action goal is currently active.
  bool has_active_goal() const;
  /// @brief Build a motion command from a planned path and goal metadata.
  amr_msgs::msg::MotionCommand build_motion_command(
    const geometry_msgs::msg::PoseStamped &goal_pose,
    const std::string &route_id,
    const nav_msgs::msg::Path &plan,
    bool align_heading_at_goal);
  /// @brief Publish a motion command when the lifecycle publisher is active.
  void publish_motion_command(const amr_msgs::msg::MotionCommand &command);
  /// @brief Publish a stop command to halt active motion.
  void publish_stop_command();

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp_action::Server<NavigateToPoses>::SharedPtr action_server_poses_;
  rclcpp::Client<amr_msgs::srv::PlanRecovery>::SharedPtr plan_recovery_client_;
  rclcpp::Client<amr_msgs::srv::PlanLocalEscape>::SharedPtr plan_local_escape_client_;
  rclcpp::Client<amr_msgs::srv::ClearCostmap>::SharedPtr clear_costmap_client_;
  rclcpp::Client<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_client_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string navigate_action_name_;
  std::string navigate_poses_action_name_;
  std::string command_topic_;
  std::string motion_status_topic_;
  std::string local_plan_status_topic_;
  std::string plan_recovery_service_;
  std::string plan_local_escape_service_;
  std::string clear_costmap_service_;
  std::string plan_segment_service_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_;
  std::string behavior_tree_xml_path_;
  std::string default_node_id_;
  int planner_wait_timeout_ms_;
  int feedback_period_ms_;
  int recovery_max_retries_;
  int recovery_retry_delay_ms_;
  int recovery_reacquire_settle_ms_;
  bool align_heading_at_goal_;
  bool structured_logging_enabled_;
  bool recovery_decision_logging_enabled_;
  double nominal_speed_;
  uint32_t next_command_id_;
  geometry_msgs::msg::PoseStamped current_pose_;
  amr_msgs::msg::MotionStatus latest_motion_status_;
  amr_msgs::msg::LocalPlanStatus latest_local_plan_status_;
  bool has_current_pose_;
  bool has_motion_status_;
  bool has_local_plan_status_;
  mutable std::mutex navigator_mutex_;
  mutable std::mutex active_goal_mutex_;
  std::weak_ptr<GoalHandleNavigateToPose> active_goal_handle_;
  std::weak_ptr<GoalHandleNavigateToPoses> active_goals_handle_;
};

}  // namespace amr::bt::navigator

#endif  // AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
