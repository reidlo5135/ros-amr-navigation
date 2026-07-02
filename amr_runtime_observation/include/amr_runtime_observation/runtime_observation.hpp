#ifndef AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_
#define AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_

/**
 * @file runtime_observation.hpp
 * @brief Runtime observer node that summarizes navigation progress, blockage, and recovery state.
 */

#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr::runtime::observation
{

/// @brief Aggregates navigation topics into compact runtime summary and event JSON streams.
class RuntimeObservation : public rclcpp::Node
{
public:
  /// @brief Construct subscriptions, publishers, timers, and observation parameters.
  explicit RuntimeObservation(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the runtime observation node.
  ~RuntimeObservation() override = default;

private:
  /// @brief NavigateToPoses feedback wrapper type used by ROS action subscriptions.
  using NavigateToPosesFeedbackMessage =
    amr_msgs::action::NavigateToPoses::Impl::FeedbackMessage;

  /// @brief Immutable observation state assembled for one publish cycle.
  struct Snapshot
  {
    /// @brief True when route execution is currently considered active.
    bool route_active{false};
    /// @brief True when route feedback has stopped making distance progress.
    bool progress_stalled{false};
    /// @brief True when the motion controller reports blocked.
    bool motion_blocked{false};
    /// @brief True when the motion controller reports stalled.
    bool motion_stalled{false};
    /// @brief True when the local planner requests recovery.
    bool local_recovery_required{false};
    /// @brief True when the motion controller has reached the active goal.
    bool motion_goal_reached{false};
    /// @brief True when the controller is executing a recovery command.
    bool controller_recovery{false};
    /// @brief Current route goal index reported by action feedback.
    uint32_t current_goal_index{0U};
    /// @brief Total number of route goals reported by action feedback.
    uint32_t goal_count{0U};
    /// @brief Number of recoveries reported by action feedback.
    int16_t number_of_recoveries{0};
    /// @brief Local planner decision code.
    uint8_t planner_decision{amr_msgs::msg::LocalPlanStatus::DECISION_OK};
    /// @brief Latest action status code.
    int8_t action_status{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
    /// @brief True when a recovery condition is currently active.
    bool recovery_triggered{false};
    /// @brief True when a local escape command is active.
    bool local_escape_active{false};
    /// @brief Human-readable blocked context label.
    std::string blocked_context{"clear"};
    /// @brief Human-readable recovery reason label.
    std::string recovery_reason{"none"};
    /// @brief Human-readable recovery phase label.
    std::string recovery_phase{"idle"};
    /// @brief Human-readable runtime state label.
    std::string runtime_state{"idle"};
    /// @brief Human-readable controller phase label.
    std::string controller_phase{"idle"};
    /// @brief Human-readable reason for clearing progress-stall context.
    std::string progress_clear_reason{"none"};
    /// @brief Delta in remaining goal distance since previous observation.
    double dist_goal_delta_m{0.0};
  };

  /// @brief Cache the latest motion command and detect local escape commands.
  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  /// @brief Cache the latest motion status and progress distance.
  void handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message);
  /// @brief Cache the latest local plan status.
  void handle_local_plan_status(const amr_msgs::msg::LocalPlanStatus::SharedPtr message);
  /// @brief Cache the latest NavigateToPoses feedback message.
  void handle_navigate_feedback(const NavigateToPosesFeedbackMessage::SharedPtr message);
  /// @brief Cache the latest NavigateToPoses action status array.
  void handle_navigate_status(const action_msgs::msg::GoalStatusArray::SharedPtr message);
  /// @brief Publish the current summary and any state-change event.
  void publish_observation();

  /// @brief Return true when recent route feedback indicates active execution.
  bool is_route_active(const rclcpp::Time &now) const;
  /// @brief Return true when route progress has stalled for the configured window.
  bool is_progress_stalled(const rclcpp::Time &now) const;
  /// @brief Return true when the controller is executing a non-navigation command.
  bool is_controller_recovery() const;
  /// @brief Return true when controller status is not blocked or stalled.
  bool is_controller_clear() const;
  /// @brief Return true when the cached local plan status is still fresh.
  bool is_local_plan_status_current() const;
  /// @brief Return true when recovery is active outside controller-owned motion.
  bool has_explicit_non_controller_recovery() const;
  /// @brief Return true when the controller is near the active goal.
  bool is_goal_approach_context() const;
  /// @brief Return true when the controller is aligning final heading.
  bool is_final_heading_alignment_context() const;
  /// @brief Resolve the most relevant action status code.
  int8_t resolve_action_status() const;
  /// @brief Resolve a user-facing controller phase label.
  std::string resolve_controller_phase() const;
  /// @brief Resolve why progress-stall context should be considered clear.
  std::string resolve_progress_clear_reason(bool progress_stalled, bool route_active) const;
  /// @brief Resolve the current blocked context label.
  std::string resolve_blocked_context(bool progress_stalled) const;
  /// @brief Resolve the current recovery reason label.
  std::string resolve_recovery_reason(bool progress_stalled) const;
  /// @brief Resolve the current recovery phase label.
  std::string resolve_recovery_phase(bool route_active, bool recovery_triggered) const;
  /// @brief Resolve the top-level runtime state label.
  std::string resolve_runtime_state(bool route_active, bool progress_stalled) const;
  /// @brief Build an observation snapshot from cached topics.
  Snapshot make_snapshot(const rclcpp::Time &now) const;
  /// @brief Publish a state-change event when the snapshot differs from the previous one.
  void publish_event_if_needed(const Snapshot &snapshot, const rclcpp::Time &now);
  /// @brief Publish one structured runtime event.
  void publish_event(
    const std::string &event_type,
    const std::string &reason,
    const Snapshot &snapshot,
    const rclcpp::Time &now);
  /// @brief Serialize an observation snapshot into summary JSON.
  std::string build_summary_json(const Snapshot &snapshot, const rclcpp::Time &now) const;
  /// @brief Serialize an observation snapshot into event JSON.
  std::string build_event_json(
    const std::string &event_type,
    const std::string &reason,
    const Snapshot &snapshot,
    const rclcpp::Time &now) const;
  /// @brief Convert a local planner decision code into a stable label.
  std::string planner_decision_label(uint8_t decision) const;
  /// @brief Convert an action status code into a stable label.
  std::string action_status_label(int8_t status) const;
  /// @brief Escape a string for embedding in JSON output.
  static std::string escape_json(const std::string &value);

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_subscription_;
  rclcpp::Subscription<NavigateToPosesFeedbackMessage>::SharedPtr navigate_feedback_subscription_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_status_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr observation_summary_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr observation_event_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string motion_command_topic_;
  std::string motion_status_topic_;
  std::string local_plan_status_topic_;
  std::string navigate_to_poses_action_name_;
  std::string observation_summary_topic_;
  std::string observation_event_topic_;
  int publish_period_ms_;
  int route_stale_timeout_ms_;
  double progress_stall_window_sec_;
  double progress_epsilon_;
  double progress_clear_delta_m_;
  double goal_approach_distance_;
  double final_heading_alignment_distance_;
  bool structured_logging_enabled_;
  double summary_log_throttle_sec_;
  bool heavy_topic_observation_enabled_;

  amr_msgs::msg::MotionCommand latest_motion_command_;
  amr_msgs::msg::MotionStatus latest_motion_status_;
  amr_msgs::msg::LocalPlanStatus latest_local_plan_status_;
  NavigateToPosesFeedbackMessage latest_navigate_feedback_;
  action_msgs::msg::GoalStatusArray latest_navigate_status_;
  bool has_motion_command_{false};
  bool has_motion_status_{false};
  bool has_local_plan_status_{false};
  bool has_navigate_feedback_{false};
  bool has_navigate_status_{false};
  rclcpp::Time last_motion_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motion_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_plan_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigate_feedback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigate_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time best_progress_time_{0, 0, RCL_ROS_TIME};
  float best_distance_remaining_{0.0f};
  bool has_progress_baseline_{false};
  double previous_motion_distance_remaining_{0.0};
  double last_dist_goal_delta_m_{0.0};
  bool has_previous_motion_distance_{false};
  uint32_t local_escape_command_id_{0U};
  bool local_escape_command_active_{false};
  Snapshot previous_snapshot_;
  bool has_previous_snapshot_{false};
};

}  // namespace amr::runtime::observation

#endif  // AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_
