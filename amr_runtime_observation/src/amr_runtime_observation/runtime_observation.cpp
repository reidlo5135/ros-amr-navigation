/**
 * @file runtime_observation.cpp
 * @brief Implementation of runtime summary and event observation.
 */

#include "amr_runtime_observation/runtime_observation.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace amr::runtime::observation
{

namespace
{

/// @brief Return a stable true/false label for structured logs.
const char *bool_label(const bool value)
{
  return value ? "true" : "false";
}

/// @brief Convert a seconds duration parameter into a positive throttle period in milliseconds.
int throttle_ms_from_sec(const double seconds)
{
  return static_cast<int>(std::max(0.1, seconds) * 1000.0);
}

}  // namespace

/// @copydoc RuntimeObservation::RuntimeObservation
RuntimeObservation::RuntimeObservation(const rclcpp::NodeOptions &options)
: rclcpp::Node("runtime_observation", options)
{
  this->declare_parameter("topics.motion_status", "/motion_status");
  this->declare_parameter("topics.motion_command", "/motion_command");
  this->declare_parameter("topics.local_plan_status", "/local_plan_status");
  this->declare_parameter("actions.navigate_to_poses", "/navigate_to_poses");
  this->declare_parameter("topics.observation_summary", "/observation/runtime/summary");
  this->declare_parameter("topics.observation_events", "/observation/runtime/events");
  this->declare_parameter("observation.publish_period_ms", 200);
  this->declare_parameter("observation.route_stale_timeout_ms", 1500);
  this->declare_parameter("observation.progress_stall_window_sec", 3.0);
  this->declare_parameter("observation.progress_epsilon", 0.05);
  this->declare_parameter("observation.progress_clear_delta_m", 0.005);
  this->declare_parameter("observation.goal_approach_distance", 0.14);
  this->declare_parameter("observation.final_heading_alignment_distance", 0.08);
  this->declare_parameter("logging.structured_enabled", true);
  this->declare_parameter("logging.summary_throttle_sec", 1.0);
  this->declare_parameter("logging.heavy_topic_observation_enabled", false);

  this->get_parameter("topics.motion_status", this->motion_status_topic_);
  this->get_parameter("topics.motion_command", this->motion_command_topic_);
  this->get_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->get_parameter("actions.navigate_to_poses", this->navigate_to_poses_action_name_);
  this->get_parameter("topics.observation_summary", this->observation_summary_topic_);
  this->get_parameter("topics.observation_events", this->observation_event_topic_);
  this->get_parameter("observation.publish_period_ms", this->publish_period_ms_);
  this->get_parameter("observation.route_stale_timeout_ms", this->route_stale_timeout_ms_);
  this->get_parameter("observation.progress_stall_window_sec", this->progress_stall_window_sec_);
  this->get_parameter("observation.progress_epsilon", this->progress_epsilon_);
  this->get_parameter("observation.progress_clear_delta_m", this->progress_clear_delta_m_);
  this->get_parameter("observation.goal_approach_distance", this->goal_approach_distance_);
  this->get_parameter(
    "observation.final_heading_alignment_distance", this->final_heading_alignment_distance_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->get_parameter("logging.summary_throttle_sec", this->summary_log_throttle_sec_);
  this->get_parameter(
    "logging.heavy_topic_observation_enabled", this->heavy_topic_observation_enabled_);

  const std::string feedback_topic = this->navigate_to_poses_action_name_ + "/_action/feedback";
  const std::string status_topic = this->navigate_to_poses_action_name_ + "/_action/status";

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->motion_command_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RuntimeObservation::handle_motion_command, this, std::placeholders::_1));
  this->motion_status_subscription_ = this->create_subscription<amr_msgs::msg::MotionStatus>(
    this->motion_status_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RuntimeObservation::handle_motion_status, this, std::placeholders::_1));
  this->local_plan_status_subscription_ =
    this->create_subscription<amr_msgs::msg::LocalPlanStatus>(
    this->local_plan_status_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RuntimeObservation::handle_local_plan_status, this, std::placeholders::_1));
  this->navigate_feedback_subscription_ =
    this->create_subscription<NavigateToPosesFeedbackMessage>(
    feedback_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RuntimeObservation::handle_navigate_feedback, this, std::placeholders::_1));
  this->navigate_status_subscription_ =
    this->create_subscription<action_msgs::msg::GoalStatusArray>(
    status_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RuntimeObservation::handle_navigate_status, this, std::placeholders::_1));

  this->observation_summary_publisher_ =
    this->create_publisher<std_msgs::msg::String>(
    this->observation_summary_topic_,
    rclcpp::SystemDefaultsQoS());
  this->observation_event_publisher_ =
    this->create_publisher<std_msgs::msg::String>(
    this->observation_event_topic_,
    rclcpp::SystemDefaultsQoS());

  this->publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(50, this->publish_period_ms_)),
    std::bind(&RuntimeObservation::publish_observation, this));

  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=runtime_observation event=runtime_summary phase=configured motion_command_topic=%s motion_status_topic=%s local_plan_status_topic=%s summary_topic=%s events_topic=%s heavy_topic_observation_enabled=%s",
      this->motion_command_topic_.c_str(),
      this->motion_status_topic_.c_str(),
      this->local_plan_status_topic_.c_str(),
      this->observation_summary_topic_.c_str(),
      this->observation_event_topic_.c_str(),
      bool_label(this->heavy_topic_observation_enabled_));
  }
}

/// @copydoc RuntimeObservation::handle_motion_command
void RuntimeObservation::handle_motion_command(
  const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  const bool new_motion_command =
    !this->has_motion_command_ || message->command_id != this->latest_motion_command_.command_id;
  this->latest_motion_command_ = *message;
  this->has_motion_command_ = true;
  this->last_motion_command_time_ = this->now();

  if (new_motion_command)
  {
    this->has_progress_baseline_ = false;
    this->best_distance_remaining_ = 0.0f;
    this->best_progress_time_ = this->last_motion_command_time_;
    this->has_previous_motion_distance_ = false;
    this->last_dist_goal_delta_m_ = 0.0;
  }

  const bool planner_recovery_context =
    this->has_local_plan_status_ &&
    this->latest_local_plan_status_.recovery_required &&
    this->latest_local_plan_status_.decision !=
      amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED;

  if (message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE && planner_recovery_context) {
    this->local_escape_command_id_ = message->command_id;
    this->local_escape_command_active_ = true;
    return;
  }

  if (message->mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE) {
    this->local_escape_command_id_ = 0U;
    this->local_escape_command_active_ = false;
    return;
  }

  if (!planner_recovery_context) {
    this->local_escape_command_id_ = 0U;
    this->local_escape_command_active_ = false;
  }
}

/// @copydoc RuntimeObservation::handle_motion_status
void RuntimeObservation::handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message)
{
  const auto now = this->now();
  const bool same_motion_command =
    this->has_motion_status_ && message->command_id == this->latest_motion_status_.command_id;
  this->last_dist_goal_delta_m_ =
    (this->has_previous_motion_distance_ && same_motion_command) ?
    this->previous_motion_distance_remaining_ - message->remaining_distance :
    0.0;
  this->previous_motion_distance_remaining_ = message->remaining_distance;
  this->has_previous_motion_distance_ = true;

  this->latest_motion_status_ = *message;
  this->has_motion_status_ = true;
  this->last_motion_status_time_ = now;

  if (
    message->active &&
    message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&
    (!this->has_progress_baseline_ ||
    message->remaining_distance <
      static_cast<double>(this->best_distance_remaining_) - this->progress_epsilon_ ||
    this->last_dist_goal_delta_m_ >= std::max(0.0, this->progress_clear_delta_m_)))
  {
    this->best_distance_remaining_ = static_cast<float>(message->remaining_distance);
    this->best_progress_time_ = now;
    this->has_progress_baseline_ = true;
  }

  if (message->goal_reached || message->command_completed)
  {
    this->best_distance_remaining_ = static_cast<float>(message->remaining_distance);
    this->best_progress_time_ = now;
    this->has_progress_baseline_ = true;
  }

  if (this->local_escape_command_active_ && message->command_id != this->local_escape_command_id_) {
    this->local_escape_command_active_ = false;
  }
  if (this->local_escape_command_active_ && (!message->active || message->command_completed)) {
    this->local_escape_command_active_ = false;
  }
}

/// @copydoc RuntimeObservation::handle_local_plan_status
void RuntimeObservation::handle_local_plan_status(
  const amr_msgs::msg::LocalPlanStatus::SharedPtr message)
{
  this->latest_local_plan_status_ = *message;
  this->has_local_plan_status_ = true;
  this->last_local_plan_status_time_ = this->now();
}

/// @copydoc RuntimeObservation::handle_navigate_feedback
void RuntimeObservation::handle_navigate_feedback(
  const NavigateToPosesFeedbackMessage::SharedPtr message)
{
  this->latest_navigate_feedback_ = *message;
  this->has_navigate_feedback_ = true;
  this->last_navigate_feedback_time_ = this->now();

  const double progress_threshold = this->is_controller_clear() ?
    std::max(0.0, this->progress_clear_delta_m_) :
    std::max(0.0, this->progress_epsilon_);
  if (!this->has_progress_baseline_ ||
    message->feedback.distance_remaining < (this->best_distance_remaining_ - progress_threshold))
  {
    this->best_distance_remaining_ = message->feedback.distance_remaining;
    this->best_progress_time_ = this->last_navigate_feedback_time_;
    this->has_progress_baseline_ = true;
  }
}

/// @copydoc RuntimeObservation::handle_navigate_status
void RuntimeObservation::handle_navigate_status(
  const action_msgs::msg::GoalStatusArray::SharedPtr message)
{
  this->latest_navigate_status_ = *message;
  this->has_navigate_status_ = true;
  this->last_navigate_status_time_ = this->now();
}

/// @copydoc RuntimeObservation::is_route_active
bool RuntimeObservation::is_route_active(const rclcpp::Time &now) const
{
  if (!this->has_navigate_feedback_) {
    return false;
  }

  const int8_t status = this->resolve_action_status();
  if (
    status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
    status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
    status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
  {
    return false;
  }

  const auto feedback_age = now - this->last_navigate_feedback_time_;
  if (feedback_age <= rclcpp::Duration::from_seconds(
      static_cast<double>(this->route_stale_timeout_ms_) / 1000.0))
  {
    return true;
  }

  return true;
}

/// @copydoc RuntimeObservation::is_progress_stalled
bool RuntimeObservation::is_progress_stalled(const rclcpp::Time &now) const
{
  if (!this->is_route_active(now) || !this->has_progress_baseline_ || !this->has_motion_status_) {
    return false;
  }
  if (!this->latest_motion_status_.active || this->latest_motion_status_.goal_reached) {
    return false;
  }
  if (this->is_goal_approach_context() || this->is_final_heading_alignment_context()) {
    return false;
  }
  if (this->last_dist_goal_delta_m_ >= std::max(0.0, this->progress_clear_delta_m_)) {
    return false;
  }
  if (this->is_controller_clear() && !this->has_explicit_non_controller_recovery()) {
    return false;
  }

  const auto time_since_progress = now - this->best_progress_time_;
  return time_since_progress.seconds() >= this->progress_stall_window_sec_;
}

/// @copydoc RuntimeObservation::is_controller_recovery
bool RuntimeObservation::is_controller_recovery() const
{
  return
    this->has_motion_status_ &&
    this->latest_motion_status_.active &&
    this->latest_motion_status_.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE;
}

/// @copydoc RuntimeObservation::is_controller_clear
bool RuntimeObservation::is_controller_clear() const
{
  return
    this->has_motion_status_ &&
    !this->latest_motion_status_.blocked &&
    !this->latest_motion_status_.stalled &&
    !this->is_controller_recovery();
}

/// @copydoc RuntimeObservation::is_local_plan_status_current
bool RuntimeObservation::is_local_plan_status_current() const
{
  return
    this->has_local_plan_status_ &&
    (!this->has_motion_status_ ||
    this->latest_local_plan_status_.command_id == this->latest_motion_status_.command_id);
}

/// @copydoc RuntimeObservation::has_explicit_non_controller_recovery
bool RuntimeObservation::has_explicit_non_controller_recovery() const
{
  return
    this->is_local_plan_status_current() &&
    this->latest_local_plan_status_.recovery_required;
}

/// @copydoc RuntimeObservation::is_goal_approach_context
bool RuntimeObservation::is_goal_approach_context() const
{
  return
    this->has_motion_status_ &&
    this->latest_motion_status_.active &&
    this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&
    !this->latest_motion_status_.goal_reached &&
    !this->latest_motion_status_.blocked &&
    !this->latest_motion_status_.stalled &&
    this->latest_motion_status_.remaining_distance <= std::max(0.0, this->goal_approach_distance_);
}

/// @copydoc RuntimeObservation::is_final_heading_alignment_context
bool RuntimeObservation::is_final_heading_alignment_context() const
{
  const bool command_matches_status =
    this->has_motion_command_ &&
    this->has_motion_status_ &&
    this->latest_motion_command_.command_id == this->latest_motion_status_.command_id;
  return
    command_matches_status &&
    this->latest_motion_command_.align_heading_at_goal &&
    this->is_goal_approach_context() &&
    this->latest_motion_status_.remaining_distance <=
      std::max(0.0, this->final_heading_alignment_distance_);
}

/// @copydoc RuntimeObservation::resolve_action_status
int8_t RuntimeObservation::resolve_action_status() const
{
  if (!this->has_navigate_status_ || !this->has_navigate_feedback_) {
    return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }

  for (const auto &item : this->latest_navigate_status_.status_list) {
    if (item.goal_info.goal_id.uuid == this->latest_navigate_feedback_.goal_id.uuid) {
      return item.status;
    }
  }

  return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
}

/// @copydoc RuntimeObservation::resolve_controller_phase
std::string RuntimeObservation::resolve_controller_phase() const
{
  if (!this->has_motion_status_)
  {
    return "idle";
  }

  if (this->latest_motion_status_.goal_reached)
  {
    return "reached";
  }

  if (!this->latest_motion_status_.active)
  {
    return "idle";
  }

  if (this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_BACKUP)
  {
    return "recovery_backup";
  }
  if (this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_SPIN)
  {
    return "recovery_spin";
  }
  if (this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_WAIT)
  {
    return "recovery_wait";
  }

  if (this->is_final_heading_alignment_context())
  {
    return "final_heading_align";
  }
  if (this->is_goal_approach_context())
  {
    return "goal_approach";
  }
  if (this->latest_motion_status_.blocked)
  {
    return "blocked";
  }
  if (this->latest_motion_status_.stalled)
  {
    return "stalled";
  }
  return "tracking";
}

/// @copydoc RuntimeObservation::resolve_progress_clear_reason
std::string RuntimeObservation::resolve_progress_clear_reason(
  const bool progress_stalled,
  const bool route_active) const
{
  if (progress_stalled)
  {
    return "none";
  }
  if (this->resolve_action_status() == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    return "route_succeeded";
  }
  if (!route_active)
  {
    return "route_inactive";
  }
  if (!this->has_motion_status_)
  {
    return "no_controller_status";
  }
  if (!this->latest_motion_status_.active)
  {
    return "controller_inactive";
  }
  if (this->latest_motion_status_.goal_reached)
  {
    return "controller_goal_reached";
  }
  if (this->is_final_heading_alignment_context())
  {
    return "final_heading_align";
  }
  if (this->is_goal_approach_context())
  {
    return "goal_approach";
  }
  if (
    this->is_controller_clear() &&
    this->last_dist_goal_delta_m_ >= std::max(0.0, this->progress_clear_delta_m_))
  {
    return "controller_normal_progress";
  }
  if (this->last_dist_goal_delta_m_ >= std::max(0.0, this->progress_clear_delta_m_))
  {
    return "distance_progress";
  }
  if (this->is_controller_clear() && !this->has_explicit_non_controller_recovery())
  {
    return "controller_clear";
  }
  return "not_stalled";
}

/// @copydoc RuntimeObservation::resolve_blocked_context
std::string RuntimeObservation::resolve_blocked_context(bool progress_stalled) const
{
  const std::string controller_phase = this->resolve_controller_phase();
  const bool goal_hold_context =
    controller_phase == "goal_approach" ||
    controller_phase == "final_heading_align" ||
    controller_phase == "reached";
  if (goal_hold_context && this->is_controller_clear())
  {
    return "clear";
  }

  if (this->is_controller_clear() && !this->has_explicit_non_controller_recovery())
  {
    return "clear";
  }

  if (this->has_motion_status_ && this->latest_motion_status_.safety_gate_blocked)
  {
    return "motion_safety_gate_blocked";
  }

  if (this->has_motion_status_ && this->latest_motion_status_.costmap_blocked)
  {
    return "motion_costmap_blocked";
  }

  if (
    this->is_local_plan_status_current() &&
    this->latest_local_plan_status_.recovery_required &&
    !goal_hold_context)
  {
    switch (this->latest_local_plan_status_.decision)
    {
      case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
        return "planner_goal_proximity_blocked";
      case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
        return "planner_global_replan_required";
      case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
        return "planner_hard_blocked";
      default:
        return "planner_recovery_required";
    }
  }

  if (this->has_motion_status_ && this->latest_motion_status_.stalled)
  {
    return "motion_stalled";
  }

  if (progress_stalled)
  {
    return "progress_stalled";
  }

  return "clear";
}

/// @copydoc RuntimeObservation::resolve_recovery_reason
std::string RuntimeObservation::resolve_recovery_reason(bool progress_stalled) const
{
  const std::string blocked_context = this->resolve_blocked_context(progress_stalled);
  return blocked_context == "clear" ? "none" : blocked_context;
}

/// @copydoc RuntimeObservation::resolve_recovery_phase
std::string RuntimeObservation::resolve_recovery_phase(
  bool route_active,
  bool recovery_triggered) const
{
  if (!route_active)
  {
    return "idle";
  }

  if (
    this->local_escape_command_active_ &&
    this->has_motion_status_ &&
    this->latest_motion_status_.active &&
    this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&
    this->latest_motion_status_.command_id == this->local_escape_command_id_)
  {
    return "local_escape_executing";
  }

  if (
    this->has_motion_status_ &&
    this->latest_motion_status_.active &&
    this->latest_motion_status_.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE)
  {
    return "recovery_executing";
  }

  if (recovery_triggered)
  {
    return "recovery_requested";
  }

  return "navigating";
}

/// @copydoc RuntimeObservation::resolve_runtime_state
std::string RuntimeObservation::resolve_runtime_state(
  bool route_active,
  bool progress_stalled) const
{
  const int8_t action_status = this->resolve_action_status();

  if (action_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    return "route_succeeded";
  }
  if (action_status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    return "route_aborted";
  }
  if (action_status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
    return "route_canceled";
  }

  if (!route_active) {
    if (this->has_motion_status_ && this->latest_motion_status_.goal_reached) {
      return "goal_reached";
    }
    return "idle";
  }

  if (this->is_final_heading_alignment_context()) {
    return "final_heading_align";
  }
  if (this->is_goal_approach_context()) {
    return "goal_approach";
  }
  if (this->has_motion_status_ && this->latest_motion_status_.goal_reached) {
    return "goal_reached";
  }
  if (this->has_explicit_non_controller_recovery()) {
    return "recovery_required";
  }
  if (this->has_motion_status_ &&
    (this->latest_motion_status_.blocked || this->latest_motion_status_.stalled))
  {
    return "motion_blocked";
  }
  if (progress_stalled) {
    return "progress_stalled";
  }
  if (this->resolve_controller_phase() == "tracking") {
    return "tracking";
  }
  return "navigating";
}

/// @copydoc RuntimeObservation::make_snapshot
RuntimeObservation::Snapshot RuntimeObservation::make_snapshot(const rclcpp::Time &now) const
{
  Snapshot snapshot;
  snapshot.route_active = this->is_route_active(now);
  snapshot.action_status = this->resolve_action_status();
  const bool action_terminal =
    snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
    snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
    snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
  if (this->has_navigate_feedback_) {
    const auto feedback_age = now - this->last_navigate_feedback_time_;
    snapshot.route_feedback_stale = feedback_age > rclcpp::Duration::from_seconds(
      static_cast<double>(this->route_stale_timeout_ms_) / 1000.0);
  }
  snapshot.route_terminal_status_missing =
    snapshot.route_active && snapshot.route_feedback_stale && !action_terminal;
  if (!this->has_navigate_feedback_) {
    snapshot.route_active_state = "no_feedback";
  } else if (action_terminal) {
    snapshot.route_active_state = "terminal_status";
  } else if (snapshot.route_feedback_stale) {
    snapshot.route_active_state = "stale_nonterminal_status";
  } else {
    snapshot.route_active_state = "active_recent_feedback";
  }
  snapshot.controller_phase = this->resolve_controller_phase();
  snapshot.controller_recovery = this->is_controller_recovery();
  snapshot.dist_goal_delta_m = this->last_dist_goal_delta_m_;
  snapshot.progress_stalled = this->is_progress_stalled(now);
  snapshot.progress_clear_reason = this->resolve_progress_clear_reason(
    snapshot.progress_stalled,
    snapshot.route_active);

  if (this->has_motion_status_) {
    snapshot.motion_blocked = this->latest_motion_status_.blocked;
    snapshot.motion_stalled = this->latest_motion_status_.stalled;
    snapshot.motion_goal_reached = this->latest_motion_status_.goal_reached;
  }

  const bool suppress_local_recovery =
    snapshot.controller_phase == "goal_approach" ||
    snapshot.controller_phase == "final_heading_align" ||
    snapshot.controller_phase == "reached";
  if (this->is_local_plan_status_current()) {
    snapshot.local_recovery_required = this->latest_local_plan_status_.recovery_required;
    snapshot.planner_decision = this->latest_local_plan_status_.decision;
  }
  if (suppress_local_recovery)
  {
    snapshot.local_recovery_required = false;
  }

  if (this->has_navigate_feedback_) {
    snapshot.current_goal_index = this->latest_navigate_feedback_.feedback.current_goal_index;
    snapshot.goal_count = this->latest_navigate_feedback_.feedback.goal_count;
    snapshot.number_of_recoveries = this->latest_navigate_feedback_.feedback.number_of_recoveries;
  }

  if (snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    snapshot.route_active = false;
    snapshot.progress_stalled = false;
    snapshot.progress_clear_reason = "route_succeeded";
    snapshot.motion_blocked = false;
    snapshot.motion_stalled = false;
    snapshot.local_recovery_required = false;
    snapshot.controller_recovery = false;
  }
  snapshot.blocked_context = this->resolve_blocked_context(snapshot.progress_stalled);
  if (snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    snapshot.blocked_context = "clear";
  }
  snapshot.recovery_reason = this->resolve_recovery_reason(snapshot.progress_stalled);
  if (snapshot.action_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    snapshot.recovery_reason = "none";
  }
  snapshot.recovery_triggered = snapshot.recovery_reason != "none";
  snapshot.local_escape_active =
    this->local_escape_command_active_ &&
    this->has_motion_status_ &&
    this->latest_motion_status_.active &&
    this->latest_motion_status_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE &&
    this->latest_motion_status_.command_id == this->local_escape_command_id_;
  snapshot.recovery_phase = this->resolve_recovery_phase(
    snapshot.route_active,
    snapshot.recovery_triggered);
  snapshot.runtime_state = this->resolve_runtime_state(
    snapshot.route_active,
    snapshot.progress_stalled);
  return snapshot;
}

/// @copydoc RuntimeObservation::publish_event_if_needed
void RuntimeObservation::publish_event_if_needed(const Snapshot &snapshot, const rclcpp::Time &now)
{
  if (!this->has_previous_snapshot_) {
    this->publish_event("observation_started", "initial_snapshot", snapshot, now);
    this->previous_snapshot_ = snapshot;
    this->has_previous_snapshot_ = true;
    return;
  }

  if (!this->previous_snapshot_.route_active && snapshot.route_active) {
    this->publish_event("route_started", "route_became_active", snapshot, now);
  }
  if (this->previous_snapshot_.route_active && !snapshot.route_active) {
    this->publish_event("route_inactive", "route_became_inactive", snapshot, now);
  }
  if (snapshot.route_active_state != this->previous_snapshot_.route_active_state) {
    this->publish_event("route_active_state_changed", "route_active_state_changed", snapshot, now);
  }
  if (snapshot.current_goal_index != this->previous_snapshot_.current_goal_index) {
    this->publish_event("goal_advanced", "current_goal_index_changed", snapshot, now);
  }
  if (snapshot.number_of_recoveries != this->previous_snapshot_.number_of_recoveries) {
    this->publish_event("recovery_count_changed", "number_of_recoveries_changed", snapshot, now);
  }
  if (snapshot.blocked_context != this->previous_snapshot_.blocked_context)
  {
    this->publish_event("blocked_context_changed", "blocked_context_changed", snapshot, now);
  }
  if (snapshot.recovery_triggered != this->previous_snapshot_.recovery_triggered)
  {
    this->publish_event("recovery_trigger_changed", "recovery_trigger_changed", snapshot, now);
  }
  if (snapshot.recovery_reason != this->previous_snapshot_.recovery_reason)
  {
    this->publish_event("recovery_reason_changed", "recovery_reason_changed", snapshot, now);
  }
  if (snapshot.recovery_phase != this->previous_snapshot_.recovery_phase)
  {
    this->publish_event("recovery_phase_changed", "recovery_phase_changed", snapshot, now);
  }
  if (snapshot.local_escape_active != this->previous_snapshot_.local_escape_active)
  {
    this->publish_event("local_escape_state_changed", "local_escape_state_changed", snapshot, now);
  }
  if (snapshot.planner_decision != this->previous_snapshot_.planner_decision) {
    this->publish_event("planner_decision_changed", "local_plan_decision_changed", snapshot, now);
  }
  if (snapshot.motion_blocked != this->previous_snapshot_.motion_blocked) {
    this->publish_event("motion_blocked_changed", "motion_blocked_changed", snapshot, now);
  }
  if (snapshot.progress_stalled != this->previous_snapshot_.progress_stalled) {
    this->publish_event("progress_stall_changed", "progress_stall_changed", snapshot, now);
  }
  if (snapshot.action_status != this->previous_snapshot_.action_status) {
    this->publish_event("action_status_changed", "route_action_status_changed", snapshot, now);
  }
  if (snapshot.runtime_state != this->previous_snapshot_.runtime_state) {
    this->publish_event("runtime_state_changed", "runtime_state_changed", snapshot, now);
  }

  this->previous_snapshot_ = snapshot;
}

/// @copydoc RuntimeObservation::publish_event
void RuntimeObservation::publish_event(
  const std::string &event_type,
  const std::string &reason,
  const Snapshot &snapshot,
  const rclcpp::Time &now)
{
  std_msgs::msg::String message;
  message.data = this->build_event_json(event_type, reason, snapshot, now);
  this->observation_event_publisher_->publish(message);
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=runtime_observation event=runtime_event phase=%s state=%s reason=%s route_active=%s route_active_state=%s route_feedback_stale=%s route_terminal_status_missing=%s recovery=%s recovery_count=%d blocked=%s blocked_count=%d dist_goal_m=%.3f dist_goal_delta_m=%.3f heading_err_rad=%.3f controller_phase=%s controller_blocked=%s controller_stalled=%s controller_recovery=%s controller_goal_reached=%s progress_stall_window_sec=%.3f progress_clear_delta_m=%.3f progress_stalled=%s progress_clear_reason=%s recovery_reason=%s",
      event_type.c_str(),
      snapshot.runtime_state.c_str(),
      reason.c_str(),
      bool_label(snapshot.route_active),
      snapshot.route_active_state.c_str(),
      bool_label(snapshot.route_feedback_stale),
      bool_label(snapshot.route_terminal_status_missing),
      bool_label(snapshot.recovery_triggered),
      snapshot.number_of_recoveries,
      bool_label(snapshot.motion_blocked || snapshot.progress_stalled || snapshot.local_recovery_required),
      (snapshot.motion_blocked || snapshot.progress_stalled || snapshot.local_recovery_required) ? 1 : 0,
      this->has_motion_status_ ? this->latest_motion_status_.remaining_distance : 0.0,
      snapshot.dist_goal_delta_m,
      this->has_motion_status_ ? this->latest_motion_status_.heading_error : 0.0,
      snapshot.controller_phase.c_str(),
      bool_label(snapshot.motion_blocked),
      bool_label(snapshot.motion_stalled),
      bool_label(snapshot.controller_recovery),
      bool_label(snapshot.motion_goal_reached),
      this->progress_stall_window_sec_,
      this->progress_clear_delta_m_,
      bool_label(snapshot.progress_stalled),
      snapshot.progress_clear_reason.c_str(),
      snapshot.recovery_reason.c_str());
  }
}

/// @copydoc RuntimeObservation::publish_observation
void RuntimeObservation::publish_observation()
{
  const auto now = this->now();
  const auto snapshot = this->make_snapshot(now);

  std_msgs::msg::String summary;
  summary.data = this->build_summary_json(snapshot, now);
  this->observation_summary_publisher_->publish(summary);
  this->publish_event_if_needed(snapshot, now);
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      throttle_ms_from_sec(this->summary_log_throttle_sec_),
      "AMR_LOG schema=v1 component=runtime_observation event=runtime_summary phase=%s route_active=%s route_active_state=%s route_feedback_stale=%s route_terminal_status_missing=%s dist_goal_m=%.3f dist_goal_delta_m=%.3f heading_err_rad=%.3f controller_phase=%s controller_blocked=%s controller_stalled=%s controller_recovery=%s controller_goal_reached=%s progress_stall_window_sec=%.3f progress_clear_delta_m=%.3f recovery_count=%d blocked_count=%d progress_stalled=%s recovery=%s reason=%s progress_clear_reason=%s recovery_reason=%s",
      snapshot.runtime_state.c_str(),
      bool_label(snapshot.route_active),
      snapshot.route_active_state.c_str(),
      bool_label(snapshot.route_feedback_stale),
      bool_label(snapshot.route_terminal_status_missing),
      this->has_motion_status_ ? this->latest_motion_status_.remaining_distance : 0.0,
      snapshot.dist_goal_delta_m,
      this->has_motion_status_ ? this->latest_motion_status_.heading_error : 0.0,
      snapshot.controller_phase.c_str(),
      bool_label(snapshot.motion_blocked),
      bool_label(snapshot.motion_stalled),
      bool_label(snapshot.controller_recovery),
      bool_label(snapshot.motion_goal_reached),
      this->progress_stall_window_sec_,
      this->progress_clear_delta_m_,
      snapshot.number_of_recoveries,
      (snapshot.motion_blocked || snapshot.progress_stalled || snapshot.local_recovery_required) ? 1 : 0,
      bool_label(snapshot.progress_stalled),
      bool_label(snapshot.recovery_triggered),
      snapshot.recovery_reason.c_str(),
      snapshot.progress_clear_reason.c_str(),
      snapshot.recovery_reason.c_str());
  }
}

/// @copydoc RuntimeObservation::build_summary_json
std::string RuntimeObservation::build_summary_json(
  const Snapshot &snapshot,
  const rclcpp::Time &now) const
{
  std::ostringstream stream;
  stream << std::boolalpha;
  stream << "{";
  stream << "\"stamp\":{\"sec\":" << (now.nanoseconds() / 1000000000LL) << ",\"nanosec\":" <<
    (now.nanoseconds() % 1000000000LL) << "},";
  stream << "\"runtime_state\":\"" << escape_json(snapshot.runtime_state) << "\",";
  stream << "\"route_active\":" << snapshot.route_active << ",";
  stream << "\"route_active_state\":\"" << escape_json(snapshot.route_active_state) << "\",";
  stream << "\"route_feedback_stale\":" << snapshot.route_feedback_stale << ",";
  stream << "\"route_terminal_status_missing\":" << snapshot.route_terminal_status_missing << ",";
  stream << "\"progress_stalled\":" << snapshot.progress_stalled << ",";
  stream << "\"progress_stall_window_sec\":" << this->progress_stall_window_sec_ << ",";
  stream << "\"progress_clear_delta_m\":" << this->progress_clear_delta_m_ << ",";
  stream << "\"progress_clear_reason\":\"" << escape_json(snapshot.progress_clear_reason) << "\",";
  stream << "\"dist_goal_delta_m\":" << snapshot.dist_goal_delta_m << ",";
  stream << "\"controller_phase\":\"" << escape_json(snapshot.controller_phase) << "\",";
  stream << "\"action_status\":" << static_cast<int>(snapshot.action_status) << ",";
  stream << "\"action_status_label\":\"" << escape_json(this->action_status_label(snapshot.action_status)) << "\",";
  stream << "\"planner_decision\":" << static_cast<int>(snapshot.planner_decision) << ",";
  stream << "\"planner_decision_label\":\"" <<
    escape_json(this->planner_decision_label(snapshot.planner_decision)) << "\",";
  stream << "\"current_goal_index\":" << snapshot.current_goal_index << ",";
  stream << "\"goal_count\":" << snapshot.goal_count << ",";
  stream << "\"number_of_recoveries\":" << snapshot.number_of_recoveries << ",";
  stream << "\"blocked_context\":\"" << escape_json(snapshot.blocked_context) << "\",";
  stream << "\"recovery_triggered\":" << snapshot.recovery_triggered << ",";
  stream << "\"recovery_reason\":\"" << escape_json(snapshot.recovery_reason) << "\",";
  stream << "\"local_escape_active\":" << snapshot.local_escape_active << ",";
  stream << "\"recovery_phase\":\"" << escape_json(snapshot.recovery_phase) << "\",";
  stream << "\"motion\":{";
  stream << "\"has_status\":" << this->has_motion_status_ << ",";
  stream << "\"active\":" << (this->has_motion_status_ ? this->latest_motion_status_.active : false) << ",";
  stream << "\"goal_reached\":" << snapshot.motion_goal_reached << ",";
  stream << "\"controller_recovery\":" << snapshot.controller_recovery << ",";
  stream << "\"controller_phase\":\"" << escape_json(snapshot.controller_phase) << "\",";
  stream << "\"blocked\":" << snapshot.motion_blocked << ",";
  stream << "\"stalled\":" << snapshot.motion_stalled << ",";
  stream << "\"local_plan_valid\":" <<
    (this->has_motion_status_ ? this->latest_motion_status_.local_plan_valid : false) << ",";
  stream << "\"costmap_blocked\":" <<
    (this->has_motion_status_ ? this->latest_motion_status_.costmap_blocked : false) << ",";
  stream << "\"safety_gate_blocked\":" <<
    (this->has_motion_status_ ? this->latest_motion_status_.safety_gate_blocked : false) << ",";
  stream << "\"remaining_distance\":" <<
    (this->has_motion_status_ ? this->latest_motion_status_.remaining_distance : 0.0) << ",";
  stream << "\"heading_error\":" <<
    (this->has_motion_status_ ? this->latest_motion_status_.heading_error : 0.0);
  stream << "},";
  stream << "\"local_plan\":{";
  stream << "\"has_status\":" << this->has_local_plan_status_ << ",";
  stream << "\"active\":" << (this->has_local_plan_status_ ? this->latest_local_plan_status_.active : false) << ",";
  stream << "\"local_plan_valid\":" <<
    (this->has_local_plan_status_ ? this->latest_local_plan_status_.local_plan_valid : false) << ",";
  stream << "\"recovery_required\":" << snapshot.local_recovery_required << ",";
  stream << "\"blocked_distance\":" <<
    (this->has_local_plan_status_ ? this->latest_local_plan_status_.blocked_distance : 0.0);
  stream << "}";
  if (this->has_navigate_feedback_) {
    stream << ",\"route_feedback\":{";
    stream << "\"distance_remaining\":" << this->latest_navigate_feedback_.feedback.distance_remaining << ",";
    stream << "\"goal_count\":" << this->latest_navigate_feedback_.feedback.goal_count << ",";
    stream << "\"current_goal_index\":" << this->latest_navigate_feedback_.feedback.current_goal_index << ",";
    stream << "\"number_of_recoveries\":" << this->latest_navigate_feedback_.feedback.number_of_recoveries;
    stream << "}";
  }
  stream << "}";
  return stream.str();
}

/// @copydoc RuntimeObservation::build_event_json
std::string RuntimeObservation::build_event_json(
  const std::string &event_type,
  const std::string &reason,
  const Snapshot &snapshot,
  const rclcpp::Time &now) const
{
  std::ostringstream stream;
  stream << std::boolalpha;
  stream << "{";
  stream << "\"event_type\":\"" << escape_json(event_type) << "\",";
  stream << "\"reason\":\"" << escape_json(reason) << "\",";
  stream << "\"runtime_state\":\"" << escape_json(snapshot.runtime_state) << "\",";
  stream << "\"route_active\":" << snapshot.route_active << ",";
  stream << "\"route_active_state\":\"" << escape_json(snapshot.route_active_state) << "\",";
  stream << "\"route_feedback_stale\":" << snapshot.route_feedback_stale << ",";
  stream << "\"route_terminal_status_missing\":" << snapshot.route_terminal_status_missing << ",";
  stream << "\"current_goal_index\":" << snapshot.current_goal_index << ",";
  stream << "\"goal_count\":" << snapshot.goal_count << ",";
  stream << "\"number_of_recoveries\":" << snapshot.number_of_recoveries << ",";
  stream << "\"blocked_context\":\"" << escape_json(snapshot.blocked_context) << "\",";
  stream << "\"recovery_triggered\":" << snapshot.recovery_triggered << ",";
  stream << "\"recovery_reason\":\"" << escape_json(snapshot.recovery_reason) << "\",";
  stream << "\"local_escape_active\":" << snapshot.local_escape_active << ",";
  stream << "\"recovery_phase\":\"" << escape_json(snapshot.recovery_phase) << "\",";
  stream << "\"action_status\":" << static_cast<int>(snapshot.action_status) << ",";
  stream << "\"action_status_label\":\"" << escape_json(this->action_status_label(snapshot.action_status)) << "\",";
  stream << "\"planner_decision\":" << static_cast<int>(snapshot.planner_decision) << ",";
  stream << "\"planner_decision_label\":\"" <<
    escape_json(this->planner_decision_label(snapshot.planner_decision)) << "\",";
  stream << "\"motion_blocked\":" << snapshot.motion_blocked << ",";
  stream << "\"motion_stalled\":" << snapshot.motion_stalled << ",";
  stream << "\"controller_recovery\":" << snapshot.controller_recovery << ",";
  stream << "\"controller_goal_reached\":" << snapshot.motion_goal_reached << ",";
  stream << "\"controller_phase\":\"" << escape_json(snapshot.controller_phase) << "\",";
  stream << "\"dist_goal_delta_m\":" << snapshot.dist_goal_delta_m << ",";
  stream << "\"progress_stall_window_sec\":" << this->progress_stall_window_sec_ << ",";
  stream << "\"progress_clear_delta_m\":" << this->progress_clear_delta_m_ << ",";
  stream << "\"progress_clear_reason\":\"" << escape_json(snapshot.progress_clear_reason) << "\",";
  stream << "\"progress_stalled\":" << snapshot.progress_stalled << ",";
  stream << "\"stamp_ns\":" << now.nanoseconds();
  stream << "}";
  return stream.str();
}

/// @copydoc RuntimeObservation::planner_decision_label
std::string RuntimeObservation::planner_decision_label(uint8_t decision) const
{
  switch (decision) {
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
      return "ok";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
      return "goal_proximity_blocked";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
      return "global_replan_required";
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return "hard_blocked";
    default:
      return "unknown";
  }
}

/// @copydoc RuntimeObservation::action_status_label
std::string RuntimeObservation::action_status_label(int8_t status) const
{
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      return "unknown";
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      return "accepted";
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      return "executing";
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      return "canceling";
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return "succeeded";
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return "canceled";
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return "aborted";
    default:
      return "unmapped";
  }
}

/// @copydoc RuntimeObservation::escape_json
std::string RuntimeObservation::escape_json(const std::string &value)
{
  std::string escaped;
  escaped.reserve(value.size());

  for (const char ch : value) {
    if (ch == '\\') {
      escaped += "\\\\";
    } else if (ch == '"') {
      escaped += "\\\"";
    } else {
      escaped.push_back(ch);
    }
  }

  return escaped;
}

}  // namespace amr::runtime::observation
