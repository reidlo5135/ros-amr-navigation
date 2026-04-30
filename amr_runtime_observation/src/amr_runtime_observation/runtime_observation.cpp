#include "amr_runtime_observation/runtime_observation.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace amr::runtime::observation
{

RuntimeObservation::RuntimeObservation(const rclcpp::NodeOptions & options)
: rclcpp::Node("runtime_observation", options)
{
  this->declare_parameter("topics.motion_status", "/amr/motion/status");
  this->declare_parameter("topics.local_plan_status", "/amr/planner/local_status");
  this->declare_parameter("actions.navigate_to_poses", "/amr/navigator/navigate_to_poses");
  this->declare_parameter("topics.observation_summary", "/amr/observation/runtime/summary");
  this->declare_parameter("topics.observation_events", "/amr/observation/runtime/events");
  this->declare_parameter("observation.publish_period_ms", 200);
  this->declare_parameter("observation.route_stale_timeout_ms", 1500);
  this->declare_parameter("observation.progress_stall_window_sec", 3.0);
  this->declare_parameter("observation.progress_epsilon", 0.05);

  this->get_parameter("topics.motion_status", this->motion_status_topic_);
  this->get_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->get_parameter("actions.navigate_to_poses", this->navigate_to_poses_action_name_);
  this->get_parameter("topics.observation_summary", this->observation_summary_topic_);
  this->get_parameter("topics.observation_events", this->observation_event_topic_);
  this->get_parameter("observation.publish_period_ms", this->publish_period_ms_);
  this->get_parameter("observation.route_stale_timeout_ms", this->route_stale_timeout_ms_);
  this->get_parameter("observation.progress_stall_window_sec", this->progress_stall_window_sec_);
  this->get_parameter("observation.progress_epsilon", this->progress_epsilon_);

  const std::string feedback_topic = this->navigate_to_poses_action_name_ + "/_action/feedback";
  const std::string status_topic = this->navigate_to_poses_action_name_ + "/_action/status";

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

  RCLCPP_INFO(
    this->get_logger(),
    "Configured runtime observation with motion_status='%s', local_plan_status='%s', "
    "navigate_feedback='%s', navigate_status='%s', summary='%s', events='%s'",
    this->motion_status_topic_.c_str(),
    this->local_plan_status_topic_.c_str(),
    feedback_topic.c_str(),
    status_topic.c_str(),
    this->observation_summary_topic_.c_str(),
    this->observation_event_topic_.c_str());
}

void RuntimeObservation::handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message)
{
  this->latest_motion_status_ = *message;
  this->has_motion_status_ = true;
  this->last_motion_status_time_ = this->now();
}

void RuntimeObservation::handle_local_plan_status(
  const amr_msgs::msg::LocalPlanStatus::SharedPtr message)
{
  this->latest_local_plan_status_ = *message;
  this->has_local_plan_status_ = true;
  this->last_local_plan_status_time_ = this->now();
}

void RuntimeObservation::handle_navigate_feedback(
  const NavigateToPosesFeedbackMessage::SharedPtr message)
{
  this->latest_navigate_feedback_ = *message;
  this->has_navigate_feedback_ = true;
  this->last_navigate_feedback_time_ = this->now();

  if (!this->has_progress_baseline_ ||
    message->feedback.distance_remaining < (this->best_distance_remaining_ - this->progress_epsilon_))
  {
    this->best_distance_remaining_ = message->feedback.distance_remaining;
    this->best_progress_time_ = this->last_navigate_feedback_time_;
    this->has_progress_baseline_ = true;
  }
}

void RuntimeObservation::handle_navigate_status(
  const action_msgs::msg::GoalStatusArray::SharedPtr message)
{
  this->latest_navigate_status_ = *message;
  this->has_navigate_status_ = true;
  this->last_navigate_status_time_ = this->now();
}

bool RuntimeObservation::is_route_active(const rclcpp::Time & now) const
{
  if (!this->has_navigate_feedback_) {
    return false;
  }

  const auto feedback_age = now - this->last_navigate_feedback_time_;
  if (feedback_age <= rclcpp::Duration::from_seconds(
      static_cast<double>(this->route_stale_timeout_ms_) / 1000.0))
  {
    return true;
  }

  const int8_t status = this->resolve_action_status();
  return
    status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
    status != action_msgs::msg::GoalStatus::STATUS_CANCELED &&
    status != action_msgs::msg::GoalStatus::STATUS_ABORTED;
}

bool RuntimeObservation::is_progress_stalled(const rclcpp::Time & now) const
{
  if (!this->is_route_active(now) || !this->has_progress_baseline_ || !this->has_motion_status_) {
    return false;
  }
  if (!this->latest_motion_status_.active || this->latest_motion_status_.goal_reached) {
    return false;
  }

  const auto time_since_progress = now - this->best_progress_time_;
  return time_since_progress.seconds() >= this->progress_stall_window_sec_;
}

int8_t RuntimeObservation::resolve_action_status() const
{
  if (!this->has_navigate_status_ || !this->has_navigate_feedback_) {
    return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }

  for (const auto & item : this->latest_navigate_status_.status_list) {
    if (item.goal_info.goal_id.uuid == this->latest_navigate_feedback_.goal_id.uuid) {
      return item.status;
    }
  }

  return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
}

std::string RuntimeObservation::resolve_recovery_reason(bool progress_stalled) const
{
  if (this->has_local_plan_status_ && this->latest_local_plan_status_.recovery_required)
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

  if (this->has_motion_status_ && this->latest_motion_status_.blocked)
  {
    if (this->latest_motion_status_.safety_gate_blocked)
    {
      return "motion_safety_gate_blocked";
    }
    if (this->latest_motion_status_.costmap_blocked)
    {
      return "motion_costmap_blocked";
    }
    return "motion_blocked";
  }

  if (this->has_motion_status_ && this->latest_motion_status_.stalled)
  {
    return "motion_stalled";
  }

  if (progress_stalled)
  {
    return "progress_stalled";
  }

  return "none";
}

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

  if (this->has_local_plan_status_ && this->latest_local_plan_status_.recovery_required) {
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
  return "navigating";
}

RuntimeObservation::Snapshot RuntimeObservation::make_snapshot(const rclcpp::Time & now) const
{
  Snapshot snapshot;
  snapshot.route_active = this->is_route_active(now);
  snapshot.progress_stalled = this->is_progress_stalled(now);

  if (this->has_motion_status_) {
    snapshot.motion_blocked = this->latest_motion_status_.blocked;
    snapshot.motion_stalled = this->latest_motion_status_.stalled;
    snapshot.motion_goal_reached = this->latest_motion_status_.goal_reached;
  }

  if (this->has_local_plan_status_) {
    snapshot.local_recovery_required = this->latest_local_plan_status_.recovery_required;
    snapshot.planner_decision = this->latest_local_plan_status_.decision;
  }

  if (this->has_navigate_feedback_) {
    snapshot.current_goal_index = this->latest_navigate_feedback_.feedback.current_goal_index;
    snapshot.goal_count = this->latest_navigate_feedback_.feedback.goal_count;
    snapshot.number_of_recoveries = this->latest_navigate_feedback_.feedback.number_of_recoveries;
  }

  snapshot.action_status = this->resolve_action_status();
  snapshot.recovery_reason = this->resolve_recovery_reason(snapshot.progress_stalled);
  snapshot.recovery_triggered = snapshot.recovery_reason != "none";
  snapshot.runtime_state = this->resolve_runtime_state(
    snapshot.route_active,
    snapshot.progress_stalled);
  return snapshot;
}

void RuntimeObservation::publish_event_if_needed(const Snapshot & snapshot, const rclcpp::Time & now)
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
  if (snapshot.current_goal_index != this->previous_snapshot_.current_goal_index) {
    this->publish_event("goal_advanced", "current_goal_index_changed", snapshot, now);
  }
  if (snapshot.number_of_recoveries != this->previous_snapshot_.number_of_recoveries) {
    this->publish_event("recovery_count_changed", "number_of_recoveries_changed", snapshot, now);
  }
  if (snapshot.recovery_triggered != this->previous_snapshot_.recovery_triggered)
  {
    this->publish_event("recovery_trigger_changed", "recovery_trigger_changed", snapshot, now);
  }
  if (snapshot.recovery_reason != this->previous_snapshot_.recovery_reason)
  {
    this->publish_event("recovery_reason_changed", "recovery_reason_changed", snapshot, now);
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

void RuntimeObservation::publish_event(
  const std::string & event_type,
  const std::string & reason,
  const Snapshot & snapshot,
  const rclcpp::Time & now)
{
  std_msgs::msg::String message;
  message.data = this->build_event_json(event_type, reason, snapshot, now);
  this->observation_event_publisher_->publish(message);
}

void RuntimeObservation::publish_observation()
{
  const auto now = this->now();
  const auto snapshot = this->make_snapshot(now);

  std_msgs::msg::String summary;
  summary.data = this->build_summary_json(snapshot, now);
  this->observation_summary_publisher_->publish(summary);
  this->publish_event_if_needed(snapshot, now);
}

std::string RuntimeObservation::build_summary_json(
  const Snapshot & snapshot,
  const rclcpp::Time & now) const
{
  std::ostringstream stream;
  stream << std::boolalpha;
  stream << "{";
  stream << "\"stamp\":{\"sec\":" << (now.nanoseconds() / 1000000000LL) << ",\"nanosec\":" <<
    (now.nanoseconds() % 1000000000LL) << "},";
  stream << "\"runtime_state\":\"" << escape_json(snapshot.runtime_state) << "\",";
  stream << "\"route_active\":" << snapshot.route_active << ",";
  stream << "\"progress_stalled\":" << snapshot.progress_stalled << ",";
  stream << "\"action_status\":" << static_cast<int>(snapshot.action_status) << ",";
  stream << "\"action_status_label\":\"" << escape_json(this->action_status_label(snapshot.action_status)) << "\",";
  stream << "\"planner_decision\":" << static_cast<int>(snapshot.planner_decision) << ",";
  stream << "\"planner_decision_label\":\"" <<
    escape_json(this->planner_decision_label(snapshot.planner_decision)) << "\",";
  stream << "\"current_goal_index\":" << snapshot.current_goal_index << ",";
  stream << "\"goal_count\":" << snapshot.goal_count << ",";
  stream << "\"number_of_recoveries\":" << snapshot.number_of_recoveries << ",";
  stream << "\"recovery_triggered\":" << snapshot.recovery_triggered << ",";
  stream << "\"recovery_reason\":\"" << escape_json(snapshot.recovery_reason) << "\",";
  stream << "\"motion\":{";
  stream << "\"has_status\":" << this->has_motion_status_ << ",";
  stream << "\"active\":" << (this->has_motion_status_ ? this->latest_motion_status_.active : false) << ",";
  stream << "\"goal_reached\":" << snapshot.motion_goal_reached << ",";
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

std::string RuntimeObservation::build_event_json(
  const std::string & event_type,
  const std::string & reason,
  const Snapshot & snapshot,
  const rclcpp::Time & now) const
{
  std::ostringstream stream;
  stream << std::boolalpha;
  stream << "{";
  stream << "\"event_type\":\"" << escape_json(event_type) << "\",";
  stream << "\"reason\":\"" << escape_json(reason) << "\",";
  stream << "\"runtime_state\":\"" << escape_json(snapshot.runtime_state) << "\",";
  stream << "\"route_active\":" << snapshot.route_active << ",";
  stream << "\"current_goal_index\":" << snapshot.current_goal_index << ",";
  stream << "\"goal_count\":" << snapshot.goal_count << ",";
  stream << "\"number_of_recoveries\":" << snapshot.number_of_recoveries << ",";
  stream << "\"recovery_triggered\":" << snapshot.recovery_triggered << ",";
  stream << "\"recovery_reason\":\"" << escape_json(snapshot.recovery_reason) << "\",";
  stream << "\"action_status\":" << static_cast<int>(snapshot.action_status) << ",";
  stream << "\"action_status_label\":\"" << escape_json(this->action_status_label(snapshot.action_status)) << "\",";
  stream << "\"planner_decision\":" << static_cast<int>(snapshot.planner_decision) << ",";
  stream << "\"planner_decision_label\":\"" <<
    escape_json(this->planner_decision_label(snapshot.planner_decision)) << "\",";
  stream << "\"motion_blocked\":" << snapshot.motion_blocked << ",";
  stream << "\"motion_stalled\":" << snapshot.motion_stalled << ",";
  stream << "\"progress_stalled\":" << snapshot.progress_stalled << ",";
  stream << "\"stamp_ns\":" << now.nanoseconds();
  stream << "}";
  return stream.str();
}

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

std::string RuntimeObservation::escape_json(const std::string & value)
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
