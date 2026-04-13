#ifndef AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_
#define AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_

#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr::runtime::observation
{

class RuntimeObservation : public rclcpp::Node
{
public:
  explicit RuntimeObservation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RuntimeObservation() override = default;

private:
  using NavigateToPosesFeedbackMessage =
    amr_msgs::action::NavigateToPoses::Impl::FeedbackMessage;

  struct Snapshot
  {
    bool route_active{false};
    bool progress_stalled{false};
    bool motion_blocked{false};
    bool motion_stalled{false};
    bool local_recovery_required{false};
    bool motion_goal_reached{false};
    uint32_t current_goal_index{0U};
    uint32_t goal_count{0U};
    int16_t number_of_recoveries{0};
    uint8_t planner_decision{amr_msgs::msg::LocalPlanStatus::DECISION_OK};
    int8_t action_status{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};
    std::string runtime_state{"idle"};
  };

  void handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message);
  void handle_local_plan_status(const amr_msgs::msg::LocalPlanStatus::SharedPtr message);
  void handle_navigate_feedback(const NavigateToPosesFeedbackMessage::SharedPtr message);
  void handle_navigate_status(const action_msgs::msg::GoalStatusArray::SharedPtr message);
  void publish_observation();

  bool is_route_active(const rclcpp::Time & now) const;
  bool is_progress_stalled(const rclcpp::Time & now) const;
  int8_t resolve_action_status() const;
  std::string resolve_runtime_state(bool route_active, bool progress_stalled) const;
  Snapshot make_snapshot(const rclcpp::Time & now) const;
  void publish_event_if_needed(const Snapshot & snapshot, const rclcpp::Time & now);
  void publish_event(
    const std::string & event_type,
    const std::string & reason,
    const Snapshot & snapshot,
    const rclcpp::Time & now);
  std::string build_summary_json(const Snapshot & snapshot, const rclcpp::Time & now) const;
  std::string build_event_json(
    const std::string & event_type,
    const std::string & reason,
    const Snapshot & snapshot,
    const rclcpp::Time & now) const;
  std::string planner_decision_label(uint8_t decision) const;
  std::string action_status_label(int8_t status) const;
  static std::string escape_json(const std::string & value);

  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_subscription_;
  rclcpp::Subscription<NavigateToPosesFeedbackMessage>::SharedPtr navigate_feedback_subscription_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_status_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr observation_summary_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr observation_event_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string motion_status_topic_;
  std::string local_plan_status_topic_;
  std::string navigate_to_poses_action_name_;
  std::string observation_summary_topic_;
  std::string observation_event_topic_;
  int publish_period_ms_;
  int route_stale_timeout_ms_;
  double progress_stall_window_sec_;
  double progress_epsilon_;

  amr_msgs::msg::MotionStatus latest_motion_status_;
  amr_msgs::msg::LocalPlanStatus latest_local_plan_status_;
  NavigateToPosesFeedbackMessage latest_navigate_feedback_;
  action_msgs::msg::GoalStatusArray latest_navigate_status_;
  bool has_motion_status_{false};
  bool has_local_plan_status_{false};
  bool has_navigate_feedback_{false};
  bool has_navigate_status_{false};
  rclcpp::Time last_motion_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_plan_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigate_feedback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigate_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time best_progress_time_{0, 0, RCL_ROS_TIME};
  float best_distance_remaining_{0.0f};
  bool has_progress_baseline_{false};
  Snapshot previous_snapshot_;
  bool has_previous_snapshot_{false};
};

}  // namespace amr::runtime::observation

#endif  // AMR_RUNTIME_OBSERVATION__RUNTIME_OBSERVATION_HPP_
