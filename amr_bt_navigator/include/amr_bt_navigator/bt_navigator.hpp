#ifndef AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/msg/local_plan_status.hpp"
#include "amr_msgs/msg/localization_status.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/clear_costmap.hpp"
#include "amr_msgs/srv/plan_recovery.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include "amr_msgs/srv/trigger_global_localization.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_bt_navigator
{

class Btnavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Btnavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message);
  void handle_local_plan_status(const amr_msgs::msg::LocalPlanStatus::SharedPtr message);
  void handle_localization_status(const amr_msgs::msg::LocalizationStatus::SharedPtr message);
  geometry_msgs::msg::PoseStamped get_current_pose_copy() const;
  amr_msgs::msg::MotionStatus get_motion_status_copy() const;
  amr_msgs::msg::LocalPlanStatus get_local_plan_status_copy() const;
  amr_msgs::msg::LocalizationStatus get_localization_status_copy() const;
  bool is_navigator_ready(std::string & error_message) const;
  bool wait_for_planner_service(std::string & error_message);
  bool wait_for_recovery_services(std::string & error_message);
  bool request_global_plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan,
    std::string & error_message);
  bool request_recovery_command(
    const std::string & behavior,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    amr_msgs::msg::MotionCommand & command,
    std::string & error_message);
  bool trigger_global_localization(const std::string & reason, std::string & error_message);
  bool clear_local_costmap(std::string & error_message);
  bool wait_for_command_completion(
    uint32_t command_id,
    int timeout_ms,
    std::string & error_message);
  amr_msgs::msg::MotionCommand build_motion_command(
    const NavigateToPose::Goal & goal,
    const nav_msgs::msg::Path & plan);
  void publish_motion_command(const amr_msgs::msg::MotionCommand & command);
  void publish_stop_command();
  void run_active_relocalization_supervisor();
  void request_active_relocalization_behavior_async(
    const std::string & behavior,
    const geometry_msgs::msg::PoseStamped & current_pose,
    std::size_t behavior_index);
  void trigger_global_localization_async(const std::string & reason);
  bool has_active_goal() const;

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Client<amr_msgs::srv::PlanRecovery>::SharedPtr plan_recovery_client_;
  rclcpp::Client<amr_msgs::srv::ClearCostmap>::SharedPtr clear_costmap_client_;
  rclcpp::Client<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_client_;
  rclcpp::Client<amr_msgs::srv::TriggerGlobalLocalization>::SharedPtr trigger_global_localization_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::LocalPlanStatus>::SharedPtr local_plan_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::LocalizationStatus>::SharedPtr localization_status_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_publisher_;
  rclcpp::TimerBase::SharedPtr active_relocalization_timer_;
  std::string navigate_action_name_;
  std::string command_topic_;
  std::string current_pose_topic_;
  std::string localization_status_topic_;
  std::string motion_status_topic_;
  std::string local_plan_status_topic_;
  std::string plan_recovery_service_;
  std::string clear_costmap_service_;
  std::string plan_segment_service_;
  std::string trigger_global_localization_service_;
  std::string startup_localization_mode_;
  std::string behavior_tree_xml_path_;
  std::string default_node_id_;
  int planner_wait_timeout_ms_;
  int feedback_period_ms_;
  int recovery_max_retries_;
  int recovery_retry_delay_ms_;
  int arl_check_period_ms_;
  int arl_command_timeout_ms_;
  int arl_retrigger_interval_ms_;
  uint32_t next_command_id_;
  geometry_msgs::msg::PoseStamped current_pose_;
  amr_msgs::msg::MotionStatus latest_motion_status_;
  amr_msgs::msg::LocalPlanStatus latest_local_plan_status_;
  amr_msgs::msg::LocalizationStatus latest_localization_status_;
  bool has_current_pose_;
  bool has_motion_status_;
  bool has_local_plan_status_;
  bool has_localization_status_;
  mutable std::mutex navigator_mutex_;
  mutable std::mutex active_goal_mutex_;
  mutable std::mutex command_mutex_;
  std::mutex active_relocalization_mutex_;
  std::weak_ptr<GoalHandleNavigateToPose> active_goal_handle_;
  bool active_relocalization_command_active_;
  uint32_t active_relocalization_command_id_;
  std::size_t active_relocalization_phase_index_;
  int64_t active_relocalization_command_started_ns_;
  int64_t active_relocalization_last_trigger_ns_;
  bool active_relocalization_request_pending_;
  int64_t active_relocalization_request_started_ns_;
  std::string active_relocalization_pending_behavior_;
  bool active_relocalization_trigger_pending_;
  int64_t active_relocalization_trigger_started_ns_;
};

}  // namespace amr_bt_navigator

#endif  // AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
