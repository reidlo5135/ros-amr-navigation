#ifndef AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/obstacle_report.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/plan_local_escape.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
  void handle_obstacle_report(const amr_msgs::msg::ObstacleReport::SharedPtr message);
  geometry_msgs::msg::PoseStamped get_current_pose_copy() const;
  amr_msgs::msg::MotionStatus get_motion_status_copy() const;
  amr_msgs::msg::ObstacleReport get_obstacle_report_copy() const;
  bool is_navigator_ready(std::string & error_message) const;
  bool wait_for_planner_service(std::string & error_message);
  bool wait_for_local_escape_service(std::string & error_message);
  bool request_global_plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan,
    std::string & error_message);
  bool request_local_escape_plan(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & source_plan,
    nav_msgs::msg::Path & plan,
    std::string & error_message);
  amr_msgs::msg::MotionCommand build_motion_command(
    const NavigateToPose::Goal & goal,
    const nav_msgs::msg::Path & plan);
  void publish_motion_command(const amr_msgs::msg::MotionCommand & command);
  void publish_stop_command();

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Client<amr_msgs::srv::PlanLocalEscape>::SharedPtr local_escape_client_;
  rclcpp::Client<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<amr_msgs::msg::ObstacleReport>::SharedPtr obstacle_report_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_publisher_;
  std::string navigate_action_name_;
  std::string command_topic_;
  std::string current_pose_topic_;
  std::string motion_status_topic_;
  std::string obstacle_report_topic_;
  std::string local_escape_service_;
  std::string plan_segment_service_;
  std::string behavior_tree_xml_path_;
  std::string default_node_id_;
  int planner_wait_timeout_ms_;
  int feedback_period_ms_;
  int recovery_max_retries_;
  int recovery_retry_delay_ms_;
  uint32_t next_command_id_;
  geometry_msgs::msg::PoseStamped current_pose_;
  amr_msgs::msg::MotionStatus latest_motion_status_;
  amr_msgs::msg::ObstacleReport latest_obstacle_report_;
  bool has_current_pose_;
  bool has_motion_status_;
  bool has_obstacle_report_;
  mutable std::mutex navigator_mutex_;
};

}  // namespace amr_bt_navigator

#endif  // AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
