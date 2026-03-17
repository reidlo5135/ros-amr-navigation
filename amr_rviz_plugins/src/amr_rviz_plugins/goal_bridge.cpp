#include "amr_rviz_plugins/goal_bridge.hpp"

#include <chrono>
#include <stdexcept>

namespace amr_rviz_plugins
{

GoalBridge::GoalBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("rviz_bridge", options),
  goal_pose_topic_(""),
  navigate_to_pose_action_(""),
  action_wait_timeout_ms_(1000)
{
  this->declare_parameter("topics.goal_pose", this->goal_pose_topic_);
  this->declare_parameter("actions.navigate_to_pose", this->navigate_to_pose_action_);
  this->declare_parameter("execution.action_wait_timeout_ms", this->action_wait_timeout_ms_);

  this->get_parameter("topics.goal_pose", this->goal_pose_topic_);
  this->get_parameter("actions.navigate_to_pose", this->navigate_to_pose_action_);
  this->get_parameter("execution.action_wait_timeout_ms", this->action_wait_timeout_ms_);

  if (this->goal_pose_topic_.empty() || this->navigate_to_pose_action_.empty()) {
    RCLCPP_FATAL(
      this->get_logger(),
      "RViz goal bridge parameters must not be empty: goal_pose='%s' action='%s'",
      this->goal_pose_topic_.c_str(),
      this->navigate_to_pose_action_.c_str());
    throw std::runtime_error("RViz goal bridge parameters are empty");
  }

  this->navigate_to_pose_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, this->navigate_to_pose_action_);
  this->goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->goal_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_goal_pose(message);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured RViz goal bridge with goal_pose='%s' action='%s'",
    this->goal_pose_topic_.c_str(),
    this->navigate_to_pose_action_.c_str());
}

void GoalBridge::handle_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Ignoring RViz goal pose with empty frame_id");
    return;
  }

  if (!this->navigate_to_pose_client_->wait_for_action_server(
      std::chrono::milliseconds(this->action_wait_timeout_ms_)))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "NavigateToPose action server '%s' is not available",
      this->navigate_to_pose_action_.c_str());
    return;
  }

  NavigateToPose::Goal goal;
  goal.goal_pose = *message;

  RCLCPP_INFO(
    this->get_logger(),
    "Forwarding RViz 2D goal pose to action: frame='%s' x=%.3f y=%.3f",
    goal.goal_pose.header.frame_id.c_str(),
    goal.goal_pose.pose.position.x,
    goal.goal_pose.pose.position.y);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "RViz goal was rejected by navigator");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "RViz goal accepted by navigator");
    };
  options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "RViz goal succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "RViz goal aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "RViz goal canceled");
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "RViz goal finished with unknown result code");
          break;
      }
    };

  this->navigate_to_pose_client_->async_send_goal(goal, options);
}

}  // namespace amr_rviz_plugins
