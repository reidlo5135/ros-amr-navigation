#include "amr_bt_navigator/bt_navigator.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"

namespace amr_bt_navigator
{

Btnavigator::Btnavigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bt_navigator", options),
  action_name_("execute_route"),
  command_topic_("motion_command"),
  default_node_id_("start"),
  next_command_id_(1U)
{
  this->declare_parameter("execute_route_action", this->action_name_);
  this->declare_parameter("command_topic", this->command_topic_);
  this->declare_parameter("default_node_id", this->default_node_id_);
}

Btnavigator::CallbackReturn Btnavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("execute_route_action", this->action_name_);
  this->get_parameter("command_topic", this->command_topic_);
  this->get_parameter("default_node_id", this->default_node_id_);

  this->motion_command_publisher_ = this->create_publisher<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS());
  this->action_server_ = rclcpp_action::create_server<ExecuteRoute>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->action_name_,
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ExecuteRoute::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleExecuteRoute> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleExecuteRoute> goal_handle) {
      this->handle_accepted(goal_handle);
    });

  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_publisher_->on_activate();
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->action_server_.reset();
  this->motion_command_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->action_server_.reset();
  this->motion_command_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse Btnavigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteRoute::Goal> goal)
{
  (void)uuid;
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal while navigator is inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->route_id.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Btnavigator::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteRoute> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Btnavigator::handle_accepted(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void Btnavigator::execute(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle)
{
  if (!this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated()) {
    auto result = std::make_shared<ExecuteRoute::Result>();
    result->success = false;
    result->message = "Navigator is not active.";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();

  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id = "map";
  command.command_id = this->next_command_id_++;
  command.route_id = goal->route_id;
  command.node_id = this->default_node_id_;
  command.goal_pose.header = command.header;
  command.goal_pose.pose.orientation.w = 1.0;
  command.align_heading_at_goal = true;
  this->motion_command_publisher_->publish(command);

  auto feedback = std::make_shared<ExecuteRoute::Feedback>();
  feedback->route_id = goal->route_id;
  feedback->current_node_id = this->default_node_id_;
  feedback->current_pose = command.goal_pose;
  feedback->remaining_distance = 0.0;
  feedback->heading_error = 0.0;
  feedback->current_waypoint_index = 0U;
  feedback->waypoint_count = 1U;
  goal_handle->publish_feedback(feedback);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<ExecuteRoute::Result>();
    result->success = false;
    result->message = "Route execution canceled.";
    goal_handle->canceled(result);
    return;
  }

  auto result = std::make_shared<ExecuteRoute::Result>();
  result->success = true;
  result->message = "Stub BT navigator accepted the route.";
  goal_handle->succeed(result);
}

}  // namespace amr_bt_navigator
