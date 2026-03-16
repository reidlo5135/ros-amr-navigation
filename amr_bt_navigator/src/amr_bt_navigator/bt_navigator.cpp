#include "amr_bt_navigator/bt_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"

namespace amr_bt_navigator
{

Btnavigator::Btnavigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", options),
  navigate_action_name_("/amr/navigator/navigate_to_pose"),
  command_topic_("/amr/motion_controller/command"),
  current_pose_topic_("/amr/localization/pose"),
  motion_status_topic_("/amr/motion_controller/status"),
  plan_segment_service_("/amr/global_planner/plan_segment"),
  default_node_id_("start"),
  planner_wait_timeout_ms_(2000),
  feedback_period_ms_(100),
  next_command_id_(1U),
  has_current_pose_(false),
  has_motion_status_(false)
{
  this->declare_parameter("navigate_action_name", this->navigate_action_name_);
  this->declare_parameter("command_topic", this->command_topic_);
  this->declare_parameter("current_pose_topic", this->current_pose_topic_);
  this->declare_parameter("motion_status_topic", this->motion_status_topic_);
  this->declare_parameter("plan_segment_service", this->plan_segment_service_);
  this->declare_parameter("default_node_id", this->default_node_id_);
  this->declare_parameter("planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->declare_parameter("feedback_period_ms", this->feedback_period_ms_);
}

Btnavigator::CallbackReturn Btnavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("navigate_action_name", this->navigate_action_name_);
  this->get_parameter("command_topic", this->command_topic_);
  this->get_parameter("current_pose_topic", this->current_pose_topic_);
  this->get_parameter("motion_status_topic", this->motion_status_topic_);
  this->get_parameter("plan_segment_service", this->plan_segment_service_);
  this->get_parameter("default_node_id", this->default_node_id_);
  this->get_parameter("planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->get_parameter("feedback_period_ms", this->feedback_period_ms_);

  this->motion_command_publisher_ = this->create_publisher<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS());
  this->plan_segment_client_ = this->create_client<amr_msgs::srv::PlanSegment>(
    this->plan_segment_service_);
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->motion_status_subscription_ = this->create_subscription<amr_msgs::msg::MotionStatus>(
    this->motion_status_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionStatus::SharedPtr message) {
      this->handle_motion_status(message);
    });
  this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->navigate_action_name_,
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      this->handle_accepted(goal_handle);
    });

  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_activate();
  }
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
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->motion_command_publisher_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->action_server_.reset();
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->motion_command_publisher_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse Btnavigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal while navigator is inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->goal_pose.header.frame_id.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Btnavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Btnavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void Btnavigator::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  if (
    !this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated() ||
    !this->plan_segment_client_)
  {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = "Navigator is not active.";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  const auto current_pose = this->get_current_pose_copy();
  if (current_pose.header.frame_id.empty()) {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = "Current pose is not available yet.";
    goal_handle->abort(result);
    return;
  }

  if (!this->plan_segment_client_->wait_for_service(
      std::chrono::milliseconds(this->planner_wait_timeout_ms_)))
  {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = "Global planner service is not available.";
    goal_handle->abort(result);
    return;
  }

  auto request = std::make_shared<amr_msgs::srv::PlanSegment::Request>();
  request->start = current_pose;
  request->goal = goal->goal_pose;

  auto future = this->plan_segment_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(this->planner_wait_timeout_ms_)) !=
      std::future_status::ready)
  {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = "Timed out while waiting for a global plan.";
    goal_handle->abort(result);
    return;
  }

  const auto response = future.get();
  if (!response->success) {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = response->message;
    goal_handle->abort(result);
    return;
  }

  auto command = this->build_motion_command(*goal, response->plan);
  this->motion_command_publisher_->publish(command);
  const auto waypoint_count = std::max<std::size_t>(1U, response->plan.poses.size());

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      this->publish_stop_command();
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = false;
      result->message = "Route execution canceled.";
      goal_handle->canceled(result);
      return;
    }

    const auto status = this->get_motion_status_copy();
    const auto pose = this->get_current_pose_copy();

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->route_id = goal->route_id;
    feedback->current_node_id = this->default_node_id_;
    feedback->current_pose = pose;
    feedback->remaining_distance = status.remaining_distance;
    feedback->heading_error = status.heading_error;
    feedback->current_waypoint_index = status.goal_reached ? waypoint_count : 0U;
    feedback->waypoint_count = static_cast<uint32_t>(waypoint_count);
    goal_handle->publish_feedback(feedback);

    if (status.command_id == command.command_id && status.goal_reached) {
      this->publish_stop_command();
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = true;
      result->message = "Goal reached.";
      goal_handle->succeed(result);
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(this->feedback_period_ms_));
  }
}

void Btnavigator::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void Btnavigator::handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_motion_status_ = *message;
  this->has_motion_status_ = true;
}

geometry_msgs::msg::PoseStamped Btnavigator::get_current_pose_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->current_pose_;
}

amr_msgs::msg::MotionStatus Btnavigator::get_motion_status_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_motion_status_;
}

amr_msgs::msg::MotionCommand Btnavigator::build_motion_command(
  const NavigateToPose::Goal & goal,
  const nav_msgs::msg::Path & plan)
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    goal.goal_pose.header.frame_id.empty() ? std::string("map") : goal.goal_pose.header.frame_id;
  command.command_id = this->next_command_id_++;
  command.route_id = goal.route_id;
  command.node_id = this->default_node_id_;
  command.plan = plan;
  command.goal_pose = goal.goal_pose;
  command.align_heading_at_goal = true;
  return command;
}

void Btnavigator::publish_stop_command()
{
  if (!this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated()) {
    return;
  }

  const auto current_pose = this->get_current_pose_copy();
  amr_msgs::msg::MotionCommand stop_command;
  stop_command.header.stamp = this->now();
  stop_command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  stop_command.command_id = this->next_command_id_++;
  stop_command.route_id = "stop";
  stop_command.node_id = this->default_node_id_;
  stop_command.goal_pose = current_pose;
  stop_command.align_heading_at_goal = false;
  this->motion_command_publisher_->publish(stop_command);
}

}  // namespace amr_bt_navigator
