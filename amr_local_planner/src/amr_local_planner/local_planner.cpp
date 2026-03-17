#include "amr_local_planner/local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace amr_local_planner
{

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner", options),
  command_topic_(""),
  current_pose_topic_(""),
  local_plan_topic_(""),
  publish_period_ms_(100),
  lookahead_distance_(0.8),
  goal_tolerance_(0.15),
  last_command_id_(0U),
  last_progress_index_(0U),
  has_command_(false),
  has_current_pose_(false)
{
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.plan", this->local_plan_topic_);
  this->declare_parameter("planner.publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("planner.lookahead_distance", this->lookahead_distance_);
  this->declare_parameter("planner.goal_tolerance", this->goal_tolerance_);
}

LocalPlanner::CallbackReturn LocalPlanner::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.plan", this->local_plan_topic_);
  this->get_parameter("planner.publish_period_ms", this->publish_period_ms_);
  this->get_parameter("planner.lookahead_distance", this->lookahead_distance_);
  this->get_parameter("planner.goal_tolerance", this->goal_tolerance_);

  if (
    this->command_topic_.empty() || this->current_pose_topic_.empty() ||
    this->local_plan_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Local planner topics must not be empty: command='%s' pose='%s' local_plan='%s'",
      this->command_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->local_plan_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->local_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
    this->local_plan_topic_, rclcpp::SystemDefaultsQoS());
  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(this->publish_period_ms_),
    [this]() { this->publish_local_plan(); });
  this->timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(),
    "Configured local planner with command='%s', pose='%s', plan='%s', lookahead=%.2f, tolerance=%.2f",
    this->command_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->local_plan_topic_.c_str(),
    this->lookahead_distance_,
    this->goal_tolerance_);

  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->local_plan_publisher_->on_activate();
  this->timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->timer_) {
    this->timer_->cancel();
  }
  if (this->local_plan_publisher_) {
    this->local_plan_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated local planner");
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->local_plan_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->last_command_id_ = 0U;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->local_plan_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->last_command_id_ = 0U;
  this->last_progress_index_ = 0U;
  this->has_command_ = false;
  this->has_current_pose_ = false;
  return CallbackReturn::SUCCESS;
}

void LocalPlanner::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  if (message->command_id != this->last_command_id_) {
    this->last_progress_index_ = 0U;
    this->last_command_id_ = message->command_id;
  }
  this->latest_command_ = *message;
  this->has_command_ = true;
  RCLCPP_INFO(
    this->get_logger(), "Received command %u for route '%s' and updating local plan",
    message->command_id, message->route_id.c_str());
  this->publish_local_plan();
}

void LocalPlanner::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void LocalPlanner::publish_local_plan()
{
  if (
    !this->local_plan_publisher_ || !this->local_plan_publisher_->is_activated() ||
    !this->has_command_ || !this->has_current_pose_)
  {
    return;
  }

  this->local_plan_publisher_->publish(
    this->build_local_plan(this->latest_command_, this->current_pose_));

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Publishing local plan for command %u from progress index %zu",
    this->latest_command_.command_id,
    this->last_progress_index_);
}

nav_msgs::msg::Path LocalPlanner::build_local_plan(
  const amr_msgs::msg::MotionCommand & command,
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  const auto source_plan = this->build_source_plan(command);
  nav_msgs::msg::Path local_plan;
  local_plan.header = source_plan.header;
  if (local_plan.header.frame_id.empty()) {
    local_plan.header.frame_id = current_pose.header.frame_id;
  }
  local_plan.header.stamp = this->now();

  if (source_plan.poses.empty()) {
    return local_plan;
  }

  const auto & goal_pose = source_plan.poses.back();
  if (this->pose_distance(current_pose, goal_pose) <= this->goal_tolerance_) {
    this->last_progress_index_ = source_plan.poses.empty() ? 0U : source_plan.poses.size() - 1U;
    return local_plan;
  }

  const auto closest_index =
    this->find_closest_pose_index(source_plan, current_pose, this->last_progress_index_);
  this->last_progress_index_ = closest_index;
  local_plan.poses.push_back(current_pose);

  double accumulated_distance = 0.0;
  geometry_msgs::msg::PoseStamped segment_start = current_pose;
  for (std::size_t index = closest_index; index < source_plan.poses.size(); ++index) {
    const auto & target_pose = source_plan.poses[index];
    const double segment_distance = this->pose_distance(segment_start, target_pose);

    if (segment_distance <= 1e-6) {
      segment_start = target_pose;
      continue;
    }

    if (accumulated_distance + segment_distance >= this->lookahead_distance_) {
      const double remaining_distance = this->lookahead_distance_ - accumulated_distance;
      const double interpolation_ratio =
        std::clamp(remaining_distance / segment_distance, 0.0, 1.0);
      local_plan.poses.push_back(
        this->interpolate_pose(segment_start, target_pose, interpolation_ratio));
      return local_plan;
    }

    local_plan.poses.push_back(target_pose);
    accumulated_distance += segment_distance;
    segment_start = target_pose;
  }

  return local_plan;
}

nav_msgs::msg::Path LocalPlanner::build_source_plan(const amr_msgs::msg::MotionCommand & command) const
{
  nav_msgs::msg::Path source_plan = command.plan;
  if (source_plan.header.frame_id.empty()) {
    source_plan.header = command.header;
  }
  if (source_plan.header.stamp.sec == 0 && source_plan.header.stamp.nanosec == 0U) {
    source_plan.header.stamp = this->now();
  }
  if (source_plan.poses.empty()) {
    source_plan.poses.push_back(command.goal_pose);
  }
  return source_plan;
}

std::size_t LocalPlanner::find_closest_pose_index(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::size_t start_index) const
{
  if (plan.poses.empty()) {
    return 0U;
  }

  const auto search_start = std::min(start_index, plan.poses.size() - 1U);
  std::size_t closest_index = search_start;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t index = search_start; index < plan.poses.size(); ++index) {
    const double distance = this->pose_distance(current_pose, plan.poses[index]);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = index;
    }
  }

  return closest_index;
}

geometry_msgs::msg::PoseStamped LocalPlanner::interpolate_pose(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const double ratio) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = goal.header;
  pose.header.stamp = this->now();
  pose.pose.position.x = start.pose.position.x + ((goal.pose.position.x - start.pose.position.x) * ratio);
  pose.pose.position.y = start.pose.position.y + ((goal.pose.position.y - start.pose.position.y) * ratio);
  pose.pose.position.z = start.pose.position.z + ((goal.pose.position.z - start.pose.position.z) * ratio);
  pose.pose.orientation = ratio < 1.0 ? start.pose.orientation : goal.pose.orientation;
  return pose;
}

double LocalPlanner::pose_distance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr_local_planner
