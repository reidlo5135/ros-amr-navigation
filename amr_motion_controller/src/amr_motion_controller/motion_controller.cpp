#include "amr_motion_controller/motion_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace amr_motion_controller
{

MotionController::MotionController(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("motion_controller", options),
  command_topic_("/amr/motion_controller/command"),
  local_plan_topic_("/amr/local_planner/plan"),
  current_pose_topic_("/amr/localization/pose"),
  status_topic_("/amr/motion_controller/status"),
  cmd_vel_topic_("/cmd_vel"),
  publish_period_ms_(100),
  nominal_linear_velocity_(0.3),
  max_linear_velocity_(0.3),
  min_linear_velocity_(0.05),
  max_angular_velocity_(1.5),
  heading_gain_(2.0),
  rotate_in_place_threshold_(0.6),
  goal_tolerance_(0.15),
  has_command_(false),
  has_local_plan_(false),
  has_current_pose_(false)
{
  this->declare_parameter("command_topic", this->command_topic_);
  this->declare_parameter("local_plan_topic", this->local_plan_topic_);
  this->declare_parameter("current_pose_topic", this->current_pose_topic_);
  this->declare_parameter("status_topic", this->status_topic_);
  this->declare_parameter("cmd_vel_topic", this->cmd_vel_topic_);
  this->declare_parameter("publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("nominal_linear_velocity", this->nominal_linear_velocity_);
  this->declare_parameter("max_linear_velocity", this->max_linear_velocity_);
  this->declare_parameter("min_linear_velocity", this->min_linear_velocity_);
  this->declare_parameter("max_angular_velocity", this->max_angular_velocity_);
  this->declare_parameter("heading_gain", this->heading_gain_);
  this->declare_parameter("rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->declare_parameter("goal_tolerance", this->goal_tolerance_);
}

MotionController::CallbackReturn MotionController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("command_topic", this->command_topic_);
  this->get_parameter("local_plan_topic", this->local_plan_topic_);
  this->get_parameter("current_pose_topic", this->current_pose_topic_);
  this->get_parameter("status_topic", this->status_topic_);
  this->get_parameter("cmd_vel_topic", this->cmd_vel_topic_);
  this->get_parameter("publish_period_ms", this->publish_period_ms_);
  this->get_parameter("nominal_linear_velocity", this->nominal_linear_velocity_);
  this->get_parameter("max_linear_velocity", this->max_linear_velocity_);
  this->get_parameter("min_linear_velocity", this->min_linear_velocity_);
  this->get_parameter("max_angular_velocity", this->max_angular_velocity_);
  this->get_parameter("heading_gain", this->heading_gain_);
  this->get_parameter("rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->get_parameter("goal_tolerance", this->goal_tolerance_);

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
    });
  this->local_plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    this->local_plan_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Path::SharedPtr message) {
      this->handle_local_plan(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    this->cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  this->motion_status_publisher_ = this->create_publisher<amr_msgs::msg::MotionStatus>(
    this->status_topic_, rclcpp::SystemDefaultsQoS());

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(this->publish_period_ms_),
    [this]() { this->publish_control(); });
  this->timer_->cancel();

  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->cmd_vel_publisher_->on_activate();
  this->motion_status_publisher_->on_activate();
  this->timer_->reset();
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->timer_) {
    this->timer_->cancel();
  }
  if (this->cmd_vel_publisher_) {
    this->cmd_vel_publisher_->on_deactivate();
  }
  if (this->motion_status_publisher_) {
    this->motion_status_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  return CallbackReturn::SUCCESS;
}

void MotionController::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  this->latest_command_ = *message;
  this->has_command_ = true;
}

void MotionController::handle_local_plan(const nav_msgs::msg::Path::SharedPtr message)
{
  this->latest_local_plan_ = *message;
  this->has_local_plan_ = true;
}

void MotionController::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void MotionController::publish_control()
{
  if (
    !this->cmd_vel_publisher_ || !this->cmd_vel_publisher_->is_activated() ||
    !this->motion_status_publisher_ || !this->motion_status_publisher_->is_activated())
  {
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  amr_msgs::msg::MotionStatus status;
  status.header.stamp = this->now();
  status.header.frame_id = "map";
  status.current_pose = this->current_pose_;

  if (this->has_command_ && this->has_local_plan_ && this->has_current_pose_) {
    const auto tracking_target = this->select_tracking_target();
    const auto remaining_distance = this->estimate_remaining_distance(this->latest_local_plan_);
    const auto goal_distance = this->pose_distance(this->current_pose_, this->latest_command_.goal_pose);
    const auto current_yaw = this->quaternion_yaw(this->current_pose_.pose.orientation);
    const auto target_heading = std::atan2(
      tracking_target.pose.position.y - this->current_pose_.pose.position.y,
      tracking_target.pose.position.x - this->current_pose_.pose.position.x);
    const auto heading_error = this->normalize_angle(target_heading - current_yaw);

    status.command_id = this->latest_command_.command_id;
    status.active = true;
    status.goal_reached =
      goal_distance <= this->goal_tolerance_ ||
      this->latest_local_plan_.poses.empty();
    status.current_pose = this->current_pose_;
    status.remaining_distance = remaining_distance;
    status.heading_error = heading_error;

    if (status.goal_reached) {
      this->has_command_ = false;
      this->has_local_plan_ = false;
    } else {
      if (std::abs(heading_error) > this->rotate_in_place_threshold_) {
        cmd_vel.angular.z = this->clamp(
          this->heading_gain_ * heading_error,
          -this->max_angular_velocity_,
          this->max_angular_velocity_);
      } else {
        const auto lookahead_distance = std::max(0.001, this->pose_distance(this->current_pose_, tracking_target));
        const auto curvature = (2.0 * std::sin(heading_error)) / lookahead_distance;
        const auto linear_speed = this->clamp(
          this->nominal_linear_velocity_ * std::max(0.0, std::cos(heading_error)),
          this->min_linear_velocity_,
          this->max_linear_velocity_);
        cmd_vel.linear.x = linear_speed;
        cmd_vel.angular.z = this->clamp(
          linear_speed * curvature,
          -this->max_angular_velocity_,
          this->max_angular_velocity_);
      }
    }
  } else {
    status.active = false;
    status.goal_reached = true;
  }

  this->cmd_vel_publisher_->publish(cmd_vel);
  this->motion_status_publisher_->publish(status);
}

double MotionController::estimate_remaining_distance(const nav_msgs::msg::Path & path) const
{
  double distance = 0.0;
  if (path.poses.size() < 2U) {
    return distance;
  }

  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto dx = current.x - previous.x;
    const auto dy = current.y - previous.y;
    distance += std::sqrt((dx * dx) + (dy * dy));
  }

  return distance;
}

double MotionController::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MotionController::normalize_angle(double angle) const
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MotionController::clamp(
  const double value,
  const double min_value,
  const double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

geometry_msgs::msg::PoseStamped MotionController::select_tracking_target() const
{
  if (!this->latest_local_plan_.poses.empty()) {
    return this->latest_local_plan_.poses.back();
  }
  return this->latest_command_.goal_pose;
}

double MotionController::pose_distance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr_motion_controller
