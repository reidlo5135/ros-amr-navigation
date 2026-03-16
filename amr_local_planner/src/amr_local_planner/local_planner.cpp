#include "amr_local_planner/local_planner.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace amr_local_planner
{

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("local_planner", options),
  command_topic_("motion_command"),
  status_topic_("motion_status"),
  cmd_vel_topic_("cmd_vel"),
  publish_period_ms_(100),
  nominal_linear_velocity_(0.3),
  has_command_(false)
{
  this->declare_parameter("command_topic", this->command_topic_);
  this->declare_parameter("status_topic", this->status_topic_);
  this->declare_parameter("cmd_vel_topic", this->cmd_vel_topic_);
  this->declare_parameter("publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("nominal_linear_velocity", this->nominal_linear_velocity_);
}

LocalPlanner::CallbackReturn LocalPlanner::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("command_topic", this->command_topic_);
  this->get_parameter("status_topic", this->status_topic_);
  this->get_parameter("cmd_vel_topic", this->cmd_vel_topic_);
  this->get_parameter("publish_period_ms", this->publish_period_ms_);
  this->get_parameter("nominal_linear_velocity", this->nominal_linear_velocity_);

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
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

LocalPlanner::CallbackReturn LocalPlanner::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->cmd_vel_publisher_->on_activate();
  this->motion_status_publisher_->on_activate();
  this->timer_->reset();
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_deactivate(const rclcpp_lifecycle::State & state)
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

LocalPlanner::CallbackReturn LocalPlanner::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->has_command_ = false;
  return CallbackReturn::SUCCESS;
}

LocalPlanner::CallbackReturn LocalPlanner::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->motion_command_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->has_command_ = false;
  return CallbackReturn::SUCCESS;
}

void LocalPlanner::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  this->latest_command_ = *message;
  this->has_command_ = true;
  RCLCPP_INFO(
    this->get_logger(), "Received command %u for route '%s'",
    this->latest_command_.command_id, this->latest_command_.route_id.c_str());
}

void LocalPlanner::publish_control()
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

  if (this->has_command_) {
    cmd_vel.linear.x = this->nominal_linear_velocity_;
    status.command_id = this->latest_command_.command_id;
    status.active = true;
    status.goal_reached = this->latest_command_.plan.poses.empty();
    status.current_pose = this->latest_command_.goal_pose;
    status.remaining_distance = this->estimate_remaining_distance(this->latest_command_.plan);
    status.heading_error = 0.0;
    if (status.goal_reached) {
      this->has_command_ = false;
    }
  } else {
    status.active = false;
    status.goal_reached = true;
  }

  this->cmd_vel_publisher_->publish(cmd_vel);
  this->motion_status_publisher_->publish(status);
}

double LocalPlanner::estimate_remaining_distance(const nav_msgs::msg::Path & path) const
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

}  // namespace amr_local_planner
