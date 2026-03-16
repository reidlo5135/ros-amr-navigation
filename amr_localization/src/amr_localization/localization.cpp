#include "amr_localization/localization.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace amr_localization
{

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("localization", options),
  odom_topic_("localization/odom"),
  map_frame_("map"),
  base_frame_("base_link"),
  publish_period_ms_(100)
{
  this->declare_parameter("odom_topic", this->odom_topic_);
  this->declare_parameter("map_frame", this->map_frame_);
  this->declare_parameter("base_frame", this->base_frame_);
  this->declare_parameter("publish_period_ms", this->publish_period_ms_);
}

Localization::CallbackReturn Localization::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("odom_topic", this->odom_topic_);
  this->get_parameter("map_frame", this->map_frame_);
  this->get_parameter("base_frame", this->base_frame_);
  this->get_parameter("publish_period_ms", this->publish_period_ms_);

  this->odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->odom_topic_, rclcpp::SystemDefaultsQoS());

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(this->publish_period_ms_),
    [this]() { this->publish_odometry(); });
  this->timer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "Configured localization on topic '%s'", this->odom_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->odometry_publisher_->on_activate();
  this->timer_->reset();
  this->publish_odometry();
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->timer_) {
    this->timer_->cancel();
  }
  if (this->odometry_publisher_) {
    this->odometry_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->timer_.reset();
  this->odometry_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->timer_.reset();
  this->odometry_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

void Localization::publish_odometry()
{
  if (!this->odometry_publisher_ || !this->odometry_publisher_->is_activated()) {
    return;
  }
  this->odometry_publisher_->publish(this->build_odometry());
}

nav_msgs::msg::Odometry Localization::build_odometry() const
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = this->now();
  odometry.header.frame_id = this->map_frame_;
  odometry.child_frame_id = this->base_frame_;
  odometry.pose.pose.orientation.w = 1.0;
  return odometry;
}

}  // namespace amr_localization
