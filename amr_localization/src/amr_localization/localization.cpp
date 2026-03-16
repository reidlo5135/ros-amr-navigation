#include "amr_localization/localization.hpp"

#include <cmath>

namespace amr_localization
{

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("localization", options),
  odom_topic_("/odom"),
  initial_pose_topic_("/amr/localization/initial_pose"),
  estimated_pose_topic_("/amr/localization/pose"),
  estimated_odom_topic_("/amr/localization/odometry"),
  map_frame_("map"),
  odom_frame_("odom"),
  base_frame_("base_link"),
  initial_x_(0.0),
  initial_y_(0.0),
  initial_yaw_(0.0),
  has_latest_odom_(false),
  has_reference_odom_(false),
  has_initial_pose_(false)
{
  this->declare_parameter("odom_topic", this->odom_topic_);
  this->declare_parameter("initial_pose_topic", this->initial_pose_topic_);
  this->declare_parameter("estimated_pose_topic", this->estimated_pose_topic_);
  this->declare_parameter("estimated_odom_topic", this->estimated_odom_topic_);
  this->declare_parameter("map_frame", this->map_frame_);
  this->declare_parameter("odom_frame", this->odom_frame_);
  this->declare_parameter("base_frame", this->base_frame_);
  this->declare_parameter("initial_pose.x", this->initial_x_);
  this->declare_parameter("initial_pose.y", this->initial_y_);
  this->declare_parameter("initial_pose.yaw", this->initial_yaw_);
}

Localization::CallbackReturn Localization::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("odom_topic", this->odom_topic_);
  this->get_parameter("initial_pose_topic", this->initial_pose_topic_);
  this->get_parameter("estimated_pose_topic", this->estimated_pose_topic_);
  this->get_parameter("estimated_odom_topic", this->estimated_odom_topic_);
  this->get_parameter("map_frame", this->map_frame_);
  this->get_parameter("odom_frame", this->odom_frame_);
  this->get_parameter("base_frame", this->base_frame_);
  this->get_parameter("initial_pose.x", this->initial_x_);
  this->get_parameter("initial_pose.y", this->initial_y_);
  this->get_parameter("initial_pose.yaw", this->initial_yaw_);

  this->initial_map_pose_.header.frame_id = this->map_frame_;
  this->initial_map_pose_.pose.position.x = this->initial_x_;
  this->initial_map_pose_.pose.position.y = this->initial_y_;
  this->initial_map_pose_.pose.position.z = 0.0;
  this->update_pose_orientation(this->initial_map_pose_, this->initial_yaw_);
  this->estimated_pose_ = this->initial_map_pose_;
  this->has_initial_pose_ = true;

  this->odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->odom_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      this->handle_odometry(message);
    });
  this->initial_pose_subscription_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->initial_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      this->handle_initial_pose(message);
    });
  this->estimated_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->estimated_pose_topic_, rclcpp::SystemDefaultsQoS());
  this->estimated_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->estimated_odom_topic_, rclcpp::SystemDefaultsQoS());
  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "Configured localization with odom topic '%s'", this->odom_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_activate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_activate();
  }
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_deactivate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
  this->transform_broadcaster_.reset();
  this->latest_odom_ = nav_msgs::msg::Odometry();
  this->reference_odom_pose_ = geometry_msgs::msg::PoseStamped();
  this->estimated_pose_ = geometry_msgs::msg::PoseStamped();
  this->has_latest_odom_ = false;
  this->has_reference_odom_ = false;
  this->has_initial_pose_ = false;
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
  this->transform_broadcaster_.reset();
  this->latest_odom_ = nav_msgs::msg::Odometry();
  this->reference_odom_pose_ = geometry_msgs::msg::PoseStamped();
  this->estimated_pose_ = geometry_msgs::msg::PoseStamped();
  this->has_latest_odom_ = false;
  this->has_reference_odom_ = false;
  this->has_initial_pose_ = false;
  return CallbackReturn::SUCCESS;
}

void Localization::handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  this->latest_odom_ = *message;
  this->has_latest_odom_ = true;

  if (!this->has_reference_odom_) {
    this->reference_odom_pose_ = this->odometry_pose_to_pose_stamped(this->latest_odom_);
    this->has_reference_odom_ = true;
  }

  if (!this->has_initial_pose_) {
    return;
  }

  this->update_estimated_pose(message->header.stamp);
  this->publish_outputs(message->header.stamp);
}

void Localization::handle_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  this->initial_map_pose_.header = message->header;
  if (this->initial_map_pose_.header.frame_id.empty()) {
    this->initial_map_pose_.header.frame_id = this->map_frame_;
  }
  this->initial_map_pose_.pose = message->pose.pose;
  this->estimated_pose_ = this->initial_map_pose_;
  this->has_initial_pose_ = true;

  if (this->has_latest_odom_) {
    this->reference_odom_pose_ = this->odometry_pose_to_pose_stamped(this->latest_odom_);
    this->has_reference_odom_ = true;
    this->update_estimated_pose(message->header.stamp);
    this->publish_outputs(message->header.stamp);
  }
}

void Localization::update_estimated_pose(const rclcpp::Time & stamp)
{
  if (!this->has_latest_odom_ || !this->has_reference_odom_ || !this->has_initial_pose_) {
    return;
  }

  const auto current_odom_pose = this->odometry_pose_to_pose_stamped(this->latest_odom_);
  const double map_yaw = this->quaternion_yaw(this->initial_map_pose_.pose.orientation);
  const double reference_odom_yaw = this->quaternion_yaw(this->reference_odom_pose_.pose.orientation);
  const double current_odom_yaw = this->quaternion_yaw(current_odom_pose.pose.orientation);

  const double delta_odom_x =
    current_odom_pose.pose.position.x - this->reference_odom_pose_.pose.position.x;
  const double delta_odom_y =
    current_odom_pose.pose.position.y - this->reference_odom_pose_.pose.position.y;

  const double local_delta_x =
    std::cos(reference_odom_yaw) * delta_odom_x + std::sin(reference_odom_yaw) * delta_odom_y;
  const double local_delta_y =
    -std::sin(reference_odom_yaw) * delta_odom_x + std::cos(reference_odom_yaw) * delta_odom_y;

  this->estimated_pose_.header.stamp = stamp;
  this->estimated_pose_.header.frame_id = this->map_frame_;
  this->estimated_pose_.pose.position.x =
    this->initial_map_pose_.pose.position.x +
    (std::cos(map_yaw) * local_delta_x - std::sin(map_yaw) * local_delta_y);
  this->estimated_pose_.pose.position.y =
    this->initial_map_pose_.pose.position.y +
    (std::sin(map_yaw) * local_delta_x + std::cos(map_yaw) * local_delta_y);
  this->estimated_pose_.pose.position.z = this->initial_map_pose_.pose.position.z;
  this->update_pose_orientation(
    this->estimated_pose_,
    this->normalize_angle(map_yaw + (current_odom_yaw - reference_odom_yaw)));
}

void Localization::publish_outputs(const rclcpp::Time & stamp)
{
  if (
    !this->estimated_pose_publisher_ || !this->estimated_pose_publisher_->is_activated() ||
    !this->estimated_odometry_publisher_ || !this->estimated_odometry_publisher_->is_activated())
  {
    return;
  }

  this->estimated_pose_publisher_->publish(this->estimated_pose_);

  nav_msgs::msg::Odometry estimated_odometry;
  estimated_odometry.header = this->estimated_pose_.header;
  estimated_odometry.child_frame_id = this->base_frame_;
  estimated_odometry.pose.pose = this->estimated_pose_.pose;
  estimated_odometry.twist = this->latest_odom_.twist;
  this->estimated_odometry_publisher_->publish(estimated_odometry);

  if (this->transform_broadcaster_) {
    this->transform_broadcaster_->sendTransform(this->build_map_to_odom_transform(stamp));
  }
}

geometry_msgs::msg::TransformStamped Localization::build_map_to_odom_transform(
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = this->map_frame_;
  transform.child_frame_id = this->odom_frame_;

  const auto current_odom_pose = this->odometry_pose_to_pose_stamped(this->latest_odom_);
  const double map_yaw = this->quaternion_yaw(this->estimated_pose_.pose.orientation);
  const double odom_yaw = this->quaternion_yaw(current_odom_pose.pose.orientation);
  const double yaw_delta = this->normalize_angle(map_yaw - odom_yaw);

  const double odom_x = current_odom_pose.pose.position.x;
  const double odom_y = current_odom_pose.pose.position.y;
  transform.transform.translation.x =
    this->estimated_pose_.pose.position.x - (std::cos(yaw_delta) * odom_x - std::sin(yaw_delta) * odom_y);
  transform.transform.translation.y =
    this->estimated_pose_.pose.position.y - (std::sin(yaw_delta) * odom_x + std::cos(yaw_delta) * odom_y);
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.z = std::sin(yaw_delta * 0.5);
  transform.transform.rotation.w = std::cos(yaw_delta * 0.5);
  return transform;
}

geometry_msgs::msg::PoseStamped Localization::odometry_pose_to_pose_stamped(
  const nav_msgs::msg::Odometry & odometry) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = this->odom_frame_;
  }
  pose.pose = odometry.pose.pose;
  return pose;
}

double Localization::normalize_angle(double angle) const
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

double Localization::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void Localization::update_pose_orientation(geometry_msgs::msg::PoseStamped & pose, const double yaw) const
{
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
}

}  // namespace amr_localization
