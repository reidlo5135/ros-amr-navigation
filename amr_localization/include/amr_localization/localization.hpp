#ifndef AMR_LOCALIZATION__LOCALIZATION_HPP_
#define AMR_LOCALIZATION__LOCALIZATION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace amr_localization
{

class Localization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Localization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void update_estimated_pose(const rclcpp::Time & stamp);
  void publish_outputs(const rclcpp::Time & stamp);
  geometry_msgs::msg::TransformStamped build_map_to_odom_transform(const rclcpp::Time & stamp) const;
  geometry_msgs::msg::PoseStamped odometry_pose_to_pose_stamped(
    const nav_msgs::msg::Odometry & odometry) const;
  double normalize_angle(double angle) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  void update_pose_orientation(geometry_msgs::msg::PoseStamped & pose, double yaw) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string estimated_pose_topic_;
  std::string estimated_odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PoseStamped initial_map_pose_;
  geometry_msgs::msg::PoseStamped reference_odom_pose_;
  geometry_msgs::msg::PoseStamped estimated_pose_;
  bool has_latest_odom_;
  bool has_reference_odom_;
  bool has_initial_pose_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__LOCALIZATION_HPP_
