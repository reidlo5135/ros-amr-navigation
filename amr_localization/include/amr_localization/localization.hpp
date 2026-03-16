#ifndef AMR_LOCALIZATION__LOCALIZATION_HPP_
#define AMR_LOCALIZATION__LOCALIZATION_HPP_

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

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

  void publish_odometry();
  nav_msgs::msg::Odometry build_odometry() const;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string base_frame_;
  int publish_period_ms_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__LOCALIZATION_HPP_
