#ifndef AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <string>

#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_local_planner
{

class LocalPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LocalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  void publish_control();
  double estimate_remaining_distance(const nav_msgs::msg::Path & path) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string status_topic_;
  std::string cmd_vel_topic_;
  int publish_period_ms_;
  double nominal_linear_velocity_;
  amr_msgs::msg::MotionCommand latest_command_;
  bool has_command_;
};

}  // namespace amr_local_planner

#endif  // AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
