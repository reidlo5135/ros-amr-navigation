#ifndef AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <cstddef>
#include <string>

#include "amr_msgs/msg/motion_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void publish_local_plan();
  nav_msgs::msg::Path build_local_plan(
    const amr_msgs::msg::MotionCommand & command,
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  nav_msgs::msg::Path build_source_plan(const amr_msgs::msg::MotionCommand & command) const;
  std::size_t find_closest_pose_index(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & current_pose) const;
  geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double ratio) const;
  double pose_distance(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string current_pose_topic_;
  std::string local_plan_topic_;
  int publish_period_ms_;
  double lookahead_distance_;
  double goal_tolerance_;
  amr_msgs::msg::MotionCommand latest_command_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_command_;
  bool has_current_pose_;
};

}  // namespace amr_local_planner

#endif  // AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
