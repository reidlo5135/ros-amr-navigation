#ifndef AMR_RVIZ_PLUGINS__GOAL_BRIDGE_HPP_
#define AMR_RVIZ_PLUGINS__GOAL_BRIDGE_HPP_

#include <string>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_rviz_plugins
{

class GoalBridge : public rclcpp::Node
{
public:
  explicit GoalBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void handle_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;

  std::string goal_pose_topic_;
  std::string navigate_to_pose_action_;
  int action_wait_timeout_ms_;
};

}  // namespace amr_rviz_plugins

#endif  // AMR_RVIZ_PLUGINS__GOAL_BRIDGE_HPP_
