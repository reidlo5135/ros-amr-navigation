/**
 * @file rviz_bridge.cpp
 * @brief Bridge from RViz goal topics to AMR navigation actions.
 */

#include <chrono>
#include <memory>
#include <string>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

using namespace std::chrono_literals;

/// @brief Forwards RViz goal topics to AMR NavigateToPose/NavigateToPoses actions.
class RvizBridge : public rclcpp::Node
{
public:
  /// @brief NavigateToPose action alias.
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  /// @brief NavigateToPoses action alias.
  using NavigateToPoses = amr_msgs::action::NavigateToPoses;

  /// @brief Construct subscriptions and action clients for RViz goal forwarding.
  RvizBridge()
  : Node("rviz_bridge")
  {
    goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/rviz/goal");
    goals_topic_ = this->declare_parameter<std::string>("goals_topic", "/rviz/goals");
    navigate_to_pose_action_ = this->declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    navigate_to_poses_action_ = this->declare_parameter<std::string>(
      "navigate_to_poses_action", "/navigate_to_poses");
    default_frame_id_ = this->declare_parameter<std::string>("default_frame_id", "map");

    navigate_to_pose_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);
    navigate_to_poses_client_ =
      rclcpp_action::create_client<NavigateToPoses>(this, navigate_to_poses_action_);

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        this->handle_goal(*message);
      });

    goals_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      goals_topic_,
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        this->handle_goals(*message);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "RViz bridge ready: goal_topic='%s', goals_topic='%s', actions='%s'/'%s'",
      goal_topic_.c_str(),
      goals_topic_.c_str(),
      navigate_to_pose_action_.c_str(),
      navigate_to_poses_action_.c_str());
  }

private:
  /// @brief Fill missing frame/stamp metadata on a pose stamped message.
  geometry_msgs::msg::PoseStamped normalize_pose(
    const geometry_msgs::msg::PoseStamped &input) const
  {
    geometry_msgs::msg::PoseStamped normalized = input;
    if (normalized.header.frame_id.empty()) {
      normalized.header.frame_id = default_frame_id_;
    }
    if (normalized.header.stamp.sec == 0 && normalized.header.stamp.nanosec == 0) {
      normalized.header.stamp = this->now();
    }
    return normalized;
  }

  /// @brief Convert a PoseArray element into a stamped pose with default metadata.
  geometry_msgs::msg::PoseStamped to_pose_stamped(
    const std_msgs::msg::Header &header,
    const geometry_msgs::msg::Pose &pose) const
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = header;
    stamped.header.frame_id =
      stamped.header.frame_id.empty() ? default_frame_id_ : stamped.header.frame_id;
    if (stamped.header.stamp.sec == 0 && stamped.header.stamp.nanosec == 0) {
      stamped.header.stamp = this->now();
    }
    stamped.pose = pose;
    return stamped;
  }

  /// @brief Wait briefly for the selected navigation action server.
  bool wait_for_server(const std::string &name, bool multi_goal)
  {
    const bool available = multi_goal ?
      navigate_to_poses_client_->wait_for_action_server(1s) :
      navigate_to_pose_client_->wait_for_action_server(1s);

    if (!available) {
      RCLCPP_ERROR(this->get_logger(), "Action server '%s' is unavailable", name.c_str());
    }
    return available;
  }

  /// @brief Forward one RViz PoseStamped goal to NavigateToPose.
  void handle_goal(const geometry_msgs::msg::PoseStamped &message)
  {
    if (!wait_for_server(navigate_to_pose_action_, false)) {
      return;
    }

    NavigateToPose::Goal goal;
    goal.goal_pose = normalize_pose(message);

    RCLCPP_INFO(
      this->get_logger(),
      "Forwarding single RViz goal: frame='%s' x=%.3f y=%.3f",
      goal.goal_pose.header.frame_id.c_str(),
      goal.goal_pose.pose.position.x,
      goal.goal_pose.pose.position.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "Navigator rejected RViz single-goal request");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Navigator accepted RViz single-goal request");
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result) {
        RCLCPP_INFO(
          this->get_logger(),
          "RViz single-goal request finished with code=%d",
          static_cast<int>(result.code));
      };
    navigate_to_pose_client_->async_send_goal(goal, options);
  }

  /// @brief Forward an RViz PoseArray route to NavigateToPoses.
  void handle_goals(const geometry_msgs::msg::PoseArray &message)
  {
    if (message.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Ignoring empty RViz multi-goal route");
      return;
    }

    if (!wait_for_server(navigate_to_poses_action_, true)) {
      return;
    }

    NavigateToPoses::Goal goal;
    goal.goal_poses.reserve(message.poses.size());
    for (const auto &pose : message.poses) {
      goal.goal_poses.push_back(to_pose_stamped(message.header, pose));
    }

    const auto &first = goal.goal_poses.front();
    const auto &last = goal.goal_poses.back();
    RCLCPP_INFO(
      this->get_logger(),
      "Forwarding RViz multi-goal route with %zu goals: first=(%.3f, %.3f) last=(%.3f, %.3f)",
      goal.goal_poses.size(),
      first.pose.position.x,
      first.pose.position.y,
      last.pose.position.x,
      last.pose.position.y);

    rclcpp_action::Client<NavigateToPoses>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::SharedPtr &goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "Navigator rejected RViz multi-goal route");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Navigator accepted RViz multi-goal route");
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::WrappedResult &result) {
        RCLCPP_INFO(
          this->get_logger(),
          "RViz multi-goal route finished with code=%d",
          static_cast<int>(result.code));
      };
    navigate_to_poses_client_->async_send_goal(goal, options);
  }

  std::string goal_topic_;
  std::string goals_topic_;
  std::string navigate_to_pose_action_;
  std::string navigate_to_poses_action_;
  std::string default_frame_id_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<NavigateToPoses>::SharedPtr navigate_to_poses_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_subscription_;
};

}  // namespace

/// @brief Initialize ROS, spin the RViz bridge node, and shut ROS down.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizBridge>());
  rclcpp::shutdown();
  return 0;
}
