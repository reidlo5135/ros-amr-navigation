#ifndef AMR_MCP_SERVER__ROS_TOOL_EXECUTOR_HPP_
#define AMR_MCP_SERVER__ROS_TOOL_EXECUTOR_HPP_

#include <memory>
#include <string>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_mcp_server/command_types.hpp"
#include "amr_mcp_server/topic_snapshot.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr::mcp
{

struct RosToolConfig
{
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string navigate_to_poses_action{"/navigate_to_poses"};
  std::string initial_pose_topic{"/initialpose"};
  std::string cmd_vel_topic{"/cmd_vel"};
  std::string default_frame_id{"map"};
  bool enable_direct_cmd_vel{false};
  double max_linear_speed{0.12};
  double max_angular_speed{0.8};
  int cmd_vel_duration_ms{500};
  bool dry_run{false};
};

class RosToolExecutor
{
public:
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  using NavigateToPoses = amr_msgs::action::NavigateToPoses;

  RosToolExecutor(
    rclcpp::Node *node,
    RosToolConfig config,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher);

  ExecutionResult execute(const AiCommand &command, const SnapshotData &snapshot);
  ExecutionResult executeRequestStatus(const AiCommand &command, const SnapshotData &snapshot);
  ExecutionResult executeNavigateToPose(const AiCommand &command);
  ExecutionResult executeNavigateRoute(const AiCommand &command);
  ExecutionResult executeCancelNavigation(const AiCommand &command);
  ExecutionResult executeSetInitialPose(const AiCommand &command);
  ExecutionResult executeSaveMap(const AiCommand &command);
  ExecutionResult executeSendMotionCommand(const AiCommand &command);
  ExecutionResult executeRequestPlanSegment(const AiCommand &command);
  ExecutionResult executeRequestPlanRoute(const AiCommand &command);
  ExecutionResult executeChangeRobotId(const AiCommand &command);

private:
  geometry_msgs::msg::PoseStamped toPoseStamped(const PoseGoal &goal, const std::string &frame_id) const;
  void publishEvent(const std::string &event_type, const std::string &message) const;
  void publishFeedback(const std::string &message) const;
  static std::string escapeJson(const std::string &text);

  rclcpp::Node *node_{nullptr};
  RosToolConfig config_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<NavigateToPoses>::SharedPtr navigate_to_poses_client_;
};

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__ROS_TOOL_EXECUTOR_HPP_
