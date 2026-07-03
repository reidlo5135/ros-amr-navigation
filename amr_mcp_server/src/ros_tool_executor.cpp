#include "amr_mcp_server/ros_tool_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>
#include <utility>

namespace amr::mcp
{

namespace
{

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

std::string result_label(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "성공";
    case rclcpp_action::ResultCode::ABORTED:
      return "실패";
    case rclcpp_action::ResultCode::CANCELED:
      return "취소";
    case rclcpp_action::ResultCode::UNKNOWN:
      break;
  }
  return "알 수 없음";
}

}  // namespace

RosToolExecutor::RosToolExecutor(
  rclcpp::Node *node,
  RosToolConfig config,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher)
: node_(node),
  config_(std::move(config)),
  events_publisher_(std::move(events_publisher)),
  feedback_publisher_(std::move(feedback_publisher))
{
  initial_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_.initial_pose_topic, rclcpp::SystemDefaultsQoS());
  cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    config_.cmd_vel_topic, rclcpp::QoS(10));
  navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    config_.navigate_to_pose_action);
  navigate_to_poses_client_ = rclcpp_action::create_client<NavigateToPoses>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    config_.navigate_to_poses_action);
}

ExecutionResult RosToolExecutor::execute(const AiCommand &command, const SnapshotData &snapshot)
{
  switch (command.type) {
    case AiCommandType::RequestStatus:
      return executeRequestStatus(command, snapshot);
    case AiCommandType::NavigateToPose:
      return executeNavigateToPose(command);
    case AiCommandType::NavigateRoute:
      return executeNavigateRoute(command);
    case AiCommandType::CancelNavigation:
      return executeCancelNavigation(command);
    case AiCommandType::SetInitialPose:
      return executeSetInitialPose(command);
    case AiCommandType::SaveMap:
      return executeSaveMap(command);
    case AiCommandType::SendMotionCommand:
      return executeSendMotionCommand(command);
    case AiCommandType::RequestPlanSegment:
      return executeRequestPlanSegment(command);
    case AiCommandType::RequestPlanRoute:
      return executeRequestPlanRoute(command);
    case AiCommandType::ChangeRobotId:
      return executeChangeRobotId(command);
    case AiCommandType::ChatOnly:
    case AiCommandType::Unknown:
      break;
  }

  ExecutionResult result;
  result.command_type = command.type;
  result.response = "명령으로 확정되지 않아 대화 응답으로 처리합니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeRequestStatus(
  const AiCommand &command,
  const SnapshotData &snapshot)
{
  (void)command;
  std::ostringstream response;
  response << "현재 수신된 AMR 상태입니다.\n";
  if (snapshot.has_battery) {
    response << "- 배터리: " << static_cast<int>(std::round(snapshot.battery_percentage)) << "%\n";
  } else {
    response << "- 배터리: 수신된 데이터 없음\n";
  }
  if (snapshot.has_pose) {
    response << "- 위치(map): x=" << snapshot.pose_x << ", y=" << snapshot.pose_y <<
      ", yaw=" << snapshot.pose_yaw << "\n";
  } else {
    response << "- 위치: 수신된 데이터 없음\n";
  }
  if (snapshot.has_motion) {
    response << "- 주행: " << (snapshot.motion_active ? "활성" : "대기") <<
      ", remaining=" << snapshot.remaining_distance <<
      ", heading_error=" << snapshot.heading_error << "\n";
    response << "- 막힘: " << (snapshot.motion_blocked ? "감지됨" : "clear") <<
      ", 정체: " << (snapshot.motion_stalled ? "감지됨" : "없음") << "\n";
  } else {
    response << "- 주행 상태: 수신된 데이터 없음\n";
  }
  if (snapshot.has_runtime_summary) {
    response << "- 런타임: " <<
      (snapshot.runtime_state.empty() ? "unknown" : snapshot.runtime_state) <<
      ", blocked=" <<
      (snapshot.blocked_context.empty() ? "unknown" : snapshot.blocked_context) <<
      ", recovery=" <<
      (snapshot.recovery_phase.empty() ? "unknown" : snapshot.recovery_phase) << "\n";
  } else {
    response << "- 런타임 요약: 수신된 데이터 없음\n";
  }

  ExecutionResult result;
  result.command_type = AiCommandType::RequestStatus;
  result.command_executed = true;
  result.response = response.str();
  return result;
}

ExecutionResult RosToolExecutor::executeNavigateToPose(const AiCommand &command)
{
  ExecutionResult result;
  result.command_type = AiCommandType::NavigateToPose;
  if (!command.has_pose) {
    result.accepted = false;
    result.error_message = "목표 좌표 x/y를 찾지 못했습니다.";
    result.response = result.error_message;
    return result;
  }
  if (config_.dry_run) {
    result.command_executed = false;
    result.response = "dry_run 모드: NavigateToPose 전송을 생략했습니다.";
    return result;
  }
  if (!navigate_to_pose_client_->action_server_is_ready()) {
    result.accepted = false;
    result.error_message = "NavigateToPose action server가 준비되지 않았습니다.";
    result.response = result.error_message;
    return result;
  }

  PoseGoal goal;
  goal.x = command.x;
  goal.y = command.y;
  goal.z = command.z;
  goal.yaw = command.yaw;
  NavigateToPose::Goal action_goal;
  action_goal.goal_pose = toPoseStamped(goal, command.frame_id);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle) {
      publishEvent("navigate_to_pose.goal_response", handle ? "accepted" : "rejected");
      if (!handle) {
        publishFeedback("NavigateToPose goal이 거부되었습니다.");
      }
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &wrapped) {
      const std::string error = wrapped.result ? wrapped.result->error_msg : "";
      const std::string error_suffix = error.empty() ? "" : " (" + error + ")";
      publishEvent(
        "navigate_to_pose.result",
        "result_code=" + std::to_string(static_cast<int>(wrapped.code)) +
        ", error=" + error);
      publishFeedback("NavigateToPose 주행 결과: " + result_label(wrapped.code) + error_suffix);
    };
  navigate_to_pose_client_->async_send_goal(action_goal, options);

  result.command_executed = true;
  result.response = "목표 좌표로 이동 명령을 전송했습니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeNavigateRoute(const AiCommand &command)
{
  ExecutionResult result;
  result.command_type = AiCommandType::NavigateRoute;
  if (command.goals.empty()) {
    result.accepted = false;
    result.error_message = "경유 좌표 목록을 찾지 못했습니다.";
    result.response = result.error_message;
    return result;
  }
  if (config_.dry_run) {
    result.response = "dry_run 모드: NavigateToPoses 전송을 생략했습니다.";
    return result;
  }
  if (!navigate_to_poses_client_->action_server_is_ready()) {
    result.accepted = false;
    result.error_message = "NavigateToPoses action server가 준비되지 않았습니다.";
    result.response = result.error_message;
    return result;
  }

  NavigateToPoses::Goal action_goal;
  action_goal.goal_poses.reserve(command.goals.size());
  for (const auto &goal : command.goals) {
    action_goal.goal_poses.push_back(toPoseStamped(goal, command.frame_id));
  }

  rclcpp_action::Client<NavigateToPoses>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::SharedPtr &handle) {
      publishEvent("navigate_route.goal_response", handle ? "accepted" : "rejected");
      if (!handle) {
        publishFeedback("NavigateToPoses route goal이 거부되었습니다.");
      }
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::WrappedResult &wrapped) {
      const std::string error = wrapped.result ? wrapped.result->error_msg : "";
      const std::string completed = wrapped.result ?
        ", completed_goals=" + std::to_string(wrapped.result->completed_goals) : "";
      const std::string error_suffix = error.empty() ? "" : " (" + error + ")";
      publishEvent(
        "navigate_route.result",
        "result_code=" + std::to_string(static_cast<int>(wrapped.code)) +
        ", error=" + error);
      publishFeedback(
        "NavigateToPoses 주행 결과: " + result_label(wrapped.code) + completed + error_suffix);
    };
  navigate_to_poses_client_->async_send_goal(action_goal, options);

  result.command_executed = true;
  result.response = "경유지 이동 명령을 전송했습니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeCancelNavigation(const AiCommand &command)
{
  (void)command;
  if (navigate_to_pose_client_) {
    navigate_to_pose_client_->async_cancel_all_goals();
  }
  if (navigate_to_poses_client_) {
    navigate_to_poses_client_->async_cancel_all_goals();
  }
  publishEvent("cancel_navigation", "cancel requested");
  ExecutionResult result;
  result.command_type = AiCommandType::CancelNavigation;
  result.command_executed = true;
  result.response = "현재 navigation 목표 취소를 요청했습니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeSetInitialPose(const AiCommand &command)
{
  ExecutionResult result;
  result.command_type = AiCommandType::SetInitialPose;
  if (!command.has_pose) {
    result.accepted = false;
    result.error_message = "초기 위치 x/y/yaw 좌표를 찾지 못했습니다.";
    result.response = result.error_message;
    return result;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = command.frame_id.empty() ? config_.default_frame_id : command.frame_id;
  message.header.stamp = node_->now();
  message.pose.pose.position.x = command.x;
  message.pose.pose.position.y = command.y;
  message.pose.pose.position.z = command.z;
  message.pose.pose.orientation = quaternion_from_yaw(command.yaw);
  message.pose.covariance[0] = 0.25;
  message.pose.covariance[7] = 0.25;
  message.pose.covariance[35] = 0.06853891945200942;
  if (!config_.dry_run) {
    initial_pose_publisher_->publish(message);
  }
  result.command_executed = !config_.dry_run;
  result.response = config_.dry_run ? "dry_run 모드: initial pose publish를 생략했습니다." :
    "초기 위치를 publish했습니다.";
  publishEvent("set_initial_pose", result.response);
  return result;
}

ExecutionResult RosToolExecutor::executeSaveMap(const AiCommand &command)
{
  (void)command;
  ExecutionResult result;
  result.command_type = AiCommandType::SaveMap;
  result.response = "맵 저장 tool은 아직 placeholder입니다. map saver service 연결이 필요합니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeSendMotionCommand(const AiCommand &command)
{
  ExecutionResult result;
  result.command_type = AiCommandType::SendMotionCommand;
  if (!config_.enable_direct_cmd_vel) {
    result.response =
      "안전 정책상 직접 /cmd_vel 명령은 비활성화되어 있습니다. 목표 좌표 기반 navigation 명령을 사용하세요.";
    return result;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::clamp(command.linear_x, -config_.max_linear_speed, config_.max_linear_speed);
  twist.angular.z = std::clamp(command.angular_z, -config_.max_angular_speed, config_.max_angular_speed);
  if (!config_.dry_run) {
    cmd_vel_publisher_->publish(twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.cmd_vel_duration_ms));
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
  }
  result.command_executed = !config_.dry_run;
  result.response = config_.dry_run ? "dry_run 모드: /cmd_vel publish를 생략했습니다." :
    "제한된 시간 동안 안전 제한 속도 명령을 publish했습니다.";
  publishEvent("send_motion_command", result.response);
  return result;
}

ExecutionResult RosToolExecutor::executeRequestPlanSegment(const AiCommand &command)
{
  (void)command;
  ExecutionResult result;
  result.command_type = AiCommandType::RequestPlanSegment;
  result.response = "PlanSegment 요청 tool은 placeholder입니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeRequestPlanRoute(const AiCommand &command)
{
  (void)command;
  ExecutionResult result;
  result.command_type = AiCommandType::RequestPlanRoute;
  result.response = "PlanRoute 요청 tool은 placeholder입니다.";
  return result;
}

ExecutionResult RosToolExecutor::executeChangeRobotId(const AiCommand &command)
{
  ExecutionResult result;
  result.command_type = AiCommandType::ChangeRobotId;
  result.response = "활성 robot_id 변경 요청을 확인했습니다: " + command.robot_id;
  return result;
}

geometry_msgs::msg::PoseStamped RosToolExecutor::toPoseStamped(
  const PoseGoal &goal,
  const std::string &frame_id) const
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = frame_id.empty() ? config_.default_frame_id : frame_id;
  stamped.header.stamp = node_->now();
  stamped.pose.position.x = goal.x;
  stamped.pose.position.y = goal.y;
  stamped.pose.position.z = goal.z;
  stamped.pose.orientation = quaternion_from_yaw(goal.yaw);
  return stamped;
}

void RosToolExecutor::publishEvent(const std::string &event_type, const std::string &message) const
{
  if (!events_publisher_) {
    return;
  }
  std_msgs::msg::String event;
  event.data = "{\"event_type\":\"" + escapeJson(event_type) + "\",\"message\":\"" +
    escapeJson(message) + "\"}";
  events_publisher_->publish(event);
}

void RosToolExecutor::publishFeedback(const std::string &message) const
{
  if (!feedback_publisher_) {
    return;
  }
  std_msgs::msg::String feedback;
  feedback.data = message;
  feedback_publisher_->publish(feedback);
}

std::string RosToolExecutor::escapeJson(const std::string &text)
{
  std::string escaped;
  for (const char ch : text) {
    if (ch == '\\') {
      escaped += "\\\\";
    } else if (ch == '"') {
      escaped += "\\\"";
    } else if (ch == '\n') {
      escaped += "\\n";
    } else {
      escaped += ch;
    }
  }
  return escaped;
}

}  // namespace amr::mcp
