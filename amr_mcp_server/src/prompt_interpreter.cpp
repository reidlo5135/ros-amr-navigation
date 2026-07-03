#include "amr_mcp_server/prompt_interpreter.hpp"

#include <algorithm>
#include <cctype>
#include <regex>

namespace amr::mcp
{

std::string to_string(AiCommandType type)
{
  switch (type) {
    case AiCommandType::ChatOnly:
      return "ChatOnly";
    case AiCommandType::RequestStatus:
      return "RequestStatus";
    case AiCommandType::NavigateToPose:
      return "NavigateToPose";
    case AiCommandType::NavigateRoute:
      return "NavigateRoute";
    case AiCommandType::CancelNavigation:
      return "CancelNavigation";
    case AiCommandType::SetInitialPose:
      return "SetInitialPose";
    case AiCommandType::SaveMap:
      return "SaveMap";
    case AiCommandType::SendMotionCommand:
      return "SendMotionCommand";
    case AiCommandType::RequestPlanSegment:
      return "RequestPlanSegment";
    case AiCommandType::RequestPlanRoute:
      return "RequestPlanRoute";
    case AiCommandType::ChangeRobotId:
      return "ChangeRobotId";
    case AiCommandType::Unknown:
      return "Unknown";
  }
  return "Unknown";
}

AiCommand PromptInterpreter::parse(
  const std::string &message,
  const std::string &robot_id,
  const std::string &frame_id,
  bool rule_based_first) const
{
  AiCommand command;
  command.robot_id = robot_id;
  command.frame_id = frame_id.empty() ? "map" : frame_id;
  command.raw_message = message;

  if (!rule_based_first) {
    command.type = AiCommandType::ChatOnly;
    command.requires_ollama = true;
    return command;
  }

  const std::string text = lower(message);
  const auto goals = extractPoseGoals(message);

  if (containsAny(text, {"취소", "멈춰", "중지", "cancel", "stop navigation", "목표 취소"})) {
    command.type = AiCommandType::CancelNavigation;
    command.response_hint = "현재 navigation goal 취소를 요청합니다.";
    return command;
  }

  if (containsAny(text, {"맵 저장", "save map"})) {
    command.type = AiCommandType::SaveMap;
    return command;
  }

  if (containsAny(text, {"초기 위치", "initial pose", "init pose"})) {
    command.type = AiCommandType::SetInitialPose;
    if (!goals.empty()) {
      command.x = goals.front().x;
      command.y = goals.front().y;
      command.z = goals.front().z;
      command.yaw = goals.front().yaw;
      command.has_pose = true;
    }
    return command;
  }

  if (containsAny(text, {"경유", "waypoint", "route"}) && goals.size() > 1U) {
    command.type = AiCommandType::NavigateRoute;
    command.goals = goals;
    command.has_pose = true;
    return command;
  }

  if (!goals.empty() && containsAny(text, {"보내", "목표", "이동", "navigate", "go to", "x="})) {
    command.type = AiCommandType::NavigateToPose;
    command.x = goals.front().x;
    command.y = goals.front().y;
    command.z = goals.front().z;
    command.yaw = goals.front().yaw;
    command.has_pose = true;
    return command;
  }

  if (containsAny(text, {"앞으로", "전진", "뒤로", "후진", "좌회전", "우회전"})) {
    command.type = AiCommandType::SendMotionCommand;
    if (containsAny(text, {"앞으로", "전진"})) {
      command.linear_x = 0.08;
    } else if (containsAny(text, {"뒤로", "후진"})) {
      command.linear_x = -0.08;
    } else if (containsAny(text, {"좌회전"})) {
      command.angular_z = 0.5;
    } else if (containsAny(text, {"우회전"})) {
      command.angular_z = -0.5;
    }
    return command;
  }

  if (containsAny(text, {"상태", "요약", "배터리", "현재", "어때", "막힘", "장애물"})) {
    command.type = AiCommandType::RequestStatus;
    return command;
  }

  command.type = AiCommandType::ChatOnly;
  command.requires_ollama = true;
  return command;
}

std::string PromptInterpreter::lower(const std::string &text)
{
  std::string lowered = text;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return lowered;
}

bool PromptInterpreter::containsAny(
  const std::string &text,
  const std::initializer_list<const char *> &needles)
{
  for (const char *needle : needles) {
    if (text.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::vector<PoseGoal> PromptInterpreter::extractPoseGoals(const std::string &message)
{
  std::vector<PoseGoal> goals;
  const std::regex xy_pattern(
    R"((?:x\s*=\s*)?([-+]?\d+(?:\.\d+)?)\s*[, ]+\s*(?:y\s*=\s*)?([-+]?\d+(?:\.\d+)?)(?:\s*[, ]+\s*(?:yaw\s*=\s*)?([-+]?\d+(?:\.\d+)?))?)",
    std::regex::icase);
  const std::regex keyed_pattern(
    R"(x\s*=\s*([-+]?\d+(?:\.\d+)?)[,\s]+y\s*=\s*([-+]?\d+(?:\.\d+)?)(?:[,\s]+yaw\s*=\s*([-+]?\d+(?:\.\d+)?))?)",
    std::regex::icase);

  auto collect = [&goals](const std::smatch &match) {
      PoseGoal goal;
      goal.x = std::stod(match[1].str());
      goal.y = std::stod(match[2].str());
      if (match.size() > 3U && match[3].matched) {
        goal.yaw = std::stod(match[3].str());
        goal.has_yaw = true;
      }
      goals.push_back(goal);
    };

  for (auto it = std::sregex_iterator(message.begin(), message.end(), keyed_pattern);
    it != std::sregex_iterator(); ++it)
  {
    collect(*it);
  }

  if (goals.empty() && message.find("x=") == std::string::npos) {
    for (auto it = std::sregex_iterator(message.begin(), message.end(), xy_pattern);
      it != std::sregex_iterator(); ++it)
    {
      collect(*it);
    }
  }

  return goals;
}

}  // namespace amr::mcp
