#ifndef AMR_MCP_SERVER__COMMAND_TYPES_HPP_
#define AMR_MCP_SERVER__COMMAND_TYPES_HPP_

#include <string>
#include <vector>

namespace amr::mcp
{

enum class AiCommandType
{
  ChatOnly,
  RequestStatus,
  NavigateToPose,
  NavigateRoute,
  CancelNavigation,
  SetInitialPose,
  SaveMap,
  SendMotionCommand,
  RequestPlanSegment,
  RequestPlanRoute,
  ChangeRobotId,
  Unknown
};

struct PoseGoal
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool has_yaw{false};
};

struct AiCommand
{
  AiCommandType type{AiCommandType::Unknown};
  std::string robot_id;
  std::string frame_id{"map"};
  std::string raw_message;
  std::string response_hint;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  bool has_pose{false};
  std::vector<PoseGoal> goals;
  double linear_x{0.0};
  double angular_z{0.0};
  bool requires_ollama{false};
  bool requires_confirmation{false};
};

struct ExecutionResult
{
  bool accepted{true};
  bool command_executed{false};
  AiCommandType command_type{AiCommandType::Unknown};
  std::string response;
  std::string error_message;
};

std::string to_string(AiCommandType type);

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__COMMAND_TYPES_HPP_
