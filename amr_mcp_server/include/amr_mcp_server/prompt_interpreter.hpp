#ifndef AMR_MCP_SERVER__PROMPT_INTERPRETER_HPP_
#define AMR_MCP_SERVER__PROMPT_INTERPRETER_HPP_

#include <string>
#include <vector>

#include "amr_mcp_server/command_types.hpp"

namespace amr::mcp
{

class PromptInterpreter
{
public:
  AiCommand parse(
    const std::string &message,
    const std::string &robot_id,
    const std::string &frame_id,
    bool rule_based_first) const;

private:
  static std::string lower(const std::string &text);
  static bool containsAny(const std::string &text, const std::initializer_list<const char *> &needles);
  static std::vector<PoseGoal> extractPoseGoals(const std::string &message);
};

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__PROMPT_INTERPRETER_HPP_
