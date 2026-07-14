#ifndef AMR_MCP_SERVER__AMR_MCP_NODE_HPP_
#define AMR_MCP_SERVER__AMR_MCP_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/ai_chat.hpp"
#include "amr_mcp_server/ollama_client.hpp"
#include "amr_mcp_server/prompt_interpreter.hpp"
#include "amr_mcp_server/ros_tool_executor.hpp"
#include "amr_mcp_server/topic_snapshot.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr::mcp
{

class AmrMcpNode : public rclcpp::Node
{
public:
  explicit AmrMcpNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using AiChat = amr_msgs::srv::AiChat;

  void declareParameters();
  void configureInterfaces();
  void handleChat(
    const std::shared_ptr<AiChat::Request> request,
    std::shared_ptr<AiChat::Response> response);
  std::string nextRequestId();
  void publishEvent(const std::string &event_type, const std::string &message);
  static std::string escapeJson(const std::string &text);

  PromptInterpreter interpreter_;
  TopicSnapshot snapshot_;
  std::unique_ptr<OllamaClient> ollama_client_;
  std::unique_ptr<RosToolExecutor> tool_executor_;

  rclcpp::Service<AiChat>::SharedPtr chat_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr runtime_summary_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr runtime_event_subscription_;

  std::string default_provider_{"ollama"};
  std::string default_robot_id_{"burger1"};
  std::string default_frame_id_{"map"};
  std::string chat_service_name_{"/mcp/chat"};
  std::string events_topic_{"/mcp/events"};
  std::string feedback_topic_{"/mcp/feedback"};
  std::string battery_topic_{"/battery_state"};
  std::string pose_topic_{"/pose"};
  std::string motion_status_topic_{"/motion_status"};
  std::string runtime_summary_topic_{"/observation/runtime/summary"};
  std::string runtime_event_topic_{"/observation/runtime/events"};
  bool rule_based_first_{true};
  bool require_confirmation_for_motion_{false};
  std::uint64_t request_counter_{0};
};

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__AMR_MCP_NODE_HPP_
