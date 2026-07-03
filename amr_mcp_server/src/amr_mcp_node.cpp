#include "amr_mcp_server/amr_mcp_node.hpp"

#include <sstream>

namespace amr::mcp
{

AmrMcpNode::AmrMcpNode(const rclcpp::NodeOptions &options)
: rclcpp::Node("amr_mcp_server", options)
{
  declareParameters();
  configureInterfaces();
}

void AmrMcpNode::declareParameters()
{
  default_provider_ = declare_parameter<std::string>("default_provider", default_provider_);
  default_robot_id_ = declare_parameter<std::string>("default_robot_id", default_robot_id_);
  default_frame_id_ = declare_parameter<std::string>("default_frame_id", default_frame_id_);
  chat_service_name_ = declare_parameter<std::string>("chat_service_name", chat_service_name_);
  events_topic_ = declare_parameter<std::string>("events_topic", events_topic_);
  feedback_topic_ = declare_parameter<std::string>("feedback_topic", feedback_topic_);
  battery_topic_ = declare_parameter<std::string>("battery_topic", battery_topic_);
  pose_topic_ = declare_parameter<std::string>("pose_topic", pose_topic_);
  motion_status_topic_ = declare_parameter<std::string>("motion_status_topic", motion_status_topic_);
  runtime_summary_topic_ = declare_parameter<std::string>("runtime_summary_topic", runtime_summary_topic_);
  runtime_event_topic_ = declare_parameter<std::string>("runtime_event_topic", runtime_event_topic_);
  rule_based_first_ = declare_parameter<bool>("rule_based_first", rule_based_first_);
  require_confirmation_for_motion_ =
    declare_parameter<bool>("require_confirmation_for_motion", require_confirmation_for_motion_);

  OllamaConfig ollama_config;
  ollama_config.base_url = declare_parameter<std::string>("ollama_base_url", ollama_config.base_url);
  ollama_config.chat_endpoint =
    declare_parameter<std::string>("ollama_chat_endpoint", ollama_config.chat_endpoint);
  ollama_config.model = declare_parameter<std::string>("ollama_model", ollama_config.model);
  ollama_config.timeout_ms = declare_parameter<int>("ollama_timeout_ms", ollama_config.timeout_ms);
  ollama_config.enabled = declare_parameter<bool>("ollama_enable", ollama_config.enabled);
  ollama_client_ = std::make_unique<OllamaClient>(ollama_config);
}

void AmrMcpNode::configureInterfaces()
{
  events_publisher_ = create_publisher<std_msgs::msg::String>(events_topic_, rclcpp::QoS(10));
  feedback_publisher_ = create_publisher<std_msgs::msg::String>(feedback_topic_, rclcpp::QoS(10));

  RosToolConfig tool_config;
  tool_config.navigate_to_pose_action =
    declare_parameter<std::string>("navigate_to_pose_action", tool_config.navigate_to_pose_action);
  tool_config.navigate_to_poses_action =
    declare_parameter<std::string>("navigate_to_poses_action", tool_config.navigate_to_poses_action);
  tool_config.initial_pose_topic =
    declare_parameter<std::string>("initial_pose_topic", tool_config.initial_pose_topic);
  tool_config.cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", tool_config.cmd_vel_topic);
  tool_config.default_frame_id = default_frame_id_;
  tool_config.enable_direct_cmd_vel =
    declare_parameter<bool>("enable_direct_cmd_vel", tool_config.enable_direct_cmd_vel);
  tool_config.max_linear_speed =
    declare_parameter<double>("max_linear_speed", tool_config.max_linear_speed);
  tool_config.max_angular_speed =
    declare_parameter<double>("max_angular_speed", tool_config.max_angular_speed);
  tool_config.cmd_vel_duration_ms =
    declare_parameter<int>("cmd_vel_duration_ms", tool_config.cmd_vel_duration_ms);
  tool_config.dry_run = declare_parameter<bool>("dry_run", tool_config.dry_run);
  tool_executor_ = std::make_unique<RosToolExecutor>(
    this, tool_config, events_publisher_, feedback_publisher_);

  const auto qos = rclcpp::SystemDefaultsQoS();
  battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, qos, [this](const sensor_msgs::msg::BatteryState::SharedPtr message) {
      snapshot_.updateBattery(*message, now());
    });
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      snapshot_.updatePose(*message, now());
    });
  motion_status_subscription_ = create_subscription<amr_msgs::msg::MotionStatus>(
    motion_status_topic_, qos, [this](const amr_msgs::msg::MotionStatus::SharedPtr message) {
      snapshot_.updateMotionStatus(*message, now());
    });
  runtime_summary_subscription_ = create_subscription<std_msgs::msg::String>(
    runtime_summary_topic_, qos, [this](const std_msgs::msg::String::SharedPtr message) {
      snapshot_.updateRuntimeSummary(*message, now());
    });
  runtime_event_subscription_ = create_subscription<std_msgs::msg::String>(
    runtime_event_topic_, qos, [this](const std_msgs::msg::String::SharedPtr message) {
      snapshot_.updateRuntimeEvent(*message, now());
    });

  chat_service_ = create_service<AiChat>(
    chat_service_name_,
    [this](
      const std::shared_ptr<AiChat::Request> request,
      std::shared_ptr<AiChat::Response> response) {
      handleChat(request, response);
    });

  RCLCPP_INFO(get_logger(), "AMR MCP server ready: %s", chat_service_name_.c_str());
}

void AmrMcpNode::handleChat(
  const std::shared_ptr<AiChat::Request> request,
  std::shared_ptr<AiChat::Response> response)
{
  const std::string request_id = nextRequestId();
  response->request_id = request_id;

  const std::string provider = request->provider.empty() ? default_provider_ : request->provider;
  const std::string robot_id = request->robot_id.empty() ? default_robot_id_ : request->robot_id;
  const std::string frame_id = request->default_frame.empty() ? default_frame_id_ : request->default_frame;
  const SnapshotData snapshot = snapshot_.data();
  AiCommand command = interpreter_.parse(request->message, robot_id, frame_id, rule_based_first_);

  publishEvent("chat.request", "request_id=" + request_id + ", command=" + to_string(command.type));

  ExecutionResult result;
  if (command.type == AiCommandType::ChatOnly || command.type == AiCommandType::Unknown ||
    command.requires_ollama)
  {
    std::string ollama_error;
    std::string answer = ollama_client_->ask(request->message, snapshot, &ollama_error);
    result.command_type = command.type;
    if (answer.empty()) {
      result.accepted = false;
      result.error_message = ollama_error;
      result.response = ollama_error.empty() ?
        "명령을 해석하지 못했습니다. 좌표나 명령을 더 구체적으로 입력해 주세요." :
        "Ollama 응답을 가져오지 못했습니다: " + ollama_error;
    } else {
      result.accepted = true;
      result.response = answer;
    }
  } else {
    result = tool_executor_->execute(command, snapshot);
  }

  response->accepted = result.accepted;
  response->command_executed = result.command_executed;
  response->command_type = to_string(result.command_type);
  response->response = result.response;
  response->error_message = result.error_message;

  publishEvent("chat.response", "request_id=" + request_id + ", accepted=" +
    std::string(result.accepted ? "true" : "false"));
}

std::string AmrMcpNode::nextRequestId()
{
  ++request_counter_;
  return "amr-mcp-" + std::to_string(request_counter_);
}

void AmrMcpNode::publishEvent(const std::string &event_type, const std::string &message)
{
  if (!events_publisher_) {
    return;
  }
  std_msgs::msg::String event;
  event.data = "{\"event_type\":\"" + escapeJson(event_type) + "\",\"message\":\"" +
    escapeJson(message) + "\"}";
  events_publisher_->publish(event);
}

std::string AmrMcpNode::escapeJson(const std::string &text)
{
  std::string escaped;
  escaped.reserve(text.size());
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
