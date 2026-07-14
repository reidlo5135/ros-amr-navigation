# amr_mcp_server

`amr_mcp_server` is a ROS 2 Humble C++ Mission/Command Processor for AMR operator AI workflows.
It sits between AMR Visualization and the ROS graph, interprets operator chat requests, executes
safe structured tools, and optionally asks Ollama for explanatory answers.

## Difference From ros-mcp-server

This package does not implement the old TypeScript/WebSocket/MQTT bridge. It is a ROS-native
`rclcpp` node that talks directly to topics, actions, and services inside this AMR stack.

- No WebSocket, MQTT, Node.js, or TypeScript runtime.
- Rule-based command parsing runs before LLM fallback.
- Robot execution only happens through structured C++ tool methods.
- Direct `/cmd_vel` control is disabled by default and safety-limited when enabled.
- Missing topic data is reported as missing instead of being invented.

## Architecture

```text
AMR Viz AI Chat Panel
  -> /mcp/chat (amr_msgs/srv/AiChat)
    -> AmrMcpNode
      -> PromptInterpreter
      -> RosToolExecutor
      -> TopicSnapshot
      -> OllamaClient (fallback/explanation only)
```

## Interfaces

Service:

- `/mcp/chat` (`amr_msgs/srv/AiChat`)

Publishers:

- `/mcp/events` (`std_msgs/msg/String`)
- `/mcp/feedback` (`std_msgs/msg/String`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`, safety-gated)
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)

Action clients:

- `/navigate_to_pose` (`amr_msgs/action/NavigateToPose`)
- `/navigate_to_poses` (`amr_msgs/action/NavigateToPoses`)

Snapshot subscriptions:

- `/battery_state`
- `/pose`
- `/motion_status`
- `/observation/runtime/summary`
- `/observation/runtime/events`

## Parameters

Default parameters live in `config/amr_mcp_server.yaml`. Key values:

- `default_provider`: `ollama`
- `default_robot_id`: `burger1`
- `default_frame_id`: `map`
- `ollama_base_url`: `http://127.0.0.1:11434`
- `ollama_model`: `qwen3`
- `chat_service_name`: `/mcp/chat`
- `enable_direct_cmd_vel`: `false`
- `max_linear_speed`: `0.12`
- `max_angular_speed`: `0.8`
- `dry_run`: `false`

## Launch

```bash
ros2 launch amr_mcp_server amr_mcp_server.launch.py
```

AMR Visualization can optionally launch it too:

```bash
ros2 launch amr_visualization amr_visualization.launch.py with_mcp_server:=true
```

## Service Examples

```bash
ros2 service call /mcp/chat amr_msgs/srv/AiChat "{provider: 'ollama', robot_id: 'burger1', default_frame: 'map', message: '현재 상태 요약해줘'}"
```

```bash
ros2 service call /mcp/chat amr_msgs/srv/AiChat "{provider: 'ollama', robot_id: 'burger1', default_frame: 'map', message: '현재 목표 취소해'}"
```

```bash
ros2 service call /mcp/chat amr_msgs/srv/AiChat "{provider: 'ollama', robot_id: 'burger1', default_frame: 'map', message: 'burger1을 map 기준 x=1.0 y=0.5 yaw=0.0으로 보내줘'}"
```

## Ollama Setup

Run Ollama separately if fallback or explanatory responses are needed:

```bash
ollama serve
ollama pull qwen3
```

If Ollama is down, deterministic commands such as status, cancel, initial pose, and coordinate
navigation still return structured responses. Chat-only fallback reports the connection error.

## Safety Policy

- Navigation actions are preferred over direct velocity commands.
- Direct `/cmd_vel` is disabled unless `enable_direct_cmd_vel` is explicitly set true.
- When direct velocity is enabled, linear and angular speeds are clamped and a zero Twist is
  published after a short duration.
- Unknown or ambiguous requests are not executed as robot commands.

## Build/Test

```bash
colcon build --packages-select amr_msgs amr_mcp_server amr_visualization
```

This README lists the intended validation commands. The package itself should also be tested with
Ollama stopped to confirm that the node stays alive and returns a clear error for chat fallback.
