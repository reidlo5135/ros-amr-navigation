# amr_mqtt_bridge

Robot-side ROS 2 <-> MQTT bridge package.

This package runs inside the robot runtime and exposes the AMR stack over MQTT for external operators such as `ros-rcs`.

## Role

- subscribes to on-robot ROS topics and republishes them to MQTT
- publishes raw ROS telemetry for machine consumers
- publishes JSON viz topics for desktop or web clients
- consumes MQTT commands and translates them into ROS publishers, services, and actions
- scopes every MQTT topic by `robot_id`

## Parameter Section

Use the central parameter block in [amr.yaml](/home/reidlo/ws/src/ros-amr-navigation/amr_bringup/params/amr.yaml):

```yaml
/amr/mqtt_bridge:
```

Important defaults in the current tree:

- MQTT root: `/amr`
- `robot_id`: `burger1`
- broker host: `192.168.61.35`
- broker port: `1883`
- MQTT QoS: `0` for telemetry, command, and service traffic

## Topic Naming

All MQTT topics are built as:

```text
<mqtt_root>/<robot_id>/<suffix>
```

With the current defaults:

```text
/amr/burger1/<suffix>
```

Examples:

- `/amr/burger1/telemetry/map`
- `/amr/burger1/viz/robot_pose`
- `/amr/burger1/command/navigate_to_poses`
- `/amr/burger1/response/navigate_to_poses`

## Payload Conventions

### Raw telemetry / raw feedback / raw status

- Payload type: ROS 2 serialized binary payload produced by `rmw_serialize(...)`
- Intended consumer: ROS-aware or native clients that know the exact message type
- These are not JSON

### Viz topics and response topics

- Payload type: UTF-8 JSON
- Intended consumer: desktop/web/operator clients

### Commands

- Most commands accept UTF-8 JSON
- `command/cmd_vel` additionally accepts raw serialized `geometry_msgs/msg/Twist`

## MQTT Topic Catalogue

### Raw Telemetry Topics

Each topic below carries a raw serialized ROS message payload.

| MQTT suffix | ROS source | ROS type |
| --- | --- | --- |
| `telemetry/map` | `/amr/map/data` | `nav_msgs/msg/OccupancyGrid` |
| `telemetry/robot_pose` | `/amr/localization/pose` | `geometry_msgs/msg/PoseStamped` |
| `telemetry/global_path` | `/amr/planner/global` | `nav_msgs/msg/Path` |
| `telemetry/local_path` | `/amr/planner/local` | `nav_msgs/msg/Path` |
| `telemetry/global_costmap` | `/amr/costmap/global` | `nav_msgs/msg/OccupancyGrid` |
| `telemetry/local_costmap` | `/amr/costmap/local` | `nav_msgs/msg/OccupancyGrid` |
| `telemetry/motion_status` | `/amr/motion/status` | `amr_msgs/msg/MotionStatus` |
| `telemetry/scan` | `/scan` | `sensor_msgs/msg/LaserScan` |
| `telemetry/odom` | `/odom` | `nav_msgs/msg/Odometry` |
| `telemetry/imu` | `/imu` | `sensor_msgs/msg/Imu` |
| `telemetry/tf` | `/tf` | `tf2_msgs/msg/TFMessage` |
| `telemetry/tf_static` | `/tf_static` | `tf2_msgs/msg/TFMessage` |
| `telemetry/joint_states` | `/joint_states` | `sensor_msgs/msg/JointState` |
| `telemetry/robot_description` | `/robot_description` | `std_msgs/msg/String` |
| `telemetry/battery_state` | `/battery_state` | `sensor_msgs/msg/BatteryState` |
| `telemetry/temp_map` | `/slam/map/temp/refined` | `nav_msgs/msg/OccupancyGrid` |
| `telemetry/temp_map_raw` | `/slam/map/temp/raw` | `nav_msgs/msg/OccupancyGrid` |
| `telemetry/mapping_pose` | `/slam/mapper/pose` | `geometry_msgs/msg/PoseStamped` |
| `telemetry/slam_graph` | `/slam/mapper/graph_debug` | `std_msgs/msg/String` |

### JSON Viz Topics

These are JSON mirrors intended for operator clients. They are published under `/viz/` instead of `/telemetry/`.

| MQTT suffix | Source ROS topic | JSON shape summary |
| --- | --- | --- |
| `viz/map` | `/amr/map/data` | `header`, `info`, full `data[]` |
| `viz/robot_pose` | `/amr/localization/pose` | `header`, `position`, `orientation`, `yaw` |
| `viz/global_path` | `/amr/planner/global` | `header`, `pose_count`, `poses[]` |
| `viz/local_path` | `/amr/planner/local` | `header`, `pose_count`, `poses[]` |
| `viz/global_costmap` | `/amr/costmap/global` | `header`, `info`, full `data[]` |
| `viz/local_costmap` | `/amr/costmap/local` | `header`, `info`, full `data[]` |
| `viz/motion_status` | `/amr/motion/status` | motion flags, current pose, blocked pose |
| `viz/scan` | `/scan` | angles, range limits, `ranges[]` |
| `viz/tf` | `/tf` | `transforms[]` |
| `viz/tf_static` | `/tf_static` | `transforms[]` |
| `viz/robot_description` | `/robot_description` | `{ "data": "...", "footprint_polygon": [...] }` |
| `viz/battery_state` | `/battery_state` | voltage, current, percentage, supply state fields |
| `viz/temp_map` | `/slam/map/temp/refined` | `header`, `info`, full `data[]` |
| `viz/temp_map_raw` | `/slam/map/temp/raw` | `header`, `info`, full `data[]` |
| `viz/mapping_pose` | `/slam/mapper/pose` | `header`, `position`, `orientation`, `yaw` |
| `viz/slam_graph` | `/slam/mapper/graph_debug` | raw JSON string payload as-is |

The current implementation does not publish JSON viz mirrors for `odom`, `imu`, or `joint_states`, even though raw telemetry exists for them.

### Navigation Topics

The bridge exposes route execution through `NavigateToPoses`.

#### Raw action transport

| MQTT suffix | Payload type |
| --- | --- |
| `feedback/navigate_to_poses` | serialized `amr_msgs/action/NavigateToPoses_FeedbackMessage` |
| `status/navigate_to_poses` | serialized `action_msgs/msg/GoalStatusArray` |

#### JSON navigation topics

| MQTT suffix | Payload type |
| --- | --- |
| `viz/navigate_to_poses/feedback` | JSON feedback summary |
| `viz/navigate_to_poses/status` | JSON goal status summary |
| `viz/navigate_to_poses/response` | JSON final result mirror |
| `response/navigate_to_poses` | JSON response/result |

## Command / Request Protocol

### `command/cmd_vel`

Publishes to ROS topic `/cmd_vel`.

Accepted payloads:

1. JSON

```json
{
  "linear": { "x": 0.1, "y": 0.0, "z": 0.0 },
  "angular": { "x": 0.0, "y": 0.0, "z": 0.3 }
}
```

2. Flat JSON

```json
{
  "linear_x": 0.1,
  "angular_z": 0.3
}
```

3. Raw serialized `geometry_msgs/msg/Twist`

No response topic is emitted for `cmd_vel`.

### `command/set_initial_pose`

Publishes a `geometry_msgs/msg/PoseWithCovarianceStamped` to `/amr/localization/initial_pose`.

Required fields:

- `request_id`
- `x`
- `y`
- `yaw`

Optional fields:

- `frame_id` default: `"map"`
- `covariance_x` default: `0.25`
- `covariance_y` default: `0.25`
- `covariance_yaw` default: `0.06853891945200942`

Example:

```json
{
  "request_id": "init-001",
  "frame_id": "map",
  "x": 1.2,
  "y": -0.5,
  "yaw": 1.57,
  "covariance_x": 0.1,
  "covariance_y": 0.1,
  "covariance_yaw": 0.05
}
```

Response topic:

- `response/set_initial_pose`

Simple response schema:

```json
{
  "request_id": "init-001",
  "success": true,
  "message": "initial pose published"
}
```

### `command/navigate_to_poses`

Dispatches the ROS action `/amr/navigator/navigate_to_poses`.

Required fields:

- `request_id`
- `goal_poses`

Rules:

- `goal_poses` must be a non-empty array
- single-goal navigation also uses this API with a one-element array
- if another route is already active, the bridge rejects the request

Example:

```json
{
  "request_id": "route-001",
  "goal_poses": [
    {
      "header": { "frame_id": "map" },
      "pose": {
        "position": { "x": 1.0, "y": 0.5, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
      }
    },
    {
      "header": { "frame_id": "map" },
      "pose": {
        "position": { "x": 2.0, "y": 1.2, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071 }
      }
    }
  ]
}
```

The command handler also accepts an inline cancel form:

```json
{
  "request_id": "route-001-cancel",
  "cancel": true
}
```

Prefer the dedicated cancel topic below for normal client behavior.

### `command/cancel_navigate_to_poses`

Cancels the currently active route goal.

Example:

```json
{
  "request_id": "route-001-cancel"
}
```

### `command/save_map`

Writes the current temporary SLAM map from `/slam/map/temp/refined` to disk as:

- `<save_directory>/<basename>.pgm`
- `<save_directory>/<basename>.yaml`

Required fields:

- `request_id`
- `basename`

Restrictions:

- `basename` must contain only letters, digits, `_`, `-`, or `.`
- the temporary map must already be available in memory

Example:

```json
{
  "request_id": "map-save-001",
  "basename": "warehouse_a"
}
```

Response topic:

- `response/save_map`

### `command/ping`

Health-check / latency probe.

Example:

```json
{
  "request_id": "ping-001",
  "sent_at_ms": 1710000000.123
}
```

Response topic:

- `response/ping`

Response schema:

```json
{
  "request_id": "ping-001",
  "success": true,
  "sent_at_ms": 1710000000.123,
  "bridge_time_ms": 1710000000,
  "message": "pong"
}
```

### `command/set_robot_id`

Requests a topic scope switch from the current `robot_id` to another value.

Required fields:

- `request_id`
- `robot_id`

Rules:

- `robot_id` may contain only letters, digits, `_`, or `-`
- the bridge first replies on the current scope, then reconnects using the new scope

Example:

```json
{
  "request_id": "robot-scope-001",
  "robot_id": "burger2"
}
```

Response topic:

- `response/set_robot_id`

### `request/plan_segment`

Dispatch-only request to ROS service `/amr/global_planner/plan_segment`.

Required fields:

- `request_id`
- `start`
- `goal`

Example:

```json
{
  "request_id": "segment-001",
  "start": {
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  },
  "goal": {
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 2.0, "y": 1.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  }
}
```

Response topic:

- `response/plan_segment`

Current behavior:

- this topic only returns dispatch/validation success or failure
- it does not publish the actual planned path result yet

### `request/plan_route`

Dispatch-only request to ROS service `/amr/global_planner/plan_route`.

Required fields:

- `request_id`
- `start`
- `waypoints`

Example:

```json
{
  "request_id": "route-plan-001",
  "start": {
    "header": { "frame_id": "map" },
    "pose": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  },
  "waypoints": [
    {
      "header": { "frame_id": "map" },
      "pose": {
        "position": { "x": 1.0, "y": 0.5, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
      }
    },
    {
      "header": { "frame_id": "map" },
      "pose": {
        "position": { "x": 2.0, "y": 1.5, "z": 0.0 },
        "orientation": { "x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071 }
      }
    }
  ]
}
```

Response topic:

- `response/plan_route`

Current behavior:

- this topic only returns dispatch/validation success or failure
- it does not publish the actual planned route result yet

## Navigation JSON Schemas

### `viz/navigate_to_poses/feedback`

Published continuously while a route is active.

Schema:

```json
{
  "goal_id": "0123456789abcdef0123456789abcdef",
  "current_goal_index": 0,
  "goal_count": 2,
  "distance_remaining": 3.42,
  "number_of_recoveries": 1,
  "navigation_time": { "sec": 12, "nanosec": 500000000 },
  "estimated_time_remaining": { "sec": 8, "nanosec": 0 },
  "current_pose": {
    "header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "map" },
    "position": { "x": 1.0, "y": 0.5, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "yaw": 0.0 }
  }
}
```

### `viz/navigate_to_poses/status`

Goal status mirror generated from `action_msgs/msg/GoalStatusArray`.

Schema:

```json
{
  "status_list": [
    {
      "goal_id": "0123456789abcdef0123456789abcdef",
      "status": 2
    }
  ]
}
```

### `response/navigate_to_poses` and `viz/navigate_to_poses/response`

Final route result schema:

```json
{
  "request_id": "route-001",
  "success": true,
  "accepted": true,
  "completed": true,
  "status_code": 4,
  "completed_goals": 2,
  "message": "navigation succeeded"
}
```

Notes:

- `accepted` reports whether the action server accepted the goal
- `completed` reports whether the bridge has received a terminal action result
- `status_code` is the ROS action result status code
- `completed_goals` is the number of route goals actually completed before termination

## Common Response Schema

Most non-navigation responses use:

```json
{
  "request_id": "req-001",
  "success": true,
  "message": "..."
}
```

This applies to:

- `response/set_initial_pose`
- `response/save_map`
- `response/set_robot_id`
- `response/plan_segment`
- `response/plan_route`

## JSON Field Shapes Reused Across Topics

### Pose-stamped JSON

```json
{
  "header": {
    "stamp": { "sec": 0, "nanosec": 0 },
    "frame_id": "map"
  },
  "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "orientation": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "w": 1.0,
    "yaw": 0.0
  }
}
```

### Path JSON

```json
{
  "header": { "...": "..." },
  "pose_count": 3,
  "poses": [ /* pose-stamped objects */ ]
}
```

### Occupancy grid JSON

```json
{
  "header": { "...": "..." },
  "info": {
    "width": 100,
    "height": 100,
    "resolution": 0.05,
    "origin": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "yaw": 0.0 }
    }
  },
  "data": [0, 0, 100, -1]
}
```

### Motion status JSON

Fields include:

- `command_id`
- `active`
- `goal_reached`
- `obstacle_detected`
- `blocked`
- `stalled`
- `local_plan_valid`
- `costmap_blocked`
- `safety_gate_blocked`
- `has_blocked_pose`
- `remaining_distance`
- `heading_error`
- `current_pose`
- `blocked_pose`

## Current Limitations

- `request/plan_segment` and `request/plan_route` currently return only dispatch acknowledgements, not full planning results
- `command/cmd_vel` is fire-and-forget and has no response topic
- all MQTT QoS values are currently configured as `0`
- the bridge exposes navigation over `NavigateToPoses` only; single-goal clients must still send a one-element `goal_poses` array

## Launch

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py params_file:=/path/to/amr.yaml
```
