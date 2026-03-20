# amr_mqtt_bridge

`amr_mqtt_bridge` is the navigation-side ROS <-> MQTT transport boundary for
the AMR stack.

It keeps ROS topics, services, and actions inside the navigation runtime and
exports only modeled AMR data over MQTT.

## Runtime Role

- ROS runtime side
  - `rclc`
- MQTT side
  - Eclipse Paho C client via `libpaho-mqtt-dev`
- typical deployment
  - `amr_navigation`
  - `amr_mqtt_bridge`
  - `mosquitto`
  - same VBox Ubuntu server

## Requirements

```bash
sudo apt install -y libpaho-mqtt-dev
sudo apt install -y mosquitto mosquitto-clients
```

Expected ROS environment:

- ROS 2 Humble
- `rclc`

## Parameters

Shared runtime mapping lives in:

- [`amr_bringup/params/amr.yaml`](/home/reidlo/ws/src/ros-amr-navigation/amr_bringup/params/amr.yaml)

Relevant section:

- `/amr/mqtt_bridge`

Tunable groups:

- broker host, port, client id, keepalive, auth
- MQTT telemetry/command/request/response topic names
- ROS source topic names
- ROS service names
- ROS action names

## Current ROS Inputs

Telemetry sources:

- `/amr/localization/pose`
- `/amr/planner/global`
- `/amr/planner/local`
- `/amr/map/data`
- `/amr/costmap/global`
- `/amr/costmap/local`
- `/amr/motion/status`
- `/amr/obstacle/report`
- `/cmd_vel`

Robot ingress republish targets:

- `/scan`
- `/odom`
- `/imu`
- `/tf`
- `/tf_static`
- `/joint_states`

Command / request targets:

- `/amr/localization/initial_pose`
- `/amr/navigator/navigate_to_pose`
- `/amr/global_planner/plan_segment`
- `/amr/global_planner/plan_route`

## Current MQTT Interface

Telemetry:

- `amr/telemetry/robot_pose`
- `amr/telemetry/global_path`
- `amr/telemetry/local_path`
- `amr/telemetry/map`
- `amr/telemetry/global_costmap`
- `amr/telemetry/local_costmap`
- `amr/telemetry/motion_status`
- `amr/telemetry/obstacle_report`

Commands / feedback / responses:

- `amr/robot/turtlebot3/command/cmd_vel`
- `amr/command/set_initial_pose`
- `amr/command/navigate_to_pose`
- `amr/request/plan_segment`
- `amr/request/plan_route`
- `amr/response/set_initial_pose`
- `amr/response/navigate_to_pose`
- `amr/feedback/navigate_to_pose`
- `amr/response/plan_segment`
- `amr/response/plan_route`

## Supported Behaviors

- ROS topic -> MQTT telemetry publish
- ROS `/cmd_vel` -> MQTT robot command publish
- MQTT robot telemetry -> ROS republish for robot bringup topics
- MQTT command -> ROS initial pose publish
- MQTT command -> ROS action goal for `navigate_to_pose`
- ROS action feedback/result -> MQTT feedback/response
- MQTT request -> ROS service call for `plan_segment`
- MQTT request -> ROS service call for `plan_route`

## Payload Examples

`set_initial_pose`

```json
{
  "request_id": "req-001",
  "frame_id": "map",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "covariance_x": 0.25,
  "covariance_y": 0.25,
  "covariance_yaw": 0.06853891945200942
}
```

`navigate_to_pose`

```json
{
  "request_id": "req-002",
  "goal_pose": {
    "header": { "frame_id": "map" },
    "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

`plan_segment`

```json
{
  "request_id": "req-003",
  "start": {
    "header": { "frame_id": "map" },
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  },
  "goal": {
    "header": { "frame_id": "map" },
    "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

`plan_route`

```json
{
  "request_id": "req-004",
  "start": {
    "header": { "frame_id": "map" },
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  },
  "waypoints": [
    {
      "header": { "frame_id": "map" },
      "position": { "x": 1.0, "y": 1.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    }
  ]
}
```

## Launch

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py
```

## Notes

- this package is the navigation-side MQTT boundary
- robot bringup transport belongs in `amr_mqtt_robot_plugin`
- MQTT topic names and ROS interface names should be tuned through `amr.yaml`
