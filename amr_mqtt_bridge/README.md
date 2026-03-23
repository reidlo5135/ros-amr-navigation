# amr_mqtt_bridge

`amr_mqtt_bridge` is the VBox/server-side MQTT <-> ROS mirror for the AMR
stack.

Its job is to receive MQTT traffic produced by the robot, reconstruct a local
VBox ROS graph for RViz and operator tools, and emit selected MQTT commands
from VBox-side ROS topics. It also publishes modeled JSON telemetry for the
web client on `amr/viz/telemetry/*`.

## Placement

- VBox Ubuntu server
  - `mosquitto`
  - `amr_mqtt_bridge`
  - `rviz2` or `amr_viz`

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

## MQTT -> ROS Mirror

Navigation telemetry reconstructed into VBox ROS:

- `amr/telemetry/robot_pose` -> `/amr/localization/pose`
- `amr/telemetry/global_path` -> `/amr/planner/global`
- `amr/telemetry/local_path` -> `/amr/planner/local`
- `amr/robot/turtlebot3/telemetry/map` -> `/amr/map/data`
- `amr/telemetry/global_costmap` -> `/amr/costmap/global`
- `amr/telemetry/local_costmap` -> `/amr/costmap/local`
- `amr/telemetry/motion_status` -> `/amr/motion/status`
- `amr/telemetry/obstacle_report` -> `/amr/obstacle/report`

Robot telemetry reconstructed into VBox ROS:

- `amr/robot/turtlebot3/telemetry/scan` -> `/scan`
- `amr/robot/turtlebot3/telemetry/odom` -> `/odom`
- `amr/robot/turtlebot3/telemetry/imu` -> `/imu`
- `amr/robot/turtlebot3/telemetry/tf` -> `/tf`
- `amr/robot/turtlebot3/telemetry/tf_static` -> `/tf_static`
- `amr/robot/turtlebot3/telemetry/joint_states` -> `/joint_states`
- `amr/robot/turtlebot3/telemetry/robot_description` -> `/robot_description`

Mirrored action monitoring:

- `amr/feedback/navigate_to_pose` -> `/amr/navigator/navigate_to_pose/feedback`
- `amr/status/navigate_to_pose` -> `/amr/navigator/navigate_to_pose/status`

## MQTT -> MQTT Modeled Viz Topics

For `amr_viz`, this package also emits browser-friendly JSON topics:

- `amr/viz/telemetry/robot_pose`
- `amr/viz/telemetry/global_path`
- `amr/viz/telemetry/local_path`
- `amr/viz/telemetry/map`
- `amr/viz/telemetry/global_costmap`
- `amr/viz/telemetry/local_costmap`
- `amr/viz/telemetry/motion_status`
- `amr/viz/telemetry/obstacle_report`

## ROS -> MQTT Emission

VBox-side ROS inputs forwarded back into MQTT:

- `/cmd_vel` -> `amr/robot/turtlebot3/command/cmd_vel`
- `/amr/localization/initial_pose` -> `amr/command/set_initial_pose`
- `/amr/rviz/goal` -> `amr/command/navigate_to_pose`

## Command Ownership

- service/action execution ownership is on the robot side in
  `amr_mqtt_robot_plugin`
- this package mirrors telemetry and emits commands
- MQTT ACK responses remain MQTT-side topics rather than full ROS result replay

## Transport Notes

- raw mirrored topics use ROS serialized binary payloads
- `map`, `costmap`, `tf_static`, and `robot_description` need late-subscriber
  durability on the VBox ROS side
- RViz should be configured with matching durability for latched-style topics

## Launch

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py
```

## Notes

- this package is VBox/server-side only
- robot-side mirroring belongs in `amr_mqtt_robot_plugin`
- topic names and ROS interface names should be tuned through `amr.yaml`
- source layout is split by responsibility:
  - `src/amr_mqtt_bridge/node.c`
  - `src/amr_mqtt_bridge/mqtt.c`
  - `include/amr_mqtt_bridge/node.h`
  - `include/amr_mqtt_bridge/mqtt.h`
