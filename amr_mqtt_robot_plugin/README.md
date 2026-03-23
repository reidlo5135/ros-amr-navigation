# amr_mqtt_robot_plugin

`amr_mqtt_robot_plugin` is the robot-side ROS <-> MQTT mirror for TurtleBot3
bringup and the AMR navigation runtime.

It keeps the high-rate ROS graph local to the robot, mirrors selected topics to
the VBox MQTT broker, and receives MQTT commands back into local ROS topics,
services, and actions.

## Placement

- TurtleBot3 / robot
  - `turtlebot3_bringup`
  - `localization.launch.py`
  - `navigation.launch.py`
  - `amr_mqtt_robot_plugin`

## Requirements

```bash
sudo apt install -y libpaho-mqtt-dev
```

Expected ROS environment:

- ROS 2 Humble
- `rclc`

## Parameters

Shared runtime mapping lives in:

- [`amr_bringup/params/amr.yaml`](/home/reidlo/ws/src/ros-amr-navigation/amr_bringup/params/amr.yaml)

Relevant section:

- `/amr/mqtt_robot_plugin`

## ROS -> MQTT Mirror

Navigation telemetry:

- `/amr/map/data` -> `amr/robot/turtlebot3/telemetry/map`
- `/amr/localization/pose` -> `amr/telemetry/robot_pose`
- `/amr/planner/global` -> `amr/telemetry/global_path`
- `/amr/planner/local` -> `amr/telemetry/local_path`
- `/amr/costmap/global` -> `amr/telemetry/global_costmap`
- `/amr/costmap/local` -> `amr/telemetry/local_costmap`
- `/amr/motion/status` -> `amr/telemetry/motion_status`
- `/amr/obstacle/report` -> `amr/telemetry/obstacle_report`

Robot telemetry:

- `/scan` -> `amr/robot/turtlebot3/telemetry/scan`
- `/odom` -> `amr/robot/turtlebot3/telemetry/odom`
- `/imu` -> `amr/robot/turtlebot3/telemetry/imu`
- `/tf` -> `amr/robot/turtlebot3/telemetry/tf`
- `/tf_static` -> `amr/robot/turtlebot3/telemetry/tf_static`
- `/joint_states` -> `amr/robot/turtlebot3/telemetry/joint_states`
- `/robot_description` -> `amr/robot/turtlebot3/telemetry/robot_description`

## MQTT -> ROS Forwarding

- `amr/robot/turtlebot3/command/cmd_vel` -> `/cmd_vel`
- `amr/command/set_initial_pose` -> `/amr/localization/initial_pose`
- `amr/command/navigate_to_pose` -> local `NavigateToPose` action client
- `amr/request/plan_segment` -> local `plan_segment` service client
- `amr/request/plan_route` -> local `plan_route` service client

## MQTT Responses

- `amr/response/*` topics return ACK-style MQTT responses only
- `amr/feedback/navigate_to_pose` mirrors raw ROS feedback payloads
- `amr/status/navigate_to_pose` mirrors raw ROS status payloads

This keeps execution local on the robot while still exposing command progress to
the VBox side.

## Transport Format

- mirrored telemetry uses raw ROS serialized binary payloads
- MQTT explorer tools will show unreadable bytes for these topics
- that is expected and means the ROS message is being mirrored without JSON
  remodeling

## Launch

```bash
ros2 launch amr_mqtt_robot_plugin amr_mqtt_robot_plugin.launch.py params_file:=/path/to/amr.yaml
```

## Notes

- this package is robot-side only
- VBox-side reconstruction belongs in `amr_mqtt_bridge`
- topic names and ROS interface names should be tuned through `amr.yaml`
- source layout is split by responsibility:
  - `src/amr_mqtt_robot_plugin/node.c`
  - `src/amr_mqtt_robot_plugin/mqtt.c`
  - `include/amr_mqtt_robot_plugin/node.h`
  - `include/amr_mqtt_robot_plugin/mqtt.h`
