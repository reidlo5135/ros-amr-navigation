# amr_mqtt_bridge

`amr_mqtt_bridge` is the VBox/server-side ROS <-> MQTT boundary for the AMR
stack.

Its current role is:

- receive mirrored MQTT topics from the robot
- reconstruct those topics into the local VBox ROS graph
- forward selected local ROS commands such as `/cmd_vel` back into MQTT
- receive robot-side navigation feedback/status mirrors

## Runtime Role

Recommended placement:

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

## Current MQTT -> ROS Mirror

Navigation telemetry mirrored into VBox ROS:

- `amr/telemetry/robot_pose` -> `/amr/localization/pose`
- `amr/telemetry/global_path` -> `/amr/planner/global`
- `amr/telemetry/local_path` -> `/amr/planner/local`
- `amr/robot/turtlebot3/telemetry/map` -> `/amr/map/data`
- `amr/telemetry/global_costmap` -> `/amr/costmap/global`
- `amr/telemetry/local_costmap` -> `/amr/costmap/local`
- `amr/telemetry/motion_status` -> `/amr/motion/status`
- `amr/telemetry/obstacle_report` -> `/amr/obstacle/report`

Robot telemetry mirrored into VBox ROS:

- `amr/robot/turtlebot3/telemetry/scan` -> `/scan`
- `amr/robot/turtlebot3/telemetry/odom` -> `/odom`
- `amr/robot/turtlebot3/telemetry/imu` -> `/imu`
- `amr/robot/turtlebot3/telemetry/tf` -> `/tf`
- `amr/robot/turtlebot3/telemetry/tf_static` -> `/tf_static`
- `amr/robot/turtlebot3/telemetry/joint_states` -> `/joint_states`
- `amr/feedback/navigate_to_pose` -> `/amr/navigator/navigate_to_pose/feedback`
- `amr/status/navigate_to_pose` -> `/amr/navigator/navigate_to_pose/status`

## Current ROS -> MQTT Forwarding

- `/cmd_vel` -> `amr/robot/turtlebot3/command/cmd_vel`

## Feature Ownership

- robot-side command/service/action ownership belongs in `amr_mqtt_robot_plugin`
- VBox-side `amr_mqtt_bridge` mirrors telemetry, feedback, and status into the local ROS graph
- ACK responses for service/action remain MQTT-side responses

## Launch

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py
```

## Notes

- this package is intended to run on VBox / server side
- robot-side topic mirroring belongs in `amr_mqtt_robot_plugin`
- topic names and ROS interface names should be tuned through `amr.yaml`
