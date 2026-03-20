# amr_mqtt_robot_plugin

`amr_mqtt_robot_plugin` is the robot-side ROS <-> MQTT mirror for TurtleBot3
bringup and AMR navigation runtime topics.

Its job is to keep the high-rate ROS graph on the robot, serialize selected ROS
messages as raw ROS binary, and mirror them into the VBox MQTT broker.

## Runtime Role

Recommended placement:

- TurtleBot3 / robot
  - `turtlebot3_bringup`
  - `localization.launch.py`
  - `navigation.launch.py`
  - `amr_mqtt_robot_plugin`

This reduces bridged-adapter DDS traffic, `ksoftirqd`, and VM discovery noise.

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

## Current ROS -> MQTT Mirror

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

## Current MQTT -> ROS Forwarding

- `amr/robot/turtlebot3/command/cmd_vel` -> `/cmd_vel`

## Current MQTT Feature Endpoints

- `amr/command/set_initial_pose` -> publish `/amr/localization/initial_pose`
- `amr/command/navigate_to_pose` -> send local `NavigateToPose` action goal
- `amr/request/plan_segment` -> send local `plan_segment` service request
- `amr/request/plan_route` -> send local `plan_route` service request
- `amr/response/*` topics return ACK-style MQTT responses only
- `amr/feedback/navigate_to_pose` is mirrored as raw ROS feedback payload
- `amr/status/navigate_to_pose` is mirrored as raw ROS status payload

## Transport Format

- telemetry topics use raw ROS serialized binary payloads
- MQTT explorer tools will show unreadable binary bytes for these topics
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
