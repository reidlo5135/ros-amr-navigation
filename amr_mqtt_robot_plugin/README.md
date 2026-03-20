# amr_mqtt_robot_plugin

`amr_mqtt_robot_plugin` is the robot-side transport adapter for platform bringup
topics such as `scan`, `odom`, `imu`, and `cmd_vel`.

The intended topology is:

- TurtleBot3 / robot:
  - local ROS bringup
  - `amr_mqtt_robot_plugin`
- VBox / navigation runtime:
  - `amr_mqtt_bridge`
  - `amr_navigation`
  - `mosquitto`

This separation keeps DDS local to each machine and moves cross-boundary
traffic to MQTT.

## Why This Package Exists

- reduce VBox bridged-adapter DDS traffic and `ksoftirqd` load
- keep TurtleBot3 bringup extensible without exposing DDS directly to VBox
- separate central navigation bridge concerns from robot platform I/O concerns

## Planned Responsibilities

- ROS to MQTT
  - `/scan`
  - `/odom`
  - `/imu`
- MQTT to ROS
  - `/cmd_vel`

Optional extensions later:

- `/battery_state`
- `/joint_states`
- robot status heartbeat
- robot-side safety or estop topics

## Shared Parameters

Runtime mapping is reserved in:

- `amr_bringup/params/amr.yaml`

Current parameter section:

- `/amr/mqtt_robot_plugin`

## Status

This package now owns the robot-side MQTT client and topic relay path.
