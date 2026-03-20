# amr_mqtt_robot_plugin

`amr_mqtt_robot_plugin` is the robot-side ROS <-> MQTT transport plugin for
platform bringup topics.

Its job is to keep TurtleBot3 or other robot-local DDS traffic on the robot and
send only selected telemetry and commands across the machine boundary.

## Runtime Role

Recommended placement:

- TurtleBot3 / robot
  - base bringup
  - local sensors
  - `amr_mqtt_robot_plugin`
- VBox Ubuntu server
  - `mosquitto`
  - `amr_navigation`
  - `amr_mqtt_bridge`

This is intended to reduce bridged-adapter DDS load and `ksoftirqd` pressure
between the robot, host, and VBox guest.

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

Tunable groups:

- broker host, port, client id, keepalive, auth
- MQTT telemetry and command topic names
- robot ROS topic names

Default intent:

- broker host points to the Host PC address forwarded into the VBox broker
- example default: `192.168.61.35:1883`

## Current Interfaces

ROS -> MQTT:

- `/scan` -> `amr/robot/turtlebot3/telemetry/scan`
- `/odom` -> `amr/robot/turtlebot3/telemetry/odom`
- `/imu` -> `amr/robot/turtlebot3/telemetry/imu`
- `/tf` -> `amr/robot/turtlebot3/telemetry/tf`
- `/tf_static` -> `amr/robot/turtlebot3/telemetry/tf_static`
- `/joint_states` -> `amr/robot/turtlebot3/telemetry/joint_states`

MQTT -> ROS:

- `amr/robot/turtlebot3/command/cmd_vel` -> `/cmd_vel`

## Current Payload Shape

`cmd_vel` command payload:

```json
{
  "x": 0.15,
  "z": 0.35
}
```

Meaning:

- `x`
  - linear velocity for `Twist.linear.x`
- `z`
  - angular velocity for `Twist.angular.z`

## Launch

```bash
ros2 launch amr_mqtt_robot_plugin amr_mqtt_robot_plugin.launch.py
```

## Notes

- this package is robot-side only
- navigation-side command/action/service bridging belongs in `amr_mqtt_bridge`
- the current implementation focuses on minimal TurtleBot3 bringup transport:
  - `scan`
  - `odom`
  - `imu`
  - `tf`
  - `tf_static`
  - `joint_states`
  - `cmd_vel`
- more robot-specific topics can be added later if needed
