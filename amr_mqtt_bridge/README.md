# amr_mqtt_bridge

Robot-side ROS <-> MQTT bridge package.

This package is the renamed and consolidated successor of the old `amr_mqtt_robot_plugin`.

## Role

- runs on TurtleBot3
- reads ROS topics, services, and actions from the on-robot stack
- publishes raw telemetry to MQTT
- publishes web-friendly JSON viz topics to MQTT
- consumes MQTT commands and dispatches them into local ROS

## Parameter Section

Use the central `amr_bringup/params/amr.yaml` section:

```yaml
/amr/mqtt_bridge:
```

## MQTT Topics

Raw telemetry:
- `amr/robot/turtlebot3/telemetry/map`
- `amr/robot/turtlebot3/telemetry/tf_static`
- `amr/robot/turtlebot3/telemetry/robot_description`
- `amr/robot/turtlebot3/telemetry/scan`
- `amr/robot/turtlebot3/telemetry/odom`
- `amr/robot/turtlebot3/telemetry/imu`
- `amr/robot/turtlebot3/telemetry/tf`
- `amr/robot/turtlebot3/telemetry/joint_states`
- `amr/robot/turtlebot3/telemetry/robot_pose`
- `amr/robot/turtlebot3/telemetry/global_costmap`
- `amr/robot/turtlebot3/telemetry/local_costmap`
- `amr/robot/turtlebot3/telemetry/global_path`
- `amr/robot/turtlebot3/telemetry/local_path`
- `amr/robot/turtlebot3/telemetry/motion_status`

Viz JSON:
- `amr/robot/turtlebot3/viz/map`
- `amr/robot/turtlebot3/viz/global_costmap`
- `amr/robot/turtlebot3/viz/local_costmap`
- `amr/robot/turtlebot3/viz/robot_pose`
- `amr/robot/turtlebot3/viz/global_path`
- `amr/robot/turtlebot3/viz/local_path`
- `amr/robot/turtlebot3/viz/motion_status`
- `amr/robot/turtlebot3/viz/scan`
- `amr/robot/turtlebot3/viz/tf`
- `amr/robot/turtlebot3/viz/tf_static`
- `amr/robot/turtlebot3/viz/robot_description`

Commands:
- `amr/command/navigate_to_poses`
- `amr/command/cancel_navigate_to_poses`
- `amr/command/set_initial_pose`

Single-goal navigation uses the same route API with a one-element `goal_poses` array.

## Launch

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py params_file:=/path/to/amr.yaml
```
