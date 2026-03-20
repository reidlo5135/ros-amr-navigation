# ros-amr-navigation

ROS 2 Humble AMR navigation stack for occupancy-grid maps, localization, A*
planning, local replanning, motion control, and MQTT-based external transport.

## Current Direction

- navigation runtime stays inside ROS 2
- cross-machine transport moves to MQTT
- robot-side bringup can be bridged through `amr_mqtt_robot_plugin`
- navigation-side telemetry, commands, services, and actions are bridged through
  `amr_mqtt_bridge`
- `amr_viz` is currently a transitional web visualization package and is being
  separated from transport concerns

## Main Packages

- `amr_navigation`
  - metapackage for the full stack
- `amr_bringup`
  - launch files and shared runtime parameters
- `amr_msgs`
  - shared AMR actions, services, and messages
- `amr_map_server`
  - static map and mapping-mode map server
- `amr_localization`
  - pose estimation and `map -> odom`
- `amr_obstacle_detection`
  - obstacle report generation from scan data
- `amr_costmap_server`
  - global and local costmaps
- `amr_global_planner`
  - A* global planner
- `amr_local_planner`
  - local replanning and escape behavior
- `amr_motion_controller`
  - local plan tracking and `/cmd_vel`
- `amr_bt_navigator`
  - goal execution and navigation orchestration
- `amr_lifecycle_manager`
  - managed startup ordering and initial pose sequencing
- `amr_rviz_plugins`
  - RViz goal bridge
- `amr_viz`
  - current web visualization package
- `amr_mqtt_bridge`
  - navigation-side ROS <-> MQTT bridge
- `amr_mqtt_robot_plugin`
  - robot-side ROS <-> MQTT plugin for platform bringup topics

## Transport Architecture

Recommended deployment:

- TurtleBot3 / robot
  - local ROS bringup
  - `amr_mqtt_robot_plugin`
- VBox Ubuntu server
  - `amr_navigation`
  - `amr_mqtt_bridge`
  - `mosquitto`
  - `amr_viz` web app
- Host PC
  - browser only

This keeps DDS local to each machine and reduces bridged-adapter traffic,
`ksoftirqd`, and ROS discovery overhead across the VM boundary.

## Shared Runtime Parameters

Most runtime wiring is centralized in:

- [`amr_bringup/params/amr.yaml`](/home/reidlo/ws/src/ros-amr-navigation/amr_bringup/params/amr.yaml)

Important parameter sections:

- `/amr/mqtt_bridge`
- `/amr/mqtt_robot_plugin`
- localization, planner, controller, and bringup node parameters

## MQTT Packages

`amr_mqtt_bridge` currently handles:

- telemetry publish from ROS topics
- MQTT command subscribe for `set_initial_pose`
- MQTT action bridge for `navigate_to_pose`
- MQTT request/response bridge for `plan_segment`
- MQTT request/response bridge for `plan_route`

`amr_mqtt_robot_plugin` currently handles:

- ROS -> MQTT
  - `/scan`
  - `/odom`
  - `/imu`
- MQTT -> ROS
  - `/cmd_vel`

Both packages use:

- `rclc`
- Eclipse Paho C client via `libpaho-mqtt-dev`

## Visualization

`amr_viz` currently contains:

- a React web client under `amr_viz/desktop`
- an existing WebSocket bridge path used during transition

Long-term direction:

- keep `amr_viz` as the visualization client
- keep transport logic in MQTT bridge packages

## Dependencies

Required system packages for MQTT path:

```bash
sudo apt install -y libpaho-mqtt-dev mosquitto mosquitto-clients
```

Typical workspace build:

```bash
colcon build --packages-up-to amr_navigation
```

## Entry Points

Navigation bringup:

```bash
ros2 launch amr_bringup total.launch.py
```

Navigation-side MQTT bridge:

```bash
ros2 launch amr_mqtt_bridge amr_mqtt_bridge.launch.py
```

Robot-side MQTT plugin:

```bash
ros2 launch amr_mqtt_robot_plugin amr_mqtt_robot_plugin.launch.py
```

## Notes

- `amr_mqtt_robot_plugin` is intended for robot-side deployment such as
  TurtleBot3 bringup.
- `amr_mqtt_bridge` is intended for the navigation runtime boundary.
- if broker placement is on the VBox guest, the common setup is:
  - broker listen on `0.0.0.0`
  - VBox NAT port forwarding for `1883`
  - robot plugin points to the Host PC address forwarded into the guest broker
