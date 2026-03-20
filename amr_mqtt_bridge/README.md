# amr_mqtt_bridge

`amr_mqtt_bridge` is the new transport boundary package for the AMR stack.

The package layout follows the same `rclc`-first C template style used in
`ros-teleop-twist`: `src/main.c` as the entry point, package-local sources under
`src/amr_mqtt_bridge/`, and exported headers under
`include/amr_mqtt_bridge/`.

Its job is to keep DDS traffic inside the ROS 2 runtime and expose only the
selected, modeled AMR data over MQTT for web clients, dashboards, or other
external integrations.

## Goals

- replace the Python WebSocket bridge path with a lower-overhead native runtime
- keep ROS 2 discovery and topic fan-out inside the AMR runtime boundary
- publish only the AMR data we actually need instead of mirroring raw ROS traffic
- provide a reusable MQTT bridge package that is not tightly coupled to `amr_viz`

## Runtime Direction

- ROS side: `rclc`
- MQTT side: Eclipse Paho C client (`paho.mqtt.c`)
- broker transport:
  - MQTT TCP for the bridge, typically `1883`
  - MQTT over WebSocket for browsers, typically `9001`

## External Libraries

Required system packages:

```bash
sudo apt install -y libpaho-mqtt-dev
```

Expected ROS runtime:

- ROS 2 Humble
- `rclc`

Recommended broker package on the Ubuntu server:

```bash
sudo apt install -y mosquitto mosquitto-clients
```

## Initial Parameter Contract

The shared runtime parameters live in:

- `amr_bringup/params/amr.yaml`

Current parameter sections relevant to this package:

- `/amr/mqtt_bridge`
  - broker connection
  - MQTT root namespace
  - ROS source topics/services/actions
  - MQTT topic mapping for telemetry and commands

## Planned ROS Inputs

- `/amr/localization/pose`
- `/amr/planner/global`
- `/amr/planner/local`
- `/amr/map/data`
- `/amr/costmap/global`
- `/amr/costmap/local`
- `/amr/motion/status`
- `/amr/obstacle/report`

## Planned ROS Command Interfaces

- `/amr/localization/initial_pose`
- `/amr/navigator/navigate_to_pose`
- `/amr/global_planner/plan_segment`
- `/amr/global_planner/plan_route`

## Planned MQTT Interface Shape

Telemetry topics:

- `amr/telemetry/robot_pose`
- `amr/telemetry/global_path`
- `amr/telemetry/local_path`
- `amr/telemetry/map`
- `amr/telemetry/global_costmap`
- `amr/telemetry/local_costmap`
- `amr/telemetry/motion_status`
- `amr/telemetry/obstacle_report`

Command topics:

- `amr/command/navigate_to_pose`
- `amr/command/set_initial_pose`
- `amr/request/plan_segment`
- `amr/request/plan_route`
- `amr/response/plan_segment`
- `amr/response/plan_route`

## Status

This package is currently in bootstrap stage:

- package skeleton created
- native bridge executable scaffold created
- launch path created
- AMR parameter schema reserved in `amr.yaml`

The next step is implementing the actual ROS subscriptions, MQTT publish path,
and MQTT command-to-ROS bridge behavior.
