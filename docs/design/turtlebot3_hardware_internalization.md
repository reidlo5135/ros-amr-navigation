# TurtleBot3 Hardware Internalization Boundary

## Current External Dependency

The current AMR runtime depends on `turtlebot3_bringup/robot.launch.py` to provide the
robot-side hardware contract before the AMR runtime starts.

Current launch split:

- `amr_bringup/launch/turtlebot3.launch.py`
  - robot-side entrypoint
  - currently includes `turtlebot3_bringup/robot.launch.py`
  - then starts delayed AMR runtime layers
- `amr_bringup/launch/navigation.launch.py`
  - remote-PC AMR runtime entrypoint
  - expects hardware topics and TF to already exist on the ROS graph

Current external TB3 dependency provides:

- `/scan`
- `/odom`
- `/imu`
- `/joint_states`
- `/tf`
- `/tf_static`
- `/cmd_vel` ingress for the base
- `robot_description`

## Current AMR Hardware Boundary

Current AMR consumers of the external hardware contract:

- `amr_localization`
  - consumes `/odom` and `/scan`
  - publishes `map -> odom`
- `amr_costmap_server`
  - consumes `/scan`
- `amr_controller_server`
  - publishes `/cmd_vel`
  - also consumes `/scan` for local control behavior
- `amr_visualization`
  - expects `base_footprint` in TF
  - subscribes to `/tf_static` and `/robot_description`
- `amr_mqtt_server`
  - exposes `/scan`, `/odom`, `/imu`, `/joint_states`, `/tf`, `/tf_static`, and `robot_description`
  - MQTT integration is out of scope for this pass

`/imu` is currently treated as an exposed hardware/telemetry lane, not a core AMR navigation
input. The primary AMR hardware-facing requirements today are `/scan`, `/odom`, `/cmd_vel`, TF,
joint states, and robot description.

## Desired AMR-Owned Replacement

Replace the direct runtime dependency on `turtlebot3_bringup` with AMR-owned packages:

- `amr_tb3_bringup`
  - TurtleBot3 Burger hardware launch composition
- `amr_tb3_base_driver`
  - OpenCR transport, command path, feedback decoding, odometry, IMU, joint states, TF
- `amr_tb3_lidar_driver`
  - LDS-class LiDAR transport and `/scan`
- `amr_description`
  - AMR-owned TurtleBot3 Burger description and `robot_state_publisher` input

The AMR-facing contract must remain root-topic compatible:

- `/cmd_vel`
- `/odom`
- `/imu`
- `/scan`
- `/joint_states`
- `/tf`
- `/tf_static`
- `/robot_description`

## Frame Contract

The AMR runtime should continue to rely on:

- `map`
- `odom`
- `base_footprint`
- `base_link`
- `base_scan`
- `imu_link`

Expected dynamic/static chain:

- `map -> odom`
  - owned by `amr_localization`
- `odom -> base_footprint`
  - owned by the AMR base driver when TF publishing is enabled
- `base_footprint -> base_link`
  - fixed transform from URDF
- `base_link -> base_scan`
  - fixed transform from URDF
- `base_link -> imu_link`
  - fixed transform from URDF

## Migration Shape

The migration should stay incremental and reviewable.

### Phase 1

- define the hardware boundary in docs
- restore `amr_bringup` as a small shared schema/config package
- add AMR-owned package skeletons
- add AMR-owned TurtleBot3 Burger description
- split launch roles clearly:
  - `turtlebot3.launch.py` becomes robot-side, pure-ROS, hardware-only
  - `navigation.launch.py` remains remote-PC navigation/runtime only
  - `turtlebot3_external.launch.py` preserves the current `turtlebot3_bringup` compatibility path

### Phase 2

- add serial transport and protocol abstractions
- add base-driver node shell with `/cmd_vel` watchdog and connection-state reporting
- add lidar-driver node shell and parser interfaces
- move configurable hardware defaults into `amr_bringup/params/amr.yaml`

### Phase 3

- extend base feedback into `/odom`, `/imu`, `/joint_states`, and TF publishing
- implement differential-drive odometry independent of ROS node logic
- implement incremental OpenCR command/feedback support with explicit hardware-verification TODOs
- implement incremental LDS parser backends with clear sensor-model boundaries

## Risks

- OpenCR packet format details may require hardware verification before full parity with
  `turtlebot3_node`
- TurtleBot3 units may use different LDS variants, so the LiDAR parser must remain model-aware
- `robot_description` and `joint_states` must remain stable enough for `amr_visualization`
  and RViz consumers
- robot-side and remote-PC launch responsibilities must stay explicit to avoid regressions in the
  split deployment topology
- no build/test/colcon execution is performed in this implementation pass; manual validation is
  required after each stage
