# amr_bringup

Central launch, parameter, and hardware-entry package for the active `0.17.x` line.

## Main Files

- `params/amr.yaml`: shared runtime parameters
- `launch/localization.launch.py`: map server, localization, costmap server, global planner
- `launch/navigation.launch.py`: remote-PC AMR runtime entrypoint
- `launch/turtlebot3.launch.py`: AMR-owned TurtleBot3 robot-side hardware bringup
- `launch/turtlebot3_external.launch.py`: legacy external `turtlebot3_bringup` compatibility path

`navigation.launch.py` starts the remote-PC navigation runtime by default:
- delayed `localization.launch.py`
- delayed controller / recovery / BT navigation nodes
- delayed runtime observation and lifecycle manager nodes

This package now treats robot-side hardware and remote-PC navigation as separate operational lanes.
The AMR-owned `turtlebot3.launch.py` is pure ROS hardware bringup only and does not start
`amr_mqtt_server`, localization, planner, controller, or navigator nodes.
All AMR-owned TurtleBot3 hardware code now lives directly under `amr_bringup/include`,
`amr_bringup/src`, and `amr_bringup/urdf`.

## Runtime Layout

`turtlebot3.launch.py` starts:
1. `amr_bringup_hardware` as `base_driver`
2. LiDAR backend selected by `lidar_backend`
3. `external_hlds` uses `hls_lfcd_lds_driver`
4. `internal` uses `amr_bringup_hardware` as `lidar_driver`
5. `robot_state_publisher` from the AMR-owned URDF in `amr_bringup/urdf`

`turtlebot3_external.launch.py` preserves the older compatibility flow:
1. `turtlebot3_bringup/robot.launch.py`
2. delayed `localization.launch.py`
3. delayed `amr_mqtt_server.launch.py`
4. delayed `navigation.launch.py`

`mapping_mode:=true` keeps:
- `amr_map_server`
- external `ros-slam-mapper` runtime is expected to be launched separately and publish `/slam/*`

and skips:
- `amr_localization`
- `amr_costmap_server`
- `amr_global_planner`
- `navigation.launch.py`

## Important Parameter Groups

- `/amr/robot_bringup`
- `/amr/base_driver`
- `/amr/lidar_driver`
- `/amr/description`
- `/amr/map_server`
- `/amr/localization`
- `/amr/costmap_server`
- `/amr/global_planner`
- `/amr/local_planner` via `amr_controller_server`
- `/amr/motion_controller` via `amr_controller_server`
- `/amr/recovery_server`
- `/amr/runtime_observation`
- `/amr/navigator`
- `/amr/mqtt_server`

## Launch

Robot-side AMR-owned hardware bringup on the TurtleBot3 RPi4:

```bash
ros2 launch amr_bringup turtlebot3.launch.py
```

Current safest TurtleBot3 Burger LiDAR path:

```bash
ros2 launch amr_bringup turtlebot3.launch.py lidar_backend:=external_hlds
```

Single-entry executable used by the robot-side launch:

```bash
ros2 run amr_bringup amr_bringup_hardware --ros-args -p bringup.component:=base_driver
```

Legacy external TurtleBot3 compatibility mode:

```bash
ros2 launch amr_bringup turtlebot3_external.launch.py
```

Remote-PC AMR navigation/runtime bringup when robot hardware topics already exist:

```bash
ros2 launch amr_bringup navigation.launch.py
```

Mapping mode on the remote-PC runtime:

```bash
ros2 launch amr_bringup navigation.launch.py mapping_mode:=true
```

In mapping mode, `navigation.launch.py` consumes external SLAM topics such as:
- `/slam/map/temp/refined`
- `/slam/map/temp/raw`
- `/slam/mapper/odometry`
- `/slam/mapper/pose`
- `/slam/mapper/graph_debug`

## Validation Checklist

- confirm robot-side hardware topics:
  - `ros2 topic list`
  - `ros2 topic hz /scan`
  - `ros2 topic hz /odom`
  - `ros2 topic echo /imu`
- confirm TF chain:
  - `ros2 run tf2_tools view_frames`
- smoke-test velocity path carefully:
  - teleop or a single bounded `/cmd_vel` command
- validate remote-PC runtime consumption separately:
  - launch `navigation.launch.py`
  - verify localization, planner, and controller subscribe cleanly to robot-side hardware topics

## Safety Notes

- Wheels can move as soon as `/cmd_vel` is sent to the base driver.
- OpenCR and LDS packet details are still under incremental verification in this pass.
- Keep the rollback path available:
  - use `turtlebot3_external.launch.py` if the AMR-owned hardware backend is not yet ready for the robot under test.
