# amr_bringup

Central launch and parameter package for the `0.15.9` AMR stack.

## Main Files

- `params/amr.yaml`: shared runtime parameters
- `launch/localization.launch.py`: map server, localization, costmap server, global planner
- `launch/navigation.launch.py`: controller server aggregation, recovery server, BT navigator
- `launch/turtlebot3.launch.py`: TurtleBot3 bringup + AMR runtime + robot-side MQTT bridge

## Runtime Layout

`turtlebot3.launch.py` starts:
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

- `/amr/map_server`
- `/amr/localization`
- `/amr/costmap_server`
- `/amr/global_planner`
- `/amr/local_planner` via `amr_controller_server`
- `/amr/motion_controller` via `amr_controller_server`
- `/amr/recovery_server`
- `/amr/runtime_observation`
- `/amr/navigator`
- `/amr/mqtt_bridge`

## Launch

```bash
ros2 launch amr_bringup turtlebot3.launch.py
```

Mapping mode:

```bash
ros2 launch amr_bringup turtlebot3.launch.py mapping_mode:=true
```

In mapping mode, `amr_bringup` consumes external SLAM topics such as:
- `/slam/map/temp/refined`
- `/slam/map/temp/raw`
- `/slam/mapper/odometry`
- `/slam/mapper/pose`
- `/slam/mapper/graph_debug`
