# amr_bringup

Central launch and parameter package for the AMR runtime stack.

## Main Files

- `params/amr.yaml`: shared runtime parameters
- `launch/localization.launch.py`: map server, localization, costmap server, global planner
- `launch/navigation.launch.py`: AMR runtime entrypoint

`navigation.launch.py` starts the AMR runtime by default:
- delayed `localization.launch.py`
- delayed `amr_mqtt_server.launch.py`
- delayed controller / recovery / BT navigation nodes
- delayed runtime observation and lifecycle manager nodes

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
- `/amr/mqtt_server`

## Launch

AMR runtime bringup:

```bash
ros2 launch amr_bringup navigation.launch.py
```

Navigation-only mode when localization and MQTT are already running elsewhere:

```bash
ros2 launch amr_bringup navigation.launch.py navigation_only:=true
```

Mapping mode:

```bash
ros2 launch amr_bringup navigation.launch.py mapping_mode:=true
```

In mapping mode, `navigation.launch.py` consumes external SLAM topics such as:
- `/slam/map/temp/refined`
- `/slam/map/temp/raw`
- `/slam/mapper/odometry`
- `/slam/mapper/pose`
- `/slam/mapper/graph_debug`
