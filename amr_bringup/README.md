# amr_bringup

Central launch and parameter package for the `0.13.2` stack.

## Main Files

- `params/amr.yaml`: shared runtime parameters
- `launch/localization.launch.py`: map server, localization, costmap server, global planner
- `launch/navigation.launch.py`: local planner, motion controller, recovery server, BT navigator
- `launch/turtlebot3.launch.py`: TurtleBot3 bringup + AMR runtime + robot-side MQTT bridge

## Runtime Layout

`turtlebot3.launch.py` starts:
1. `turtlebot3_bringup/robot.launch.py`
2. delayed `localization.launch.py`
3. delayed `amr_mqtt_bridge.launch.py`
4. delayed `navigation.launch.py`

`mapping_mode:=true` keeps:
- `amr_map_server`

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
- `/amr/local_planner`
- `/amr/motion_controller`
- `/amr/recovery_server`
- `/amr/navigator`
- `/amr/mqtt_bridge`

## Launch

```bash
ros2 launch amr_bringup turtlebot3.launch.py
```

Startup localization policy override:

```bash
ros2 launch amr_bringup turtlebot3.launch.py startup_localization_mode:=manual_set_initial_pose
```

Supported startup localization policies:

- `manual_set_initial_pose`
- `global_relocalization`
- `active_relocalization`
  - navigator-supervised spin / wait / probe startup relocalization
- `fixed_start_pose`

Mapping mode:

```bash
ros2 launch amr_bringup turtlebot3.launch.py mapping_mode:=true
```
