# amr_navigation

`amr_navigation` is the metapackage for the full AMR stack.

## Purpose

- provides a single package target for `colcon build --packages-up-to amr_navigation`
- groups the AMR runtime packages, MQTT transport packages, and bringup package

## Included Packages

- `amr_msgs`
- `amr_map_server`
- `amr_localization`
- `amr_obstacle_detection`
- `amr_costmap_server`
- `amr_global_planner`
- `amr_local_planner`
- `amr_motion_controller`
- `amr_bt_navigator`
- `amr_rviz_plugins`
- `amr_lifecycle_manager`
- `amr_bringup`
- `amr_mqtt_bridge`
- `amr_mqtt_robot_plugin`

## Build

```bash
colcon build --packages-up-to amr_navigation
```

`amr_viz` is now a standalone React project and is not part of the ROS
metapackage.
