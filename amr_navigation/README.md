# amr_navigation

`amr_navigation` is the entry metapackage for the AMR navigation stack.

## Purpose

- provides a single package target for `colcon build --packages-up-to amr_navigation`
- groups all runtime packages required for the custom AMR stack

## Included Packages

- `amr_msgs`
- `amr_map_server`
- `amr_localization`
- `amr_global_planner`
- `amr_local_planner`
- `amr_motion_controller`
- `amr_bt_navigator`
- `amr_rviz_plugins`
- `amr_bringup`

## Build

```bash
colcon build --packages-up-to amr_navigation
```
