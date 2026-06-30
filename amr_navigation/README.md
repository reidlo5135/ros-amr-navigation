# amr_navigation

Metapackage for the navigation-only AMR stack.

Core runtime packages:

- `amr_bringup`
- `amr_costmap_server`
- `amr_global_planner`
- `amr_controller_server`
- `amr_recovery_server`
- `amr_bt_navigator`
- `amr_runtime_observation`
- `amr_lifecycle_manager`
- `amr_msgs`

Optional UI/bridge packages:

- `amr_rviz`
- `amr_visualization`
- `amr_mqtt_server`

Legacy `amr_map_server` and `amr_localization` are not dependencies of this
metapackage. Online SLAM and localization are expected to be provided by an
external `slam_toolbox` `online_async_launch.py` launch using the packaged
`amr_bringup/params/slam_toolbox.yaml` defaults.
