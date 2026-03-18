# amr_bringup

`amr_bringup` owns stack startup, shared runtime parameters, RViz configuration, and helper scripts.

## Launch Files

- [localization.launch.py](./launch/localization.launch.py)
  - `amr_map_server`
  - `amr_localization`
  - `amr_global_planner`
- [navigation.launch.py](./launch/navigation.launch.py)
  - `amr_local_planner`
  - `amr_motion_controller`
  - `amr_bt_navigator`
  - `amr_rviz_plugins`

This package launches nodes directly. It does not include per-package launch files.

## Runtime Assets

- shared params: [amr.yaml](./params/amr.yaml)
- RViz config: [amr.rviz](./rviz/amr.rviz)
- helper scripts:
  - [send_goal.sh](./script/send_goal.sh)
  - [initialpose.sh](./script/initialpose.sh)

## Startup Layout

```mermaid
flowchart LR
    A[localization.launch.py] --> B[amr_map_server]
    A --> C[amr_localization]
    A --> D[amr_global_planner]
    E[navigation.launch.py] --> F[amr_local_planner]
    E --> G[amr_motion_controller]
    E --> H[amr_bt_navigator]
    E --> I[amr_rviz_plugins]
```

## Notes

- all runtime node parameters are centralized in one file
- `amr_localization` can publish an automatic initial pose from the configured origin station
- RViz goal forwarding is enabled by the `amr_rviz_plugins` bridge node
