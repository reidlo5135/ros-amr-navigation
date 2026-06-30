# amr_lifecycle_manager

Simple lifecycle bringup manager used by `amr_bringup`.

The current navigation launch manages only navigation core lifecycle nodes:

- `/costmap_server`
- `/global_planner`
- `/local_planner`
- `/motion_controller`
- `/recovery_server`
- `/navigator`

It does not manage a map server or localization node. Initial pose publication is
disabled by default; `/initialpose` belongs to RViz or external `slam_toolbox`
configuration.
