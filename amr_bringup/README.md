# amr_bringup

Navigation launch and parameters for `slam_toolbox + AMR` online-async
navigation.

`amr_bringup/launch/navigation.launch.py` starts only the AMR navigation core:

- `costmap_server`
- `global_planner`
- `local_planner`
- `motion_controller`
- `recovery_server`
- `bt_navigator`
- `ft_navigator` when `use_frontier_navigation:=true`
- `runtime_observation`
- `navigation_manager`

It does not launch `slam_toolbox`, `amr_map_server`, or `amr_localization`.

## Expected Startup

```bash
ros2 launch <robot_bringup_package> bringup.launch.py
ros2 launch slam_toolbox online_async_launch.py \
	slam_params_file:=$(ros2 pkg prefix amr_bringup)/share/amr_bringup/params/slam_toolbox.yaml \
	use_sim_time:=false
ros2 launch amr_bringup navigation.launch.py
```

`slam_toolbox` must publish `/map` and `map -> odom`. The AMR stack consumes
that TF chain and does not publish it.

`navigation.launch.py` keeps lifecycle nodes in the root ROS namespace with
`namespace=""`, so node names and lifecycle manager targets remain
`/costmap_server`, `/global_planner`, `/local_planner`, `/motion_controller`,
`/recovery_server`, and `/bt_navigator`. When frontier navigation is enabled,
the optional lifecycle target is `/ft_navigator`.

`params/slam_toolbox.yaml` contains the packaged AMR online-async defaults for
external `slam_toolbox`: `mode: mapping`, `use_sim_time: false`, longer TF buffer
duration, and scan queue settings suitable for online map updates while AMR
navigation is running.

## Parameters

`params/amr.yaml` uses standard ROS 2 style defaults:

- `/map`
- `/scan`
- `/tf`, `/tf_static`
- `/global_costmap`
- `/local_costmap`
- `/global_plan`
- `/local_plan`
- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/cmd_vel`
- `/navigate_to_pose`
- `/navigate_to_poses`
- `/plan_segment`
- `/plan_route`
- `/plan_recovery`
- `/plan_local_escape`
- `/clear_costmap`

Default frames are `map`, `odom`, and `base_footprint`.
