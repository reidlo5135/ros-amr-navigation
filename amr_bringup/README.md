# amr_bringup

Navigation launch and parameters for `slam_toolbox + ros-amr-navigation`.

`amr_bringup/launch/navigation.launch.py` starts only the AMR navigation core:

- `costmap_server`
- `global_planner`
- `local_planner`
- `motion_controller`
- `recovery_server`
- `navigator`
- `runtime_observation`
- `navigation_manager`

It does not launch `slam_toolbox`, `amr_map_server`, or `amr_localization`.

## Expected Startup

```bash
ros2 launch <robot_bringup_package> bringup.launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<slam_toolbox_online_params.yaml> use_sim_time:=false
ros2 launch amr_bringup navigation.launch.py
```

`slam_toolbox` must publish `/map` and `map -> odom`. The AMR stack consumes
that TF chain and does not publish it.

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
