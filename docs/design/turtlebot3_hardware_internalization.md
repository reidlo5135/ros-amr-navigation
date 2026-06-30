# TurtleBot3 Hardware Boundary

The current AMR navigation stack expects robot hardware bringup to run outside
`ros-amr-navigation`.

Robot bringup must provide:

- `/scan`
- `/odom` or `odom -> base_*` TF
- `/tf`
- `/tf_static`
- `/cmd_vel` ingress for the base
- optional `/imu`, `/joint_states`, `/robot_description`, `/battery_state`

External `slam_toolbox` `online_async_launch.py` consumes the robot bringup
outputs with `amr_bringup/params/slam_toolbox.yaml` and publishes:

- `/map`
- `map -> odom`

`ros-amr-navigation` then consumes `/map`, `/scan`, and TF for costmaps,
planning, navigation, and control. It does not publish `map -> odom`.

## Frame Contract

Expected chain:

- `map -> odom`: external `slam_toolbox`
- `odom -> base_footprint` or `odom -> base_link`: robot bringup
- robot fixed frames: URDF/static TF from robot bringup

Default AMR parameters use `base_footprint`. For robots that expose only
`base_link`, set `frames.base: "base_link"` in `amr_bringup/params/amr.yaml` for
costmap, local planner, motion controller, and navigator.

## Future Hardware Internalization

If TurtleBot3 hardware is internalized later, the AMR-owned bringup layer should
still preserve the same navigation-facing contract:

- publish `/scan`
- publish `odom -> base_*`
- subscribe `/cmd_vel`
- avoid publishing `map -> odom`

This keeps online SLAM/localization ownership cleanly assigned to
`slam_toolbox` while AMR navigation keeps the root topic contract.
