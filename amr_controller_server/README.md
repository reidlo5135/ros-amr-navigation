# amr_controller_server

Hosts two lifecycle nodes:

- `local_planner`
- `motion_controller`

Both nodes obtain the robot pose through TF lookup from `map` to
`base_footprint` by default. They no longer subscribe to a localization pose
topic.

## local_planner

Inputs:

- `/motion_command`
- `/local_costmap`
- `/global_plan`
- TF `map -> odom -> base_*`

Outputs:

- `/local_plan`
- `/local_plan_status`

Service:

- `/plan_local_escape`

## motion_controller

Inputs:

- `/motion_command`
- `/local_plan`
- `/scan`
- TF `map -> odom -> base_*`

Outputs:

- `/motion_status`
- `/cmd_vel`

If TF is temporarily unavailable, both nodes wait without crashing. The motion
controller publishes no active navigation command until pose lookup succeeds.
