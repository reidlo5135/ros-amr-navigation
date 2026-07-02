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

The motion controller keeps the in-package Pure Pursuit tracking flow and adds
an RPP-style regulation layer internally. It does not load Nav2
`controller_server` or `nav2_regulated_pure_pursuit_controller` plugins, and it
keeps the existing `/motion_command`, `/local_plan`, `/scan`, `/cmd_vel`, and
`/motion_status` contract.

RPP-style behavior is configured under `regulated_pure_pursuit`:

- adaptive lookahead scales the tracking lookahead with target/current linear
  speed and caps it near the goal
- curvature regulation lowers linear speed on tight turns while preserving the
  existing rotate-in-place and final alignment states
- approach regulation slows the robot as the goal/local plan remaining distance
  enters the configured approach window
- scan-based obstacle proximity regulation slows forward motion before the
  safety gate hard-stop distance
- collision projection is a first implementation of scan-based forward safety
  regulation; the controller does not subscribe to a costmap for projection

The existing safety gate remains the hard stop path. Obstacle proximity scaling
only reduces forward speed in the scan cone, while the safety gate still owns
blocked status confirmation and rotate-in-place allowance.
