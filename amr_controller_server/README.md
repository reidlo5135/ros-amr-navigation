# amr_controller_server

Controller-layer server package for the AMR navigation stack.

## Role

`amr_controller_server` owns the runtime boundary and source code for local control while
preserving the existing lifecycle node contracts:

- `/amr/local_planner`: builds the short-horizon local plan from the global route.
- `/amr/motion_controller`: tracks the local plan, applies safety gating, and publishes `cmd_vel`.
- local path refiner: prunes near-duplicate poses, densifies long segments, smooths safe corners, and assigns path headings.
- goal checker: separates XY / yaw / hold-time arrival policy from velocity tracking logic.

This mirrors the Nav2-style server file layout with one `controller_server.hpp`,
one `controller_server.cpp`, and one `main.cpp`. The process hosts both existing lifecycle nodes
so lifecycle management, parameters, and topics remain compatible.

## Launch

`amr_bringup` includes this launch file and passes the consolidated parameter file:

```bash
ros2 launch amr_controller_server controller.launch.py params_file:=/path/to/amr.yaml
```

## Parameters

`/amr/local_planner` exposes `path_refiner.*` parameters for safe path post-processing,
including collision-checked corner smoothing with fallback to the unsmoothed local plan.
It also exposes `dynamic_obstacle.*` parameters for local escape generation and blocked-state
confirmation, including the current corridor/doorway relaxation controls used by the `0.16.x`
recovery-tuning line.
`/amr/motion_controller` exposes `goal_checker.*` parameters for arrival policy tuning.
By default, waypoint-style goals can stay heading-free while command-driven final heading align
still uses a separate tighter tolerance near the goal.
When heading alignment is active near the goal, `control.goal_reach_heading_tolerance` and
`control.final_align_max_angular_speed` tune the final in-place yaw settle behavior separately
from the looser general waypoint-style yaw tolerance.
For nominal path tracking, `control.tracking_heading_deadband` suppresses tiny heading
corrections on straight segments so the robot does not visibly wag with small localization
or path-sampling noise.

## Important Interfaces

- topic in: `/amr/motion/command`
- topic in: `/amr/localization/pose`
- topic in: `/amr/costmap/local`
- topic out: `/amr/planner/local`
- topic out: `/amr/planner/local_status`
- topic out: `/amr/motion/status`
- service: `/amr/local_planner/plan_local_escape`

## Next Direction

Progress checking, safety gate, velocity control, and local planning should keep moving toward
package-local classes or plugins under this controller boundary.
