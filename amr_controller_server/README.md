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
The refiner now stages prune/interpolate, smoothing, and final handoff separately, and
`path_refiner.smoothing_max_length_ratio` plus `path_refiner.smoothing_max_pose_deviation`
bound how far a smoothed candidate may drift from the base local path before it is rejected.
It also exposes `dynamic_obstacle.*` parameters for local escape generation and blocked-state
confirmation, including the current corridor/doorway relaxation controls used by the `0.16.x`
recovery-tuning line.
`dynamic_obstacle.clear_confirm_cycles` lets the local planner require a short clear streak
before it fully drops a previously confirmed blocked decision.
`/amr/motion_controller` exposes `goal_checker.*` parameters for arrival policy tuning.
By default, waypoint-style goals can stay heading-free while command-driven final heading align
still uses a separate tighter tolerance near the goal.
`goal_checker.xy_hysteresis` keeps the XY-arrived phase latched through small localization
noise, so final heading alignment does not repeatedly fall back into path tracking.
When heading alignment is active near the goal, `control.goal_reach_heading_tolerance` and
`control.final_align_max_angular_speed` tune the final in-place yaw settle behavior separately
from the looser general waypoint-style yaw tolerance.
`control.final_align_heading_deadband` and `control.final_align_settle_time_sec` let the
controller hold a quiet final-yaw settle window before it declares the aligned goal complete.
For nominal path tracking, `control.tracking_heading_deadband` suppresses tiny heading
corrections on straight segments so the robot does not visibly wag with small localization
or path-sampling noise.
`control.tracking_heading_release_threshold` adds hysteresis so angular correction resumes
only after the heading error leaves the deadband by a clear margin.
Straight-line oscillation reduction is controlled by `control.straight_tracking_enabled`.
When enabled, the controller detects low-curvature local plan windows with
`control.straight_curvature_threshold` and `control.straight_lateral_error_threshold`, uses
`control.straight_tracking_lookahead_distance` for a less noisy target, and applies the
conservative `control.straight_heading_deadband`, `control.straight_heading_release_threshold`,
`control.straight_angular_gain`, `control.straight_max_angular_speed`, and
`control.straight_heading_filter_alpha` only during normal path tracking.
Goal approach, final heading alignment, backup, spin, and wait recovery commands continue to use
their existing control paths.
`control.tracking_progress_rollback_window` limits how far the controller may search backward
on a refreshed local plan, which helps path rejoin stay forward-progressive instead of snapping
between old and newly republished nearby poses.
`control.tracking_target_hysteresis_distance`, `control.tracking_target_reset_distance`, and
the `control.rejoin_*` parameters slow down target switching and linear push during path rejoin
so recovery exit and off-path correction do not wag left/right as aggressively.
`status.*` parameters debounce blocked/stalled publication so fresh command dispatch, safety-gate
flicker, and final-align settling do not immediately look like hard recovery conditions.
Throttled controller logs include the current tracking index, selected target point, goal phase,
rejoin phase, and blocked/safety state so path-following quality issues can be inspected without
changing the motion status topic contract.

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
