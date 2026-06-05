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
By default, AMR navigation goals are full pose targets and command-driven final heading alignment
remains enabled. `/amr/navigator.execution.align_heading_at_goal` defaults to `true`; set it to
`false` only for explicit XY-only tests or workflows.
`goal_checker.xy_hysteresis` keeps the XY-arrived phase latched through small localization
noise, so final heading alignment does not repeatedly fall back into path tracking.
When heading alignment is active near the goal, `control.goal_reach_heading_tolerance` and
`control.final_align_max_angular_speed` tune the final in-place yaw settle behavior separately
from the looser general waypoint-style yaw tolerance.
`control.final_align_heading_deadband` and `control.final_align_settle_time_sec` let the
controller hold a quiet final-yaw settle window before it declares the aligned goal complete.
For straight-line tracking tests where final yaw should be ignored at the controller layer, set
`goal_checker.ignore_yaw` to `true`; this leaves recovery and normal tracking intact but skips
goal yaw alignment only for that configuration.
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
The local planner also reduces grid stair-steps before the motion controller sees them.
`path_refiner.line_of_sight_simplification_enabled` skips intermediate poses when the footprint can
travel directly between non-adjacent path points, and `path_refiner.collinear_pruning_enabled`
removes near-collinear residual points after that line-of-sight pass.
The `local_path_quality` log reports raw, simplified, and refined point counts plus curvature,
lateral error, pruning count, and collision-check status.
`control.tracking_progress_rollback_window` limits how far the controller may search backward
on a refreshed local plan, which helps path rejoin stay forward-progressive instead of snapping
between old and newly republished nearby poses.
`control.tracking_target_hysteresis_distance` and `control.tracking_target_reset_distance` slow
down target switching during normal tracking.
The `control.rejoin_*` parameters apply only while bounded rejoin context is active after recovery,
escape, or a large target reacquisition; normal lookahead tracking no longer becomes rejoin just
because `target_dist_m` is greater than the configured target-distance threshold.
`control.rejoin_context_timeout_sec`, `control.rejoin_context_distance_m`, and
`control.rejoin_target_jump_threshold_m` bound that context so straight-line damping can remain
active during ordinary straight tracking.
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
