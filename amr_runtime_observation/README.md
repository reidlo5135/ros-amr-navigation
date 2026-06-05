# amr_runtime_observation

Runtime observation package for the AMR navigation stack.

## Role

- subscribes to navigation runtime signals that already exist inside the AMR stack
- aggregates route progress, motion status, and local-plan decisions into one summarized view
- publishes lightweight JSON summary and event topics for downstream analyzers

## Inputs

- `/amr/motion/command`
- `/amr/motion/status`
- `/amr/planner/local_status`
- `/amr/navigator/navigate_to_poses/_action/feedback`
- `/amr/navigator/navigate_to_poses/_action/status`

## Outputs

- `/amr/observation/runtime/summary`
- `/amr/observation/runtime/events`

## Current Scope

`0.1.0` is intentionally small:

- route active / inactive state
- current goal index and goal count
- number of recoveries from route feedback
- motion blocked / stalled signals
- local planner decision and recovery-required signal
- simple progress-stall heuristic
- event emission on important state changes

## 0.15.9 Observation Baseline

`0.15.9` treats this package as the first stable operator-facing diagnosis boundary for
recovery visibility.

- summary payload exposes `recovery_triggered` and `recovery_reason`
- event payload emits `recovery_trigger_changed` and `recovery_reason_changed`
- planner-driven reasons are normalized into:
  - `planner_goal_proximity_blocked`
  - `planner_global_replan_required`
  - `planner_hard_blocked`
  - `planner_recovery_required`
- motion-driven reasons are normalized into:
  - `motion_safety_gate_blocked`
  - `motion_costmap_blocked`
  - `motion_blocked`
  - `motion_stalled`
- route-level progress fallback remains `progress_stalled`

Goal approach and final heading alignment are not recovery. When the controller is intentionally
rotating in place near the goal to satisfy yaw, observation reports `controller_phase` as
`goal_approach` or `final_heading_align`, suppresses the route-level `progress_stalled` fallback,
and keeps `recovery_triggered=false` unless the controller or current local-plan status reports a
real block/recovery condition.

Runtime summary and event payloads expose `controller_phase`, `controller_recovery`,
`controller_goal_reached`, `dist_goal_delta_m`, and `progress_stall_window_sec` so stale observation
state can be distinguished from controller-owned blocked/stalled/recovery status.

This keeps `0.15.9` focused on observable recovery diagnosis. Recovery policy changes such as
`local_escape-first` were intentionally left for the next patch line.

## 0.15.10 Observation Baseline

`0.15.10` extends the baseline from "why recovery was triggered" to "what stage recovery is in"
and "which blocked layer is currently authoritative".

- summary payload now exposes `blocked_context`
- summary payload now exposes `recovery_phase`
- event payload now emits:
  - `blocked_context_changed`
  - `recovery_phase_changed`
- recovery phase labels are fixed to:
  - `idle`
  - `navigating`
  - `recovery_requested`
  - `recovery_executing`
- blocked context is synthesized in observation from planner, controller, and route signals
  while keeping planner ownership out of `MotionController`

## 0.16.x Observation Extension

`0.16.x` adds visibility for planner-local escape before heavier recovery behaviors.

- summary payload now exposes `local_escape_active`
- event payload now emits `local_escape_state_changed`
- `recovery_phase` can now report:
  - `local_escape_executing`
- local escape is inferred from a `MODE_NAVIGATE` motion-command re-dispatch that occurs while
  planner-owned recovery is active and the planner has not escalated to `global_replan_required`
- current observation still treats `NavigateToPoses` as the primary route-level action lane;
  single-goal `NavigateToPose` remains observable mainly through motion/planner state until a
  separate single-goal action summary path is added

## Launch

This node is started from `amr_bringup/launch/navigation.launch.py`.
