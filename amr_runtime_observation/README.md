# amr_runtime_observation

Runtime observation package for the AMR navigation stack.

## Role

- subscribes to navigation runtime signals that already exist inside the AMR stack
- aggregates route progress, motion status, and local-plan decisions into one summarized view
- publishes lightweight JSON summary and event topics for downstream analyzers

## Inputs

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

This keeps `0.15.9` focused on observable recovery diagnosis. Recovery policy changes such as
`local_escape-first` stay outside this patch line.

## Launch

This node is started from `amr_bringup/launch/navigation.launch.py`.
