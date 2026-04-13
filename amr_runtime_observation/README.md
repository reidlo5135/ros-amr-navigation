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

## Launch

This node is started from `amr_bringup/launch/navigation.launch.py`.
