# amr_bt_navigator

BehaviorTree.CPP v3 based navigation decision server.

## Role

- serves `NavigateToPose`
- serves `NavigateToPoses`
- requests global plans
- dispatches motion commands
- monitors progress and blocked states
- chooses recovery behaviors through BT flow

## Current Recovery Direction

Nav2-style responsibility split:
- `amr_bt_navigator`: decide
- `amr_global_planner`: global replan
- `amr_controller_server`: local replan / local escape / execute
- `amr_recovery_server`: wait / backup / spin commands

Current `0.16.x` recovery policy starts with one planner-local escape attempt before
falling back to heavier recovery behaviors or global replanning when the local planner
still reports a blocked path.
For `NavigateToPoses`, intermediate waypoint goals keep heading-free execution while only the
final goal requests explicit heading alignment at arrival.
The BT now limits local escape attempts to planner-owned recovery requests for the currently
active motion command, so controller-only blocked/stalled reports do not spuriously re-dispatch
an escape plan from stale planner state.
The fixed recovery policy is now:
- `DECISION_HARD_BLOCKED`: one planner-local escape attempt, then `backup`, then `spin`, then fresh global replan
- `DECISION_GOAL_PROXIMITY_BLOCKED`: skip local escape, `wait`, and re-dispatch the current plan before escalating further
- `DECISION_GLOBAL_REPLAN_REQUIRED`: skip intermediate recovery commands and go straight to fresh global replanning
- controller-owned `blocked/stalled`: `wait -> backup -> spin -> fresh global replan`
`/amr/local_planner/plan_local_escape` also returns stable reason labels such as
`local_escape_map_unavailable`, `local_escape_empty_source_plan`, and
`local_escape_no_valid_path` so operator-side logs can distinguish planner refusal from later
fallback recovery behavior.

## Important Files

- `config/navigate_to_pose.xml`
- `src/amr_bt_navigator/bt_navigator.cpp`

## Inputs

- `/amr/localization/pose`
- `/amr/motion/status`
- `/amr/global_planner/plan_segment`
- `/amr/local_planner/plan_local_escape`
- `/amr/recovery_server/plan_recovery`
- `/amr/costmap_server/clear_costmap`

## Output

- `/amr/motion/command`
