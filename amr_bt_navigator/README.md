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
