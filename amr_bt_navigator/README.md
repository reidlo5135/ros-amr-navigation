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
- `amr_local_planner`: local replan / local escape
- `amr_motion_controller`: execute
- `amr_recovery_server`: wait / backup / spin commands

## Important Files

- `config/navigate_to_pose.xml`
- `src/amr_bt_navigator/bt_navigator.cpp`

## Inputs

- `/amr/localization/pose`
- `/amr/motion/status`
- `/amr/global_planner/plan_segment`
- `/amr/recovery_server/plan_recovery`
- `/amr/costmap_server/clear_costmap`

## Output

- `/amr/motion/command`
