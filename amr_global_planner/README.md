# amr_global_planner

Global A* planner on the inflated global costmap.

## Role

- consumes `/amr/costmap/global`
- computes route segments and full routes
- uses exact-footprint collision checks on the inflated global costmap
- supports path simplification and turn-penalty tuning through `amr.yaml`
- publishes the currently computed global path

## Services

- `/amr/global_planner/plan_segment`
- `/amr/global_planner/plan_route`

## Output

- `/amr/planner/global`
