# amr_global_planner

Global A* planner on the inflated global costmap.

## Role

- consumes `/amr/costmap/global`
- computes route segments and full routes
- publishes the currently computed global path

## Services

- `/amr/global_planner/plan_segment`
- `/amr/global_planner/plan_route`

## Output

- `/amr/planner/global`
