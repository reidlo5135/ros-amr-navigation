# amr_global_planner

Global A* planner on the inflated global costmap.

## Role

- consumes `/amr/costmap/global`
- computes route segments and full routes
- uses exact-footprint collision checks on the inflated global costmap
- supports path simplification and turn-penalty tuning through `amr.yaml`
- can bias A* toward holding the start row early and aligning to the goal row near the end
- prefers a direct same-row/same-Y path when line-of-sight and footprint collision checks are safe
- publishes the currently computed global path

## Services

- `/amr/global_planner/plan_segment`
- `/amr/global_planner/plan_route`

## Output

- `/amr/planner/global`

## Same-Row Straightening

For `0.18.x` navigation quality stabilization, the planner checks whether the start and goal are on
the same grid row or nearly the same world Y. If the direct line is safe on the inflated global
costmap, including footprint collision checks and unknown-cell policy, the planner publishes an
interpolated straight global path before running A*. If the line is blocked or outside the map, the
existing A* path is used.

The `plan_quality` structured log records `start_row`, `goal_row`, `row_delta`,
`same_row_candidate`, `straight_line_safe`, `straight_path_used`, `fallback_reason`,
`raw_path_points`, `final_path_points`, `max_row_deviation`, and `max_lateral_deviation_m`.
