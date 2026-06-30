# amr_global_planner

Lifecycle global planner for the AMR navigation core.

Inputs:

- `/global_costmap`

Outputs:

- `/global_plan`

Services:

- `/plan_segment`
- `/plan_route`

The planner does not require a static map file. It plans on the latest
`/global_costmap` published by `amr_costmap_server`, which is built from the live
`/map` supplied by external `slam_toolbox`. If the costmap is not ready, plan
requests return failure while the node remains active.
