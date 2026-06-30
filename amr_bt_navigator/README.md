# amr_bt_navigator

Lifecycle behavior-tree navigator for the AMR navigation core.

Actions:

- `/navigate_to_pose`
- `/navigate_to_poses`

Inputs:

- `/motion_status`
- `/local_plan_status`
- TF `map -> odom -> base_*`

Outputs:

- `/motion_command`

Service clients:

- `/plan_segment`
- `/plan_recovery`
- `/plan_local_escape`
- `/clear_costmap`

The navigator gets the current robot pose from TF and no longer depends on a
localization pose topic. If TF or the global planner service is not ready, the
current navigation request fails cleanly or waits according to the behavior-tree
state instead of crashing the node.
