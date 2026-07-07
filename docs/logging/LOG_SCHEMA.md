# AMR Log Schema

Runtime logs use `AMR_LOG schema=v1` records for navigation core events.

The current runtime assumes `slam_toolbox + AMR` online-async navigation: robot
bringup publishes sensors and base TF, external `slam_toolbox` publishes `/map`
and `map -> odom`, and AMR lifecycle nodes run in the root ROS namespace.

Current core components:

| Component | Node | Responsibility |
| --- | --- | --- |
| `costmap_server` | `/costmap_server` | consume `/map`, `/scan`, TF and publish costmaps |
| `global_planner` | `/global_planner` | plan on `/global_costmap` |
| `controller` | `/local_planner`, `/motion_controller` | local planning and `/cmd_vel` control |
| `bt_navigator` | `/bt_navigator` | navigation actions, planning requests, recovery flow |
| `ft_navigator` | `/ft_navigator` | optional unknown-goal staging orchestration |
| `recovery_server` | `/recovery_server` | recovery motion command planning |
| `runtime_observation` | `/runtime_observation` | status summary and event observation |
| `lifecycle_manager` | `/navigation_manager` | lifecycle bringup for navigation core |

Primary runtime topics:

- `/map`
- `/scan`
- `/tf`
- `/tf_static`
- `/global_costmap`
- `/local_costmap`
- `/global_plan`
- `/local_plan`
- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/cmd_vel`
- `/observation/runtime/summary`
- `/observation/runtime/events`

`map -> odom` is published by external `slam_toolbox`, not by this stack. TF
lookup failures are expected during startup and should appear as throttled
warnings while the nodes wait for the `map -> odom -> base_*` chain.

Useful events:

- `costmap_state`, `costmap_error`, `costmap_clear_requested`, `costmap_clear_done`
- `plan_requested`, `plan_succeeded`, `plan_failed`, `plan_quality`
- `local_path_quality`, `local_path_degenerate`, `local_blocked_state`
- `motion_command`, `tracking_state`, `tracking_heading_debug`, `cmd_quality`, `goal_state`
- `goal_received`, `bt_phase_transition`, `recovery_decision`, `goal_succeeded`, `goal_failed`
- `runtime_summary`
- `lifecycle_configure`, `lifecycle_activate`, `lifecycle_error`
