# amr_runtime_observation

Runtime observer for navigation status and progress summaries.

Inputs:

- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/navigate_to_poses/_action/feedback`
- `/navigate_to_poses/_action/status`

Outputs:

- `/observation/runtime/summary`
- `/observation/runtime/events`

This node is observational only. It does not own map, localization, TF, planning,
or control.
