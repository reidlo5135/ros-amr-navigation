# Field Debug Scripts

These scripts provide a repeatable field workflow for running the AMR navigation stack,
capturing logs, recording rosbag2 data, and extracting navigation quality signals.

All scripts source `scripts/amr_logging_env.sh`. The shared environment script sources
`/opt/ros/humble/setup.bash` and `${AMR_WS_DIR}/install/setup.bash` when available.

## Common Environment

```bash
source scripts/amr_logging_env.sh
```

Useful overrides:

| Variable | Default | Purpose |
| --- | --- | --- |
| `AMR_WS_DIR` | parent ROS workspace when detected | Workspace containing `install/setup.bash` |
| `AMR_LOG_ROOT` | `${AMR_WS_DIR}/log/field` | Root for text logs |
| `AMR_BAG_ROOT` | `${AMR_WS_DIR}/bags` | Root for rosbag2 output |
| `AMR_NOHUP_ROOT` | `${AMR_LOG_ROOT}/nohup` | `nohup` logs and pid files |
| `AMR_RUN_ID` | timestamp | Shared run id for logs and bags |
| `ROS_DOMAIN_ID` | `0` | ROS domain id |
| `TURTLEBOT3_MODEL` | `burger` | TurtleBot3 model for TB3 bringup |

## Running Processes

Start navigation under `nohup`:

```bash
scripts/run_navigation_nohup.sh
```

Enable the optional MQTT bridge only when MQTT/remote-client operation is needed:

```bash
scripts/run_navigation_nohup.sh use_mqtt_server:=true
```

Start TurtleBot3 bringup under `nohup`:

```bash
scripts/run_turtlebot3_nohup.sh
```

Stop started processes:

```bash
scripts/stop_nohup_process.sh --label all
```

## Recording Bags

Light profile for repeated quality checks:

```bash
scripts/record_nav_bag_light.sh
```

Debug profile with paths, local costmap, scan, odometry, and TF:

```bash
scripts/record_nav_bag_debug.sh
```

Full profile for short targeted captures:

```bash
scripts/record_nav_bag_full.sh
```

Replay:

```bash
scripts/replay_nav_bag.sh /path/to/bag
```

## Watching Logs

Follow all structured AMR logs:

```bash
scripts/watch_amr_logs.sh
```

Follow recovery decisions only:

```bash
scripts/watch_amr_logs.sh --event recovery_decision
```

Filter controller command quality without following:

```bash
scripts/watch_amr_logs.sh --component controller --event cmd_quality --no-follow
```

## Extracting Navigation Quality

Extract the core quality events from `nohup` logs:

```bash
scripts/extract_nav_quality.sh --output /tmp/nav_quality.tsv
```

CSV output is also available:

```bash
scripts/extract_nav_quality.sh --format csv --output /tmp/nav_quality.csv
```

The extractor includes these event families by default:

- `goal_state`
- `goal_transition_state`
- `cmd_quality`
- `tracking_state`
- `tracking_heading_debug`
- `tracking_frame_mismatch`
- `local_path_quality`
- `local_path_degenerate`
- `target_jump_detected`
- `recovery_decision`
- `recovery_started`
- `recovery_finished`
- `recovery_skipped`
- `rejoin_state`
- `local_blocked_state`
- `localization_guard`
- `wheel_slip_state`
- `odom_motion_guard`
- `map_odom_correction`
- `plan_quality`

For straight-line oscillation checks, the extractor also carries through `rejoin_context_active`,
`straight_segment`, `path_curvature_score`, `lateral_error_m`, `heading_error_raw_rad`,
`heading_error_filtered_rad`, `steering_deadband_active`, `steering_hysteresis_state`,
`cmd_ang_sign`, `cmd_ang_flip_count`, `output_ang_sign`, and `output_ang_flip_count`.
The final `nav_quality_summary` row reports `cmd_ang_abs_avg`, `cmd_ang_abs_max`,
`output_ang_abs_avg`, `output_ang_abs_max`, `cmd_ang_sign_flip_count`,
`output_ang_sign_flip_count`, `straight_segment_ratio`, and
`straight_cmd_ang_interference_count` for before/after comparisons.
For stair-step path diagnosis, inspect `local_path_quality` columns:
`raw_path_points`, `simplified_path_points`, `refined_path_points`, `path_length_m`,
`path_curvature_score`, `lateral_error_m`, `line_of_sight_simplified`,
`collinear_pruned_count`, and `collision_check_passed`.

For degenerate local path diagnosis, inspect `local_path_degenerate` columns:
`global_path_points`, `raw_path_points`, `refined_path_points`, `path_length_m`, `pose_frame`,
`plan_frame`, `nearest_idx`, `start_idx`, `end_idx`, `lookahead_m`, `dist_goal_m`,
`xy_reached`, `fallback_used`, and `reason`.

For same-row global planning checks, inspect `plan_quality` columns:
`start_row`, `goal_row`, `row_delta`, `same_row_candidate`, `straight_line_safe`,
`straight_path_used`, `fallback_reason`, `final_path_points`, `max_row_deviation`, and
`max_lateral_deviation_m`.

For `0.18.0` path tracking checks, start with `target_jump_detected` and
`tracking_heading_debug`. A suspicious jump should include `previous_idx`, `nearest_idx`,
`candidate_idx`, `selected_idx`, `target_jump_m`, `rejoin_activated`, and
`selection_reason` so the local target choice can be reconstructed without logging full path data.

For goal approach checks, inspect `goal_state`. XY arrival should latch before final yaw alignment:
`xy_reached=true`, `final_heading_required=true`, and `phase=final_heading_align` should pair with
`cmd_lin=0.000`. Completion should end with `phase=reached`, `cmd_lin=0.000`, and `cmd_ang=0.000`.

For recovery rejoin checks, read navigator `recovery_decision`, `recovery_started`,
`recovery_finished`, controller `rejoin_state`, and `local_blocked_state` together. A clean recovery
exit has `recovery_finished result=reacquired` followed by controller tracking with
`rejoin_context_active=false`. `result=reacquire_timeout` means the post-recovery settle window
expired before motion status and local plan status both became healthy.

For `0.18.1` wheel-slip checks, filter localization logs:

```bash
scripts/watch_amr_logs.sh --component localization --event wheel_slip_state
scripts/watch_amr_logs.sh --component localization --event odom_motion_guard
scripts/watch_amr_logs.sh --component localization --event map_odom_correction
```

Compare `odom_delta_m`, `odom_delta_yaw_rad`, `pose_delta_m`, `pose_delta_yaw_rad`,
`applied_delta_m`, `applied_delta_yaw_rad`, `slip_suspected`, `stall_suspected`, `confirmed`, and
`reason`. These are summary fields only; raw scan ranges, costmap grids, and map data are not logged.

## 0.18.1 Field Validation Checklist

Do not run these scenarios from automation. Use them only as an operator checklist during an
attended field run, then inspect logs afterward.

Common log query:

```bash
grep -R "plan_quality\|local_path_quality\|local_path_degenerate\|goal_transition_state\|wheel_slip_state\|odom_motion_guard\|map_odom_correction" ~/.ros/log | tail -200
```

Scenario A: same-Y goals

- Send goal 1 and goal 2 on the same Y line or the same map row.
- Expected: `plan_quality same_row_candidate=true`.
- Expected: if line-of-sight and footprint checks are safe, `straight_path_used=true`.
- Expected: if obstacle, unknown, or map boundary checks fail, `fallback_reason` is recorded and A* is used.

Scenario B: GP/LP lateral deviation

- Use a straight global plan and compare the local plan in RViz.
- Expected: `local_path_quality lateral_error_m` stays low relative to previous runs.
- Expected: `path_curvature_score` is not excessive for the straight section.
- Expected: controller logs do not show repeated small left/right oscillation in `cmd_quality`.

Scenario C: goal transition and rotate-in-place

- Complete goal 1, then start moving toward goal 2 with an in-place turn.
- Expected: `goal_transition_state phase=new_command` is logged.
- Expected: during commanded rotate-in-place, `wheel_slip_state confirmed=true` does not appear.
- Expected: `map_odom_correction correction_limited=true` does not repeat excessively.
- Expected: RViz does not show a large `map -> odom` split during the turn.

Scenario D: wheel slip or physical stall

- With an operator present, reproduce a low obstacle or physical stall condition that LiDAR cannot
	reliably see.
- Expected: after confirm cycles, `wheel_slip_state confirmed=true` appears.
- Expected: `odom_motion_guard applied_delta_m` is smaller than `odom_delta_m`.
- Expected: `map_odom_correction` limits hard jumps instead of allowing a large TF correction.
- Expected: controller or navigator status moves toward blocked/recovery flow instead of continuing
	to push indefinitely.

Current `0.18.1` path-refiner defaults set `path_refiner.preserve_goal_orientation=false` to favor
path tangent continuity while `/amr/motion_controller.goal_checker.respect_goal_yaw=false` is used.
For docking, precise final alignment, or other yaw-critical workflows, re-enable or explicitly
optionize goal-orientation preservation before validating that workflow.
