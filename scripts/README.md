# Scripts

Helper scripts for running and observing the navigation stack.

## Launch Helpers

- `run_turtlebot3_nohup.sh`: starts robot bringup under `nohup`
- `run_navigation_nohup.sh`: starts `amr_bringup navigation.launch.py` under `nohup`

Run external `slam_toolbox` online SLAM separately between robot bringup and AMR
navigation.

## Rosbag Helpers

- `record_nav_bag_light.sh`: records observation, command/status, and `/cmd_vel`
- `record_nav_bag_debug.sh`: records light topics plus plans, local costmap, scan, and TF
- `record_nav_bag_full.sh`: records all visible topics for short debug windows

The light/debug profiles use the standard navigation contract:

- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/global_plan`
- `/local_plan`
- `/local_costmap`
- `/scan`
- `/tf`
- `/tf_static`
- `/cmd_vel`

## Log Helpers

- `watch_amr_logs.sh`: filters `AMR_LOG schema=v1` console output
- `extract_nav_quality.sh`: extracts selected navigation quality events from logs
