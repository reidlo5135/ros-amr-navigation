# Rosbag Profiles

The navigation stack now consumes external `slam_toolbox` online-async output and
standard ROS 2 topic names. Prefer recording TF, `/map`, plans, costmaps, and
controller status rather than legacy localization pose topics.

## Light Profile

Used by `scripts/record_nav_bag_light.sh`:

- `/observation/runtime/summary`
- `/observation/runtime/events`
- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/cmd_vel`

## Debug Profile

Used by `scripts/record_nav_bag_debug.sh`:

- `/observation/runtime/summary`
- `/observation/runtime/events`
- `/motion_command`
- `/motion_status`
- `/local_plan_status`
- `/global_plan`
- `/local_plan`
- `/global_costmap`
- `/local_costmap`
- `/map`
- `/scan`
- `/tf`
- `/tf_static`
- `/cmd_vel`

## Full Profile

`scripts/record_nav_bag_full.sh` records all visible topics for short debugging
windows.
