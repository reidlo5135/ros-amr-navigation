# Map Odom Debug

## Quick Checks

Run these commands while localization is active:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /tf -v
ros2 node list | grep -E "amcl|slam|localization|map"
```

## What To Watch

- `map -> odom` should not jump while the robot is stationary.
- Large yaw jumps usually mean scan-to-map localization mismatch, bad initial pose, or a frame/layout mismatch.
- Compare the live scan overlay against the map in RViz before blaming controller behavior.
- If jumps happen only after motion starts, compare `/odom` pose continuity against `map -> odom` correction size.

## Enable Debugging

Set these localization parameters when reproducing the issue:

```yaml
debug_map_odom: true
max_map_odom_jump_xy: 0.20
max_map_odom_jump_yaw: 0.20
clamp_map_odom_jump: false
max_map_odom_correction_xy_per_update: 0.05
max_map_odom_correction_yaw_per_update: 0.05
```

When `debug_map_odom` is enabled, the localization node logs:

- previous and new `map -> odom` x/y/yaw
- delta x/y/yaw per correction
- input odom pose and estimated map pose
- scan frame id
- scan timestamp age
- odom timestamp age
- update reason such as `odometry_motion_update`, `scan_measurement_update`, or `initial_pose`

## Interpretation

- Stationary robot + repeated `scan_measurement_update` yaw jumps:
  scan matching or frame geometry is likely inconsistent with the map.
- Stable odom pose + unstable `map -> odom`:
  localization correction is the primary suspect, not wheel odometry.
- Large scan age or odom age:
  stale sensor timing may be feeding correction on delayed data.
- Repeated warnings above the jump thresholds:
  enable clamp mode temporarily to confirm whether correction spikes are the direct cause of navigation instability.
