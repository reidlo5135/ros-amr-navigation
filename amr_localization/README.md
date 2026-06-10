# amr_localization

Localization node for the AMR stack.

## Role

- consumes odometry, scan, and map
- estimates the robot pose in `map`
- publishes `map -> odom`
- publishes localized pose and odometry topics used by the rest of the stack
- guards localization against wheel slip or physical stall when odometry reports motion that scan/pose progress and controller status do not confirm

## Outputs

- `/amr/localization/pose`
- `/amr/localization/odometry`
- TF `map -> odom`

## Inputs

- `/odom`
- `/scan`
- `/amr/map/data`
- `/amr/localization/initial_pose`
- `/cmd_vel` for optional command/motion consistency checks
- `/amr/motion/status` for optional blocked/stalled and goal-proximity context

## 0.18.1 Localization Guard

The `0.18.1` guard is intentionally conservative for TurtleBot3 Burger-class low-speed operation.
It does not replace costmap or recovery decisions. It only reduces localization damage when wheel
odometry moves farther than expected while the localized pose, scan likelihood, command, and motion
status do not agree that the robot really progressed.

The guard has two parts:

- `localization_guard`: detects suspected slip/stall and dampens the odometry delta used by the
  particle motion update.
- `map_odom_guard`: bounds each `map -> odom` correction step so suspected slip does not create a
  sudden TF jump. Initial pose reset bypasses the limiter.

Structured logs stay on `AMR_LOG schema=v1`:

- `localization_guard`: configuration summary at lifecycle configure.
- `wheel_slip_state`: state transition when slip/stall is confirmed or cleared.
- `odom_motion_guard`: throttled summary when odometry motion is damped or clamped.
- `map_odom_correction`: throttled WARN when `map -> odom` correction is limited or unusually large.

The logs include summary deltas and reason codes only. They do not print raw scan ranges, costmap
grid data, or map data.

## Key Parameters

Parameters live under `/amr/localization` in `amr_bringup/params/amr.yaml`.

- `topics.velocity`: command topic used for motion consistency checks. Default: `/cmd_vel`.
- `topics.motion_status`: controller status topic used for blocked/stalled and goal-proximity context. Default: `/amr/motion/status`.
- `localization_guard.odom_translation_slip_threshold_m`: odometry translation delta that can start slip suspicion.
- `localization_guard.odom_rotation_slip_threshold_rad`: odometry yaw delta that can start slip suspicion.
- `localization_guard.pose_translation_confirm_threshold_m`: localized pose movement considered too small to confirm odometry progress.
- `localization_guard.confirm_cycles`: consecutive suspect updates required before guard confirmation.
- `localization_guard.clear_cycles`: consecutive clear updates required before clearing the guard.
- `localization_guard.odom_translation_gain_when_slipping`: translation scale applied to the motion update during slip/stall.
- `localization_guard.odom_rotation_gain_when_slipping`: yaw scale applied to the motion update during slip/stall.
- `map_odom_guard.max_correction_translation_per_update_m`: maximum translation correction per `map -> odom` publish.
- `map_odom_guard.max_correction_rotation_per_update_rad`: maximum yaw correction per `map -> odom` publish.
