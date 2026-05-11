# amr_localization

Localization node for the AMR stack.

## Role

- consumes odometry, scan, and map
- estimates the robot pose in `map`
- publishes `map -> odom`
- publishes localized pose and odometry topics used by the rest of the stack

## Outputs

- `/amr/localization/pose`
- `/amr/localization/odometry`
- TF `map -> odom`
