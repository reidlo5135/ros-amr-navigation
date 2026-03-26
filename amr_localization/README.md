# amr_localization

Localization node for the AMR stack.

## Role

- consumes odometry, scan, and map
- estimates the robot pose in `map`
- owns tracking vs global-relocalization mode for kidnapped recovery
- can start directly in global relocalization instead of forcing a fixed initial pose
- publishes `map -> odom`
- publishes localized pose and odometry topics used by the rest of the stack
- publishes localization status and exposes a trigger for global relocalization

## Outputs

- `/amr/localization/pose`
- `/amr/localization/odom`
- `/amr/localization/status`
- TF `map -> odom`
- `/amr/localization/trigger_global_localization`
