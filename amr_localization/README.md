# amr_localization

Localization node for the AMR stack.

## Role

- consumes odometry, scan, and map
- estimates the robot pose in `map`
- publishes `map -> odom`
- publishes localized pose and odometry topics used by the rest of the stack

`map -> odom` is published only after the particle filter has an initial pose. For
field bringup, keep `start_pose.enabled: false` and set `/amr/localization/initial_pose`
from RViz, the operator app, or a measured launch-time pose before sending goals. This
prevents an unverified `(0, 0, 0)` map pose from rotating or translating the full
`map -> odom -> base_footprint` chain.

## Outputs

- `/amr/localization/pose`
- `/amr/localization/odometry`
- TF `map -> odom`

## TF Ownership

- `amr_localization` is the only in-repository node that broadcasts `map -> odom` in
  localization mode.
- `robot_base_driver` owns `odom -> base_footprint`.
- `robot_state_publisher` owns the URDF-derived robot tree such as
  `base_footprint -> base_link -> base_scan`.
- Do not add a static `map -> odom` publisher to make RViz look aligned. A jump in
  `map -> odom` is a localization/map/initial-pose problem, not an odometry sign fix.
