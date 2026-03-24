# amr_motion_controller

Motion execution and progress checking node.

## Role

- follows the local path
- publishes `/cmd_vel`
- reports motion status
- enforces near-field stop logic from current scan
- participates in recovery execution flow

## Inputs

- `/amr/motion/command`
- `/amr/planner/local`
- `/amr/localization/pose`
- `/scan`

## Outputs

- `/cmd_vel`
- `/amr/motion/status`
