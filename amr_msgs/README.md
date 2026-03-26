# amr_msgs

Custom ROS interfaces used by the AMR stack.

## Includes

- actions:
  - `NavigateToPose`
- messages:
  - `MotionCommand`
  - `MotionStatus`
  - `LocalizationStatus`
- services:
  - `PlanSegment`
  - `PlanRoute`
  - `PlanLocalEscape`
  - `PlanRecovery`
  - `ClearCostmap`
  - `TriggerGlobalLocalization`

These interfaces are shared across bringup, planning, recovery, MQTT, and visualization paths.
