# amr_msgs

Custom ROS interfaces used by the AMR stack.

## Includes

- actions:
  - `NavigateToPose`
  - `NavigateToPoses`
  - `NavigateToUnknownPose`
- messages:
  - `FrontierNavigationStatus`
  - `LocalPlanStatus`
  - `MotionCommand`
  - `MotionStatus`
  - `ObstacleReport`
  - `SpatialSegment`
  - `SpatialSegmentArray`
- services:
  - `PlanSegment`
  - `PlanRoute`
  - `PlanLocalEscape`
  - `PlanRecovery`
  - `ClearCostmap`

These interfaces are shared across bringup, planning, recovery, MQTT, and visualization paths.
