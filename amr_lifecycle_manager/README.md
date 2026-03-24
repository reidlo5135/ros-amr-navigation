# amr_lifecycle_manager

Lifecycle orchestration utility for AMR nodes.

## Role

- configures and activates managed lifecycle nodes
- can publish the initial pose during bringup
- is used by both localization and navigation launch flows

## Typical Usage

- localization manager activates:
  - map server
  - localization
  - costmap server
  - global planner
- navigation manager activates:
  - local planner
  - motion controller
  - recovery server
  - BT navigator
