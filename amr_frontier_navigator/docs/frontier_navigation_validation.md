# Frontier Navigation Validation

## Test status

`NOT RUN — hardware unavailable`

Run these checks only with a low speed limit, in an open area, with an operator able to
stop the robot immediately. This document does not claim that SLAM or hardware behavior
has been validated by the automated fake integration tests.

## Startup checks

After sourcing ROS 2 Humble and the workspace, run:

```bash
ros2 run amr_frontier_navigator validate_frontier_navigation.sh check
```

Or inspect each contract directly:

```bash
ros2 lifecycle get /ft_navigator
ros2 action list -t
ros2 topic list
ros2 service list
ros2 topic info -v /frontier/status
ros2 topic info -v /frontier/unknown_goal
ros2 topic info -v /frontier/known_goal
ros2 topic info -v /frontier/global_plan
ros2 topic info -v /frontier/local_plan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_footprint
```

Confirm that `/ft_navigator` is `active`, both navigation actions and `/plan_segment`
exist, `/map` is updating, and the `map -> odom -> base_footprint` TF chain resolves.
The goal/status frontier publishers and subscribers should report reliable,
transient-local QoS; path overlays should remain volatile.

## Monitoring

Use separate terminals so none of the streams hides another:

```bash
ros2 topic echo /frontier/status
ros2 topic echo /frontier/unknown_goal
ros2 topic echo /frontier/known_goal
ros2 topic echo /frontier/global_plan
ros2 topic echo /frontier/local_plan
```

## Send an unknown-capable goal

Replace `GOAL_X`, `GOAL_Y`, and the quaternion with a safe test pose. The example keeps
the reserved retry field disabled and lets zero-valued policy fields use node defaults.

```bash
ros2 action send_goal --feedback /navigate_to_unknown_pose \
  amr_msgs/action/NavigateToUnknownPose \
  "{unknown_goal: {header: {frame_id: map}, pose: {position: {x: GOAL_X, y: GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, allow_final_unknown_retry: false, max_iterations: 0, goal_known_wait_timeout_sec: 0.0, max_staging_search_radius_m: 0.0, min_staging_progress_m: 0.0}"
```

Do not enable `allow_final_unknown_retry`; it remains reserved and has no runtime
meaning in the current orchestrator.

## Record a diagnostic bag

The helper records the minimum navigation, SLAM, overlay, motion, and hidden action
topics needed to reconstruct a transition:

```bash
ros2 run amr_frontier_navigator validate_frontier_navigation.sh record frontier_run_01
```

Equivalent explicit command:

```bash
ros2 bag record --include-hidden-topics -o frontier_run_01 \
  /map /tf /tf_static /scan \
  /global_costmap /local_costmap /global_plan /local_plan \
  /frontier/unknown_goal /frontier/known_goal \
  /frontier/global_plan /frontier/local_plan /frontier/status \
  /motion_status /cmd_vel \
  /navigate_to_unknown_pose/_action/feedback \
  /navigate_to_unknown_pose/_action/status \
  /navigate_to_unknown_pose/_action/send_goal \
  /navigate_to_unknown_pose/_action/get_result \
  /navigate_to_unknown_pose/_action/cancel_goal \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/send_goal \
  /navigate_to_pose/_action/get_result \
  /navigate_to_pose/_action/cancel_goal
```

## Manual scenario sequence

For each case, record the requested pose, phase sequence, iteration count, delegated
goal poses, cancel timestamps, terminal action code, and whether overlays cleared.

1. Send a known/free goal inside the current map and verify one direct delegation.
2. Send a goal just outside the map boundary and verify it is not directly delegated.
3. Verify the selected staging cell is known/free, has clearance, and has a successful
   `/plan_segment` response before `/navigate_to_pose` receives it.
4. Move the robot toward the staging goal and observe SLAM map expansion.
5. Confirm `/frontier/status.original_goal_known` becomes true at the original cell.
6. Confirm staging cancellation reaches a terminal state before the original goal is
   sent, and verify the original orientation is unchanged.
7. Cancel while staging navigation is active; verify both actions terminate canceled.
8. Cancel during `WAITING_FOR_MAP_EXPANSION`.
9. Immediately send the same or a different goal and verify it is accepted cleanly.
10. Temporarily deactivate the planner or delegated action server, then verify bounded
    failure with a useful error and no stale overlays.
11. Repeat a mixture of success, cancel, and failure at least ten times; confirm no
    duplicate terminal results, repeated staging point, or rejected clean follow-up.
12. Start `amr_visualization` after navigation has begun and confirm the latched original
    goal, known/staging goal, and status appear; then confirm all overlays clear at the
    terminal result.

The expected transition for map expansion is:

```text
WAITING_FOR_MAP
RESOLVING_STAGING_GOAL
NAVIGATING_TO_STAGING_GOAL
(cancel staging and wait for its terminal result)
NAVIGATING_TO_ORIGINAL_GOAL
SUCCEEDED
```
