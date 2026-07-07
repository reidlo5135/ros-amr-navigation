# amr_frontier_navigator

`amr_frontier_navigator` provides the optional unknown-goal navigation layer for the
`slam_toolbox + AMR` online navigation stack.

It does not publish `/motion_command`. It accepts an original goal that may be in
unknown SLAM space, resolves a reachable known/free staging goal from the latest
`/map`, then delegates movement to the existing `/navigate_to_pose` action server.

## Node

- package: `amr_frontier_navigator`
- executable: `ft_navigator`
- node name: `/ft_navigator`
- lifecycle node: yes

Enable it from bringup:

```bash
ros2 launch amr_bringup navigation.launch.py use_frontier_navigation:=true
```

## Algorithm

1. Receive `/navigate_to_unknown_pose`.
2. Transform the requested goal into the `map` frame.
3. Wait for `/map` and the `map -> odom -> base_*` TF chain.
4. Classify the original goal in the latest `nav_msgs/OccupancyGrid`.
5. If the original goal is known/free, send it directly to `/navigate_to_pose`.
6. If it is occupied or non-traversable, abort with an action result error.
7. If it is unknown or outside the map, search around the original goal for known/free cells.
8. If local candidates are not reachable, fall back to map-wide frontier-adjacent free cells.
9. Reject cells without obstacle clearance or without a valid `/plan_segment` from the current pose.
10. Score candidates by distance to the original goal, plan length, and frontier adjacency.
11. Navigate to the selected staging goal through `/navigate_to_pose`.
12. While staging navigation runs, monitor `/map`; if the original goal becomes known/free, cancel staging and switch to the original goal.
13. After staging completes, wait briefly for map expansion, then repeat until the original goal is reached or limits are hit.

The staging goal orientation faces the original goal. When final navigation to the
original goal starts, the original user-provided orientation is preserved.

## Interfaces

Inputs:

| Interface | Default |
| --- | --- |
| map | `/map` |
| local plan proxy source | `/local_plan` |
| delegated action | `/navigate_to_pose` |
| reachability service | `/plan_segment` |
| TF | `map -> odom -> base_footprint` or `base_link` fallback |

Outputs:

| Interface | Type |
| --- | --- |
| `/navigate_to_unknown_pose` | `amr_msgs/action/NavigateToUnknownPose` |
| `/frontier/unknown_goal` | `geometry_msgs/msg/PoseStamped` |
| `/frontier/known_goal` | `geometry_msgs/msg/PoseStamped` |
| `/frontier/global_plan` | `nav_msgs/msg/Path` |
| `/frontier/local_plan` | `nav_msgs/msg/Path` |
| `/frontier/status` | `amr_msgs/msg/FrontierNavigationStatus` |

`/frontier/local_plan` proxies `/local_plan` only while an unknown-goal action is active.
Inactive cleanup publishes empty paths and inactive status so visualization clients can
hide overlay layers.

## Parameters

Main parameters are grouped under `/ft_navigator` in
`amr_bringup/params/amr.yaml`.

| Parameter | Default |
| --- | --- |
| `resolver.free_threshold` | `25` |
| `resolver.occupied_threshold` | `65` |
| `resolver.candidate_step_cells` | `1` |
| `resolver.min_obstacle_clearance_m` | `0.20` |
| `resolver.max_staging_search_radius_m` | `3.0` |
| `resolver.goal_tolerance_m` | `0.15` |
| `resolver.map_wait_timeout_sec` | `5.0` |
| `resolver.goal_known_wait_timeout_sec` | `2.0` |
| `resolver.max_iterations` | `8` |
| `resolver.min_staging_progress_m` | `0.15` |
| `resolver.max_candidate_checks` | `500` |
| `resolver.use_plan_segment_validation` | `true` |

All topic, action, service, and frame names are parameterized.

## Failure Behavior

The node reports failures through the action result instead of throwing process-killing
exceptions. Common result codes include:

- `MAP_NOT_READY`: no valid SLAM map arrived before timeout
- `TF_ERROR`: current pose or goal transform failed
- `OCCUPIED_GOAL`: the original goal is known but non-traversable
- `NO_REACHABLE_STAGING_GOAL`: no candidate passed clearance and `/plan_segment`
- `STAGING_NAVIGATION_FAILED`: delegated navigation failed before map expansion
- `FINAL_NAVIGATION_FAILED`: delegated final navigation failed
- `CANCELED`: unknown-goal action cancel was requested, and the nested goal was canceled

## Manual Checks

1. Known goal: send a normal single goal from `amr_visualization` and confirm the backend delegates to `/navigate_to_pose`.
2. Unknown goal: send a single goal in unknown `/map` space and verify `/frontier/unknown_goal`, `/frontier/known_goal`, and `/frontier/global_plan`.
3. Map expansion: drive to the staging goal and confirm final navigation starts when the original goal becomes known/free.
4. Cancel: cancel the unknown-goal action and confirm the delegated `/navigate_to_pose` goal is canceled.
5. Failure: choose an unreachable unknown goal and confirm the action aborts without a node crash.
