# amr_local_planner

`amr_local_planner` converts a global path into a short-horizon local plan using the current pose, the local costmap, and local A* replanning.

## Interfaces

- subscribes: `/amr/motion/command`
- subscribes: `/amr/localization/pose`
- subscribes: `/amr/costmap/local`
- subscribes: `/amr/obstacle/report`
- publishes: `/amr/planner/local`

## Local Planning Logic

- receives the latest `MotionCommand` from the navigator
- finds progress on the global path relative to the current pose
- selects a short-horizon local goal using `lookahead_distance`
- consumes the local costmap from `amr_costmap_server`
- runs local A* from the current pose to the local goal on the local costmap
- falls back to sliced path output if local replanning fails

## Local Replanning Flow

```mermaid
flowchart LR
    A[MotionCommand.global path] --> B[find current progress]
    C[/amr/localization/pose/] --> B
    D[/amr/costmap/local/] --> E[use local costmap]
    J[/amr/obstacle/report/] --> E
    B --> F[select lookahead goal]
    E --> G[local A* replan]
    F --> G
    G --> H[/amr/planner/local/]
```

## Notes

- the current implementation is still a lightweight local A* replanner rather than a full DWB or TEB controller
- local replanning is now triggered from obstacle reports rather than raw scan interpretation inside the planner
