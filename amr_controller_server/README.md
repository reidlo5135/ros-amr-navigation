# amr_controller_server

Controller-layer server package for the AMR navigation stack.

## Role

`amr_controller_server` owns the runtime boundary and source code for local control while
preserving the existing lifecycle node contracts:

- `/amr/local_planner`: builds the short-horizon local plan from the global route.
- `/amr/motion_controller`: tracks the local plan, applies safety gating, and publishes `cmd_vel`.
- local path refiner: prunes near-duplicate poses, densifies long segments, smooths safe corners, and assigns path headings.
- goal checker: separates XY / yaw / hold-time arrival policy from velocity tracking logic.

This mirrors the Nav2-style server file layout with one `controller_server.hpp`,
one `controller_server.cpp`, and one `main.cpp`. The process hosts both existing lifecycle nodes
so lifecycle management, parameters, and topics remain compatible.

## Launch

`amr_bringup` includes this launch file and passes the consolidated parameter file:

```bash
ros2 launch amr_controller_server controller.launch.py params_file:=/path/to/amr.yaml
```

## Parameters

`/amr/local_planner` exposes `path_refiner.*` parameters for safe path post-processing,
including collision-checked corner smoothing with fallback to the unsmoothed local plan.
`/amr/motion_controller` exposes `goal_checker.*` parameters for arrival policy tuning.
By default, the checker respects the incoming goal yaw with a loose Nav2-style tolerance,
while `ignore_yaw` can still disable yaw checking for loose waypoint-style goals.

## Next Direction

Progress checking, safety gate, velocity control, and local planning should keep moving toward
package-local classes or plugins under this controller boundary.
