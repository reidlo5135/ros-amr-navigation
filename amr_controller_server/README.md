# amr_controller_server

Controller-layer server package for the AMR navigation stack.

## Role

`amr_controller_server` owns the runtime boundary and source code for local control while
preserving the existing lifecycle node contracts:

- `/amr/local_planner`: builds the short-horizon local plan from the global route.
- `/amr/motion_controller`: tracks the local plan, applies safety gating, and publishes `cmd_vel`.

This mirrors the Nav2-style server file layout with one `controller_server.hpp`,
one `controller_server.cpp`, and one `main.cpp`. The process hosts both existing lifecycle nodes
so lifecycle management, parameters, and topics remain compatible.

## Launch

`amr_bringup` includes this launch file and passes the consolidated parameter file:

```bash
ros2 launch amr_controller_server controller.launch.py params_file:=/path/to/amr.yaml
```

## Next Direction

Goal checking, progress checking, safety gate, velocity control, and local planning should keep
moving toward package-local classes or plugins under this controller boundary.
