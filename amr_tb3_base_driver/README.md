# amr_tb3_base_driver

AMR-owned TurtleBot3 Burger base driver package.

## Responsibility

- subscribe to `/cmd_vel`
- own OpenCR serial transport and protocol boundary
- publish `/odom`
- publish `/imu`
- publish `/joint_states`
- optionally publish `odom -> base_footprint`
- apply stale command watchdog and conservative safety behavior

The first implementation pass focuses on transport, protocol structure, and the ROS-facing node shell.
