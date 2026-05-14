# amr_tb3_bringup

AMR-owned TurtleBot3 Burger hardware bringup package.

## Responsibility

- compose the AMR-owned TurtleBot3 base driver
- compose the AMR-owned TurtleBot3 LiDAR driver
- start `robot_state_publisher` from the AMR-owned description
- own robot-side TurtleBot3 hardware profiles and launch layouts

This package is the robot-side hardware boundary below `amr_bringup`.

Current default launch entrypoint:

- `ros2 launch amr_tb3_bringup turtlebot3.launch.py`

The top-level recommended robot-side entrypoint remains:

- `ros2 launch amr_bringup turtlebot3.launch.py`
