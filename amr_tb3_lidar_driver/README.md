# amr_tb3_lidar_driver

AMR-owned TurtleBot3 LDS-class LiDAR driver package.

## Responsibility

- own LiDAR serial transport separately from the base driver
- publish `sensor_msgs/msg/LaserScan` on `/scan`
- keep sensor-model-specific packet parsing isolated from ROS publishing
- support TurtleBot3 LDS variants through explicit parser backends

The first implementation pass focuses on the node shell and parser structure, not full protocol parity.
