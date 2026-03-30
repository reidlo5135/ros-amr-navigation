# amr_slam_mapper

Pure SLAM mapping runtime for temporary occupancy-grid generation.

## Role

- subscribes to `/odom`, `/imu`, and `/scan`
- runs a local scan-matching front-end
- accumulates keyframes into a pose-graph structure
- searches loop-closure candidates from scan descriptors
- applies lightweight graph optimization when a loop closure is accepted
- publishes the live temporary SLAM map on `/amr/map/temp`
- publishes corrected mapping odometry on `/amr/slam_mapper/odometry`

## Important Topics

- `/amr/map/temp`
- `/amr/slam_mapper/odometry`

## Algorithm Summary

The current `amr_slam_mapper` pipeline is organized as a lightweight pose-graph SLAM flow:

1. Read the latest raw odometry pose for translation.
2. Reconstruct yaw using IMU delta yaw relative to the startup heading, rather than trusting wheel odometry yaw alone.
3. Apply local scan matching around the predicted pose and choose the highest-scoring candidate against the current temporary map.
4. Integrate the corrected scan into the temporary occupancy-grid map as free/occupied evidence.
5. Filter poses into keyframes and append them as pose-graph nodes with odometry relative edges.
6. Search loop-closure candidates using a compact scan descriptor, then locally rescore nearby poses with scan matching.
7. When a loop closure is accepted, run lightweight graph optimization and rebuild the temporary map from optimized node poses.

## Why Rotation Drift Stays Low

- Translation mainly follows raw odometry.
- Heading is stabilized by IMU delta yaw compensation.
- Each incoming scan is locally re-aligned against the current map before integration.
- Accepted loop closures add additional yaw and position constraints into the pose graph.

## Notes

- this package is focused on pure SLAM mapping only
- frontier / explore / completion-to-nav conversion should stay outside this package
