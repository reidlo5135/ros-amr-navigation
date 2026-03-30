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

## Notes

- this package is focused on pure SLAM mapping only
- frontier / explore / completion-to-nav conversion should stay outside this package
