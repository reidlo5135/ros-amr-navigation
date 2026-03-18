Changelog
=========

2026-03-18
----------

- Started the ``0.3.1`` responsibility refactor:

  - added ``amr_obstacle_detection`` for scan-based obstacle reporting
  - added ``amr_costmap_server`` as the new single owner of global and local costmaps
  - added ``ObstacleReport.msg`` to ``amr_msgs``
  - updated ``amr_global_planner`` to consume the global costmap instead of inflating the raw map internally
  - updated ``amr_local_planner`` to consume the local costmap and obstacle reports instead of interpreting raw scans directly
  - updated ``amr_bt_navigator`` to subscribe to obstacle reports as the first step toward navigator-owned obstacle-response decisions

- Restored lifecycle-manager-driven bringup after a local reset:

  - ``localization.launch.py`` now delegates lifecycle sequencing to ``amr_lifecycle_manager``
  - ``navigation.launch.py`` now delegates navigation-node activation to ``amr_lifecycle_manager``
  - manager-driven initial pose publication is used again instead of localization-side timing

- Restored the last dynamic-obstacle navigation split after a local reset:

  - ``amr_local_planner`` again subscribes to ``/scan`` and performs dynamic-obstacle-triggered local replanning
  - fixed obstacle stop handling in ``amr_motion_controller`` remains separate from dynamic replanning parameters
  - dynamic obstacle triggers now filter against the static inflated map so known walls are less likely to trigger dynamic replans
- Added a root README responsibility table to clarify package ownership boundaries across mapping, localization, planning, control, lifecycle, and orchestration.

- Added dual-mode operation to ``amr_map_server``:

  - static map publishing from YAML, as before
  - live ``mapping_mode`` based on ``/odom`` and ``/scan`` raytracing

- Added ``mapping_mode:=true`` support to ``amr_bringup/launch/localization.launch.py`` so the stack can start in a lightweight mapping-only flow.
- Updated shared parameters in ``amr_bringup/params/amr.yaml`` for mapping mode configuration, including grid size, origin, sensor topics, and update range.
- Split map publishing roles inside ``amr_map_server``:

  - ``/amr/map/data`` remains the official navigation map
  - ``/amr/map/temporary`` publishes the in-progress mapping result

- Added ``/amr/map_server/freeze_temporary_map`` to promote the current temporary map into the official navigation map topic.
- Improved mapping-mode RViz usability by publishing an identity ``map -> odom`` TF and changing temporary occupancy updates from permanent painting to score-based accumulation with decay.
- Reworked mapping mode toward a SLAM-lite feedback loop:

  - ``amr_localization`` can run against ``/amr/map/temporary``
  - ``amr_map_server`` consumes corrected ``/amr/localization/odometry`` instead of raw ``/odom``
  - mapping mode is now structured around localization-assisted temporary map refinement

- After validating the temporary-map feedback loop, the mapping bootstrap direction was corrected back toward raw ``/odom``-based accumulation for stability.
- Improved bootstrap mapping with IMU-assisted heading estimation:

  - ``/odom`` remains the translation source
  - ``/imu`` is now used to stabilize yaw while drawing the temporary map
- Added lightweight local scan matching to bootstrap mapping:

  - ``amr_map_server`` now searches around the ``odom + imu`` predicted pose
  - the best local candidate is selected against the current temporary map before raytracing the scan
  - scan matching only activates after enough occupied cells exist in the temporary map
- Replaced the mapping-mode identity TF with a corrected ``map -> odom`` transform derived from the latest scan-matched mapping pose, improving RViz alignment between the temporary map, robot model, and laser scan.
- Added temporary-map quality evaluation and save flow to ``amr_map_server``:

  - coverage and obstacle/free ratios are checked before saving
  - an inflation-aware free-space ratio is used as a lightweight planning-readiness metric
  - passing maps can be promoted to ``/amr/map/data`` and saved as ``pgm + yaml``
- Added optional automatic map saving in mapping mode:

  - quality checks now run periodically during mapping
  - the map can be auto-saved after a configurable number of consecutive passes
  - manual save and auto-save now share the same promotion-and-save path
- Added and expanded English README files across the workspace, including Mermaid diagrams for the root stack flow and major runtime packages.

2026-03-17
----------

- Promoted the AMR stack to a working goal-to-motion MVP pipeline:

  - ``NavigateToPose`` action orchestration
  - A* global planning
  - local replanning
  - velocity control to ``/cmd_vel``

- Added AMCL-lite style localization with particle-based pose estimation from ``/odom``, ``/scan``, and the occupancy map.
- Added automatic initial pose publishing from the configured origin station.
- Added RViz integration improvements:

  - stack-focused ``amr.rviz``
  - ``2D Goal Pose`` bridge to the AMR action server

- Added PID-based motion control, LiDAR obstacle stop handling, runtime logs, and topic/parameter cleanup under ``amr.yaml``.

2026-03-16
----------

- Reorganized the AMR package structure to match the AGV architecture style while keeping AMR-specific roles:

  - ``amr_map_server``
  - ``amr_localization``
  - ``amr_global_planner``
  - ``amr_local_planner``
  - ``amr_motion_controller``
  - ``amr_bt_navigator``
  - ``amr_bringup``
  - ``amr_msgs``
  - ``amr_navigation``

- Replaced route-centric AGV concepts with AMR-friendly goal navigation structure.
- Introduced the first AMR planning/control skeleton and shared runtime parameter layout.
