Changelog
=========

2026-06-10
----------

- Started the ``0.18.1`` localization slip-guard follow-up:

  - added conservative wheel-slip and physical-stall detection in ``amr_localization`` by comparing odometry deltas with localized pose progress, scan likelihood health, ``/cmd_vel``, and ``/amr/motion/status``
  - damped particle motion updates during suspected slip/stall so a spinning or stalled wheel does not immediately drag localization away from scan/map evidence
  - bounded per-update ``map -> odom`` correction to reduce sudden TF jumps after slip, while bypassing the limiter for initial-pose reset
  - kept ``AMR_LOG schema=v1`` and existing topic, service, and action names stable while adding localization guard diagnostics

- Started the ``0.18.0`` navigation quality stabilization line:

  - made motion-controller tracking target selection more forward-progressive by preserving target context across refreshed local plans and applying ``tracking_progress_rollback_window`` to candidate rollback limiting
  - reduced straight-segment and goal-approach oscillation with more conservative TurtleBot3 Burger-class default gains, final-align speed limits, target hysteresis, and near-goal linear-speed rampdown
  - tightened post-recovery rejoin semantics so reacquire success requires matching motion status and local plan status, with explicit timeout diagnostics for failed navigation reacquisition
  - kept ``AMR_LOG schema=v1`` and existing topic, service, and action names stable while adding diagnostic fields for target jumps and recovery reacquire outcomes

2026-05-11
----------

- Completed the remaining ``0.16.x`` planner-local escape wiring cleanup:

  - restricted local escape re-dispatch to planner-owned recovery for the currently active motion command so stale planner state does not hijack controller-only blocked or stalled cases
  - normalized ``/amr/local_planner/plan_local_escape`` responses onto stable reason labels for map-unavailable, empty-source-plan, and no-valid-path failures
  - surfaced local escape rejection context into the BT status message before fallback recovery behaviors continue

- Fixed the ``0.16.x`` local-escape-first recovery policy:

  - hard-blocked planner recovery now tries one local escape, then `backup`, then `spin`, before escalating to a fresh global replan
  - near-goal blocked planner recovery now skips local escape, uses `wait`, and re-dispatches the current plan before stronger escalation
  - planner-owned `global_replan_required` now bypasses intermediate recovery commands and consumes a bounded recovery attempt
  - controller-owned blocked/stalled recovery keeps a deterministic `wait -> backup -> spin -> fresh global replan` sequence

- Started the ``0.16.x`` corridor / doorway blocked-state tuning pass:

  - made dynamic-obstacle recovery confirmation depend on blocked distance so farther-ahead corridor and doorway collisions need longer persistence before escalating
  - added separate `dynamic_obstacle.corridor_relax_distance` and `dynamic_obstacle.corridor_confirm_cycles` parameters to preserve quicker near-field reactions while reducing false blocked decisions deeper in narrow lanes

2026-05-08
----------

- Started the ``0.16.3`` patch branch:

  - reduced false blocked-state escalation in corridor and doorway cases by requiring short persistence before dynamic obstacle recovery decisions trigger
  - kept ``NavigateToPoses`` intermediate waypoints heading-free while reserving explicit final yaw alignment for the last goal
  - tightened final yaw settle behavior and suppressed tiny straight-line heading corrections that made nominal path tracking visibly wag

2026-05-08
----------

- Started the ``0.16.2`` patch branch:

  - wired ``amr_bt_navigator`` to try planner-local escape before heavier recovery behaviors when the local planner reports a blocked path
  - added escalation guards so one local-escape re-dispatch does not loop indefinitely before fallback recovery behaviors or replanning
  - extended ``amr_runtime_observation`` and ``amr_visualization`` with ``local_escape_active`` and ``local_escape_executing`` visibility for operator-side diagnosis

2026-04-30
----------

- Froze the ``0.15.x`` runtime-hardening line at ``0.15.10``:

  - kept the stack on the measured runtime-hardening baseline after confirming recovery observation and dynamic interrupt behavior looked stable enough for the freeze point
  - closed ``0.15.x`` as the handoff line for documentation, diagnosis fields, and current operator/runtime semantics
  - moved the next active implementation direction to ``0.16.x`` recovery-policy refinement

2026-04-30
----------

- Started the ``0.15.10`` patch branch:

  - kept ``amr_controller_server`` focused on executing BT-dispatched motion commands instead of mirroring planner-side blocked semantics into motion status
  - expanded ``amr_runtime_observation`` with stable ``blocked_context`` and ``recovery_phase`` fields that synthesize planner, controller, and route-layer signals without pushing planner ownership into motion control
  - kept ``0.15.10`` focused on recovery-phase semantics and blocked-state interpretation ahead of future recovery-policy changes

2026-04-30
----------

- Started the ``0.15.9`` patch branch:

  - aligned the workspace docs and bringup docs onto the active ``0.15.9`` runtime baseline
  - expanded ``amr_runtime_observation`` summary/event payloads with structured recovery trigger and reason labels for easier operator diagnosis
  - kept ``0.15.9`` scoped to recovery trigger visibility and structured diagnosis while deferring local-escape-first recovery flow changes to the next patch line

2026-04-20
----------

- Started the ``0.15.8`` patch branch:

  - changed goal checking to respect incoming goal yaw by default with a loose Nav2-style yaw tolerance

- Started the ``0.15.7`` patch branch:

  - extended the local path refiner with conservative corner smoothing
  - added path-refiner collision validation so smoothed paths fall back to the unsmoothed plan when unsafe

- Started the ``0.15.6`` patch branch:

  - added controller-local ``goal_checker`` parameters for XY / yaw / hold-time arrival policy
  - added a conservative local path refiner for duplicate pruning, segment interpolation, and heading assignment

- Started the ``0.15.5`` patch branch:

  - introduced ``amr_controller_server`` as the controller-layer aggregation boundary
  - moved the local planner and motion controller runtime source into a Nav2-style ``controller_server.hpp`` / ``controller_server.cpp`` / ``main.cpp`` layout while preserving the existing ``/amr/local_planner`` and ``/amr/motion_controller`` lifecycle nodes
  - aligned the package dependency surface toward a Nav2-style controller boundary without changing motion-control behavior

2026-04-13
----------

- Started the ``0.15.4`` patch branch:

  - preparing the next robot-side MQTT direction beyond a thin bridge-only role
  - evaluating performance-first restructuring around a stronger robot-side API/runtime boundary
  - renamed the robot-side package direction to ``amr_mqtt_server`` and began moving the control API toward non-ROS-facing topic paths such as ``navigation/command`` and ``pose/set``

2026-04-13
----------

- Started the ``0.15.3`` patch branch:

  - added the initial ``amr_runtime_observation`` package for lightweight runtime summaries and event emission
  - wired the observation node into navigation bringup without changing the core planner/controller contract
  - kept the first scope focused on route progress, motion blocking, planner decision, and recovery-count observation

2026-04-10
----------

- Started the ``0.15.2`` patch branch:

  - removed the in-repo ``amr_viz`` app from this workspace
  - moved the operator UI direction to the external ``ros-rcs`` desktop app repository
  - kept the robot-side MQTT contract in ``amr_mqtt_bridge`` as the integration boundary

2026-04-02
----------

- Started the ``0.15.0`` development branch:

  - removed the in-repo ``amr_slam_mapper`` runtime in favor of consuming external ``ros-slam-mapper`` topics
  - switched AMR-side mapping consumers from ``/amr/*`` SLAM sources to external ``/slam/*`` sources
  - kept MQTT and ``amr_viz`` contracts stable by retargeting the bridge's ROS source topics only

- Started the ``0.14.4`` patch branch:

  - continued ``amr_slam_mapper`` refinement after stabilizing straight-line motion priors
  - began separating rotation-only keyframe behavior from straight/arc motion behavior
  - limited IMU heading usage to rotation-biased prior assistance instead of full yaw override

2026-03-30
----------

- Started the ``0.14.2`` patch branch:

  - split the temporary SLAM map into raw and refined publish layers
  - kept the legacy ``/amr/map/temp`` topic as a refined compatibility alias
  - exposed raw and refined SLAM map layers through MQTT and ``amr_viz`` mapping mode

- Started the ``0.14.1`` patch branch:

  - documented the current ``amr_slam_mapper`` pose-graph SLAM pipeline in its README
  - continued the mapping-focused ``0.14.x`` line on top of the stable navigation baseline

- Started the ``0.14.0`` development branch:

  - reset the main AMR line back onto the stable ``0.12.4`` navigation baseline
  - began separating future SLAM mapping work into ``amr_slam_mapper``
  - kept ``amr_map_server`` focused on map lifecycle, publishing, evaluation, and save flows
  - prepared the next mapping iteration around pure SLAM mapping and temporary-map publishing
2026-03-25
----------

- Started the ``0.12.4`` patch branch:
  - reorganized ``amr_viz`` into an app/features-based React structure
  - split the operator UI into page, hook, layout, and panel components
  - moved shared constants and dashboard state handling out of the root ``App.tsx``
  - split global and feature CSS so the web client matches a more standard frontend layout

2026-03-25
----------

- Started the ``0.12.3`` patch branch:
  - Next step is to move recovery branching from BT heuristics toward local-planner decision semantics.
  - Aim to reduce hard-coded near-goal handling and make planner output the recovery rationale.

2026-03-25
----------

- Started the ``0.12.2`` patch branch:
  - Routed local-costmap authority through ``amr_local_planner`` status reporting.
  - Kept controller-side blocking conservative to avoid breaking baseline straight driving.

2026-03-25
----------

- Started the ``0.12.1`` patch branch:
  - guarded ``amr_bt_navigator`` against overlapping active goals so stale recovery state does not leak into a new navigate request

2026-03-25
----------

- Started the ``0.12.0`` exact-footprint branch:

  - added exact footprint collision helpers in ``amr_geometry``
  - applied polygon-based collision checks in ``amr_global_planner`` and ``amr_local_planner``
  - published footprint metadata through ``amr_mqtt_bridge`` for operator visualization
  - added an ``Exact Footprint`` overlay layer to ``amr_viz``

2026-03-24
----------

- Started the ``0.11.0`` cleanup-and-consolidation branch:

  - removed unused packages such as ``amr_rviz_plugins`` and ``amr_obstacle_detection``
  - removed the legacy server-side MQTT bridge package and consolidated the robot-side bridge into ``amr_mqtt_bridge``
  - normalized include hygiene so implementation files now include only their paired headers
  - refreshed all package ``README.md`` files to match the active MQTT-first ``0.11.0`` stack

- Started the ``0.8.0`` planning-safety refinement branch:

  - introduced ``footprint.polygon`` and ``footprint.padding`` parameters in ``amr_costmap_server``
  - compute a circumscribed footprint radius from the configured polygon
  - apply the effective robot size to global and local obstacle inflation as a first step toward explicit footprint-aware planning

2026-03-23
----------

- Reworked ``amr_viz`` into a direct robot-topic MQTT web client:

  - converted ``amr_viz`` into a standalone React/Vite app with versioned web packaging
  - switched the web client to subscribe directly to ``amr/robot/turtlebot3/viz/*``
  - updated ``amr_mqtt_robot_plugin`` to emit web-friendly JSON visualization topics alongside raw ROS telemetry

- Expanded AMR web visualization and interaction:

  - added direct goal / initial-pose controls with map interaction and goal markers
  - added layer toggles, denser path rendering, stronger scan rendering, and richer TF visualization
  - added URDF-driven robot-model parsing with material-color application and proxy geometry rendering
  - improved sidebar layout and responsive control grouping for the operator panel

- Improved MQTT reliability and browser-side connection behavior:

  - lengthened keepalive and improved reconnect handling for both the robot plugin and the web client
  - added WebSocket endpoint fallback handling in ``amr_viz`` MQTT transport
  - reduced browser-side visualization lag by batching latest telemetry per animation frame
  - throttled high-rate robot visualization topics so the Raspberry Pi no longer over-publishes web telemetry

- Updated project planning docs:

  - refreshed the roadmap/TODO after the MQTT-first visualization transition
  - moved the BT navigator and local escaping work items into the next-day queue

2026-03-20
----------

- Reworked the AMR remote-operations transport around MQTT:

  - added ``amr_mqtt_bridge`` as the VBox-side MQTT <-> ROS mirror and command bridge
  - added ``amr_mqtt_robot_plugin`` as the TurtleBot3-side ROS <-> MQTT transport/plugin
  - moved away from the earlier Python WebSocket bridge approach after observing high CPU usage
  - added raw ROS message mirroring for high-value robot telemetry instead of JSON remodeling

- Expanded MQTT mirroring across the AMR stack:

  - mirrored robot-side ``scan``, ``odom``, ``imu``, ``tf``, ``tf_static``, ``joint_states``, and ``robot_description``
  - mirrored navigation-side ``map``, ``robot_pose``, ``global_path``, ``local_path``, ``global_costmap``, ``local_costmap``, ``motion_status``, and ``obstacle_report``
  - restored VBox-side republishing so RViz can consume mirrored robot and navigation topics
  - fixed QoS durability for mirrored ``map``, ``costmap``, and ``robot_description`` topics to match late-subscriber RViz behavior

- Added MQTT command and request handling across the split TB3/VBox deployment:

  - VBox ``amr_mqtt_bridge`` now emits MQTT commands from local ROS inputs such as RViz goal and initial pose
  - TurtleBot3 ``amr_mqtt_robot_plugin`` now consumes MQTT commands and dispatches them into local ROS topics, services, and actions
  - ``/cmd_vel`` forwarding was restored over MQTT for robot actuation
  - service/action request handling now returns MQTT ACK responses while feedback/status are mirrored in ROS-compatible form

- Reorganized launch and deployment roles for the split runtime:

  - converted ``turtlebot3.launch.py`` into the TurtleBot3 bringup entry point sequencing ``robot.launch.py``, localization, navigation, and the robot MQTT plugin
  - removed the earlier ``total.launch.py`` path in favor of the TurtleBot3-specific bringup flow
  - separated TurtleBot3 transport and VBox bridge responsibilities after clarifying that TB3 publishes to the broker and VBox reconstructs the mirror
  - removed the ``amr_bringup`` dependency cycle by separating MQTT package defaults and launch parameter ownership

- Updated workspace documentation for the MQTT-first architecture:

  - refreshed the root README and package READMEs to reflect the new TB3/VBox/MQTT deployment model
  - documented runtime requirements and package roles for ``amr_mqtt_bridge``, ``amr_mqtt_robot_plugin``, and ``amr_viz``
  - recorded the next-step refactor and viz migration direction in ``TODO.md``

2026-03-19
----------

- Added combined bringup support in ``amr_bringup``:

  - added ``total.launch.py`` to sequence localization before navigation
  - delayed navigation startup so lifecycle-driven initial pose publication can complete first

- Improved dynamic-obstacle handling in the navigation stack:

  - increased obstacle detection distance and blocking thresholds
  - updated ``amr_local_planner`` to attempt a more explicit lateral escape and rejoin flow
  - reduced local-planner costmap log spam after observing launch-side Python CPU spikes

- Started the first custom visualization package scaffold:

  - added ``amr_viz`` with a ROS-facing bridge prototype and React/Three.js renderer
  - added development scripts, bridge configuration, and initial runtime documentation
  - switched the visualization client toward a host-accessible pure web flow served on ``0.0.0.0``

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
