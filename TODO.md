# TODO

Current architecture target: external `slam_toolbox` online-async
SLAM/localization plus `ros-amr-navigation` navigation core.

## Near Term

- Field-test startup order with robot bringup, external `slam_toolbox`
  `online_async_launch.py` using `amr_bringup/params/slam_toolbox.yaml`, then
  `amr_bringup navigation.launch.py`.
- Validate TF lookup behavior with both `base_footprint` and `base_link` robot
  configurations.
- Tune local costmap behavior while `/map` is changing during online mapping.
- Exercise global planning failure responses before `/global_costmap` is ready.
- Verify RViz and optional MQTT bridge defaults against the new topic contract.

## Follow-Up

- Consider a lightweight optional `/pose` publisher for visualization-only
  consumers that need pose as a topic instead of TF.
- Add launch tests or static checks for navigation-only launch composition.
- Add static checks that keep documented topic names aligned with
  `amr_bringup/params/amr.yaml`.
- Revisit multi-robot namespacing once the single-robot standard contract is
  stable.
