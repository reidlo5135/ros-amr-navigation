# Rosbag2 Recording Profiles

The v0.17.5 field workflow uses three rosbag2 profiles. The default profile should be
`light` for repeated start-to-goal quality tests. Escalate to `debug` or `full` only when
the light profile and structured logs do not explain the behavior.

## Light

Script:

```bash
scripts/record_nav_bag_light.sh
```

Purpose:

- Compare repeated navigation runs without recording heavy sensor or grid streams.
- Preserve runtime observation, planner status, motion status, command, pose, and `/cmd_vel`.

Topics:

- `/amr/observation/runtime/summary`
- `/amr/observation/runtime/events`
- `/amr/motion/command`
- `/amr/motion/status`
- `/amr/planner/local_status`
- `/amr/localization/pose`
- `/cmd_vel`

## Debug

Script:

```bash
scripts/record_nav_bag_debug.sh
```

Purpose:

- Diagnose planner/controller/costmap interactions with enough context for replay analysis.
- Add paths, local costmap, scan, odometry, and TF while avoiding every-topic capture.

Topics:

- all `light` topics
- `/amr/planner/global`
- `/amr/planner/local`
- `/amr/localization/odometry`
- `/amr/costmap/local`
- `/scan`
- `/tf`
- `/tf_static`

## Full

Script:

```bash
scripts/record_nav_bag_full.sh
```

Purpose:

- Capture short, targeted windows when topic-level omissions are suspected.
- This may become large quickly on TurtleBot3-class hardware.

Command behavior:

```bash
ros2 bag record --all
```

## Replay

Script:

```bash
scripts/replay_nav_bag.sh /path/to/bag
```

The replay script passes `--clock` to `ros2 bag play` so analysis nodes can use simulated
time when they opt in to `use_sim_time`.

## Escalation Rule

Start with `light` for repeatability. Use `debug` when structured logs indicate planner,
costmap, scan, or TF involvement. Use `full` only for short windows where the missing topic is
not yet known.
