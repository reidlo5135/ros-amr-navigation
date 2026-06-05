# Field Debug Scripts

These scripts provide a repeatable field workflow for running the AMR navigation stack,
capturing logs, recording rosbag2 data, and extracting navigation quality signals.

All scripts source `scripts/amr_logging_env.sh`. The shared environment script sources
`/opt/ros/humble/setup.bash` and `${AMR_WS_DIR}/install/setup.bash` when available.

## Common Environment

```bash
source scripts/amr_logging_env.sh
```

Useful overrides:

| Variable | Default | Purpose |
| --- | --- | --- |
| `AMR_WS_DIR` | parent ROS workspace when detected | Workspace containing `install/setup.bash` |
| `AMR_LOG_ROOT` | `${AMR_WS_DIR}/log/field` | Root for text logs |
| `AMR_BAG_ROOT` | `${AMR_WS_DIR}/bags` | Root for rosbag2 output |
| `AMR_NOHUP_ROOT` | `${AMR_LOG_ROOT}/nohup` | `nohup` logs and pid files |
| `AMR_RUN_ID` | timestamp | Shared run id for logs and bags |
| `ROS_DOMAIN_ID` | `0` | ROS domain id |
| `TURTLEBOT3_MODEL` | `burger` | TurtleBot3 model for TB3 bringup |

## Running Processes

Start navigation under `nohup`:

```bash
scripts/run_navigation_nohup.sh
```

Start TurtleBot3 bringup under `nohup`:

```bash
scripts/run_turtlebot3_nohup.sh
```

Stop started processes:

```bash
scripts/stop_nohup_process.sh --label all
```

## Recording Bags

Light profile for repeated quality checks:

```bash
scripts/record_nav_bag_light.sh
```

Debug profile with paths, local costmap, scan, odometry, and TF:

```bash
scripts/record_nav_bag_debug.sh
```

Full profile for short targeted captures:

```bash
scripts/record_nav_bag_full.sh
```

Replay:

```bash
scripts/replay_nav_bag.sh /path/to/bag
```

## Watching Logs

Follow all structured AMR logs:

```bash
scripts/watch_amr_logs.sh
```

Follow recovery decisions only:

```bash
scripts/watch_amr_logs.sh --event recovery_decision
```

Filter controller command quality without following:

```bash
scripts/watch_amr_logs.sh --component controller --event cmd_quality --no-follow
```

## Extracting Navigation Quality

Extract the core quality events from `nohup` logs:

```bash
scripts/extract_nav_quality.sh --output /tmp/nav_quality.tsv
```

CSV output is also available:

```bash
scripts/extract_nav_quality.sh --format csv --output /tmp/nav_quality.csv
```

The extractor includes these event families by default:

- `goal_state`
- `cmd_quality`
- `tracking_state`
- `target_jump_detected`
- `recovery_decision`
