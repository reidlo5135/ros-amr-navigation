# amr_costmap_server

Lifecycle costmap server for the navigation-only AMR stack.

Inputs:

- `/map` from external `slam_toolbox`
- `/scan`
- `/tf`, `/tf_static`
- TF lookup from `map` to `base_footprint` by default

Outputs:

- `/global_costmap`
- `/local_costmap`

Service:

- `/clear_costmap`

The server no longer depends on a pose topic or the legacy map server. If `/map`
has not arrived yet, it waits without publishing costmaps. If TF lookup fails
temporarily, it keeps the node alive and skips robot-centered local updates until
the `map -> odom -> base_*` chain is available.

Relevant parameters:

- `topics.map`
- `topics.scan`
- `topics.global`
- `topics.local`
- `services.clear_costmap`
- `frames.map`
- `frames.odom`
- `frames.base`
- `tf.lookup_timeout_sec`
- `local_window.*`
- `inflation.*`
- `dynamic.*`
