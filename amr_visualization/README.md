# amr_visualization

Qt6 operator visualization app for the AMR navigation stack.

## Safe-mode parameters

The default runtime prioritizes responsiveness over detailed mesh rendering.

Key parameters:

- `enable_robot_model`: default `true`
- `enable_robot_meshes`: default `false`
- `enable_tf_visualization`: default `true`
- `enable_scan_visualization`: default `true`
- `enable_map_visualization`: default `true`
- `enable_costmap_visualization`: default `true`
- `robot_model_emit_period_ms`: default `250`
- `tf_emit_period_ms`: default `100`
- `scan_emit_period_ms`: default `100`
- `mesh_load_async`: default `false`
- `mesh_max_loaded_triangles`: default `5000`
- `mesh_max_rendered_faces`: default `500`
- `mesh_max_file_size_mb`: default `64`

Detailed STL rendering is opt-in with `enable_robot_meshes:=true`. If `mesh_load_async:=true`
is requested, mesh file loading is disabled for safety because no asynchronous loader is currently
implemented.

## Runtime isolation recipes

Use these commands to isolate live-data freezes without changing source code.

UI only, no robot description:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p robot_description_topic:=/disabled_robot_description \
  -p subscribe_scan:=false \
  -p subscribe_global_costmap:=false \
  -p subscribe_local_costmap:=false
```

Robot description but no mesh loading:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_model:=true \
  -p enable_robot_meshes:=false \
  -p subscribe_scan:=false \
  -p subscribe_global_costmap:=false \
  -p subscribe_local_costmap:=false
```

TF and scan only:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_model:=false \
  -p subscribe_scan:=true \
  -p subscribe_global_costmap:=false \
  -p subscribe_local_costmap:=false
```

Costmaps only:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_model:=false \
  -p subscribe_scan:=false \
  -p subscribe_global_costmap:=true \
  -p subscribe_local_costmap:=true
```

Mesh opt-in stress test:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_model:=true \
  -p enable_robot_meshes:=true \
  -p mesh_max_loaded_triangles:=1000 \
  -p mesh_max_rendered_faces:=200
```

## Crash backtrace

If the process crashes or hangs, collect a backtrace:

```bash
gdb --args ros2 run amr_visualization amr_visualization
run
thread apply all bt full
```

If `ros2 run` is awkward under `gdb`, resolve the executable path:

```bash
ros2 pkg prefix amr_visualization
gdb --args <prefix>/lib/amr_visualization/amr_visualization
```

Core dump workflow:

```bash
ulimit -c unlimited
coredumpctl list | grep amr_visualization
coredumpctl gdb <PID-or-exe>
```
