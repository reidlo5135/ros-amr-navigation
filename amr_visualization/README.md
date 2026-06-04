# amr_visualization

Qt6 operator visualization app for the AMR navigation stack.

## Safe-mode parameters

The packaged launch file uses a low-CPU OpenGL profile by default. It keeps URDF
STL robot meshes enabled while avoiding debug overlays, verbose per-frame logs,
aggressive geometry truncation, and excessive repaint rates. The profile lives in
`config/low_cpu_opengl.yaml` and can be overridden from the CLI.

Key parameters:

- `enable_robot_model`: default `true`
- `enable_robot_meshes`: launch default `true`
- `enable_tf_visualization`: default `true`
- `enable_scan_visualization`: default `true`
- `enable_map_visualization`: default `true`
- `enable_costmap_visualization`: default `true`
- `robot_model_emit_period_ms`: launch default `500`
- `tf_emit_period_ms`: launch default `200`
- `scan_emit_period_ms`: default `100`
- `mesh_load_async`: default `false`
- `robot_model_renderer_backend`: launch default `opengl`, allowed `proxy`, `qpainter_wireframe`, `opengl`
- `robot_mesh_render_mode`: internal render mode derived from the backend
- `mesh_max_loaded_triangles`: launch default `200000`
- `mesh_max_rendered_faces`: default `500`
- `mesh_max_file_size_mb`: launch default `32`
- `mesh_max_extent_m`: launch default `100.0`
- `mesh_max_abs_coordinate_m`: launch default `100.0`
- `mesh_max_projected_extent_px`: default `3000`
- `robot_mesh_auto_unit_scale`: default `false`
- `robot_mesh_unit_scale`: default `1.0`
- `robot_opengl_debug_camera`: default `false`, centers the OpenGL robot view on the robot mesh
  or pose instead of the map camera for visibility diagnosis.
- `robot_opengl_debug_axes`: default `false`, draws small RGB axes at each OpenGL robot visual
  pose to prove that the overlay, camera, and shader path are visible.
- `robot_opengl_debug_cube`: default `false`, draws a solid debug cube at each OpenGL robot visual
  pose even when STL loading fails.
- `robot_opengl_force_visible`: default `false`, keeps the OpenGL overlay visible for widget/camera
  diagnosis even when there are no renderable STL meshes.
- `robot_opengl_stl_only_debug`: default `false`, suppresses proxy/debug clutter and renders only
  STL meshes, with a red OpenGL fallback cube if no STL mesh reaches draw.
- `robot_opengl_debug_mesh_bbox`: default `false`, draws each accepted STL mesh bounding box with
  the same pose path as the mesh to separate transform/camera problems from mesh draw problems.
- `robot_opengl_verbose_diagnostics`: launch default `false`, enables detailed OpenGL/STL status
  tables, file probes, and paint diagnostics when set to `true`.
- `robot_opengl_auto_software_profile`: launch default `true`, selects the software FPS profile
  when the OpenGL renderer string reports llvmpipe, softpipe, or a software rasterizer.
- `robot_opengl_target_fps`: launch default `0`, where `0` means auto-select software/hardware
  FPS. Set a positive value to force an explicit FPS cap.
- `robot_opengl_software_target_fps`: launch default `8`, used for software OpenGL when no
  explicit `robot_opengl_target_fps` override is set.
- `robot_opengl_hardware_target_fps`: launch default `30`, used for hardware OpenGL when no
  explicit `robot_opengl_target_fps` override is set.
- `robot_model_pose_epsilon_m`: launch default `0.003`, suppresses tiny robot model pose updates.
- `robot_model_yaw_epsilon_rad`: launch default `0.003`, suppresses tiny robot model yaw updates.
- `robot_opengl_debug_size_m`: default `0.20`, controls the size of the OpenGL debug axis/cube.

On llvmpipe, reducing `robot_opengl_software_target_fps` is the safer first CPU lever. Lowering
`mesh_max_loaded_triangles` too far can truncate STL files and remove visible body plates,
support/frame geometry, wheel detail, or LDS geometry. For best performance, use hardware GPU
OpenGL instead of llvmpipe when the Box PC platform allows it.

Detailed STL rendering is enabled by the packaged low-CPU launch profile. For direct `ros2 run`
invocations without a parameter file, enable it with `enable_robot_meshes:=true` and either
`robot_model_renderer_backend:=opengl` or `robot_model_renderer_backend:=qpainter_wireframe`.
The `proxy` backend does not load or draw STL triangles. If `mesh_load_async:=true` is requested,
mesh file loading is disabled for safety because no asynchronous loader is currently implemented.

## Renderer backend notes

Current backends:

- `proxy`: stable fallback primitives/proxies; this is not RViz-equivalent mesh rendering.
- `qpainter_wireframe`: bounded STL loading plus guarded wireframe drawing for diagnostics.
- `opengl`: `QOpenGLWidget` overlay that renders URDF STL visuals as shaded solid geometry.

OpenGL backend notes:

- Uses a `QOpenGLWidget`-based renderer overlaid with the existing Qt Widgets scene.
- Loads mesh vertices once, uploads vertex/index buffers once, and updates per-link transforms.
- Uses OpenGL depth testing, view/projection matrices, and GPU clipping instead of raw `QPainter`
  triangle fills.
- Supports STL visual meshes first. DAE and OBJ URDF meshes are reported as unsupported and fall
  back to proxy rendering until a robust loader is added.
- Keeps map/costmap/scan/path/waypoint rendering in `SceneWidget`.
- Falls back to proxy visuals when a mesh cannot be loaded or validated.

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
  -p robot_model_renderer_backend:=qpainter_wireframe \
  -p mesh_max_loaded_triangles:=1000 \
  -p mesh_max_rendered_faces:=200
```

Launch default low-CPU OpenGL profile:

```bash
ros2 launch amr_visualization amr_visualization.launch.py
```

Runtime comparison recipes:

```bash
# Proxy only
ros2 launch amr_visualization amr_visualization.launch.py \
  robot_model_renderer_backend:=proxy \
  enable_robot_meshes:=false

# OpenGL robot only, with map/costmap/scan/TF frame drawing disabled
ros2 launch amr_visualization amr_visualization.launch.py \
  enable_map_visualization:=false \
  enable_costmap_visualization:=false \
  enable_scan_visualization:=false \
  enable_tf_visualization:=false \
  robot_opengl_software_target_fps:=3

# Robot overlay off
ros2 launch amr_visualization amr_visualization.launch.py \
  enable_robot_model:=false \
  enable_robot_meshes:=false
```

Use a custom parameter file:

```bash
ros2 launch amr_visualization amr_visualization.launch.py \
  params_file:=/path/to/custom.yaml
```

Direct executable without launch defaults:

```bash
ros2 run amr_visualization amr_visualization
```

Low-CPU OpenGL equivalent with `ros2 run`:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl \
  -p robot_opengl_force_visible:=false \
  -p robot_opengl_debug_camera:=false \
  -p robot_opengl_stl_only_debug:=false \
  -p robot_opengl_debug_mesh_bbox:=false \
  -p robot_opengl_debug_axes:=false \
  -p robot_opengl_debug_cube:=false \
  -p robot_opengl_verbose_diagnostics:=false \
  -p robot_opengl_auto_software_profile:=true \
  -p robot_opengl_target_fps:=0 \
  -p robot_opengl_software_target_fps:=8 \
  -p robot_opengl_hardware_target_fps:=30 \
  -p robot_model_emit_period_ms:=500 \
  -p tf_emit_period_ms:=200 \
  -p robot_model_pose_epsilon_m:=0.003 \
  -p robot_model_yaw_epsilon_rad:=0.003 \
  -p mesh_max_loaded_triangles:=200000 \
  -p mesh_max_file_size_mb:=32 \
  -p mesh_max_extent_m:=100.0 \
  -p mesh_max_abs_coordinate_m:=100.0
```

Low CPU software OpenGL:

```bash
ros2 launch amr_visualization amr_visualization.launch.py \
  robot_opengl_software_target_fps:=8 \
  mesh_max_loaded_triangles:=200000
```

Higher quality / hardware OpenGL:

```bash
ros2 launch amr_visualization amr_visualization.launch.py \
  robot_opengl_target_fps:=30 \
  mesh_max_loaded_triangles:=200000
```

Emergency lowest CPU:

```bash
ros2 launch amr_visualization amr_visualization.launch.py \
  robot_opengl_software_target_fps:=3 \
  mesh_max_loaded_triangles:=200000
```

Mesh proxy mode:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=proxy
```

Mesh wireframe debug:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=qpainter_wireframe \
  -p mesh_max_loaded_triangles:=1000 \
  -p mesh_max_rendered_faces:=200
```

OpenGL target:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl
```

OpenGL overlay visibility probe:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl \
  -p robot_opengl_force_visible:=true \
  -p robot_opengl_debug_axes:=true \
  -p robot_opengl_debug_cube:=true \
  -p robot_opengl_debug_camera:=true
```

OpenGL STL-only visibility probe:

```bash
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl \
  -p robot_opengl_force_visible:=true \
  -p robot_opengl_debug_camera:=true \
  -p robot_opengl_stl_only_debug:=true \
  -p robot_opengl_debug_mesh_bbox:=true \
  -p robot_opengl_debug_axes:=false \
  -p robot_opengl_debug_cube:=false \
  -p mesh_max_loaded_triangles:=50000 \
  -p mesh_max_extent_m:=10.0 \
  -p mesh_max_abs_coordinate_m:=10.0
```

Verbose mesh and OpenGL diagnostics are printed to terminal logs instead of the
Events/Feedback panel. Enable Qt logging categories when collecting renderer details:

```bash
QT_LOGGING_RULES="amr_visualization.opengl.debug=true;amr_visualization.mesh.debug=true" \
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl
```

To show all visualization Qt categories:

```bash
QT_LOGGING_RULES="amr_visualization.*=true" \
ros2 run amr_visualization amr_visualization --ros-args \
  -p enable_robot_meshes:=true \
  -p robot_model_renderer_backend:=opengl
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
