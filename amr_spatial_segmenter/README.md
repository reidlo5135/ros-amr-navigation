# amr_spatial_segmenter

`amr_spatial_segmenter` is a ROS 2 Humble lifecycle node that consumes the live
SLAM `/map` (`nav_msgs/msg/OccupancyGrid`) and publishes a deterministic,
rule-based spatial overlay.

It is not semantic AI recognition. The MVP keeps raw SLAM map geometry separate
from traversability validation: raw `/map` free and occupied cells drive area
boundaries, axis-aligned corridor centerlines, and door bottleneck candidates,
while an eroded free-space mask is used only to reject corridor candidates that
the robot cannot actually traverse.

## Node

- package: `amr_spatial_segmenter`
- executable: `amr_spatial_segmenter`
- lifecycle node name: `spatial_segmenter`

## Inputs

| Topic | Type | Notes |
| --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Subscribed with `KeepLast(1).transient_local().reliable()` QoS. |

## Outputs

| Topic | Type | Notes |
| --- | --- | --- |
| `/spatial_segments` | `amr_msgs/msg/SpatialSegmentArray` | Structured segment metadata. |
| `/spatial_segment_map` | `nav_msgs/msg/OccupancyGrid` | Labeled segment map. |
| `/spatial_segment_markers` | `visualization_msgs/msg/MarkerArray` | Combined black overlay markers. |
| `/corridor_centerlines` | `visualization_msgs/msg/MarkerArray` | Corridor centerline markers only. |
| `/area_boundaries` | `visualization_msgs/msg/MarkerArray` | Area boundary markers only. |
| `/door_candidates` | `visualization_msgs/msg/MarkerArray` | Reserved for future door candidates. |

`/spatial_segment_map` labels:

- `-1`: unknown
- `0`: unclassified free
- `10`: open area
- `20`: corridor
- `30`: junction, reserved
- `40`: door candidate, reserved
- `50`: narrow passage, reserved
- `100`: occupied or ambiguous occupied-for-segmentation cell

## Current Limitations

- Segmentation runs at a low default rate (`update.period_sec: 5.0`) to avoid
  repainting heavy map overlays every SLAM update.
- Corridor detection is an axis-aligned scanline pass over raw map pixels. It
  publishes only horizontal or vertical centerlines by default and does not draw
  a single PCA line through a large irregular free-space component.
- Open-area markers draw exposed cell-boundary outlines. The structured
  message keeps an axis-aligned bbox polygon as a coarse fallback. Boundary
  edges are merged into longer line segments before publishing.
- Door/portal markers are lightweight candidates from sharp width bottlenecks
  on accepted axis-aligned corridor scanlines.
- Unknown and ambiguous cells are treated as blocked for this MVP.

## Bringup

The segmenter is enabled by default in `amr_bringup` navigation launch. Disable
it explicitly when the overlay is not needed:

```bash
ros2 launch amr_bringup navigation.launch.py use_spatial_segmenter:=false
```

The navigation launch loads `amr_bringup/params/spatial_segmenter.yaml` for this node.

## Quick Overlay Isolation

Turn off `Spatial Overlay` in AMR Visualization. The map and costmaps should
remain, while the black semantic overlay disappears.

Check the marker stream:

```bash
ros2 topic echo /spatial_segment_markers --once
```

Default `semantic_summary` output should contain only these namespaces:

- `spatial_corridors`
- `spatial_area_boxes`
- `spatial_doors`
- optional `spatial_labels` when labels are enabled

The default total point count is capped at `visualization.max_total_marker_points`
and should be `<= 300`. Useful parameter checks:

```bash
ros2 param get /spatial_segmenter overlay.mode
ros2 param get /spatial_segmenter corridor.axis_aligned_only
ros2 param get /spatial_segmenter corridor.allow_diagonal
ros2 param get /spatial_segmenter area.draw_contour
ros2 param get /spatial_segmenter visualization.max_total_marker_points
```
