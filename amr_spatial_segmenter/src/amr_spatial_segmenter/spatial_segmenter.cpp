/**
 * @file spatial_segmenter.cpp
 * @brief Implementation of the OccupancyGrid spatial segmentation lifecycle node.
 */

#include "amr_spatial_segmenter/spatial_segmenter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

#include "amr_msgs/msg/spatial_segment.hpp"
#include "amr_spatial_segmenter/distance_map.hpp"
#include "amr_spatial_segmenter/occupancy_grid_view.hpp"
#include "amr_spatial_segmenter/segment_classifier.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace amr::spatial_segmenter
{

namespace
{

/// @brief Marker point counters grouped by semantic namespace.
struct MarkerPointCounts
{
  int corridor_points{0};
  int area_points{0};
  int door_points{0};
  int total_points{0};
};

/// @brief Return a true/false label for structured logs.
const char * bool_label(const bool value)
{
  return value ? "true" : "false";
}

/// @brief Count marker points in the fixed semantic overlay namespaces.
MarkerPointCounts count_marker_points(const visualization_msgs::msg::MarkerArray & markers)
{
  MarkerPointCounts counts;
  for (const auto & marker : markers.markers) {
    const int points = static_cast<int>(marker.points.size());
    counts.total_points += points;
    if (marker.ns == "spatial_corridors") {
      counts.corridor_points += points;
    } else if (marker.ns == "spatial_area_boxes") {
      counts.area_points += points;
    } else if (marker.ns == "spatial_doors") {
      counts.door_points += points;
    }
  }
  return counts;
}

/// @brief Convert an internal segment type to the custom message constant value.
std::uint8_t to_message_type(const SegmentType type)
{
  switch (type) {
    case SegmentType::OpenArea:
      return amr_msgs::msg::SpatialSegment::OPEN_AREA;
    case SegmentType::Corridor:
      return amr_msgs::msg::SpatialSegment::CORRIDOR;
    case SegmentType::Junction:
      return amr_msgs::msg::SpatialSegment::JUNCTION;
    case SegmentType::DoorCandidate:
      return amr_msgs::msg::SpatialSegment::DOOR_CANDIDATE;
    case SegmentType::NarrowPassage:
      return amr_msgs::msg::SpatialSegment::NARROW_PASSAGE;
    case SegmentType::Unknown:
    default:
      return amr_msgs::msg::SpatialSegment::UNKNOWN;
  }
}

}  // namespace

/// @copydoc SpatialSegmenter::SpatialSegmenter
SpatialSegmenter::SpatialSegmenter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("spatial_segmenter", options)
{
  this->declare_parameter("topics.map", params_.map_topic);
  this->declare_parameter("topics.segments", params_.segments_topic);
  this->declare_parameter("topics.segment_map", params_.segment_map_topic);
  this->declare_parameter("topics.markers", params_.markers_topic);
  this->declare_parameter("topics.corridor_centerlines", params_.corridor_centerlines_topic);
  this->declare_parameter("topics.area_boundaries", params_.area_boundaries_topic);
  this->declare_parameter("topics.door_candidates", params_.door_candidates_topic);
  this->declare_parameter("topics.debug_cells", params_.debug_cells_topic);
  this->declare_parameter("topics.debug_rejected_areas", params_.debug_rejected_areas_topic);
  this->declare_parameter("topics.debug_internal_barriers", params_.debug_internal_barriers_topic);
  this->declare_parameter(
    "topics.debug_scanline_intervals",
    params_.debug_scanline_intervals_topic);
  this->declare_parameter("frames.map", params_.map_frame);
  this->declare_parameter("overlay.mode", params_.overlay_mode);
  this->declare_parameter("overlay.publish_debug", params_.overlay_publish_debug);
  this->declare_parameter("thresholds.free_threshold", params_.free_threshold);
  this->declare_parameter("thresholds.occupied_threshold", params_.occupied_threshold);
  this->declare_parameter("robot.footprint_radius_m", params_.footprint_radius_m);
  this->declare_parameter("robot.safety_margin_m", params_.safety_margin_m);
  this->declare_parameter("update.period_sec", params_.period_sec);
  this->declare_parameter(
    "update.min_map_update_interval_sec", params_.min_map_update_interval_sec);
  this->declare_parameter("corridor.min_length_m", params_.corridor_min_length_m);
  this->declare_parameter("corridor.min_width_m", params_.corridor_min_width_m);
  this->declare_parameter("corridor.max_width_m", params_.corridor_max_width_m);
  this->declare_parameter("corridor.detection_mode", params_.corridor_detection_mode);
  this->declare_parameter("corridor.axis_aligned_only", params_.corridor_axis_aligned_only);
  this->declare_parameter("corridor.allow_diagonal", params_.corridor_allow_diagonal);
  this->declare_parameter(
    "corridor.parallel_angle_tolerance_deg", params_.corridor_parallel_angle_tolerance_deg);
  this->declare_parameter("corridor.min_wall_overlap_m", params_.corridor_min_wall_overlap_m);
  this->declare_parameter(
    "corridor.max_centerline_gap_m", params_.corridor_max_centerline_gap_m);
  this->declare_parameter(
    "corridor.min_free_ratio_between_walls", params_.corridor_min_free_ratio_between_walls);
  this->declare_parameter(
    "corridor.min_traversable_ratio_on_centerline",
    params_.corridor_min_traversable_ratio_on_centerline);
  this->declare_parameter("corridor.skeleton_enabled", params_.corridor_skeleton_enabled);
  this->declare_parameter(
    "corridor.skeleton_min_width_variance_m", params_.corridor_skeleton_min_width_variance_m);
  this->declare_parameter(
    "corridor.split_and_merge_angle_deg", params_.corridor_split_and_merge_angle_deg);
  this->declare_parameter("corridor.min_aspect_ratio", params_.corridor_min_aspect_ratio);
  this->declare_parameter("corridor.min_free_ratio", params_.corridor_min_free_ratio);
  this->declare_parameter(
    "corridor.min_boundary_evidence_ratio", params_.corridor_min_boundary_evidence_ratio);
  this->declare_parameter("corridor.min_confidence", params_.corridor_min_confidence);
  this->declare_parameter("corridor.max_corridors", params_.corridor_max_corridors);
  this->declare_parameter("scanline.max_gap_cells", params_.scanline_max_gap_cells);
  this->declare_parameter(
    "scanline.min_free_run_length_m", params_.scanline_min_free_run_length_m);
  this->declare_parameter("scanline.min_band_height_m", params_.scanline_min_band_height_m);
  this->declare_parameter("scanline.min_band_width_m", params_.scanline_min_band_width_m);
  this->declare_parameter(
    "scanline.run_endpoint_tolerance_m", params_.scanline_run_endpoint_tolerance_m);
  this->declare_parameter(
    "scanline.run_overlap_ratio_threshold", params_.scanline_run_overlap_ratio_threshold);
  this->declare_parameter(
    "scanline.min_rows_per_horizontal_band", params_.scanline_min_rows_per_horizontal_band);
  this->declare_parameter(
    "scanline.min_cols_per_vertical_band", params_.scanline_min_cols_per_vertical_band);
  this->declare_parameter("area.detection_mode", params_.area_detection_mode);
  this->declare_parameter("area.min_area_m2", params_.area_min_area_m2);
  this->declare_parameter("area.min_side_m", params_.area_min_side_m);
  this->declare_parameter("area.min_width_m", params_.area_min_width_m);
  this->declare_parameter("area.min_height_m", params_.area_min_height_m);
  this->declare_parameter("area.max_width_m", params_.area_max_width_m);
  this->declare_parameter("area.max_height_m", params_.area_max_height_m);
  this->declare_parameter("area.window_sizes_m", params_.area_window_sizes_m);
  this->declare_parameter("area.window_stride_m", params_.area_window_stride_m);
  this->declare_parameter("area.min_local_free_ratio", params_.area_min_local_free_ratio);
  this->declare_parameter("area.max_local_occupied_ratio", params_.area_max_local_occupied_ratio);
  this->declare_parameter("area.max_local_unknown_ratio", params_.area_max_local_unknown_ratio);
  this->declare_parameter("area.merge_iou_threshold", params_.area_merge_iou_threshold);
  this->declare_parameter("area.min_confidence", params_.area_min_confidence);
  this->declare_parameter("area.min_free_ratio", params_.area_min_free_ratio);
  this->declare_parameter("area.max_aspect_ratio", params_.area_max_aspect_ratio);
  this->declare_parameter(
    "area.corridor_exclusion_radius_m", params_.area_corridor_exclusion_radius_m);
  this->declare_parameter("area.portal_exclusion_radius_m", params_.area_portal_exclusion_radius_m);
  this->declare_parameter("area.reject_map_sized_bbox", params_.area_reject_map_sized_bbox);
  this->declare_parameter(
    "area.max_bbox_known_extent_ratio", params_.area_max_bbox_known_extent_ratio);
  this->declare_parameter("area.max_bbox_area_m2", params_.area_max_bbox_area_m2);
  this->declare_parameter("area.max_bbox_aspect_ratio", params_.area_max_bbox_aspect_ratio);
  this->declare_parameter("area.min_bbox_fill_ratio", params_.area_min_bbox_fill_ratio);
  this->declare_parameter(
    "area.interior_validation_enabled", params_.area_interior_validation_enabled);
  this->declare_parameter("area.max_occupied_ratio", params_.area_max_occupied_ratio);
  this->declare_parameter("area.max_unknown_ratio", params_.area_max_unknown_ratio);
  this->declare_parameter("area.max_boundary_ratio", params_.area_max_boundary_ratio);
  this->declare_parameter("area.internal_barrier_enabled", params_.area_internal_barrier_enabled);
  this->declare_parameter(
    "area.internal_barrier_min_span_ratio", params_.area_internal_barrier_min_span_ratio);
  this->declare_parameter(
    "area.internal_barrier_max_gap_m", params_.area_internal_barrier_max_gap_m);
  this->declare_parameter(
    "area.internal_barrier_margin_m", params_.area_internal_barrier_margin_m);
  this->declare_parameter("area.split_on_internal_barrier", params_.area_split_on_internal_barrier);
  this->declare_parameter("area.reject_if_split_fails", params_.area_reject_if_split_fails);
  this->declare_parameter("area.max_split_depth", params_.area_max_split_depth);
  this->declare_parameter("area.min_child_area_m2", params_.area_min_child_area_m2);
  this->declare_parameter("area.reject_tiny_ambiguous", params_.area_reject_tiny_ambiguous);
  this->declare_parameter("area.tiny_area_threshold_m2", params_.area_tiny_area_threshold_m2);
  this->declare_parameter(
    "area.tiny_area_min_boundary_evidence", params_.area_tiny_area_min_boundary_evidence);
  this->declare_parameter(
    "area.tiny_area_min_free_ratio", params_.area_tiny_area_min_free_ratio);
  this->declare_parameter(
    "area.reject_largest_component_without_split",
    params_.area_reject_largest_component_without_split);
  this->declare_parameter(
    "area.max_component_area_m2_without_split",
    params_.area_max_component_area_m2_without_split);
  this->declare_parameter("area.use_distance_seed", params_.area_use_distance_seed);
  this->declare_parameter("area.min_open_radius_m", params_.area_min_open_radius_m);
  this->declare_parameter(
    "area.seed_suppression_radius_m", params_.area_seed_suppression_radius_m);
  this->declare_parameter("area.publish_local_windows", params_.area_publish_local_windows);
  this->declare_parameter(
    "area.use_local_windows_as_debug_only", params_.area_use_local_windows_as_debug_only);
  this->declare_parameter("area.draw_contour", params_.area_draw_contour);
  this->declare_parameter("area.draw_bbox_fallback", params_.area_draw_bbox_fallback);
  this->declare_parameter("area.max_areas", params_.area_max_areas);
  this->declare_parameter("postprocess.enabled", params_.postprocess_enabled);
  this->declare_parameter(
    "postprocess.nms_iou_threshold", params_.postprocess_nms_iou_threshold);
  this->declare_parameter(
    "postprocess.merge_adjacent_bands", params_.postprocess_merge_adjacent_bands);
  this->declare_parameter("postprocess.merge_gap_m", params_.postprocess_merge_gap_m);
  this->declare_parameter(
    "postprocess.merge_same_orientation_only", params_.postprocess_merge_same_orientation_only);
  this->declare_parameter(
    "postprocess.max_published_segments", params_.postprocess_max_published_segments);
  this->declare_parameter(
    "postprocess.prefer_larger_coherent_cells", params_.postprocess_prefer_larger_coherent_cells);
  this->declare_parameter(
    "postprocess.suppress_nested_boxes",
    params_.postprocess_suppress_nested_boxes);
  this->declare_parameter(
    "postprocess.nested_box_iou_threshold", params_.postprocess_nested_box_iou_threshold);
  this->declare_parameter("cell_decomposition.enabled", params_.cell_decomposition_enabled);
  this->declare_parameter("cell_decomposition.method", params_.cell_decomposition_method);
  this->declare_parameter("cell_decomposition.sweep_axis", params_.cell_decomposition_sweep_axis);
  this->declare_parameter(
    "cell_decomposition.interval_overlap_ratio",
    params_.cell_decomposition_interval_overlap_ratio);
  this->declare_parameter(
    "cell_decomposition.endpoint_shift_threshold_m",
    params_.cell_decomposition_endpoint_shift_threshold_m);
  this->declare_parameter(
    "cell_decomposition.min_cell_area_m2", params_.cell_decomposition_min_cell_area_m2);
  this->declare_parameter(
    "cell_decomposition.min_cell_width_m", params_.cell_decomposition_min_cell_width_m);
  this->declare_parameter(
    "cell_decomposition.min_cell_height_m", params_.cell_decomposition_min_cell_height_m);
  this->declare_parameter(
    "cell_decomposition.max_cell_area_m2", params_.cell_decomposition_max_cell_area_m2);
  this->declare_parameter(
    "cell_decomposition.merge_adjacent_cells", params_.cell_decomposition_merge_adjacent_cells);
  this->declare_parameter(
    "cell_decomposition.merge_iou_threshold", params_.cell_decomposition_merge_iou_threshold);
  this->declare_parameter("cell_decomposition.merge_gap_m", params_.cell_decomposition_merge_gap_m);
  this->declare_parameter("debug.enabled", params_.debug_enabled);
  this->declare_parameter("debug.publish_cells", params_.debug_publish_cells);
  this->declare_parameter("debug.publish_rejected_areas", params_.debug_publish_rejected_areas);
  this->declare_parameter(
    "debug.publish_internal_barriers",
    params_.debug_publish_internal_barriers);
  this->declare_parameter(
    "debug.publish_scanline_intervals",
    params_.debug_publish_scanline_intervals);
  this->declare_parameter("door.enabled", params_.door_enabled);
  this->declare_parameter("door.min_width_m", params_.door_min_width_m);
  this->declare_parameter("door.max_width_m", params_.door_max_width_m);
  this->declare_parameter("door.min_confidence", params_.door_min_confidence);
  this->declare_parameter("door.max_doors", params_.door_max_doors);
  this->declare_parameter("visualization.marker_line_width_m", params_.marker_line_width_m);
  this->declare_parameter("visualization.marker_height_z", params_.marker_height_z);
  this->declare_parameter("visualization.max_area_boxes", params_.visualization_max_area_boxes);
  this->declare_parameter(
    "visualization.max_area_contour_points", params_.visualization_max_area_contour_points);
  this->declare_parameter(
    "visualization.use_fixed_marker_ids", params_.visualization_use_fixed_marker_ids);
  this->declare_parameter("visualization.publish_labels", params_.visualization_publish_labels);
  this->declare_parameter(
    "visualization.use_batched_line_list", params_.visualization_use_batched_line_list);
  this->declare_parameter(
    "visualization.max_total_marker_points", params_.visualization_max_total_marker_points);
  this->declare_parameter(
    "visualization.max_corridor_points",
    params_.visualization_max_corridor_points);
  this->declare_parameter("visualization.max_area_points", params_.visualization_max_area_points);
  this->declare_parameter("visualization.max_door_points", params_.visualization_max_door_points);
}

/// @copydoc SpatialSegmenter::on_configure
SpatialSegmenter::CallbackReturn SpatialSegmenter::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  load_parameters();
  if (
    params_.map_topic.empty() || params_.segments_topic.empty() ||
    params_.segment_map_topic.empty() || params_.markers_topic.empty() ||
    params_.map_frame.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Spatial segmenter topics and frame names must not be empty");
    return CallbackReturn::FAILURE;
  }

  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.map_topic,
    latched_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      handle_map(message);
    });
  segment_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    params_.segment_map_topic,
    latched_qos);
  markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.markers_topic,
    latched_qos);
  corridor_centerlines_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.corridor_centerlines_topic,
    latched_qos);
  area_boundaries_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.area_boundaries_topic,
    latched_qos);
  door_candidates_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.door_candidates_topic,
    latched_qos);
  debug_cells_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.debug_cells_topic,
    latched_qos);
  debug_rejected_areas_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.debug_rejected_areas_topic,
    latched_qos);
  debug_internal_barriers_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.debug_internal_barriers_topic,
    latched_qos);
  debug_scanline_intervals_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.debug_scanline_intervals_topic,
    latched_qos);
  segments_publisher_ = this->create_publisher<amr_msgs::msg::SpatialSegmentArray>(
    params_.segments_topic,
    latched_qos);

  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=configure map_topic=%s segment_map_topic=%s markers_topic=%s segments_topic=%s door_enabled=%s",
    params_.map_topic.c_str(),
    params_.segment_map_topic.c_str(),
    params_.markers_topic.c_str(),
    params_.segments_topic.c_str(),
    bool_label(params_.door_enabled));
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=config overlay_mode=%s corridor_detection_mode=%s area_detection_mode=%s axis_aligned_only=%s allow_diagonal=%s area_draw_contour=%s area_draw_bbox=%s reject_map_sized_bbox=%s publish_local_windows=%s use_batched_line_list=%s publish_debug=%s",
    params_.overlay_mode.c_str(),
    params_.corridor_detection_mode.c_str(),
    params_.area_detection_mode.c_str(),
    bool_label(params_.corridor_axis_aligned_only),
    bool_label(params_.corridor_allow_diagonal),
    bool_label(params_.area_draw_contour),
    bool_label(params_.area_draw_bbox_fallback),
    bool_label(params_.area_reject_map_sized_bbox),
    bool_label(params_.area_publish_local_windows),
    bool_label(params_.visualization_use_batched_line_list),
    bool_label(params_.overlay_publish_debug));
  return CallbackReturn::SUCCESS;
}

/// @copydoc SpatialSegmenter::on_activate
SpatialSegmenter::CallbackReturn SpatialSegmenter::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (segment_map_publisher_) {
    segment_map_publisher_->on_activate();
  }
  if (markers_publisher_) {
    markers_publisher_->on_activate();
  }
  if (corridor_centerlines_publisher_) {
    corridor_centerlines_publisher_->on_activate();
  }
  if (area_boundaries_publisher_) {
    area_boundaries_publisher_->on_activate();
  }
  if (door_candidates_publisher_) {
    door_candidates_publisher_->on_activate();
  }
  if (debug_cells_publisher_) {
    debug_cells_publisher_->on_activate();
  }
  if (debug_rejected_areas_publisher_) {
    debug_rejected_areas_publisher_->on_activate();
  }
  if (debug_internal_barriers_publisher_) {
    debug_internal_barriers_publisher_->on_activate();
  }
  if (debug_scanline_intervals_publisher_) {
    debug_scanline_intervals_publisher_->on_activate();
  }
  if (segments_publisher_) {
    segments_publisher_->on_activate();
  }

  const auto period_ms = std::chrono::milliseconds(
    std::max(1, static_cast<int>(std::round(std::max(params_.period_sec, 0.05) * 1000.0))));
  processing_timer_ = this->create_wall_timer(period_ms, [this]() {process_latest_map();});
  process_latest_map();
  RCLCPP_INFO(this->get_logger(), "Activated spatial segmenter");
  return CallbackReturn::SUCCESS;
}

/// @copydoc SpatialSegmenter::on_deactivate
SpatialSegmenter::CallbackReturn SpatialSegmenter::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  processing_timer_.reset();
  if (segment_map_publisher_) {
    segment_map_publisher_->on_deactivate();
  }
  if (markers_publisher_) {
    markers_publisher_->on_deactivate();
  }
  if (corridor_centerlines_publisher_) {
    corridor_centerlines_publisher_->on_deactivate();
  }
  if (area_boundaries_publisher_) {
    area_boundaries_publisher_->on_deactivate();
  }
  if (door_candidates_publisher_) {
    door_candidates_publisher_->on_deactivate();
  }
  if (debug_cells_publisher_) {
    debug_cells_publisher_->on_deactivate();
  }
  if (debug_rejected_areas_publisher_) {
    debug_rejected_areas_publisher_->on_deactivate();
  }
  if (debug_internal_barriers_publisher_) {
    debug_internal_barriers_publisher_->on_deactivate();
  }
  if (debug_scanline_intervals_publisher_) {
    debug_scanline_intervals_publisher_->on_deactivate();
  }
  if (segments_publisher_) {
    segments_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

/// @copydoc SpatialSegmenter::on_cleanup
SpatialSegmenter::CallbackReturn SpatialSegmenter::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  processing_timer_.reset();
  map_subscription_.reset();
  segment_map_publisher_.reset();
  markers_publisher_.reset();
  corridor_centerlines_publisher_.reset();
  area_boundaries_publisher_.reset();
  door_candidates_publisher_.reset();
  debug_cells_publisher_.reset();
  debug_rejected_areas_publisher_.reset();
  debug_internal_barriers_publisher_.reset();
  debug_scanline_intervals_publisher_.reset();
  segments_publisher_.reset();
  latest_map_.reset();
  latest_map_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_processed_map_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  map_dirty_ = false;
  return CallbackReturn::SUCCESS;
}

/// @copydoc SpatialSegmenter::on_shutdown
SpatialSegmenter::CallbackReturn SpatialSegmenter::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

/// @brief Load ROS parameters into the internal parameter block.
void SpatialSegmenter::load_parameters()
{
  this->get_parameter("topics.map", params_.map_topic);
  this->get_parameter("topics.segments", params_.segments_topic);
  this->get_parameter("topics.segment_map", params_.segment_map_topic);
  this->get_parameter("topics.markers", params_.markers_topic);
  this->get_parameter("topics.corridor_centerlines", params_.corridor_centerlines_topic);
  this->get_parameter("topics.area_boundaries", params_.area_boundaries_topic);
  this->get_parameter("topics.door_candidates", params_.door_candidates_topic);
  this->get_parameter("topics.debug_cells", params_.debug_cells_topic);
  this->get_parameter("topics.debug_rejected_areas", params_.debug_rejected_areas_topic);
  this->get_parameter("topics.debug_internal_barriers", params_.debug_internal_barriers_topic);
  this->get_parameter("topics.debug_scanline_intervals", params_.debug_scanline_intervals_topic);
  this->get_parameter("frames.map", params_.map_frame);
  this->get_parameter("overlay.mode", params_.overlay_mode);
  this->get_parameter("overlay.publish_debug", params_.overlay_publish_debug);
  this->get_parameter("thresholds.free_threshold", params_.free_threshold);
  this->get_parameter("thresholds.occupied_threshold", params_.occupied_threshold);
  this->get_parameter("robot.footprint_radius_m", params_.footprint_radius_m);
  this->get_parameter("robot.safety_margin_m", params_.safety_margin_m);
  this->get_parameter("update.period_sec", params_.period_sec);
  this->get_parameter(
    "update.min_map_update_interval_sec", params_.min_map_update_interval_sec);
  this->get_parameter("corridor.min_length_m", params_.corridor_min_length_m);
  this->get_parameter("corridor.min_width_m", params_.corridor_min_width_m);
  this->get_parameter("corridor.max_width_m", params_.corridor_max_width_m);
  this->get_parameter("corridor.detection_mode", params_.corridor_detection_mode);
  this->get_parameter("corridor.axis_aligned_only", params_.corridor_axis_aligned_only);
  this->get_parameter("corridor.allow_diagonal", params_.corridor_allow_diagonal);
  this->get_parameter(
    "corridor.parallel_angle_tolerance_deg", params_.corridor_parallel_angle_tolerance_deg);
  this->get_parameter("corridor.min_wall_overlap_m", params_.corridor_min_wall_overlap_m);
  this->get_parameter("corridor.max_centerline_gap_m", params_.corridor_max_centerline_gap_m);
  this->get_parameter(
    "corridor.min_free_ratio_between_walls", params_.corridor_min_free_ratio_between_walls);
  this->get_parameter(
    "corridor.min_traversable_ratio_on_centerline",
    params_.corridor_min_traversable_ratio_on_centerline);
  this->get_parameter("corridor.skeleton_enabled", params_.corridor_skeleton_enabled);
  this->get_parameter(
    "corridor.skeleton_min_width_variance_m", params_.corridor_skeleton_min_width_variance_m);
  this->get_parameter(
    "corridor.split_and_merge_angle_deg", params_.corridor_split_and_merge_angle_deg);
  this->get_parameter("corridor.min_aspect_ratio", params_.corridor_min_aspect_ratio);
  this->get_parameter("corridor.min_free_ratio", params_.corridor_min_free_ratio);
  this->get_parameter(
    "corridor.min_boundary_evidence_ratio", params_.corridor_min_boundary_evidence_ratio);
  this->get_parameter("corridor.min_confidence", params_.corridor_min_confidence);
  this->get_parameter("corridor.max_corridors", params_.corridor_max_corridors);
  this->get_parameter("scanline.max_gap_cells", params_.scanline_max_gap_cells);
  this->get_parameter(
    "scanline.min_free_run_length_m", params_.scanline_min_free_run_length_m);
  this->get_parameter("scanline.min_band_height_m", params_.scanline_min_band_height_m);
  this->get_parameter("scanline.min_band_width_m", params_.scanline_min_band_width_m);
  this->get_parameter(
    "scanline.run_endpoint_tolerance_m", params_.scanline_run_endpoint_tolerance_m);
  this->get_parameter(
    "scanline.run_overlap_ratio_threshold", params_.scanline_run_overlap_ratio_threshold);
  this->get_parameter(
    "scanline.min_rows_per_horizontal_band", params_.scanline_min_rows_per_horizontal_band);
  this->get_parameter(
    "scanline.min_cols_per_vertical_band", params_.scanline_min_cols_per_vertical_band);
  this->get_parameter("area.detection_mode", params_.area_detection_mode);
  this->get_parameter("area.min_area_m2", params_.area_min_area_m2);
  this->get_parameter("area.min_side_m", params_.area_min_side_m);
  this->get_parameter("area.min_width_m", params_.area_min_width_m);
  this->get_parameter("area.min_height_m", params_.area_min_height_m);
  this->get_parameter("area.max_width_m", params_.area_max_width_m);
  this->get_parameter("area.max_height_m", params_.area_max_height_m);
  this->get_parameter("area.window_sizes_m", params_.area_window_sizes_m);
  this->get_parameter("area.window_stride_m", params_.area_window_stride_m);
  this->get_parameter("area.min_local_free_ratio", params_.area_min_local_free_ratio);
  this->get_parameter("area.max_local_occupied_ratio", params_.area_max_local_occupied_ratio);
  this->get_parameter("area.max_local_unknown_ratio", params_.area_max_local_unknown_ratio);
  this->get_parameter("area.merge_iou_threshold", params_.area_merge_iou_threshold);
  this->get_parameter("area.min_confidence", params_.area_min_confidence);
  this->get_parameter("area.min_free_ratio", params_.area_min_free_ratio);
  this->get_parameter("area.max_aspect_ratio", params_.area_max_aspect_ratio);
  this->get_parameter(
    "area.corridor_exclusion_radius_m", params_.area_corridor_exclusion_radius_m);
  this->get_parameter("area.portal_exclusion_radius_m", params_.area_portal_exclusion_radius_m);
  this->get_parameter("area.reject_map_sized_bbox", params_.area_reject_map_sized_bbox);
  this->get_parameter(
    "area.max_bbox_known_extent_ratio", params_.area_max_bbox_known_extent_ratio);
  this->get_parameter("area.max_bbox_area_m2", params_.area_max_bbox_area_m2);
  this->get_parameter("area.max_bbox_aspect_ratio", params_.area_max_bbox_aspect_ratio);
  this->get_parameter("area.min_bbox_fill_ratio", params_.area_min_bbox_fill_ratio);
  this->get_parameter(
    "area.interior_validation_enabled", params_.area_interior_validation_enabled);
  this->get_parameter("area.max_occupied_ratio", params_.area_max_occupied_ratio);
  this->get_parameter("area.max_unknown_ratio", params_.area_max_unknown_ratio);
  this->get_parameter("area.max_boundary_ratio", params_.area_max_boundary_ratio);
  this->get_parameter("area.internal_barrier_enabled", params_.area_internal_barrier_enabled);
  this->get_parameter(
    "area.internal_barrier_min_span_ratio", params_.area_internal_barrier_min_span_ratio);
  this->get_parameter(
    "area.internal_barrier_max_gap_m", params_.area_internal_barrier_max_gap_m);
  this->get_parameter(
    "area.internal_barrier_margin_m", params_.area_internal_barrier_margin_m);
  this->get_parameter("area.split_on_internal_barrier", params_.area_split_on_internal_barrier);
  this->get_parameter("area.reject_if_split_fails", params_.area_reject_if_split_fails);
  this->get_parameter("area.max_split_depth", params_.area_max_split_depth);
  this->get_parameter("area.min_child_area_m2", params_.area_min_child_area_m2);
  this->get_parameter("area.reject_tiny_ambiguous", params_.area_reject_tiny_ambiguous);
  this->get_parameter("area.tiny_area_threshold_m2", params_.area_tiny_area_threshold_m2);
  this->get_parameter(
    "area.tiny_area_min_boundary_evidence", params_.area_tiny_area_min_boundary_evidence);
  this->get_parameter(
    "area.tiny_area_min_free_ratio", params_.area_tiny_area_min_free_ratio);
  this->get_parameter(
    "area.reject_largest_component_without_split",
    params_.area_reject_largest_component_without_split);
  this->get_parameter(
    "area.max_component_area_m2_without_split",
    params_.area_max_component_area_m2_without_split);
  this->get_parameter("area.use_distance_seed", params_.area_use_distance_seed);
  this->get_parameter("area.min_open_radius_m", params_.area_min_open_radius_m);
  this->get_parameter(
    "area.seed_suppression_radius_m", params_.area_seed_suppression_radius_m);
  this->get_parameter("area.publish_local_windows", params_.area_publish_local_windows);
  this->get_parameter(
    "area.use_local_windows_as_debug_only", params_.area_use_local_windows_as_debug_only);
  this->get_parameter("area.draw_contour", params_.area_draw_contour);
  this->get_parameter("area.draw_bbox_fallback", params_.area_draw_bbox_fallback);
  this->get_parameter("area.max_areas", params_.area_max_areas);
  this->get_parameter("postprocess.enabled", params_.postprocess_enabled);
  this->get_parameter(
    "postprocess.nms_iou_threshold", params_.postprocess_nms_iou_threshold);
  this->get_parameter(
    "postprocess.merge_adjacent_bands", params_.postprocess_merge_adjacent_bands);
  this->get_parameter("postprocess.merge_gap_m", params_.postprocess_merge_gap_m);
  this->get_parameter(
    "postprocess.merge_same_orientation_only", params_.postprocess_merge_same_orientation_only);
  this->get_parameter(
    "postprocess.max_published_segments", params_.postprocess_max_published_segments);
  this->get_parameter(
    "postprocess.prefer_larger_coherent_cells", params_.postprocess_prefer_larger_coherent_cells);
  this->get_parameter(
    "postprocess.suppress_nested_boxes",
    params_.postprocess_suppress_nested_boxes);
  this->get_parameter(
    "postprocess.nested_box_iou_threshold", params_.postprocess_nested_box_iou_threshold);
  this->get_parameter("cell_decomposition.enabled", params_.cell_decomposition_enabled);
  this->get_parameter("cell_decomposition.method", params_.cell_decomposition_method);
  this->get_parameter("cell_decomposition.sweep_axis", params_.cell_decomposition_sweep_axis);
  this->get_parameter(
    "cell_decomposition.interval_overlap_ratio",
    params_.cell_decomposition_interval_overlap_ratio);
  this->get_parameter(
    "cell_decomposition.endpoint_shift_threshold_m",
    params_.cell_decomposition_endpoint_shift_threshold_m);
  this->get_parameter(
    "cell_decomposition.min_cell_area_m2", params_.cell_decomposition_min_cell_area_m2);
  this->get_parameter(
    "cell_decomposition.min_cell_width_m", params_.cell_decomposition_min_cell_width_m);
  this->get_parameter(
    "cell_decomposition.min_cell_height_m", params_.cell_decomposition_min_cell_height_m);
  this->get_parameter(
    "cell_decomposition.max_cell_area_m2", params_.cell_decomposition_max_cell_area_m2);
  this->get_parameter(
    "cell_decomposition.merge_adjacent_cells", params_.cell_decomposition_merge_adjacent_cells);
  this->get_parameter(
    "cell_decomposition.merge_iou_threshold", params_.cell_decomposition_merge_iou_threshold);
  this->get_parameter("cell_decomposition.merge_gap_m", params_.cell_decomposition_merge_gap_m);
  this->get_parameter("debug.enabled", params_.debug_enabled);
  this->get_parameter("debug.publish_cells", params_.debug_publish_cells);
  this->get_parameter("debug.publish_rejected_areas", params_.debug_publish_rejected_areas);
  this->get_parameter("debug.publish_internal_barriers", params_.debug_publish_internal_barriers);
  this->get_parameter("debug.publish_scanline_intervals", params_.debug_publish_scanline_intervals);
  this->get_parameter("door.enabled", params_.door_enabled);
  this->get_parameter("door.min_width_m", params_.door_min_width_m);
  this->get_parameter("door.max_width_m", params_.door_max_width_m);
  this->get_parameter("door.min_confidence", params_.door_min_confidence);
  this->get_parameter("door.max_doors", params_.door_max_doors);
  this->get_parameter("visualization.marker_line_width_m", params_.marker_line_width_m);
  this->get_parameter("visualization.marker_height_z", params_.marker_height_z);
  this->get_parameter("visualization.max_area_boxes", params_.visualization_max_area_boxes);
  this->get_parameter(
    "visualization.max_area_contour_points", params_.visualization_max_area_contour_points);
  this->get_parameter(
    "visualization.use_fixed_marker_ids", params_.visualization_use_fixed_marker_ids);
  this->get_parameter("visualization.publish_labels", params_.visualization_publish_labels);
  this->get_parameter(
    "visualization.use_batched_line_list", params_.visualization_use_batched_line_list);
  this->get_parameter(
    "visualization.max_total_marker_points", params_.visualization_max_total_marker_points);
  this->get_parameter(
    "visualization.max_corridor_points",
    params_.visualization_max_corridor_points);
  this->get_parameter("visualization.max_area_points", params_.visualization_max_area_points);
  this->get_parameter("visualization.max_door_points", params_.visualization_max_door_points);
}

/// @brief Cache the latest map for periodic processing.
void SpatialSegmenter::handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  latest_map_ = message;
  latest_map_time_ = this->now();
  map_dirty_ = true;
}

/// @brief Process the latest map when active and throttling allows it.
void SpatialSegmenter::process_latest_map()
{
  if (!should_process_map()) {
    return;
  }

  const auto source_map = *latest_map_;
  const auto width = static_cast<std::size_t>(source_map.info.width);
  const auto height = static_cast<std::size_t>(source_map.info.height);
  if (
    width == 0U || height == 0U || source_map.info.resolution <= 0.0F ||
    source_map.data.size() != width * height)
  {
    (void)analyze_map(source_map);
    map_dirty_ = false;
    last_processed_map_time_ = latest_map_time_;
    return;
  }

  map_dirty_ = false;
  last_processed_map_time_ = latest_map_time_;
  const SegmentationResult result = analyze_map(source_map);
  auto segment_map = build_segment_map(source_map, result);
  auto markers = build_markers(segment_map.header, result.segments, false, false, false);
  auto corridor_markers = build_markers(segment_map.header, result.segments, true, false, false);
  auto area_markers = build_markers(segment_map.header, result.segments, false, true, false);
  auto door_markers = build_markers(segment_map.header, result.segments, false, false, true);
  auto segment_array = build_segment_array(segment_map.header, result.segments);
  const MarkerPointCounts semantic_points = count_marker_points(markers);
  const int corridors_published = semantic_points.corridor_points / 2;
  const int areas_published = semantic_points.area_points / 8;
  const int doors_published = semantic_points.door_points / 2;
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=spatial_stats raw_horizontal_runs=%d raw_vertical_runs=%d corridor_candidates=%d corridors_published=%d area_components=%d areas_published=%d door_candidates=%d doors_published=%d",
    result.stats.raw_horizontal_runs,
    result.stats.raw_vertical_runs,
    result.stats.corridor_candidates,
    corridors_published,
    result.stats.area_components,
    areas_published,
    result.stats.door_candidates,
    doors_published);
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=area_stats raw_components=%d large_components=%d unresolved_components=%d local_candidates=%d accepted_areas=%d rejected_areas=%d areas_published=%d",
    result.stats.raw_components,
    result.stats.large_components,
    result.stats.unresolved_components,
    result.stats.local_candidates,
    result.stats.areas_published,
    result.stats.rejected_areas,
    areas_published);
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=band_stats horizontal_runs=%d vertical_runs=%d horizontal_bands=%d vertical_bands=%d corridor_bands=%d area_bands=%d local_windows=%d",
    result.stats.raw_horizontal_runs,
    result.stats.raw_vertical_runs,
    result.stats.horizontal_bands,
    result.stats.vertical_bands,
    result.stats.corridor_bands,
    result.stats.area_bands,
    result.stats.local_windows);
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=cell_decomposition_stats sweep_axis=%s intervals=%d raw_cells=%d merged_cells=%d split_cells=%d final_cells=%d",
    params_.cell_decomposition_sweep_axis.c_str(),
    result.stats.cell_intervals,
    result.stats.raw_cells,
    result.stats.merged_cells,
    result.stats.split_cells,
    result.stats.final_cells);
  RCLCPP_INFO(
    this->get_logger(),
    "AMR_LOG schema=v1 component=spatial_segmenter event=spatial_marker_publish mode=%s corridors=%d areas=%d doors=%d corridor_points=%d area_points=%d door_points=%d total_markers=%d total_points=%d corridors_dropped=%d areas_dropped=%d doors_dropped=%d",
    params_.overlay_mode.c_str(),
    corridors_published,
    areas_published,
    doors_published,
    semantic_points.corridor_points,
    semantic_points.area_points,
    semantic_points.door_points,
    static_cast<int>(markers.markers.size()),
    semantic_points.total_points,
    std::max(0, result.stats.corridors_published - corridors_published),
    std::max(0, result.stats.areas_published - areas_published),
    std::max(0, result.stats.doors_published - doors_published));
  publish_result(segment_map, markers, corridor_markers, area_markers, door_markers, segment_array);
}

/// @brief Return true when the cached map should be processed.
bool SpatialSegmenter::should_process_map() const
{
  if (!map_dirty_ || !latest_map_) {
    return false;
  }
  if (last_processed_map_time_.nanoseconds() == 0) {
    return true;
  }
  const double elapsed = (latest_map_time_ - last_processed_map_time_).seconds();
  return elapsed >= std::max(0.0, params_.min_map_update_interval_sec);
}

/// @brief Run the rule-based segmentation pipeline for one map.
SegmentationResult SpatialSegmenter::analyze_map(const nav_msgs::msg::OccupancyGrid & map) const
{
  OccupancyGridView view(map, params_.free_threshold, params_.occupied_threshold);
  SegmentationResult empty_result;
  if (!view.valid()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid map for spatial segmentation: width=%u height=%u resolution=%.4f data=%zu",
      map.info.width,
      map.info.height,
      map.info.resolution,
      map.data.size());
    return empty_result;
  }

  const auto raw_free = view.raw_free_mask();
  const auto raw_occupied = view.raw_occupied_mask();
  DistanceMap raw_obstacle_distance;
  raw_obstacle_distance.compute(view.width(), view.height(), view.resolution(), raw_occupied);

  DistanceMap validation_distance;
  validation_distance.compute(view.width(), view.height(), view.resolution(), view.blocked_mask());
  const double clearance_m = std::max(0.0, params_.footprint_radius_m + params_.safety_margin_m);
  std::vector<bool> traversable(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    traversable[static_cast<std::size_t>(index)] =
      view.is_free(index) && validation_distance.at(index) >= clearance_m;
  }

  SegmentClassifier classifier(params_);
  return classifier.classify(view, raw_free, traversable, raw_obstacle_distance);
}

/// @brief Build a labeled OccupancyGrid from segmentation labels.
nav_msgs::msg::OccupancyGrid SpatialSegmenter::build_segment_map(
  const nav_msgs::msg::OccupancyGrid & source,
  const SegmentationResult & result) const
{
  nav_msgs::msg::OccupancyGrid segment_map = source;
  segment_map.header = source.header;
  if (segment_map.header.frame_id.empty()) {
    segment_map.header.frame_id = params_.map_frame;
  }
  if (result.labels.size() == source.data.size()) {
    segment_map.data = result.labels;
  } else {
    OccupancyGridView view(source, params_.free_threshold, params_.occupied_threshold);
    segment_map.data.assign(source.data.size(), 0);
    if (view.valid()) {
      for (int index = 0; index < view.cell_count(); ++index) {
        if (view.is_unknown(index)) {
          segment_map.data[static_cast<std::size_t>(index)] = -1;
        } else if (view.is_occupied(index)) {
          segment_map.data[static_cast<std::size_t>(index)] = 100;
        }
      }
    }
  }
  return segment_map;
}

/// @brief Convert segments into visualization markers.
visualization_msgs::msg::MarkerArray SpatialSegmenter::build_markers(
  const std_msgs::msg::Header & header,
  const std::vector<Segment> & segments,
  const bool corridor_only,
  const bool area_only,
  const bool door_only) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  const bool all_types = !corridor_only && !area_only && !door_only;
  const bool include_corridors = all_types || corridor_only;
  const bool include_areas = all_types || area_only;
  const bool include_doors = all_types || door_only;
  int remaining_total_points = std::max(0, params_.visualization_max_total_marker_points);

  auto make_line_marker = [this, &header](
    const char * ns,
    const int id) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.header.frame_id =
        marker.header.frame_id.empty() ? params_.map_frame : marker.header.frame_id;
      marker.ns = ns;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(0.001, params_.marker_line_width_m);
      marker.color.r = 0.0F;
      marker.color.g = 0.0F;
      marker.color.b = 0.0F;
      marker.color.a = 1.0F;
      return marker;
    };
  auto make_label_marker = [this, &header]() {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.header.frame_id =
        marker.header.frame_id.empty() ? params_.map_frame : marker.header.frame_id;
      marker.ns = "spatial_labels";
      marker.id = 3;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.16;
      marker.color.r = 0.0F;
      marker.color.g = 0.0F;
      marker.color.b = 0.0F;
      marker.color.a = 1.0F;
      return marker;
    };
  auto add_point =
    [this](visualization_msgs::msg::Marker & marker, geometry_msgs::msg::Point point) {
      point.z = params_.marker_height_z;
      marker.points.push_back(point);
    };
  auto append_bbox =
    [&add_point](visualization_msgs::msg::Marker & marker, const Segment & segment) {
      geometry_msgs::msg::Point p0 = segment.bbox_min;
      geometry_msgs::msg::Point p1 = segment.bbox_min;
      geometry_msgs::msg::Point p2 = segment.bbox_max;
      geometry_msgs::msg::Point p3 = segment.bbox_max;
      p1.x = segment.bbox_max.x;
      p3.x = segment.bbox_min.x;
      add_point(marker, p0);
      add_point(marker, p1);
      add_point(marker, p1);
      add_point(marker, p2);
      add_point(marker, p2);
      add_point(marker, p3);
      add_point(marker, p3);
      add_point(marker, p0);
    };

  std::vector<const Segment *> corridors;
  std::vector<const Segment *> areas;
  std::vector<const Segment *> doors;
  for (const auto & segment : segments) {
    if (segment.type == SegmentType::Corridor && segment.centerline.size() >= 2) {
      corridors.push_back(&segment);
    } else if (segment.type == SegmentType::OpenArea) {
      areas.push_back(&segment);
    } else if (segment.type == SegmentType::DoorCandidate && segment.centerline.size() >= 2) {
      doors.push_back(&segment);
    }
  }
  std::sort(
    corridors.begin(), corridors.end(), [](const Segment * lhs, const Segment * rhs) {
      if (std::abs(lhs->confidence - rhs->confidence) > 1.0e-6) {
        return lhs->confidence > rhs->confidence;
      }
      return lhs->length_m > rhs->length_m;
    });
  std::sort(
    areas.begin(), areas.end(), [](const Segment * lhs, const Segment * rhs) {
      if (std::abs(lhs->confidence - rhs->confidence) > 1.0e-6) {
        return lhs->confidence > rhs->confidence;
      }
      return lhs->area_m2 > rhs->area_m2;
    });
  std::sort(
    doors.begin(), doors.end(), [](const Segment * lhs, const Segment * rhs) {
      return lhs->confidence > rhs->confidence;
    });

  auto corridor_marker = make_line_marker("spatial_corridors", 0);
  auto area_marker = make_line_marker("spatial_area_boxes", 1);
  auto door_marker = make_line_marker("spatial_doors", 2);

  if (include_corridors) {
    const int max_corridors = std::min(
      {
        std::max(0, params_.corridor_max_corridors),
        std::max(0, params_.visualization_max_corridor_points) / 2,
        remaining_total_points / 2});
    const int count = std::min(max_corridors, static_cast<int>(corridors.size()));
    for (int i = 0; i < count; ++i) {
      const auto & centerline = corridors[static_cast<std::size_t>(i)]->centerline;
      add_point(corridor_marker, centerline.front());
      add_point(corridor_marker, centerline.back());
      remaining_total_points -= 2;
    }
  }

  if (include_areas) {
    const int max_area_boxes = std::min(
      {
        std::max(0, params_.area_max_areas),
        std::max(0, params_.visualization_max_area_boxes),
        std::max(0, params_.visualization_max_area_points) / 8,
        remaining_total_points / 8});
    const int count = std::min(max_area_boxes, static_cast<int>(areas.size()));
    for (int i = 0; i < count; ++i) {
      append_bbox(area_marker, *areas[static_cast<std::size_t>(i)]);
      remaining_total_points -= 8;
    }
  }

  if (include_doors) {
    const int max_doors = std::min(
      {
        std::max(0, params_.door_max_doors),
        std::max(0, params_.visualization_max_door_points) / 2,
        remaining_total_points / 2});
    const int count = std::min(max_doors, static_cast<int>(doors.size()));
    for (int i = 0; i < count; ++i) {
      const auto & centerline = doors[static_cast<std::size_t>(i)]->centerline;
      add_point(door_marker, centerline.front());
      add_point(door_marker, centerline.back());
      remaining_total_points -= 2;
    }
  }

  if (all_types) {
    marker_array.markers.push_back(corridor_marker);
    marker_array.markers.push_back(area_marker);
    marker_array.markers.push_back(door_marker);
    if (params_.visualization_publish_labels) {
      marker_array.markers.push_back(make_label_marker());
    }
  } else if (corridor_only) {
    marker_array.markers.push_back(corridor_marker);
  } else if (area_only) {
    marker_array.markers.push_back(area_marker);
  } else if (door_only) {
    marker_array.markers.push_back(door_marker);
  }

  return marker_array;
}

/// @brief Convert internal segments into the custom ROS message array.
amr_msgs::msg::SpatialSegmentArray SpatialSegmenter::build_segment_array(
  const std_msgs::msg::Header & header,
  const std::vector<Segment> & segments) const
{
  amr_msgs::msg::SpatialSegmentArray array;
  array.header = header;
  array.header.frame_id = array.header.frame_id.empty() ? params_.map_frame : array.header.frame_id;
  array.segments.reserve(segments.size());
  for (const auto & segment : segments) {
    amr_msgs::msg::SpatialSegment message;
    message.id = segment.id;
    message.type = to_message_type(segment.type);
    message.confidence = static_cast<float>(segment.confidence);
    message.centroid = segment.centroid;
    message.polygon = segment.polygon;
    message.centerline = segment.centerline;
    message.bbox_min = segment.bbox_min;
    message.bbox_max = segment.bbox_max;
    message.area_m2 = static_cast<float>(segment.area_m2);
    message.length_m = static_cast<float>(segment.length_m);
    message.width_m = static_cast<float>(segment.width_m);
    array.segments.push_back(message);
  }
  return array;
}

/// @brief Publish all spatial segmentation outputs.
void SpatialSegmenter::publish_result(
  const nav_msgs::msg::OccupancyGrid & segment_map,
  const visualization_msgs::msg::MarkerArray & markers,
  const visualization_msgs::msg::MarkerArray & corridor_markers,
  const visualization_msgs::msg::MarkerArray & area_markers,
  const visualization_msgs::msg::MarkerArray & door_markers,
  const amr_msgs::msg::SpatialSegmentArray & segments)
{
  if (segment_map_publisher_ && segment_map_publisher_->is_activated()) {
    segment_map_publisher_->publish(segment_map);
  }
  if (markers_publisher_ && markers_publisher_->is_activated()) {
    markers_publisher_->publish(markers);
  }
  if (corridor_centerlines_publisher_ && corridor_centerlines_publisher_->is_activated()) {
    corridor_centerlines_publisher_->publish(corridor_markers);
  }
  if (area_boundaries_publisher_ && area_boundaries_publisher_->is_activated()) {
    area_boundaries_publisher_->publish(area_markers);
  }
  if (door_candidates_publisher_ && door_candidates_publisher_->is_activated()) {
    door_candidates_publisher_->publish(door_markers);
  }
  if (params_.debug_enabled) {
    visualization_msgs::msg::MarkerArray empty_debug_markers;
    if (params_.debug_publish_cells && debug_cells_publisher_ &&
      debug_cells_publisher_->is_activated())
    {
      debug_cells_publisher_->publish(empty_debug_markers);
    }
    if (params_.debug_publish_rejected_areas && debug_rejected_areas_publisher_ &&
      debug_rejected_areas_publisher_->is_activated())
    {
      debug_rejected_areas_publisher_->publish(empty_debug_markers);
    }
    if (params_.debug_publish_internal_barriers && debug_internal_barriers_publisher_ &&
      debug_internal_barriers_publisher_->is_activated())
    {
      debug_internal_barriers_publisher_->publish(empty_debug_markers);
    }
    if (params_.debug_publish_scanline_intervals && debug_scanline_intervals_publisher_ &&
      debug_scanline_intervals_publisher_->is_activated())
    {
      debug_scanline_intervals_publisher_->publish(empty_debug_markers);
    }
  }
  if (segments_publisher_ && segments_publisher_->is_activated()) {
    segments_publisher_->publish(segments);
  }
}

}  // namespace amr::spatial_segmenter
