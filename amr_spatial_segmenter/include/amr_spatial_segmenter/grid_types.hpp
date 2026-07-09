#ifndef AMR_SPATIAL_SEGMENTER__GRID_TYPES_HPP_
#define AMR_SPATIAL_SEGMENTER__GRID_TYPES_HPP_

/**
 * @file grid_types.hpp
 * @brief Shared value types for OccupancyGrid spatial segmentation.
 */

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace amr::spatial_segmenter
{

/// @brief OccupancyGrid cell state after conservative thresholding.
enum class CellState : std::uint8_t
{
  Unknown,
  Free,
  Ambiguous,
  Occupied,
};

/// @brief Segment class labels mirrored by amr_msgs/msg/SpatialSegment.
enum class SegmentType : std::uint8_t
{
  Unknown = 0,
  OpenArea = 1,
  Corridor = 2,
  Junction = 3,
  DoorCandidate = 4,
  NarrowPassage = 5,
  UnresolvedFreeSpace = 6,
};

/// @brief Integer grid coordinate.
struct GridIndex
{
  int x{0};
  int y{0};
};

/// @brief Connected component extracted from a binary grid mask.
struct Component
{
  std::vector<int> cells;
  int min_x{0};
  int min_y{0};
  int max_x{0};
  int max_y{0};
};

/// @brief Runtime parameters used by the classifier.
struct SegmenterParams
{
  std::string map_topic{"/map"};
  std::string segments_topic{"/spatial_segments"};
  std::string segment_map_topic{"/spatial_segment_map"};
  std::string markers_topic{"/spatial_segment_markers"};
  std::string corridor_centerlines_topic{"/corridor_centerlines"};
  std::string area_boundaries_topic{"/area_boundaries"};
  std::string door_candidates_topic{"/door_candidates"};
  std::string debug_cells_topic{"/spatial_debug_cells"};
  std::string debug_rejected_areas_topic{"/spatial_debug_rejected_areas"};
  std::string debug_internal_barriers_topic{"/spatial_debug_internal_barriers"};
  std::string debug_scanline_intervals_topic{"/spatial_debug_scanline_intervals"};
  std::string map_frame{"map"};

  std::string overlay_mode{"semantic_summary"};
  bool overlay_publish_debug{false};

  int free_threshold{25};
  int occupied_threshold{65};

  double footprint_radius_m{0.18};
  double safety_margin_m{0.10};
  double period_sec{5.0};
  double min_map_update_interval_sec{3.0};

  double corridor_min_length_m{1.0};
  double corridor_min_width_m{0.45};
  double corridor_max_width_m{2.20};
  std::string corridor_detection_mode{"scanline_band"};
  bool corridor_axis_aligned_only{true};
  bool corridor_allow_diagonal{false};
  double corridor_parallel_angle_tolerance_deg{12.0};
  double corridor_min_wall_overlap_m{0.80};
  double corridor_max_centerline_gap_m{0.35};
  double corridor_min_free_ratio_between_walls{0.75};
  double corridor_min_traversable_ratio_on_centerline{0.80};
  bool corridor_skeleton_enabled{true};
  double corridor_skeleton_min_width_variance_m{0.30};
  double corridor_split_and_merge_angle_deg{15.0};
  double corridor_min_aspect_ratio{2.2};
  double corridor_min_free_ratio{0.70};
  double corridor_min_boundary_evidence_ratio{0.35};
  double corridor_min_confidence{0.45};
  int corridor_max_corridors{12};

  int scanline_max_gap_cells{2};
  double scanline_min_free_run_length_m{0.80};
  double scanline_min_band_height_m{0.35};
  double scanline_min_band_width_m{0.35};
  double scanline_run_endpoint_tolerance_m{0.35};
  double scanline_run_overlap_ratio_threshold{0.60};
  int scanline_min_rows_per_horizontal_band{4};
  int scanline_min_cols_per_vertical_band{4};

  std::string area_detection_mode{"cell_decomposition"};
  double area_min_area_m2{0.50};
  double area_min_side_m{0.60};
  double area_min_width_m{0.60};
  double area_min_height_m{0.60};
  double area_max_width_m{3.00};
  double area_max_height_m{3.00};
  std::vector<double> area_window_sizes_m{0.8, 1.2, 1.6, 2.0};
  double area_window_stride_m{0.20};
  double area_min_local_free_ratio{0.70};
  double area_max_local_occupied_ratio{0.05};
  double area_max_local_unknown_ratio{0.30};
  double area_merge_iou_threshold{0.35};
  double area_min_confidence{0.45};
  double area_min_free_ratio{0.70};
  double area_max_aspect_ratio{3.5};
  double area_corridor_exclusion_radius_m{0.25};
  double area_portal_exclusion_radius_m{0.20};
  bool area_reject_map_sized_bbox{true};
  double area_max_bbox_known_extent_ratio{0.45};
  double area_max_bbox_area_m2{10.0};
  double area_max_bbox_aspect_ratio{4.0};
  double area_min_bbox_fill_ratio{0.35};
  bool area_interior_validation_enabled{true};
  double area_max_occupied_ratio{0.08};
  double area_max_unknown_ratio{0.35};
  double area_max_boundary_ratio{0.30};
  bool area_internal_barrier_enabled{true};
  double area_internal_barrier_min_span_ratio{0.55};
  double area_internal_barrier_max_gap_m{0.15};
  double area_internal_barrier_margin_m{0.10};
  bool area_split_on_internal_barrier{true};
  bool area_reject_if_split_fails{true};
  int area_max_split_depth{2};
  double area_min_child_area_m2{0.50};
  bool area_reject_tiny_ambiguous{true};
  double area_tiny_area_threshold_m2{0.80};
  double area_tiny_area_min_boundary_evidence{0.55};
  double area_tiny_area_min_free_ratio{0.80};
  bool area_reject_largest_component_without_split{true};
  double area_max_component_area_m2_without_split{4.0};
  bool area_use_distance_seed{true};
  double area_min_open_radius_m{0.45};
  double area_seed_suppression_radius_m{0.80};
  bool area_publish_local_windows{false};
  bool area_use_local_windows_as_debug_only{true};
  bool area_draw_contour{false};
  bool area_draw_bbox_fallback{true};
  int area_max_areas{8};

  bool postprocess_enabled{true};
  double postprocess_nms_iou_threshold{0.20};
  bool postprocess_merge_adjacent_bands{true};
  double postprocess_merge_gap_m{0.30};
  bool postprocess_merge_same_orientation_only{true};
  int postprocess_max_published_segments{20};
  bool postprocess_prefer_larger_coherent_cells{true};
  bool postprocess_suppress_nested_boxes{true};
  double postprocess_nested_box_iou_threshold{0.60};

  bool cell_decomposition_enabled{true};
  std::string cell_decomposition_method{"boustrophedon_scanline"};
  std::string cell_decomposition_sweep_axis{"both"};
  double cell_decomposition_interval_overlap_ratio{0.45};
  double cell_decomposition_endpoint_shift_threshold_m{0.45};
  double cell_decomposition_min_cell_area_m2{0.50};
  double cell_decomposition_min_cell_width_m{0.60};
  double cell_decomposition_min_cell_height_m{0.60};
  double cell_decomposition_max_cell_area_m2{10.0};
  bool cell_decomposition_merge_adjacent_cells{true};
  double cell_decomposition_merge_iou_threshold{0.20};
  double cell_decomposition_merge_gap_m{0.20};

  bool debug_enabled{false};
  bool debug_publish_cells{false};
  bool debug_publish_rejected_areas{false};
  bool debug_publish_internal_barriers{false};
  bool debug_publish_scanline_intervals{false};

  bool door_enabled{true};
  double door_min_width_m{0.55};
  double door_max_width_m{1.20};
  double door_min_confidence{0.35};
  int door_max_doors{20};

  double marker_line_width_m{0.045};
  double marker_height_z{0.03};
  int visualization_max_area_boxes{12};
  int visualization_max_area_contour_points{300};
  bool visualization_use_fixed_marker_ids{true};
  bool visualization_publish_labels{false};
  bool visualization_use_batched_line_list{true};
  int visualization_max_total_marker_points{300};
  int visualization_max_corridor_points{40};
  int visualization_max_area_points{96};
  int visualization_max_door_points{40};
};

/// @brief Lightweight counters for segmentation and marker diagnostics.
struct SegmentationStats
{
  int raw_horizontal_runs{0};
  int raw_vertical_runs{0};
  int horizontal_bands{0};
  int vertical_bands{0};
  int corridor_bands{0};
  int area_bands{0};
  int local_windows{0};
  int cell_intervals{0};
  int raw_cells{0};
  int merged_cells{0};
  int split_cells{0};
  int final_cells{0};
  int corridor_candidates{0};
  int corridors_published{0};
  int raw_components{0};
  int large_components{0};
  int unresolved_components{0};
  int local_candidates{0};
  int rejected_areas{0};
  int portal_cut_cells{0};
  int area_components{0};
  int areas_published{0};
  int door_candidates{0};
  int doors_published{0};
};

/// @brief One classified spatial feature in world coordinates.
struct Segment
{
  std::uint32_t id{0};
  SegmentType type{SegmentType::Unknown};
  double confidence{0.0};
  std::vector<int> cells;
  geometry_msgs::msg::Pose centroid;
  std::vector<geometry_msgs::msg::Point> polygon;
  std::vector<geometry_msgs::msg::Point> boundary_edges;
  std::vector<geometry_msgs::msg::Point> centerline;
  geometry_msgs::msg::Point bbox_min;
  geometry_msgs::msg::Point bbox_max;
  double area_m2{0.0};
  double length_m{0.0};
  double width_m{0.0};
};

/// @brief Complete segmentation result for one OccupancyGrid update.
struct SegmentationResult
{
  std::vector<Segment> segments;
  std::vector<int8_t> labels;
  SegmentationStats stats;
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__GRID_TYPES_HPP_
