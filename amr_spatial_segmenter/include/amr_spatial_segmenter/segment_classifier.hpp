#ifndef AMR_SPATIAL_SEGMENTER__SEGMENT_CLASSIFIER_HPP_
#define AMR_SPATIAL_SEGMENTER__SEGMENT_CLASSIFIER_HPP_

/**
 * @file segment_classifier.hpp
 * @brief Deterministic corridor/open-area classifier over traversable map components.
 */

#include <string>
#include <vector>

#include "amr_spatial_segmenter/distance_map.hpp"
#include "amr_spatial_segmenter/grid_types.hpp"
#include "amr_spatial_segmenter/occupancy_grid_view.hpp"

namespace amr::spatial_segmenter
{

/// @brief Rule-based spatial segment classifier.
class SegmentClassifier
{
public:
  /// @brief Construct a classifier with immutable runtime parameters.
  explicit SegmentClassifier(const SegmenterParams & params);

  /// @brief Classify raw free-space structure and validate centerlines against traversability.
  SegmentationResult classify(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    const std::vector<bool> & traversable,
    const DistanceMap & raw_obstacle_distance) const;

private:
  struct ComponentStats
  {
    geometry_msgs::msg::Point centroid;
    geometry_msgs::msg::Point bbox_min;
    geometry_msgs::msg::Point bbox_max;
    std::vector<geometry_msgs::msg::Point> bbox_polygon;
    double area_m2{0.0};
    double bbox_width_m{0.0};
    double bbox_height_m{0.0};
    double length_m{0.0};
    double width_m{0.0};
    double aspect_ratio{0.0};
    double axis_x{1.0};
    double axis_y{0.0};
    double min_projection{0.0};
    double max_projection{0.0};
  };

  struct GridBounds
  {
    int min_x{0};
    int min_y{0};
    int max_x{0};
    int max_y{0};
    bool valid{false};
  };

  struct AreaCandidate
  {
    std::uint32_t diagnostic_id{0};
    int min_x{0};
    int min_y{0};
    int max_x{0};
    int max_y{0};
    int source_component{-1};
    double component_area_m2{0.0};
    double bbox_area_m2{0.0};
    double known_extent_ratio{0.0};
    double free_extent_ratio{0.0};
    double fill_ratio{0.0};
    double free_ratio{0.0};
    double occupied_ratio{0.0};
    double unknown_ratio{0.0};
    double corridor_overlap_ratio{0.0};
    double portal_overlap_ratio{0.0};
    double confidence{0.0};
    bool from_component_bbox{false};
    bool largest_component_without_split{false};
  };

  struct SkeletonLine
  {
    std::vector<int> cells;
    std::vector<double> width_samples_m;
    GridIndex start;
    GridIndex end;
    double length_m{0.0};
    double width_m{0.0};
    double width_stddev_m{0.0};
    double traversable_ratio{0.0};
    double confidence{0.0};
    bool horizontal{true};
  };

  struct AxisAlignedDetection
  {
    std::vector<SkeletonLine> lines;
    int raw_horizontal_runs{0};
    int raw_vertical_runs{0};
    int corridor_candidates{0};
  };

  struct ScanlineRun
  {
    bool horizontal{true};
    int line{0};
    int start{0};
    int end{0};
    double free_ratio{0.0};
    double boundary_evidence{0.0};
  };

  struct ScanlineBand
  {
    bool horizontal{true};
    std::vector<ScanlineRun> runs;
    int min_x{0};
    int min_y{0};
    int max_x{0};
    int max_y{0};
    double length_m{0.0};
    double width_m{0.0};
    double area_m2{0.0};
    double aspect_ratio{0.0};
    double free_fill_ratio{0.0};
    double occupied_ratio{0.0};
    double unknown_ratio{0.0};
    double boundary_ratio{0.0};
    int internal_barrier_count{0};
    double boundary_evidence{0.0};
    double endpoint_stability{0.0};
    double confidence{0.0};
    SegmentType type{SegmentType::Unknown};
  };

  struct CellInterval
  {
    int y{0};
    int start{0};
    int end{0};
  };

  struct CellCandidate
  {
    std::uint32_t diagnostic_id{0};
    std::vector<CellInterval> intervals;
    int min_x{0};
    int min_y{0};
    int max_x{0};
    int max_y{0};
    double endpoint_stability{0.0};
  };

  struct BarrierInfo
  {
    bool found{false};
    bool vertical{true};
    int line{0};
    double span_ratio{0.0};
  };

  struct AreaValidation
  {
    bool accepted{false};
    std::string reason{"unknown"};
    double area_m2{0.0};
    double free_ratio{0.0};
    double occupied_ratio{0.0};
    double unknown_ratio{0.0};
    double boundary_ratio{0.0};
    double boundary_evidence{0.0};
    double endpoint_stability{0.0};
    double confidence{0.0};
    int internal_barriers{0};
    BarrierInfo barrier;
  };

  struct CellDecompositionDetection
  {
    std::vector<ScanlineBand> cells;
    int intervals{0};
    int raw_cells{0};
    int merged_cells{0};
    int split_cells{0};
    int final_cells{0};
    int rejected_cells{0};
  };

  struct ScanlineBandDetection
  {
    std::vector<ScanlineRun> horizontal_runs;
    std::vector<ScanlineRun> vertical_runs;
    std::vector<ScanlineBand> horizontal_bands;
    std::vector<ScanlineBand> vertical_bands;
    int local_windows{0};
    int rejected_bands{0};
  };

  Segment build_corridor_segment(
    std::uint32_t id,
    const SkeletonLine & line,
    const OccupancyGridView & view) const;
  Segment build_door_segment(
    std::uint32_t id,
    const std::vector<int> & cells,
    double width_m,
    double confidence,
    const OccupancyGridView & view) const;
  Segment build_open_area_segment(
    std::uint32_t id,
    const Component & component,
    const ComponentStats & stats,
    const OccupancyGridView & view) const;
  Segment build_open_area_segment(
    std::uint32_t id,
    const AreaCandidate & candidate,
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    const std::vector<bool> & area_mask) const;
  std::vector<geometry_msgs::msg::Point> build_boundary_edges(
    const Component & component,
    const OccupancyGridView & view) const;
  AxisAlignedDetection detect_axis_aligned_corridors(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    const std::vector<bool> & traversable) const;
  std::vector<SkeletonLine> detect_skeleton_corridors(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    const std::vector<bool> & traversable,
    const DistanceMap & raw_obstacle_distance) const;
  std::vector<SkeletonLine> split_skeleton_component(
    const Component & component,
    const OccupancyGridView & view,
    const std::vector<bool> & traversable,
    const DistanceMap & raw_obstacle_distance) const;
  SkeletonLine build_skeleton_line(
    const std::vector<int> & ordered_cells,
    int start_index,
    int end_index,
    const OccupancyGridView & view,
    const std::vector<bool> & traversable,
    const DistanceMap & raw_obstacle_distance) const;
  double traversable_ratio_on_line(
    const GridIndex & start,
    const GridIndex & end,
    const OccupancyGridView & view,
    const std::vector<bool> & traversable) const;
  double traversable_ratio_for_cells(
    const std::vector<int> & cells,
    const std::vector<bool> & traversable) const;
  std::vector<Segment> detect_door_candidates(
    std::uint32_t & next_id,
    const std::vector<SkeletonLine> & corridor_lines,
    const OccupancyGridView & view) const;
  ScanlineBandDetection detect_scanline_bands(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free) const;
  Segment build_band_segment(
    std::uint32_t id,
    const ScanlineBand & band,
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free) const;
  std::vector<ScanlineBand> postprocess_bands(
    std::vector<ScanlineBand> bands,
    const OccupancyGridView & view) const;
  CellDecompositionDetection detect_cell_decomposition(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free) const;
  std::vector<ScanlineBand> validate_area_bbox_recursive(
    std::uint32_t & next_id,
    const ScanlineBand & candidate,
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    int depth,
    int & split_count,
    int & rejected_count) const;
  AreaValidation validate_area_candidate(
    std::uint32_t id,
    const ScanlineBand & candidate,
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free) const;
  BarrierInfo detect_internal_barrier(
    const ScanlineBand & candidate,
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free) const;
  void log_area_validation(
    std::uint32_t id,
    const ScanlineBand & candidate,
    const AreaValidation & validation) const;
  std::vector<bool> expanded_mask(
    const OccupancyGridView & view,
    const std::vector<bool> & seeds,
    double radius_m) const;
  GridBounds mask_bounds(
    const OccupancyGridView & view,
    const std::vector<bool> & mask) const;
  AreaCandidate component_bbox_candidate(
    std::uint32_t diagnostic_id,
    const Component & component,
    const ComponentStats & stats,
    const GridBounds & known_extent,
    const std::vector<bool> & raw_free,
    const OccupancyGridView & view,
    bool largest_component_without_split) const;
  std::vector<AreaCandidate> detect_local_area_candidates(
    const OccupancyGridView & view,
    const std::vector<bool> & raw_free,
    const std::vector<bool> & area_mask,
    const std::vector<bool> & corridor_exclusion_mask,
    const std::vector<bool> & portal_exclusion_mask,
    const DistanceMap & raw_obstacle_distance,
    const GridBounds & known_extent,
    const std::vector<int> & component_ids,
    const std::vector<double> & component_areas_m2,
    int & local_candidate_count,
    int & rejected_count) const;
  bool reject_area_candidate(
    const AreaCandidate & candidate,
    const GridBounds & known_extent,
    const GridBounds & free_extent,
    std::string & reason) const;
  void log_area_rejected(
    const AreaCandidate & candidate,
    const std::string & reason) const;
  ComponentStats analyze_component(
    const Component & component,
    const OccupancyGridView & view) const;
  bool is_open_area(const ComponentStats & stats) const;
  double corridor_confidence(const SkeletonLine & line) const;
  double open_area_confidence(const ComponentStats & stats) const;
  static double clamp01(double value);

  SegmenterParams params_;
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__SEGMENT_CLASSIFIER_HPP_
