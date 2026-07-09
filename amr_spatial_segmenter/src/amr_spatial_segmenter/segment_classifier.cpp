/**
 * @file segment_classifier.cpp
 * @brief Implementation of deterministic spatial segment classification.
 */

#include "amr_spatial_segmenter/segment_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <utility>

#include "amr_spatial_segmenter/connected_components.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr::spatial_segmenter
{

namespace
{

constexpr double k_epsilon = 1.0e-6;

geometry_msgs::msg::Point make_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace

/// @copydoc SegmentClassifier::SegmentClassifier
SegmentClassifier::SegmentClassifier(const SegmenterParams & params)
: params_(params)
{
}

/// @copydoc SegmentClassifier::classify
SegmentationResult SegmentClassifier::classify(
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free,
  const std::vector<bool> & traversable,
  const DistanceMap & raw_obstacle_distance) const
{
  SegmentationResult result;
  if (
    !view.valid() ||
    raw_free.size() != static_cast<std::size_t>(view.cell_count()) ||
    traversable.size() != static_cast<std::size_t>(view.cell_count()))
  {
    return result;
  }

  result.labels.assign(static_cast<std::size_t>(view.cell_count()), 0);
  for (int index = 0; index < view.cell_count(); ++index) {
    if (view.is_unknown(index)) {
      result.labels[static_cast<std::size_t>(index)] = -1;
    } else if (view.is_occupied(index)) {
      result.labels[static_cast<std::size_t>(index)] = 100;
    }
  }

  const double min_corridor_area =
    params_.corridor_min_length_m * params_.corridor_min_width_m;

  const bool use_scanline_bands =
    params_.corridor_detection_mode == "scanline_band" ||
    params_.area_detection_mode == "scanline_band" ||
    params_.area_detection_mode == "cell_decomposition";
  if (use_scanline_bands) {
    const double min_corridor_or_area = std::min(params_.area_min_area_m2, min_corridor_area);
    const int raw_min_cells = std::max(
      1, static_cast<int>(std::floor(
        min_corridor_or_area / std::max(view.resolution() * view.resolution(), k_epsilon))));
    const auto raw_components =
      ConnectedComponents::extract(raw_free, view.width(), view.height(), raw_min_cells);
    result.stats.raw_components = static_cast<int>(raw_components.size());
    result.stats.area_components = result.stats.raw_components;
    for (const auto & component : raw_components) {
      const double component_area_m2 =
        static_cast<double>(component.cells.size()) * view.resolution() * view.resolution();
      if (component_area_m2 > params_.area_max_component_area_m2_without_split) {
        ++result.stats.large_components;
      }
    }

    const ScanlineBandDetection detection = detect_scanline_bands(view, raw_free);
    result.stats.raw_horizontal_runs = static_cast<int>(detection.horizontal_runs.size());
    result.stats.raw_vertical_runs = static_cast<int>(detection.vertical_runs.size());
    result.stats.horizontal_bands = static_cast<int>(detection.horizontal_bands.size());
    result.stats.vertical_bands = static_cast<int>(detection.vertical_bands.size());
    result.stats.local_windows = detection.local_windows;
    result.stats.local_candidates = detection.local_windows;
    result.stats.rejected_areas = detection.rejected_bands;

    std::vector<ScanlineBand> bands = detection.horizontal_bands;
    bands.insert(bands.end(), detection.vertical_bands.begin(), detection.vertical_bands.end());
    if (params_.area_detection_mode == "cell_decomposition") {
      bands.erase(
        std::remove_if(
          bands.begin(), bands.end(), [](const ScanlineBand & band) {
            return band.type == SegmentType::OpenArea;
          }),
        bands.end());
      const CellDecompositionDetection cells = detect_cell_decomposition(view, raw_free);
      result.stats.cell_intervals = cells.intervals;
      result.stats.raw_cells = cells.raw_cells;
      result.stats.merged_cells = cells.merged_cells;
      result.stats.split_cells = cells.split_cells;
      result.stats.final_cells = cells.final_cells;
      result.stats.rejected_areas += cells.rejected_cells;
      bands.insert(bands.end(), cells.cells.begin(), cells.cells.end());
    }
    bands = postprocess_bands(std::move(bands), view);

    std::uint32_t next_id = 1U;
    for (const auto & band : bands) {
      Segment segment = build_band_segment(next_id++, band, view, raw_free);
      for (const int cell : segment.cells) {
        if (cell < 0 || cell >= static_cast<int>(result.labels.size())) {
          continue;
        }
        if (segment.type == SegmentType::Corridor) {
          if (result.labels[static_cast<std::size_t>(cell)] == 0) {
            result.labels[static_cast<std::size_t>(cell)] = 20;
          }
        } else if (segment.type == SegmentType::OpenArea) {
          if (result.labels[static_cast<std::size_t>(cell)] == 0) {
            result.labels[static_cast<std::size_t>(cell)] = 10;
          }
        }
      }
      if (segment.type == SegmentType::Corridor) {
        ++result.stats.corridor_bands;
        ++result.stats.corridors_published;
      } else if (segment.type == SegmentType::OpenArea) {
        ++result.stats.area_bands;
        ++result.stats.areas_published;
      }
      result.segments.push_back(segment);
    }

    if (result.stats.large_components > 0 && result.stats.corridor_bands == 0 &&
      result.stats.area_bands == 0)
    {
      result.stats.unresolved_components = result.stats.large_components;
    }
    return result;
  }

  auto expanded_axis_aligned_line = [&view, &raw_free](const SkeletonLine & line) {
      SkeletonLine expanded = line;
      std::vector<bool> member(static_cast<std::size_t>(view.cell_count()), false);
      for (const int cell : expanded.cells) {
        if (cell >= 0 && cell < view.cell_count()) {
          member[static_cast<std::size_t>(cell)] = true;
        }
      }

      const int half_width_cells = std::max(
        0, static_cast<int>(std::ceil(
          (line.width_m * 0.5) / std::max(
            view.resolution(), k_epsilon))));
      const int min_x = std::min(line.start.x, line.end.x);
      const int max_x = std::max(line.start.x, line.end.x);
      const int min_y = std::min(line.start.y, line.end.y);
      const int max_y = std::max(line.start.y, line.end.y);
      const int x0 = line.horizontal ? min_x : line.start.x - half_width_cells;
      const int x1 = line.horizontal ? max_x : line.start.x + half_width_cells;
      const int y0 = line.horizontal ? line.start.y - half_width_cells : min_y;
      const int y1 = line.horizontal ? line.start.y + half_width_cells : max_y;

      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          if (!view.in_bounds(x, y)) {
            continue;
          }
          const int index = view.to_index(x, y);
          if (!raw_free[static_cast<std::size_t>(index)] ||
            member[static_cast<std::size_t>(index)])
          {
            continue;
          }
          expanded.cells.push_back(index);
          member[static_cast<std::size_t>(index)] = true;
        }
      }

      return expanded;
    };

  std::uint32_t next_id = 1U;
  std::vector<bool> corridor_mask(static_cast<std::size_t>(view.cell_count()), false);
  std::vector<SkeletonLine> corridor_lines;
  const bool use_axis_aligned =
    params_.corridor_axis_aligned_only ||
    !params_.corridor_allow_diagonal ||
    params_.corridor_detection_mode == "axis_aligned_scanline";
  if (use_axis_aligned) {
    const AxisAlignedDetection detection =
      detect_axis_aligned_corridors(view, raw_free, traversable);
    corridor_lines = detection.lines;
    result.stats.raw_horizontal_runs = detection.raw_horizontal_runs;
    result.stats.raw_vertical_runs = detection.raw_vertical_runs;
    result.stats.corridor_candidates = detection.corridor_candidates;
  } else if (
    params_.corridor_skeleton_enabled &&
    (params_.corridor_detection_mode.empty() ||
    params_.corridor_detection_mode == "skeleton" ||
    params_.corridor_detection_mode == "wall_pair_then_skeleton"))
  {
    corridor_lines = detect_skeleton_corridors(
      view, raw_free, traversable, raw_obstacle_distance);
    result.stats.corridor_candidates = static_cast<int>(corridor_lines.size());
  }

  for (const auto & line : corridor_lines) {
    if (
      params_.corridor_axis_aligned_only &&
      line.start.x != line.end.x &&
      line.start.y != line.end.y)
    {
      continue;
    }
    SkeletonLine expanded = expanded_axis_aligned_line(line);
    Segment segment = build_corridor_segment(next_id++, expanded, view);
    for (const int cell : expanded.cells) {
      if (cell >= 0 && cell < static_cast<int>(result.labels.size())) {
        corridor_mask[static_cast<std::size_t>(cell)] = true;
        if (result.labels[static_cast<std::size_t>(cell)] == 0) {
          result.labels[static_cast<std::size_t>(cell)] = 20;
        }
      }
    }
    result.segments.push_back(segment);
    ++result.stats.corridors_published;
  }

  std::vector<bool> portal_mask(static_cast<std::size_t>(view.cell_count()), false);
  if (params_.door_enabled) {
    for (auto & door : detect_door_candidates(next_id, corridor_lines, view)) {
      ++result.stats.door_candidates;
      for (const int cell : door.cells) {
        if (cell >= 0 && cell < static_cast<int>(result.labels.size()) &&
          result.labels[static_cast<std::size_t>(cell)] >= 0 &&
          result.labels[static_cast<std::size_t>(cell)] < 100)
        {
          result.labels[static_cast<std::size_t>(cell)] = 40;
        }
        if (cell >= 0 && cell < view.cell_count()) {
          portal_mask[static_cast<std::size_t>(cell)] = true;
        }
      }
      result.segments.push_back(door);
      ++result.stats.doors_published;
    }
  }

  const std::vector<bool> corridor_exclusion_mask = expanded_mask(
    view, corridor_mask, params_.area_corridor_exclusion_radius_m);
  const std::vector<bool> portal_exclusion_mask = expanded_mask(
    view, portal_mask, params_.area_portal_exclusion_radius_m);
  std::vector<bool> area_mask(raw_free.size(), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    area_mask[static_cast<std::size_t>(index)] =
      raw_free[static_cast<std::size_t>(index)] &&
      !corridor_exclusion_mask[static_cast<std::size_t>(index)] &&
      !portal_exclusion_mask[static_cast<std::size_t>(index)];
    if (portal_exclusion_mask[static_cast<std::size_t>(index)] &&
      raw_free[static_cast<std::size_t>(index)])
    {
      ++result.stats.portal_cut_cells;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("spatial_segmenter"),
    "AMR_LOG schema=v1 component=spatial_segmenter event=portal_cuts_applied doors=%d cut_cells=%d",
    result.stats.doors_published,
    result.stats.portal_cut_cells);

  const double min_corridor_or_area = std::min(params_.area_min_area_m2, min_corridor_area);
  const int raw_min_cells = std::max(
    1, static_cast<int>(std::floor(
      min_corridor_or_area / std::max(view.resolution() * view.resolution(), k_epsilon))));
  const auto raw_components =
    ConnectedComponents::extract(raw_free, view.width(), view.height(), raw_min_cells);
  result.stats.raw_components = static_cast<int>(raw_components.size());
  result.stats.area_components = result.stats.raw_components;

  std::vector<bool> known_mask(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    known_mask[static_cast<std::size_t>(index)] = !view.is_unknown(index);
  }
  const GridBounds known_extent = mask_bounds(view, known_mask);
  const GridBounds free_extent = mask_bounds(view, raw_free);

  std::vector<int> component_ids(static_cast<std::size_t>(view.cell_count()), -1);
  std::vector<double> component_areas_m2(raw_components.size(), 0.0);
  int largest_component = -1;
  double largest_component_area_m2 = 0.0;
  const double cell_area_m2 = view.resolution() * view.resolution();
  for (int component_index = 0; component_index < static_cast<int>(raw_components.size());
    ++component_index)
  {
    const auto & component = raw_components[static_cast<std::size_t>(component_index)];
    const double component_area_m2 = static_cast<double>(component.cells.size()) * cell_area_m2;
    component_areas_m2[static_cast<std::size_t>(component_index)] = component_area_m2;
    if (component_area_m2 > params_.area_max_component_area_m2_without_split) {
      ++result.stats.large_components;
    }
    if (component_area_m2 > largest_component_area_m2) {
      largest_component_area_m2 = component_area_m2;
      largest_component = component_index;
    }
    for (const int cell : component.cells) {
      if (cell >= 0 && cell < view.cell_count()) {
        component_ids[static_cast<std::size_t>(cell)] = component_index;
      }
    }
  }

  std::vector<bool> component_has_area(raw_components.size(), false);
  std::uint32_t diagnostic_id = next_id;
  const bool has_split_cuts = result.stats.corridors_published > 0 ||
    result.stats.portal_cut_cells > 0;
  for (int component_index = 0; component_index < static_cast<int>(raw_components.size());
    ++component_index)
  {
    const auto & component = raw_components[static_cast<std::size_t>(component_index)];
    const ComponentStats stats = analyze_component(component, view);
    const AreaCandidate candidate = component_bbox_candidate(
      diagnostic_id++,
      component,
      stats,
      known_extent,
      raw_free,
      view,
      component_index == largest_component && !has_split_cuts);
    std::string rejection_reason;
    if (reject_area_candidate(candidate, known_extent, free_extent, rejection_reason)) {
      ++result.stats.rejected_areas;
      log_area_rejected(candidate, rejection_reason);
      continue;
    }
    if (params_.area_detection_mode != "local_free_rectangles") {
      Segment segment = build_open_area_segment(next_id++, component, stats, view);
      for (const int cell : component.cells) {
        if (result.labels[static_cast<std::size_t>(cell)] == 0) {
          result.labels[static_cast<std::size_t>(cell)] = 10;
        }
      }
      result.segments.push_back(segment);
      if (component_index >= 0 && component_index < static_cast<int>(component_has_area.size())) {
        component_has_area[static_cast<std::size_t>(component_index)] = true;
      }
      ++result.stats.areas_published;
    }
  }

  if (params_.area_detection_mode == "local_free_rectangles") {
    int local_candidate_count = 0;
    int local_rejected_count = 0;
    const auto local_areas = detect_local_area_candidates(
      view,
      raw_free,
      area_mask,
      corridor_exclusion_mask,
      portal_exclusion_mask,
      raw_obstacle_distance,
      known_extent,
      component_ids,
      component_areas_m2,
      local_candidate_count,
      local_rejected_count);
    result.stats.local_candidates = local_candidate_count;
    result.stats.rejected_areas += local_rejected_count;
    for (const auto & candidate : local_areas) {
      Segment segment = build_open_area_segment(next_id++, candidate, view, raw_free, area_mask);
      for (const int cell : segment.cells) {
        if (cell >= 0 && cell < static_cast<int>(result.labels.size()) &&
          result.labels[static_cast<std::size_t>(cell)] == 0)
        {
          result.labels[static_cast<std::size_t>(cell)] = 10;
        }
      }
      if (candidate.source_component >= 0 &&
        candidate.source_component < static_cast<int>(component_has_area.size()))
      {
        component_has_area[static_cast<std::size_t>(candidate.source_component)] = true;
      }
      result.segments.push_back(segment);
      ++result.stats.areas_published;
    }
  }

  for (int component_index = 0; component_index < static_cast<int>(component_areas_m2.size());
    ++component_index)
  {
    if (component_areas_m2[static_cast<std::size_t>(component_index)] >
      params_.area_max_component_area_m2_without_split &&
      !component_has_area[static_cast<std::size_t>(component_index)])
    {
      ++result.stats.unresolved_components;
    }
  }

  return result;
}

/// @brief Build a corridor segment from component stats.
Segment SegmentClassifier::build_corridor_segment(
  const std::uint32_t id,
  const SkeletonLine & line,
  const OccupancyGridView & view) const
{
  Segment segment;
  segment.id = id;
  segment.type = SegmentType::Corridor;
  segment.confidence = line.confidence;
  segment.cells = line.cells;
  const auto start = view.cell_center(view.to_index(line.start.x, line.start.y));
  const auto end = view.cell_center(view.to_index(line.end.x, line.end.y));
  segment.centroid.position = make_point(
    (start.x + end.x) * 0.5,
    (start.y + end.y) * 0.5,
    params_.marker_height_z);
  segment.centroid.orientation.w = 1.0;
  segment.centerline.push_back(view.cell_center(view.to_index(line.start.x, line.start.y)));
  segment.centerline.push_back(view.cell_center(view.to_index(line.end.x, line.end.y)));
  for (auto & point : segment.centerline) {
    point.z = params_.marker_height_z;
  }
  segment.bbox_min = make_point(
    std::min(segment.centerline.front().x, segment.centerline.back().x),
    std::min(segment.centerline.front().y, segment.centerline.back().y),
    params_.marker_height_z);
  segment.bbox_max = make_point(
    std::max(segment.centerline.front().x, segment.centerline.back().x),
    std::max(segment.centerline.front().y, segment.centerline.back().y),
    params_.marker_height_z);
  segment.area_m2 = line.length_m * line.width_m;
  segment.length_m = line.length_m;
  segment.width_m = line.width_m;
  return segment;
}

/// @brief Build a compact door/portal candidate segment from bottleneck cells.
Segment SegmentClassifier::build_door_segment(
  const std::uint32_t id,
  const std::vector<int> & cells,
  const double width_m,
  const double confidence,
  const OccupancyGridView & view) const
{
  Segment segment;
  segment.id = id;
  segment.type = SegmentType::DoorCandidate;
  segment.confidence = confidence;
  segment.cells = cells;
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const int cell : cells) {
    const auto point = view.cell_center(cell);
    sum_x += point.x;
    sum_y += point.y;
  }
  const double count = static_cast<double>(std::max<std::size_t>(1U, cells.size()));
  segment.centroid.position = make_point(
    sum_x / count,
    sum_y / count,
    params_.marker_height_z);
  segment.centroid.orientation.w = 1.0;
  segment.bbox_min = segment.centroid.position;
  segment.bbox_max = segment.centroid.position;
  segment.area_m2 = width_m * view.resolution();
  segment.length_m = static_cast<double>(cells.size()) * view.resolution();
  segment.width_m = width_m;
  if (!cells.empty()) {
    const GridIndex first = view.to_grid(cells.front());
    const GridIndex last = view.to_grid(cells.back());
    const bool corridor_horizontal = std::abs(last.x - first.x) >= std::abs(last.y - first.y);
    geometry_msgs::msg::Point a = segment.centroid.position;
    geometry_msgs::msg::Point b = segment.centroid.position;
    const double half_width = width_m * 0.5;
    if (corridor_horizontal) {
      a.y -= half_width;
      b.y += half_width;
    } else {
      a.x -= half_width;
      b.x += half_width;
    }
    segment.centerline = {a, b};
  }
  return segment;
}

/// @brief Build an open-area segment from component stats.
Segment SegmentClassifier::build_open_area_segment(
  const std::uint32_t id,
  const Component & component,
  const ComponentStats & stats,
  const OccupancyGridView & view) const
{
  Segment segment;
  segment.id = id;
  segment.type = SegmentType::OpenArea;
  segment.confidence = open_area_confidence(stats);
  segment.cells = component.cells;
  segment.centroid.position = stats.centroid;
  segment.centroid.orientation.w = 1.0;
  segment.polygon = stats.bbox_polygon;
  for (auto & point : segment.polygon) {
    point.z = params_.marker_height_z;
  }
  segment.boundary_edges = build_boundary_edges(component, view);
  segment.bbox_min = stats.bbox_min;
  segment.bbox_max = stats.bbox_max;
  segment.area_m2 = stats.area_m2;
  segment.length_m = std::max(stats.bbox_width_m, stats.bbox_height_m);
  segment.width_m = std::min(stats.bbox_width_m, stats.bbox_height_m);
  return segment;
}

/// @brief Build an open-area segment from an accepted local rectangle proposal.
Segment SegmentClassifier::build_open_area_segment(
  const std::uint32_t id,
  const AreaCandidate & candidate,
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free,
  const std::vector<bool> & area_mask) const
{
  Segment segment;
  segment.id = id;
  segment.type = SegmentType::OpenArea;
  segment.confidence = candidate.confidence;
  const auto p0 = view.grid_corner(candidate.min_x, candidate.min_y);
  const auto p1 = view.grid_corner(candidate.max_x + 1, candidate.min_y);
  const auto p2 = view.grid_corner(candidate.max_x + 1, candidate.max_y + 1);
  const auto p3 = view.grid_corner(candidate.min_x, candidate.max_y + 1);
  segment.polygon = {p0, p1, p2, p3, p0};
  for (auto & point : segment.polygon) {
    point.z = params_.marker_height_z;
  }
  segment.bbox_min = make_point(
    std::min({p0.x, p1.x, p2.x, p3.x}),
    std::min({p0.y, p1.y, p2.y, p3.y}),
    params_.marker_height_z);
  segment.bbox_max = make_point(
    std::max({p0.x, p1.x, p2.x, p3.x}),
    std::max({p0.y, p1.y, p2.y, p3.y}),
    params_.marker_height_z);
  segment.centroid.position = make_point(
    (segment.bbox_min.x + segment.bbox_max.x) * 0.5,
    (segment.bbox_min.y + segment.bbox_max.y) * 0.5,
    params_.marker_height_z);
  segment.centroid.orientation.w = 1.0;
  segment.length_m = std::max(
    segment.bbox_max.x - segment.bbox_min.x,
    segment.bbox_max.y - segment.bbox_min.y);
  segment.width_m = std::min(
    segment.bbox_max.x - segment.bbox_min.x,
    segment.bbox_max.y - segment.bbox_min.y);
  segment.area_m2 = candidate.bbox_area_m2;
  for (int y = candidate.min_y; y <= candidate.max_y; ++y) {
    for (int x = candidate.min_x; x <= candidate.max_x; ++x) {
      if (!view.in_bounds(x, y)) {
        continue;
      }
      const int index = view.to_index(x, y);
      if (index >= 0 && index < static_cast<int>(area_mask.size()) &&
        area_mask[static_cast<std::size_t>(index)] &&
        raw_free[static_cast<std::size_t>(index)])
      {
        segment.cells.push_back(index);
      }
    }
  }
  return segment;
}

/// @brief Build exposed grid-cell boundary edges for a free-space component.
std::vector<geometry_msgs::msg::Point> SegmentClassifier::build_boundary_edges(
  const Component & component,
  const OccupancyGridView & view) const
{
  std::vector<geometry_msgs::msg::Point> edges;
  if (component.cells.empty() || !view.valid()) {
    return edges;
  }

  std::vector<bool> member(static_cast<std::size_t>(view.cell_count()), false);
  for (const int cell : component.cells) {
    if (cell >= 0 && cell < view.cell_count()) {
      member[static_cast<std::size_t>(cell)] = true;
    }
  }

  auto is_member = [&member, &view](const int x, const int y) -> bool {
      if (!view.in_bounds(x, y)) {
        return false;
      }
      return static_cast<bool>(member[static_cast<std::size_t>(view.to_index(x, y))]);
    };
  auto add_edge = [this, &edges](geometry_msgs::msg::Point a, geometry_msgs::msg::Point b) {
      a.z = params_.marker_height_z;
      b.z = params_.marker_height_z;
      edges.push_back(a);
      edges.push_back(b);
    };

  edges.reserve(component.cells.size() / 2U);
  for (int y = component.min_y; y <= component.max_y + 1; ++y) {
    int x = component.min_x;
    while (x <= component.max_x) {
      while (x <= component.max_x && is_member(x, y - 1) == is_member(x, y)) {
        ++x;
      }
      const int run_start = x;
      while (x <= component.max_x && is_member(x, y - 1) != is_member(x, y)) {
        ++x;
      }
      if (run_start < x) {
        add_edge(view.grid_corner(run_start, y), view.grid_corner(x, y));
      }
    }
  }

  for (int x = component.min_x; x <= component.max_x + 1; ++x) {
    int y = component.min_y;
    while (y <= component.max_y) {
      while (y <= component.max_y && is_member(x - 1, y) == is_member(x, y)) {
        ++y;
      }
      const int run_start = y;
      while (y <= component.max_y && is_member(x - 1, y) != is_member(x, y)) {
        ++y;
      }
      if (run_start < y) {
        add_edge(view.grid_corner(x, run_start), view.grid_corner(x, y));
      }
    }
  }

  return edges;
}

/// @brief Detect clean horizontal/vertical corridor candidates with raw-map scanlines.
SegmentClassifier::AxisAlignedDetection SegmentClassifier::detect_axis_aligned_corridors(
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free,
  const std::vector<bool> & traversable) const
{
  AxisAlignedDetection detection;
  if (!view.valid() || raw_free.size() != static_cast<std::size_t>(view.cell_count())) {
    return detection;
  }

  const int width = view.width();
  const int height = view.height();
  const double resolution = std::max(view.resolution(), k_epsilon);
  std::vector<double> vertical_widths(static_cast<std::size_t>(view.cell_count()), 0.0);
  std::vector<double> horizontal_widths(static_cast<std::size_t>(view.cell_count()), 0.0);
  std::vector<bool> horizontal_centers(static_cast<std::size_t>(view.cell_count()), false);
  std::vector<bool> vertical_centers(static_cast<std::size_t>(view.cell_count()), false);

  for (int x = 0; x < width; ++x) {
    int y = 0;
    while (y < height) {
      while (y < height && !raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++y;
      }
      const int run_start = y;
      while (y < height && raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++y;
      }
      const int run_end = y - 1;
      if (run_start > run_end) {
        continue;
      }
      ++detection.raw_vertical_runs;
      const int run_cells = run_end - run_start + 1;
      const double run_width = static_cast<double>(run_cells) * resolution;
      const int center_y = (run_start + run_end) / 2;
      for (int cell_y = run_start; cell_y <= run_end; ++cell_y) {
        vertical_widths[static_cast<std::size_t>(view.to_index(x, cell_y))] = run_width;
      }
      if (run_width >= params_.corridor_min_width_m &&
        run_width <= params_.corridor_max_width_m)
      {
        horizontal_centers[static_cast<std::size_t>(view.to_index(x, center_y))] = true;
      }
    }
  }

  for (int y = 0; y < height; ++y) {
    int x = 0;
    while (x < width) {
      while (x < width && !raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++x;
      }
      const int run_start = x;
      while (x < width && raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++x;
      }
      const int run_end = x - 1;
      if (run_start > run_end) {
        continue;
      }
      ++detection.raw_horizontal_runs;
      const int run_cells = run_end - run_start + 1;
      const double run_width = static_cast<double>(run_cells) * resolution;
      const int center_x = (run_start + run_end) / 2;
      for (int cell_x = run_start; cell_x <= run_end; ++cell_x) {
        horizontal_widths[static_cast<std::size_t>(view.to_index(cell_x, y))] = run_width;
      }
      if (run_width >= params_.corridor_min_width_m &&
        run_width <= params_.corridor_max_width_m)
      {
        vertical_centers[static_cast<std::size_t>(view.to_index(center_x, y))] = true;
      }
    }
  }

  auto make_line =
    [this, &view, &traversable, resolution](
    const std::vector<int> & cells,
    const std::vector<double> & width_samples,
    const bool horizontal) -> SkeletonLine
    {
      SkeletonLine line;
      if (cells.size() < 2U || cells.size() != width_samples.size()) {
        return line;
      }
      line.cells = cells;
      line.width_samples_m = width_samples;
      line.horizontal = horizontal;
      line.start = view.to_grid(cells.front());
      line.end = view.to_grid(cells.back());
      line.length_m = static_cast<double>(cells.size()) * resolution;
      const double width_sum = std::accumulate(width_samples.begin(), width_samples.end(), 0.0);
      line.width_m = width_sum / static_cast<double>(width_samples.size());
      double variance = 0.0;
      for (const double sample : width_samples) {
        const double delta = sample - line.width_m;
        variance += delta * delta;
      }
      line.width_stddev_m = std::sqrt(variance / static_cast<double>(width_samples.size()));
      line.traversable_ratio = traversable_ratio_for_cells(cells, traversable);
      line.confidence = corridor_confidence(line);
      return line;
    };

  auto accept_line = [this](const SkeletonLine & line) {
      return line.length_m >= params_.corridor_min_length_m &&
             line.width_m >= params_.corridor_min_width_m &&
             line.width_m <= params_.corridor_max_width_m &&
             (line.length_m / std::max(line.width_m, k_epsilon)) >=
             params_.corridor_min_aspect_ratio &&
             line.width_stddev_m <= params_.corridor_skeleton_min_width_variance_m &&
             line.traversable_ratio >= params_.corridor_min_traversable_ratio_on_centerline &&
             line.confidence >= params_.corridor_min_confidence;
    };

  for (int y = 0; y < height; ++y) {
    int x = 0;
    while (x < width) {
      while (x < width && !horizontal_centers[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++x;
      }
      std::vector<int> cells;
      std::vector<double> width_samples;
      while (x < width && horizontal_centers[static_cast<std::size_t>(view.to_index(x, y))]) {
        const int index = view.to_index(x, y);
        cells.push_back(index);
        width_samples.push_back(vertical_widths[static_cast<std::size_t>(index)]);
        ++x;
      }
      const SkeletonLine line = make_line(cells, width_samples, true);
      if (!line.cells.empty()) {
        ++detection.corridor_candidates;
      }
      if (accept_line(line)) {
        detection.lines.push_back(line);
      }
    }
  }

  for (int x = 0; x < width; ++x) {
    int y = 0;
    while (y < height) {
      while (y < height && !vertical_centers[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++y;
      }
      std::vector<int> cells;
      std::vector<double> width_samples;
      while (y < height && vertical_centers[static_cast<std::size_t>(view.to_index(x, y))]) {
        const int index = view.to_index(x, y);
        cells.push_back(index);
        width_samples.push_back(horizontal_widths[static_cast<std::size_t>(index)]);
        ++y;
      }
      const SkeletonLine line = make_line(cells, width_samples, false);
      if (!line.cells.empty()) {
        ++detection.corridor_candidates;
      }
      if (accept_line(line)) {
        detection.lines.push_back(line);
      }
    }
  }

  std::sort(
    detection.lines.begin(), detection.lines.end(),
    [](const SkeletonLine & lhs, const SkeletonLine & rhs) {
      return lhs.length_m > rhs.length_m;
    });
  return detection;
}

/// @brief Detect corridor centerline candidates from raw free-space distance ridges.
std::vector<SegmentClassifier::SkeletonLine> SegmentClassifier::detect_skeleton_corridors(
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free,
  const std::vector<bool> & traversable,
  const DistanceMap & raw_obstacle_distance) const
{
  std::vector<SkeletonLine> lines;
  if (!view.valid() || !raw_obstacle_distance.valid()) {
    return lines;
  }

  auto distance_at = [&raw_obstacle_distance, &view](const int x, const int y) {
      if (!view.in_bounds(x, y)) {
        return -std::numeric_limits<double>::infinity();
      }
      return raw_obstacle_distance.at(view.to_index(x, y));
    };

  std::vector<bool> skeleton(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    if (!raw_free[static_cast<std::size_t>(index)]) {
      continue;
    }
    const double distance = raw_obstacle_distance.at(index);
    const double width = distance * 2.0;
    if (!std::isfinite(distance) ||
      width < params_.corridor_min_width_m ||
      width > params_.corridor_max_width_m)
    {
      continue;
    }

    const GridIndex grid = view.to_grid(index);
    const double left = distance_at(grid.x - 1, grid.y);
    const double right = distance_at(grid.x + 1, grid.y);
    const double down = distance_at(grid.x, grid.y - 1);
    const double up = distance_at(grid.x, grid.y + 1);
    const double down_left = distance_at(grid.x - 1, grid.y - 1);
    const double up_right = distance_at(grid.x + 1, grid.y + 1);
    const double up_left = distance_at(grid.x - 1, grid.y + 1);
    const double down_right = distance_at(grid.x + 1, grid.y - 1);

    const bool local_max_x = distance >= left && distance >= right &&
      (distance > left || distance > right);
    const bool local_max_y = distance >= down && distance >= up &&
      (distance > down || distance > up);
    const bool local_max_diag_a = distance >= down_left && distance >= up_right &&
      (distance > down_left || distance > up_right);
    const bool local_max_diag_b = distance >= up_left && distance >= down_right &&
      (distance > up_left || distance > down_right);
    skeleton[static_cast<std::size_t>(index)] =
      local_max_x || local_max_y || local_max_diag_a || local_max_diag_b;
  }

  const int min_skeleton_cells = std::max(
    2,
    static_cast<int>(std::floor(
      params_.corridor_min_length_m / std::max(view.resolution(), k_epsilon))));
  const auto components =
    ConnectedComponents::extract(skeleton, view.width(), view.height(), min_skeleton_cells);
  for (const auto & component : components) {
    for (const auto & line : split_skeleton_component(
        component, view, traversable, raw_obstacle_distance))
    {
      if (line.length_m < params_.corridor_min_length_m ||
        line.width_m < params_.corridor_min_width_m ||
        line.width_m > params_.corridor_max_width_m ||
        line.width_stddev_m > params_.corridor_skeleton_min_width_variance_m ||
        line.traversable_ratio < params_.corridor_min_traversable_ratio_on_centerline ||
        line.confidence < params_.corridor_min_confidence)
      {
        continue;
      }
      lines.push_back(line);
    }
  }

  return lines;
}

/// @brief Split a skeleton component into locally straight corridor line candidates.
std::vector<SegmentClassifier::SkeletonLine> SegmentClassifier::split_skeleton_component(
  const Component & component,
  const OccupancyGridView & view,
  const std::vector<bool> & traversable,
  const DistanceMap & raw_obstacle_distance) const
{
  std::vector<SkeletonLine> lines;
  if (component.cells.size() < 2U) {
    return lines;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const int cell : component.cells) {
    const auto point = view.cell_center(cell);
    sum_x += point.x;
    sum_y += point.y;
  }
  const double mean_x = sum_x / static_cast<double>(component.cells.size());
  const double mean_y = sum_y / static_cast<double>(component.cells.size());
  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  for (const int cell : component.cells) {
    const auto point = view.cell_center(cell);
    const double dx = point.x - mean_x;
    const double dy = point.y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  const double axis_angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
  const double axis_x = std::cos(axis_angle);
  const double axis_y = std::sin(axis_angle);

  std::vector<int> ordered_cells = component.cells;
  std::sort(
    ordered_cells.begin(),
    ordered_cells.end(),
    [&view, mean_x, mean_y, axis_x, axis_y](const int lhs, const int rhs) {
      const auto left = view.cell_center(lhs);
      const auto right = view.cell_center(rhs);
      const double left_projection =
      ((left.x - mean_x) * axis_x) + ((left.y - mean_y) * axis_y);
      const double right_projection =
      ((right.x - mean_x) * axis_x) + ((right.y - mean_y) * axis_y);
      return left_projection < right_projection;
    });

  std::vector<std::pair<int, int>> runs;
  int run_start = 0;
  for (int i = 1; i < static_cast<int>(ordered_cells.size()); ++i) {
    const auto previous = view.cell_center(ordered_cells[static_cast<std::size_t>(i - 1)]);
    const auto current = view.cell_center(ordered_cells[static_cast<std::size_t>(i)]);
    const double gap = std::hypot(current.x - previous.x, current.y - previous.y);
    if (gap > params_.corridor_max_centerline_gap_m) {
      runs.push_back({run_start, i - 1});
      run_start = i;
    }
  }
  runs.push_back({run_start, static_cast<int>(ordered_cells.size()) - 1});

  const double max_deviation =
    std::max(view.resolution() * 2.0, params_.corridor_max_centerline_gap_m);
  std::function<void(int, int)> split_run = [&](const int start, const int end) {
      if ((end - start) < 1) {
        return;
      }
      const auto start_point = view.cell_center(ordered_cells[static_cast<std::size_t>(start)]);
      const auto end_point = view.cell_center(ordered_cells[static_cast<std::size_t>(end)]);
      const double dx = end_point.x - start_point.x;
      const double dy = end_point.y - start_point.y;
      const double length = std::hypot(dx, dy);
      if (length < view.resolution()) {
        return;
      }

      double max_lateral_error = 0.0;
      int split_index = -1;
      for (int i = start + 1; i < end; ++i) {
        const auto point = view.cell_center(ordered_cells[static_cast<std::size_t>(i)]);
        const double error = std::abs(
          (dy * point.x) - (dx * point.y) +
          (end_point.x * start_point.y) - (end_point.y * start_point.x)) / length;
        if (error > max_lateral_error) {
          max_lateral_error = error;
          split_index = i;
        }
      }

      if (max_lateral_error > max_deviation && split_index > start && split_index < end) {
        split_run(start, split_index);
        split_run(split_index, end);
        return;
      }

      SkeletonLine line = build_skeleton_line(
        ordered_cells, start, end, view, traversable, raw_obstacle_distance);
      line.confidence = corridor_confidence(line);
      lines.push_back(line);
    };

  for (const auto &[start, end] : runs) {
    split_run(start, end);
  }

  return lines;
}

/// @brief Build one line candidate from ordered skeleton cells.
SegmentClassifier::SkeletonLine SegmentClassifier::build_skeleton_line(
  const std::vector<int> & ordered_cells,
  const int start_index,
  const int end_index,
  const OccupancyGridView & view,
  const std::vector<bool> & traversable,
  const DistanceMap & raw_obstacle_distance) const
{
  SkeletonLine line;
  if (start_index < 0 || end_index >= static_cast<int>(ordered_cells.size()) ||
    start_index >= end_index)
  {
    return line;
  }

  line.start = view.to_grid(ordered_cells[static_cast<std::size_t>(start_index)]);
  line.end = view.to_grid(ordered_cells[static_cast<std::size_t>(end_index)]);
  const auto start_point = view.cell_center(ordered_cells[static_cast<std::size_t>(start_index)]);
  const auto end_point = view.cell_center(ordered_cells[static_cast<std::size_t>(end_index)]);
  line.length_m = std::hypot(end_point.x - start_point.x, end_point.y - start_point.y);
  line.horizontal = std::abs(line.end.x - line.start.x) >= std::abs(line.end.y - line.start.y);

  std::vector<double> widths;
  widths.reserve(static_cast<std::size_t>(end_index - start_index + 1));
  for (int i = start_index; i <= end_index; ++i) {
    const int cell = ordered_cells[static_cast<std::size_t>(i)];
    line.cells.push_back(cell);
    const double distance = raw_obstacle_distance.at(cell);
    if (std::isfinite(distance)) {
      const double width = distance * 2.0;
      widths.push_back(width);
      line.width_samples_m.push_back(width);
    }
  }
  if (!widths.empty()) {
    const double sum = std::accumulate(widths.begin(), widths.end(), 0.0);
    line.width_m = sum / static_cast<double>(widths.size());
    double variance = 0.0;
    for (const double width : widths) {
      const double delta = width - line.width_m;
      variance += delta * delta;
    }
    line.width_stddev_m = std::sqrt(variance / static_cast<double>(widths.size()));
  }
  line.traversable_ratio = traversable_ratio_on_line(line.start, line.end, view, traversable);
  return line;
}

/// @brief Return sampled traversable ratio along one grid line.
double SegmentClassifier::traversable_ratio_on_line(
  const GridIndex & start,
  const GridIndex & end,
  const OccupancyGridView & view,
  const std::vector<bool> & traversable) const
{
  const double dx = static_cast<double>(end.x - start.x);
  const double dy = static_cast<double>(end.y - start.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy))));
  int valid_samples = 0;
  int traversable_samples = 0;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const int x = static_cast<int>(std::round(static_cast<double>(start.x) + (dx * t)));
    const int y = static_cast<int>(std::round(static_cast<double>(start.y) + (dy * t)));
    if (!view.in_bounds(x, y)) {
      continue;
    }
    ++valid_samples;
    const int index = view.to_index(x, y);
    if (traversable[static_cast<std::size_t>(index)]) {
      ++traversable_samples;
    }
  }
  if (valid_samples == 0) {
    return 0.0;
  }
  return static_cast<double>(traversable_samples) / static_cast<double>(valid_samples);
}

/// @brief Return traversable ratio over explicit centerline cells.
double SegmentClassifier::traversable_ratio_for_cells(
  const std::vector<int> & cells,
  const std::vector<bool> & traversable) const
{
  if (cells.empty()) {
    return 0.0;
  }
  int valid_samples = 0;
  int traversable_samples = 0;
  for (const int cell : cells) {
    if (cell < 0 || cell >= static_cast<int>(traversable.size())) {
      continue;
    }
    ++valid_samples;
    if (traversable[static_cast<std::size_t>(cell)]) {
      ++traversable_samples;
    }
  }
  if (valid_samples == 0) {
    return 0.0;
  }
  return static_cast<double>(traversable_samples) / static_cast<double>(valid_samples);
}

/// @brief Detect door/portal candidates as short sharp bottlenecks on axis-aligned corridors.
std::vector<Segment> SegmentClassifier::detect_door_candidates(
  std::uint32_t & next_id,
  const std::vector<SkeletonLine> & corridor_lines,
  const OccupancyGridView & view) const
{
  std::vector<Segment> doors;
  if (!params_.door_enabled || corridor_lines.empty() || !view.valid()) {
    return doors;
  }

  std::vector<bool> used(static_cast<std::size_t>(view.cell_count()), false);
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int lookahead_cells = std::max(
    2,
    static_cast<int>(std::ceil(params_.door_max_width_m / resolution)));
  const double required_drop = std::max(0.10, resolution * 2.0);

  for (const auto & line : corridor_lines) {
    if (line.cells.size() != line.width_samples_m.size() ||
      line.cells.size() < static_cast<std::size_t>((lookahead_cells * 2) + 1))
    {
      continue;
    }

    std::vector<int> cluster_cells;
    std::vector<double> cluster_widths;
    std::vector<double> cluster_confidences;
    auto flush_cluster = [&]() {
        if (cluster_cells.empty()) {
          return;
        }
        const double cluster_length =
          static_cast<double>(cluster_cells.size()) * resolution;
        const double width_sum =
          std::accumulate(cluster_widths.begin(), cluster_widths.end(), 0.0);
        const double mean_width = width_sum / static_cast<double>(cluster_widths.size());
        const double confidence =
          *std::max_element(cluster_confidences.begin(), cluster_confidences.end());
        if (cluster_length <= params_.door_max_width_m &&
          mean_width >= params_.door_min_width_m &&
          mean_width <= params_.door_max_width_m &&
          confidence >= params_.door_min_confidence)
        {
          for (const int cell : cluster_cells) {
            if (cell >= 0 && cell < static_cast<int>(used.size())) {
              used[static_cast<std::size_t>(cell)] = true;
            }
          }
          doors.push_back(
            build_door_segment(
              next_id++, cluster_cells, mean_width, confidence,
              view));
        }
        cluster_cells.clear();
        cluster_widths.clear();
        cluster_confidences.clear();
      };

    for (int i = 0; i < static_cast<int>(line.cells.size()); ++i) {
      const int before = i - lookahead_cells;
      const int after = i + lookahead_cells;
      if (before < 0 || after >= static_cast<int>(line.width_samples_m.size())) {
        flush_cluster();
        continue;
      }

      const double current_width = line.width_samples_m[static_cast<std::size_t>(i)];
      const double before_width = line.width_samples_m[static_cast<std::size_t>(before)];
      const double after_width = line.width_samples_m[static_cast<std::size_t>(after)];
      const double neighboring_width = std::min(before_width, after_width);
      const bool is_bottleneck =
        current_width >= params_.door_min_width_m &&
        current_width <= params_.door_max_width_m &&
        neighboring_width >= current_width + required_drop;
      const int cell = line.cells[static_cast<std::size_t>(i)];
      if (is_bottleneck &&
        cell >= 0 &&
        cell < static_cast<int>(used.size()) &&
        !used[static_cast<std::size_t>(cell)])
      {
        const double confidence = clamp01(
          (neighboring_width - current_width) /
          std::max(params_.door_max_width_m - params_.door_min_width_m, k_epsilon));
        cluster_cells.push_back(cell);
        cluster_widths.push_back(current_width);
        cluster_confidences.push_back(confidence);
      } else {
        flush_cluster();
      }
    }
    flush_cluster();
  }

  return doors;
}

/// @brief Extract horizontal/vertical free-run bands from the raw map.
SegmentClassifier::ScanlineBandDetection SegmentClassifier::detect_scanline_bands(
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free) const
{
  ScanlineBandDetection detection;
  if (!view.valid() || raw_free.size() != static_cast<std::size_t>(view.cell_count())) {
    return detection;
  }

  const int width = view.width();
  const int height = view.height();
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int max_gap = std::max(0, params_.scanline_max_gap_cells);
  const int min_run_cells = std::max(
    1, static_cast<int>(std::ceil(params_.scanline_min_free_run_length_m / resolution)));
  const int endpoint_tolerance_cells = std::max(
    1, static_cast<int>(std::ceil(params_.scanline_run_endpoint_tolerance_m / resolution)));

  auto is_boundary = [&view, &raw_free](const int x, const int y) {
      if (!view.in_bounds(x, y)) {
        return true;
      }
      const int index = view.to_index(x, y);
      return !raw_free[static_cast<std::size_t>(index)] &&
             (view.is_raw_occupied(index) || view.is_unknown(index) || view.is_occupied(index));
    };
  auto horizontal_boundary_evidence = [&is_boundary](
    const int y,
    const int start,
    const int end)
    {
      int evidence = 0;
      const int length = std::max(1, end - start + 1);
      for (int x = start; x <= end; ++x) {
        if (is_boundary(x, y - 1)) {
          ++evidence;
        }
        if (is_boundary(x, y + 1)) {
          ++evidence;
        }
      }
      return static_cast<double>(evidence) / static_cast<double>(length * 2);
    };
  auto vertical_boundary_evidence = [&is_boundary](
    const int x,
    const int start,
    const int end)
    {
      int evidence = 0;
      const int length = std::max(1, end - start + 1);
      for (int y = start; y <= end; ++y) {
        if (is_boundary(x - 1, y)) {
          ++evidence;
        }
        if (is_boundary(x + 1, y)) {
          ++evidence;
        }
      }
      return static_cast<double>(evidence) / static_cast<double>(length * 2);
    };

  for (int y = 0; y < height; ++y) {
    int x = 0;
    while (x < width) {
      while (x < width && !raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++x;
      }
      if (x >= width) {
        break;
      }
      const int start = x;
      int last_free = x;
      int free_cells = 0;
      int gap_cells = 0;
      while (x < width) {
        const int index = view.to_index(x, y);
        if (raw_free[static_cast<std::size_t>(index)]) {
          ++free_cells;
          gap_cells = 0;
          last_free = x;
        } else {
          ++gap_cells;
          if (gap_cells > max_gap) {
            break;
          }
        }
        ++x;
      }
      const int end = last_free;
      const int length_cells = end - start + 1;
      if (length_cells >= min_run_cells) {
        ScanlineRun run;
        run.horizontal = true;
        run.line = y;
        run.start = start;
        run.end = end;
        run.free_ratio = static_cast<double>(free_cells) / static_cast<double>(length_cells);
        run.boundary_evidence = horizontal_boundary_evidence(y, start, end);
        detection.horizontal_runs.push_back(run);
      }
    }
  }

  for (int x = 0; x < width; ++x) {
    int y = 0;
    while (y < height) {
      while (y < height && !raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
        ++y;
      }
      if (y >= height) {
        break;
      }
      const int start = y;
      int last_free = y;
      int free_cells = 0;
      int gap_cells = 0;
      while (y < height) {
        const int index = view.to_index(x, y);
        if (raw_free[static_cast<std::size_t>(index)]) {
          ++free_cells;
          gap_cells = 0;
          last_free = y;
        } else {
          ++gap_cells;
          if (gap_cells > max_gap) {
            break;
          }
        }
        ++y;
      }
      const int end = last_free;
      const int length_cells = end - start + 1;
      if (length_cells >= min_run_cells) {
        ScanlineRun run;
        run.horizontal = false;
        run.line = x;
        run.start = start;
        run.end = end;
        run.free_ratio = static_cast<double>(free_cells) / static_cast<double>(length_cells);
        run.boundary_evidence = vertical_boundary_evidence(x, start, end);
        detection.vertical_runs.push_back(run);
      }
    }
  }

  auto overlap_ratio = [](const ScanlineRun & lhs, const ScanlineRun & rhs) {
      const int overlap = std::max(
        0, std::min(lhs.end, rhs.end) - std::max(
          lhs.start,
          rhs.start) + 1);
      const int min_length =
        std::max(1, std::min(lhs.end - lhs.start + 1, rhs.end - rhs.start + 1));
      return static_cast<double>(overlap) / static_cast<double>(min_length);
    };
  auto percentile = [](std::vector<int> values, const double quantile) {
      if (values.empty()) {
        return 0;
      }
      std::sort(values.begin(), values.end());
      const auto last = static_cast<double>(values.size() - 1U);
      const auto index =
        static_cast<std::size_t>(std::round(std::max(0.0, std::min(1.0, quantile)) * last));
      return values[index];
    };
  auto stability_score =
    [resolution](const std::vector<int> & starts, const std::vector<int> & ends,
      const double tolerance_m) {
      auto stddev_cells = [](const std::vector<int> & values) {
          if (values.size() < 2U) {
            return 0.0;
          }
          const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
            static_cast<double>(values.size());
          double variance = 0.0;
          for (const int value : values) {
            const double delta = static_cast<double>(value) - mean;
            variance += delta * delta;
          }
          return std::sqrt(variance / static_cast<double>(values.size()));
        };
      const double stddev_m = std::max(stddev_cells(starts), stddev_cells(ends)) * resolution;
      return SegmentClassifier::clamp01(1.0 - (stddev_m / std::max(tolerance_m, resolution)));
    };
  auto fill_ratio = [&view, &raw_free](
    const int min_x,
    const int min_y,
    const int max_x,
    const int max_y)
    {
      int free_cells = 0;
      int total_cells = 0;
      for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
          if (!view.in_bounds(x, y)) {
            continue;
          }
          ++total_cells;
          if (raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
            ++free_cells;
          }
        }
      }
      return total_cells > 0 ?
             static_cast<double>(free_cells) / static_cast<double>(total_cells) : 0.0;
    };
  auto bbox_boundary_evidence = [&is_boundary](
    const bool horizontal,
    const int min_x,
    const int min_y,
    const int max_x,
    const int max_y)
    {
      int evidence = 0;
      int samples = 0;
      if (horizontal) {
        for (int x = min_x; x <= max_x; ++x) {
          ++samples;
          if (is_boundary(x, min_y - 1)) {
            ++evidence;
          }
          ++samples;
          if (is_boundary(x, max_y + 1)) {
            ++evidence;
          }
        }
      } else {
        for (int y = min_y; y <= max_y; ++y) {
          ++samples;
          if (is_boundary(min_x - 1, y)) {
            ++evidence;
          }
          ++samples;
          if (is_boundary(max_x + 1, y)) {
            ++evidence;
          }
        }
      }
      return samples > 0 ? static_cast<double>(evidence) / static_cast<double>(samples) : 0.0;
    };
  auto bounds_area_m2 = [resolution](const GridBounds & bounds) {
      if (!bounds.valid) {
        return 0.0;
      }
      const int width_cells = std::max(0, bounds.max_x - bounds.min_x + 1);
      const int height_cells = std::max(0, bounds.max_y - bounds.min_y + 1);
      return static_cast<double>(width_cells * height_cells) * resolution * resolution;
    };

  std::vector<bool> known_mask(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    known_mask[static_cast<std::size_t>(index)] = !view.is_unknown(index);
  }
  const GridBounds known_extent = mask_bounds(view, known_mask);
  const GridBounds free_extent = mask_bounds(view, raw_free);
  const double known_area_m2 = bounds_area_m2(known_extent);
  const double free_extent_area_m2 = bounds_area_m2(free_extent);

  std::uint32_t rejection_id = 1U;
  auto classify_band = [
    this, &detection, &rejection_id, &fill_ratio, &bbox_boundary_evidence, &known_extent,
    &free_extent, known_area_m2, free_extent_area_m2, resolution](ScanlineBand & band) {
      const double bbox_width_m = static_cast<double>(band.max_x - band.min_x + 1) * resolution;
      const double bbox_height_m = static_cast<double>(band.max_y - band.min_y + 1) * resolution;
      band.length_m = band.horizontal ? bbox_width_m : bbox_height_m;
      band.width_m = band.horizontal ? bbox_height_m : bbox_width_m;
      band.area_m2 = bbox_width_m * bbox_height_m;
      band.aspect_ratio = band.length_m / std::max(band.width_m, resolution);
      band.free_fill_ratio = fill_ratio(band.min_x, band.min_y, band.max_x, band.max_y);
      band.boundary_evidence = bbox_boundary_evidence(
        band.horizontal, band.min_x, band.min_y, band.max_x, band.max_y);
      band.confidence = clamp01(
        (0.45 * band.free_fill_ratio) +
        (0.35 * band.boundary_evidence) +
        (0.20 * band.endpoint_stability));

      const bool has_min_band_width = band.horizontal ?
        band.width_m >= params_.scanline_min_band_height_m :
        band.width_m >= params_.scanline_min_band_width_m;
      const bool corridor_like =
        has_min_band_width &&
        band.length_m >= params_.corridor_min_length_m &&
        band.width_m >= params_.corridor_min_width_m &&
        band.width_m <= params_.corridor_max_width_m &&
        band.aspect_ratio >= params_.corridor_min_aspect_ratio &&
        band.free_fill_ratio >= params_.corridor_min_free_ratio &&
        band.boundary_evidence >= params_.corridor_min_boundary_evidence_ratio &&
        band.confidence >= params_.corridor_min_confidence;
      if (corridor_like) {
        band.type = SegmentType::Corridor;
        return true;
      }

      AreaCandidate candidate;
      candidate.min_x = band.min_x;
      candidate.min_y = band.min_y;
      candidate.max_x = band.max_x;
      candidate.max_y = band.max_y;
      candidate.diagnostic_id = rejection_id++;
      candidate.bbox_area_m2 = band.area_m2;
      candidate.fill_ratio = band.free_fill_ratio;
      candidate.free_ratio = band.free_fill_ratio;
      candidate.confidence = band.confidence;
      candidate.known_extent_ratio = known_area_m2 > k_epsilon ? band.area_m2 / known_area_m2 : 0.0;
      candidate.free_extent_ratio = free_extent_area_m2 >
        k_epsilon ? band.area_m2 / free_extent_area_m2 : 0.0;
      std::string rejection_reason;
      if (reject_area_candidate(candidate, known_extent, free_extent, rejection_reason)) {
        ++detection.rejected_bands;
        log_area_rejected(candidate, rejection_reason);
        return false;
      }

      const double area_aspect_ratio =
        std::max(bbox_width_m, bbox_height_m) / std::max(
        std::min(
          bbox_width_m,
          bbox_height_m), resolution);
      const bool area_like =
        band.area_m2 >= params_.area_min_area_m2 &&
        bbox_width_m >= params_.area_min_side_m &&
        bbox_height_m >= params_.area_min_side_m &&
        area_aspect_ratio <= params_.area_max_aspect_ratio &&
        band.free_fill_ratio >= params_.area_min_free_ratio &&
        band.confidence >= params_.area_min_confidence;
      if (!area_like) {
        return false;
      }
      band.type = SegmentType::OpenArea;
      return true;
    };

  auto merge_runs = [this, endpoint_tolerance_cells, max_gap, &overlap_ratio, &percentile,
      &stability_score, &classify_band](
    const std::vector<ScanlineRun> & runs,
    const bool horizontal)
    {
      std::vector<ScanlineBand> active_bands;
      for (const auto & run : runs) {
        int best_index = -1;
        double best_overlap = 0.0;
        for (int index = 0; index < static_cast<int>(active_bands.size()); ++index) {
          auto & band = active_bands[static_cast<std::size_t>(index)];
          if (band.runs.empty()) {
            continue;
          }
          const auto & previous = band.runs.back();
          if (run.line - previous.line > max_gap + 1) {
            continue;
          }
          const double overlap = overlap_ratio(run, previous);
          const bool endpoints_stable =
            std::abs(run.start - previous.start) <= endpoint_tolerance_cells &&
            std::abs(run.end - previous.end) <= endpoint_tolerance_cells;
          if (overlap >= params_.scanline_run_overlap_ratio_threshold && endpoints_stable &&
            overlap > best_overlap)
          {
            best_index = index;
            best_overlap = overlap;
          }
        }
        if (best_index >= 0) {
          active_bands[static_cast<std::size_t>(best_index)].runs.push_back(run);
        } else {
          ScanlineBand band;
          band.horizontal = horizontal;
          band.runs.push_back(run);
          active_bands.push_back(band);
        }
      }

      std::vector<ScanlineBand> merged;
      for (auto & band : active_bands) {
        const int min_required = horizontal ?
          std::max(1, params_.scanline_min_rows_per_horizontal_band) :
          std::max(1, params_.scanline_min_cols_per_vertical_band);
        if (static_cast<int>(band.runs.size()) < min_required) {
          continue;
        }
        std::vector<int> starts;
        std::vector<int> ends;
        std::vector<int> lines;
        starts.reserve(band.runs.size());
        ends.reserve(band.runs.size());
        lines.reserve(band.runs.size());
        for (const auto & run : band.runs) {
          starts.push_back(run.start);
          ends.push_back(run.end);
          lines.push_back(run.line);
        }
        const int start_low = percentile(starts, 0.10);
        const int end_high = percentile(ends, 0.90);
        const int line_min = *std::min_element(lines.begin(), lines.end());
        const int line_max = *std::max_element(lines.begin(), lines.end());
        if (horizontal) {
          band.min_x = start_low;
          band.max_x = end_high;
          band.min_y = line_min;
          band.max_y = line_max;
        } else {
          band.min_x = line_min;
          band.max_x = line_max;
          band.min_y = start_low;
          band.max_y = end_high;
        }
        if (band.min_x > band.max_x || band.min_y > band.max_y) {
          continue;
        }
        band.endpoint_stability = stability_score(
          starts, ends, params_.scanline_run_endpoint_tolerance_m);
        if (classify_band(band)) {
          merged.push_back(band);
        }
      }
      return merged;
    };

  detection.horizontal_bands = merge_runs(detection.horizontal_runs, true);
  detection.vertical_bands = merge_runs(detection.vertical_runs, false);
  return detection;
}

/// @brief Build a final semantic segment from a merged scanline band envelope.
Segment SegmentClassifier::build_band_segment(
  const std::uint32_t id,
  const ScanlineBand & band,
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free) const
{
  Segment segment;
  segment.id = id;
  segment.type = band.type;
  segment.confidence = band.confidence;
  const auto p0 = view.grid_corner(band.min_x, band.min_y);
  const auto p1 = view.grid_corner(band.max_x + 1, band.min_y);
  const auto p2 = view.grid_corner(band.max_x + 1, band.max_y + 1);
  const auto p3 = view.grid_corner(band.min_x, band.max_y + 1);
  segment.polygon = {p0, p1, p2, p3, p0};
  for (auto & point : segment.polygon) {
    point.z = params_.marker_height_z;
  }
  segment.bbox_min = make_point(
    std::min({p0.x, p1.x, p2.x, p3.x}),
    std::min({p0.y, p1.y, p2.y, p3.y}),
    params_.marker_height_z);
  segment.bbox_max = make_point(
    std::max({p0.x, p1.x, p2.x, p3.x}),
    std::max({p0.y, p1.y, p2.y, p3.y}),
    params_.marker_height_z);
  segment.centroid.position = make_point(
    (segment.bbox_min.x + segment.bbox_max.x) * 0.5,
    (segment.bbox_min.y + segment.bbox_max.y) * 0.5,
    params_.marker_height_z);
  segment.centroid.orientation.w = 1.0;
  segment.area_m2 = band.area_m2;
  segment.length_m = band.length_m;
  segment.width_m = band.width_m;
  if (segment.type == SegmentType::Corridor) {
    geometry_msgs::msg::Point a;
    geometry_msgs::msg::Point b;
    if (band.horizontal) {
      a.x = (p0.x + p3.x) * 0.5;
      a.y = (p0.y + p3.y) * 0.5;
      b.x = (p1.x + p2.x) * 0.5;
      b.y = (p1.y + p2.y) * 0.5;
    } else {
      a.x = (p0.x + p1.x) * 0.5;
      a.y = (p0.y + p1.y) * 0.5;
      b.x = (p3.x + p2.x) * 0.5;
      b.y = (p3.y + p2.y) * 0.5;
    }
    a.z = params_.marker_height_z;
    b.z = params_.marker_height_z;
    segment.centerline = {a, b};
  }
  for (int y = band.min_y; y <= band.max_y; ++y) {
    for (int x = band.min_x; x <= band.max_x; ++x) {
      if (!view.in_bounds(x, y)) {
        continue;
      }
      const int index = view.to_index(x, y);
      if (raw_free[static_cast<std::size_t>(index)]) {
        segment.cells.push_back(index);
      }
    }
  }
  return segment;
}

/// @brief Suppress overlapping final bands and optionally merge adjacent same-orientation bands.
std::vector<SegmentClassifier::ScanlineBand> SegmentClassifier::postprocess_bands(
  std::vector<ScanlineBand> bands,
  const OccupancyGridView & view) const
{
  if (bands.empty() || !params_.postprocess_enabled) {
    return bands;
  }

  const int merge_gap_cells = std::max(
    0,
    static_cast<int>(std::ceil(
      params_.postprocess_merge_gap_m /
      std::max(view.resolution(), k_epsilon))));
  auto bbox_iou = [](const ScanlineBand & lhs, const ScanlineBand & rhs) {
      const int ix0 = std::max(lhs.min_x, rhs.min_x);
      const int iy0 = std::max(lhs.min_y, rhs.min_y);
      const int ix1 = std::min(lhs.max_x, rhs.max_x);
      const int iy1 = std::min(lhs.max_y, rhs.max_y);
      const int iw = std::max(0, ix1 - ix0 + 1);
      const int ih = std::max(0, iy1 - iy0 + 1);
      const double intersection = static_cast<double>(iw * ih);
      const double lhs_area = static_cast<double>(
        std::max(0, lhs.max_x - lhs.min_x + 1) *
        std::max(0, lhs.max_y - lhs.min_y + 1));
      const double rhs_area = static_cast<double>(
        std::max(0, rhs.max_x - rhs.min_x + 1) *
        std::max(0, rhs.max_y - rhs.min_y + 1));
      const double union_area = lhs_area + rhs_area - intersection;
      return union_area > k_epsilon ? intersection / union_area : 0.0;
    };
  auto interval_overlap_ratio = [](const int a0, const int a1, const int b0, const int b1) {
      const int overlap = std::max(0, std::min(a1, b1) - std::max(a0, b0) + 1);
      const int min_length = std::max(1, std::min(a1 - a0 + 1, b1 - b0 + 1));
      return static_cast<double>(overlap) / static_cast<double>(min_length);
    };
  auto gap_between = [](const int a0, const int a1, const int b0, const int b1) {
      if (a1 < b0) {
        return b0 - a1 - 1;
      }
      if (b1 < a0) {
        return a0 - b1 - 1;
      }
      return 0;
    };

  if (params_.postprocess_merge_adjacent_bands) {
    bool merged_any = true;
    while (merged_any) {
      merged_any = false;
      for (int i = 0; i < static_cast<int>(bands.size()) && !merged_any; ++i) {
        for (int j = i + 1; j < static_cast<int>(bands.size()); ++j) {
          auto & lhs = bands[static_cast<std::size_t>(i)];
          auto & rhs = bands[static_cast<std::size_t>(j)];
          if (lhs.type != rhs.type) {
            continue;
          }
          if (lhs.type == SegmentType::OpenArea) {
            continue;
          }
          if (params_.postprocess_merge_same_orientation_only && lhs.horizontal != rhs.horizontal) {
            continue;
          }
          const bool close_x =
            gap_between(lhs.min_x, lhs.max_x, rhs.min_x, rhs.max_x) <= merge_gap_cells;
          const bool close_y =
            gap_between(lhs.min_y, lhs.max_y, rhs.min_y, rhs.max_y) <= merge_gap_cells;
          const bool similar_horizontal_band = lhs.horizontal && close_x && close_y &&
            interval_overlap_ratio(lhs.min_y, lhs.max_y, rhs.min_y, rhs.max_y) >= 0.60;
          const bool similar_vertical_band = !lhs.horizontal && close_x && close_y &&
            interval_overlap_ratio(lhs.min_x, lhs.max_x, rhs.min_x, rhs.max_x) >= 0.60;
          if (!similar_horizontal_band && !similar_vertical_band) {
            continue;
          }
          lhs.min_x = std::min(lhs.min_x, rhs.min_x);
          lhs.min_y = std::min(lhs.min_y, rhs.min_y);
          lhs.max_x = std::max(lhs.max_x, rhs.max_x);
          lhs.max_y = std::max(lhs.max_y, rhs.max_y);
          lhs.confidence = std::max(lhs.confidence, rhs.confidence);
          const double bbox_width_m = static_cast<double>(lhs.max_x - lhs.min_x + 1) *
            view.resolution();
          const double bbox_height_m = static_cast<double>(lhs.max_y - lhs.min_y + 1) *
            view.resolution();
          lhs.length_m = lhs.horizontal ? bbox_width_m : bbox_height_m;
          lhs.width_m = lhs.horizontal ? bbox_height_m : bbox_width_m;
          lhs.area_m2 = bbox_width_m * bbox_height_m;
          lhs.aspect_ratio = lhs.length_m / std::max(lhs.width_m, view.resolution());
          bands.erase(bands.begin() + j);
          merged_any = true;
          break;
        }
      }
    }
  }

  std::sort(
    bands.begin(), bands.end(), [](const ScanlineBand & lhs, const ScanlineBand & rhs) {
      if (std::abs(lhs.confidence - rhs.confidence) > 1.0e-6) {
        return lhs.confidence > rhs.confidence;
      }
      return lhs.area_m2 > rhs.area_m2;
    });

  std::vector<ScanlineBand> selected;
  for (const auto & band : bands) {
    bool suppressed = false;
    for (const auto & accepted : selected) {
      if (bbox_iou(band, accepted) > params_.postprocess_nms_iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) {
      continue;
    }
    selected.push_back(band);
    if (static_cast<int>(selected.size()) >=
      std::max(0, params_.postprocess_max_published_segments))
    {
      break;
    }
  }
  return selected;
}

/// @brief Extract area cells with a lightweight Boustrophedon row sweep.
SegmentClassifier::CellDecompositionDetection SegmentClassifier::detect_cell_decomposition(
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free) const
{
  CellDecompositionDetection detection;
  if (!params_.cell_decomposition_enabled || !view.valid() ||
    raw_free.size() != static_cast<std::size_t>(view.cell_count()))
  {
    return detection;
  }

  const int width = view.width();
  const int height = view.height();
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int max_gap = std::max(0, params_.scanline_max_gap_cells);
  const int min_interval_cells = std::max(
    1, static_cast<int>(std::ceil(
      params_.scanline_min_free_run_length_m / resolution)));
  const int endpoint_shift_cells = std::max(
    1, static_cast<int>(std::ceil(
      params_.cell_decomposition_endpoint_shift_threshold_m / resolution)));

  auto extract_row_intervals = [&](const int y) {
      std::vector<CellInterval> intervals;
      int x = 0;
      while (x < width) {
        while (x < width && !raw_free[static_cast<std::size_t>(view.to_index(x, y))]) {
          ++x;
        }
        if (x >= width) {
          break;
        }
        const int start = x;
        int last_free = x;
        int free_cells = 0;
        int gap_cells = 0;
        while (x < width) {
          const int index = view.to_index(x, y);
          if (raw_free[static_cast<std::size_t>(index)]) {
            ++free_cells;
            gap_cells = 0;
            last_free = x;
          } else {
            ++gap_cells;
            if (gap_cells > max_gap) {
              break;
            }
          }
          ++x;
        }
        const int end = last_free;
        if (end - start + 1 >= min_interval_cells && free_cells >= min_interval_cells) {
          intervals.push_back(CellInterval{y, start, end});
        }
      }
      return intervals;
    };
  auto overlap_ratio = [](const CellInterval & lhs, const CellInterval & rhs) {
      const int overlap = std::max(
        0, std::min(lhs.end, rhs.end) - std::max(lhs.start, rhs.start) + 1);
      const int min_length =
        std::max(1, std::min(lhs.end - lhs.start + 1, rhs.end - rhs.start + 1));
      return static_cast<double>(overlap) / static_cast<double>(min_length);
    };
  auto recompute_bounds = [](CellCandidate & cell) {
      if (cell.intervals.empty()) {
        return;
      }
      cell.min_x = cell.intervals.front().start;
      cell.max_x = cell.intervals.front().end;
      cell.min_y = cell.intervals.front().y;
      cell.max_y = cell.intervals.front().y;
      for (const auto & interval : cell.intervals) {
        cell.min_x = std::min(cell.min_x, interval.start);
        cell.max_x = std::max(cell.max_x, interval.end);
        cell.min_y = std::min(cell.min_y, interval.y);
        cell.max_y = std::max(cell.max_y, interval.y);
      }
    };

  std::vector<CellCandidate> active_cells;
  std::vector<CellCandidate> raw_cells;
  std::uint32_t next_cell_id = 1U;
  for (int y = 0; y < height; ++y) {
    const auto intervals = extract_row_intervals(y);
    detection.intervals += static_cast<int>(intervals.size());
    std::vector<bool> active_used(active_cells.size(), false);
    std::vector<CellCandidate> next_active;

    for (const auto & interval : intervals) {
      int best_index = -1;
      double best_overlap = 0.0;
      for (int index = 0; index < static_cast<int>(active_cells.size()); ++index) {
        const auto & cell = active_cells[static_cast<std::size_t>(index)];
        if (cell.intervals.empty()) {
          continue;
        }
        const auto & previous = cell.intervals.back();
        if (interval.y - previous.y != 1) {
          continue;
        }
        const double overlap = overlap_ratio(interval, previous);
        const bool endpoints_stable =
          std::abs(interval.start - previous.start) <= endpoint_shift_cells &&
          std::abs(interval.end - previous.end) <= endpoint_shift_cells;
        if (overlap >= params_.cell_decomposition_interval_overlap_ratio && endpoints_stable &&
          overlap > best_overlap)
        {
          best_index = index;
          best_overlap = overlap;
        }
      }

      if (best_index >= 0 && !active_used[static_cast<std::size_t>(best_index)]) {
        CellCandidate cell = active_cells[static_cast<std::size_t>(best_index)];
        cell.intervals.push_back(interval);
        recompute_bounds(cell);
        active_used[static_cast<std::size_t>(best_index)] = true;
        next_active.push_back(std::move(cell));
      } else {
        CellCandidate cell;
        cell.diagnostic_id = next_cell_id++;
        cell.intervals.push_back(interval);
        recompute_bounds(cell);
        next_active.push_back(std::move(cell));
      }
    }

    for (int index = 0; index < static_cast<int>(active_cells.size()); ++index) {
      if (!active_used[static_cast<std::size_t>(index)]) {
        raw_cells.push_back(std::move(active_cells[static_cast<std::size_t>(index)]));
      }
    }
    active_cells = std::move(next_active);
  }
  raw_cells.insert(raw_cells.end(), active_cells.begin(), active_cells.end());
  detection.raw_cells = static_cast<int>(raw_cells.size());

  std::uint32_t validation_id = 1U;
  for (auto & cell : raw_cells) {
    recompute_bounds(cell);
    const double bbox_width_m = static_cast<double>(cell.max_x - cell.min_x + 1) * resolution;
    const double bbox_height_m = static_cast<double>(cell.max_y - cell.min_y + 1) * resolution;
    const double area_m2 = bbox_width_m * bbox_height_m;
    if (bbox_width_m < params_.cell_decomposition_min_cell_width_m ||
      bbox_height_m < params_.cell_decomposition_min_cell_height_m ||
      area_m2 < params_.cell_decomposition_min_cell_area_m2 ||
      area_m2 > params_.cell_decomposition_max_cell_area_m2)
    {
      ++detection.rejected_cells;
      continue;
    }

    ScanlineBand candidate;
    candidate.horizontal = bbox_width_m >= bbox_height_m;
    candidate.min_x = cell.min_x;
    candidate.min_y = cell.min_y;
    candidate.max_x = cell.max_x;
    candidate.max_y = cell.max_y;
    candidate.endpoint_stability = 1.0;

    auto valid_cells = validate_area_bbox_recursive(
      validation_id, candidate, view, raw_free, 0, detection.split_cells, detection.rejected_cells);
    detection.cells.insert(detection.cells.end(), valid_cells.begin(), valid_cells.end());
  }

  detection.merged_cells = static_cast<int>(detection.cells.size());
  detection.final_cells = static_cast<int>(detection.cells.size());
  RCLCPP_INFO(
    rclcpp::get_logger(
      "spatial_segmenter"),
    "AMR_LOG schema=v1 component=spatial_segmenter event=cell_decomposition_stats sweep_axis=%s intervals=%d raw_cells=%d merged_cells=%d split_cells=%d final_cells=%d",
    params_.cell_decomposition_sweep_axis.c_str(),
    detection.intervals,
    detection.raw_cells,
    detection.merged_cells,
    detection.split_cells,
    detection.final_cells);
  return detection;
}

/// @brief Validate an area bbox, recursively splitting along internal barriers when possible.
std::vector<SegmentClassifier::ScanlineBand> SegmentClassifier::validate_area_bbox_recursive(
  std::uint32_t & next_id,
  const ScanlineBand & candidate,
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free,
  const int depth,
  int & split_count,
  int & rejected_count) const
{
  const std::uint32_t id = next_id++;
  const AreaValidation validation = validate_area_candidate(id, candidate, view, raw_free);
  log_area_validation(id, candidate, validation);
  if (validation.accepted) {
    ScanlineBand accepted = candidate;
    accepted.type = SegmentType::OpenArea;
    accepted.area_m2 = validation.area_m2;
    accepted.free_fill_ratio = validation.free_ratio;
    accepted.occupied_ratio = validation.occupied_ratio;
    accepted.unknown_ratio = validation.unknown_ratio;
    accepted.boundary_ratio = validation.boundary_ratio;
    accepted.boundary_evidence = validation.boundary_evidence;
    accepted.confidence = validation.confidence;
    accepted.internal_barrier_count = validation.internal_barriers;
    const double resolution = std::max(view.resolution(), k_epsilon);
    const double bbox_width_m = static_cast<double>(accepted.max_x - accepted.min_x + 1) *
      resolution;
    const double bbox_height_m = static_cast<double>(accepted.max_y - accepted.min_y + 1) *
      resolution;
    accepted.length_m = std::max(bbox_width_m, bbox_height_m);
    accepted.width_m = std::min(bbox_width_m, bbox_height_m);
    accepted.aspect_ratio = accepted.length_m / std::max(accepted.width_m, resolution);
    return {accepted};
  }

  if (validation.reason == "internal_barrier" && params_.area_split_on_internal_barrier &&
    depth < params_.area_max_split_depth)
  {
    const double resolution = std::max(view.resolution(), k_epsilon);
    std::vector<ScanlineBand> children;
    if (validation.barrier.vertical) {
      ScanlineBand left = candidate;
      ScanlineBand right = candidate;
      left.max_x = validation.barrier.line - 1;
      right.min_x = validation.barrier.line + 1;
      children = {left, right};
    } else {
      ScanlineBand bottom = candidate;
      ScanlineBand top = candidate;
      bottom.max_y = validation.barrier.line - 1;
      top.min_y = validation.barrier.line + 1;
      children = {bottom, top};
    }

    std::vector<ScanlineBand> accepted_children;
    for (const auto & child : children) {
      if (child.min_x > child.max_x || child.min_y > child.max_y) {
        continue;
      }
      const double child_area_m2 = static_cast<double>(
        (child.max_x - child.min_x + 1) * (child.max_y - child.min_y + 1)) * resolution *
        resolution;
      if (child_area_m2 < params_.area_min_child_area_m2) {
        continue;
      }
      auto valid_child = validate_area_bbox_recursive(
        next_id, child, view, raw_free, depth + 1, split_count, rejected_count);
      accepted_children.insert(accepted_children.end(), valid_child.begin(), valid_child.end());
    }
    if (!accepted_children.empty()) {
      ++split_count;
      return accepted_children;
    }
  }

  ++rejected_count;
  return {};
}

/// @brief Validate fill, ambiguity, and internal barriers inside one area bbox.
SegmentClassifier::AreaValidation SegmentClassifier::validate_area_candidate(
  const std::uint32_t id,
  const ScanlineBand & candidate,
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free) const
{
  (void)id;
  AreaValidation validation;
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int min_x = std::max(0, candidate.min_x);
  const int min_y = std::max(0, candidate.min_y);
  const int max_x = std::min(view.width() - 1, candidate.max_x);
  const int max_y = std::min(view.height() - 1, candidate.max_y);
  if (min_x > max_x || min_y > max_y) {
    validation.reason = "invalid_bbox";
    return validation;
  }

  int free_cells = 0;
  int occupied_cells = 0;
  int unknown_cells = 0;
  int boundary_cells = 0;
  int total_cells = 0;
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      ++total_cells;
      const int index = view.to_index(x, y);
      if (raw_free[static_cast<std::size_t>(index)]) {
        ++free_cells;
      }
      if (view.is_raw_occupied(index)) {
        ++occupied_cells;
      }
      if (view.is_unknown(index)) {
        ++unknown_cells;
      }
      if (!raw_free[static_cast<std::size_t>(index)] &&
        (view.is_raw_occupied(index) || view.is_unknown(index) || view.is_occupied(index)))
      {
        ++boundary_cells;
      }
    }
  }
  const double inv_total = 1.0 / static_cast<double>(std::max(1, total_cells));
  validation.free_ratio = static_cast<double>(free_cells) * inv_total;
  validation.occupied_ratio = static_cast<double>(occupied_cells) * inv_total;
  validation.unknown_ratio = static_cast<double>(unknown_cells) * inv_total;
  validation.boundary_ratio = static_cast<double>(boundary_cells) * inv_total;

  auto is_boundary = [&view, &raw_free](const int x, const int y) {
      if (!view.in_bounds(x, y)) {
        return true;
      }
      const int index = view.to_index(x, y);
      return !raw_free[static_cast<std::size_t>(index)] &&
             (view.is_raw_occupied(index) || view.is_unknown(index) || view.is_occupied(index));
    };
  int side_hits = 0;
  int sides = 4;
  int top_hits = 0;
  int bottom_hits = 0;
  int left_hits = 0;
  int right_hits = 0;
  for (int x = min_x; x <= max_x; ++x) {
    if (is_boundary(x, min_y - 1)) {
      ++bottom_hits;
    }
    if (is_boundary(x, max_y + 1)) {
      ++top_hits;
    }
  }
  for (int y = min_y; y <= max_y; ++y) {
    if (is_boundary(min_x - 1, y)) {
      ++left_hits;
    }
    if (is_boundary(max_x + 1, y)) {
      ++right_hits;
    }
  }
  const int width_cells = std::max(1, max_x - min_x + 1);
  const int height_cells = std::max(1, max_y - min_y + 1);
  side_hits += static_cast<double>(bottom_hits) / static_cast<double>(width_cells) >= 0.35 ? 1 : 0;
  side_hits += static_cast<double>(top_hits) / static_cast<double>(width_cells) >= 0.35 ? 1 : 0;
  side_hits += static_cast<double>(left_hits) / static_cast<double>(height_cells) >= 0.35 ? 1 : 0;
  side_hits += static_cast<double>(right_hits) / static_cast<double>(height_cells) >= 0.35 ? 1 : 0;
  validation.boundary_evidence = static_cast<double>(side_hits) / static_cast<double>(sides);

  validation.area_m2 = static_cast<double>(width_cells * height_cells) * resolution * resolution;
  const double bbox_width_m = static_cast<double>(width_cells) * resolution;
  const double bbox_height_m = static_cast<double>(height_cells) * resolution;
  const double aspect_ratio = std::max(bbox_width_m, bbox_height_m) /
    std::max(std::min(bbox_width_m, bbox_height_m), resolution);
  validation.endpoint_stability = std::max(0.0, std::min(1.0, candidate.endpoint_stability));

  validation.barrier = detect_internal_barrier(candidate, view, raw_free);
  validation.internal_barriers = validation.barrier.found ? 1 : 0;
  const double size_score = clamp01(
    validation.area_m2 / std::max(params_.area_min_area_m2 * 2.0, k_epsilon));
  const double no_barrier_score = validation.barrier.found ? 0.0 : 1.0;
  validation.confidence = clamp01(
    (0.35 * validation.free_ratio) +
    (0.20 * validation.boundary_evidence) +
    (0.15 * validation.endpoint_stability) +
    (0.15 * size_score) +
    (0.15 * no_barrier_score) -
    (validation.barrier.found ? 0.50 : 0.0));
  if (validation.barrier.found) {
    validation.reason = "internal_barrier";
    return validation;
  }

  std::vector<bool> known_mask(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    known_mask[static_cast<std::size_t>(index)] = !view.is_unknown(index);
  }
  const GridBounds known_extent = mask_bounds(view, known_mask);
  const GridBounds free_extent = mask_bounds(view, raw_free);
  auto bounds_area_m2 = [resolution](const GridBounds & bounds) {
      if (!bounds.valid) {
        return 0.0;
      }
      const int bounds_width_cells = std::max(0, bounds.max_x - bounds.min_x + 1);
      const int bounds_height_cells = std::max(0, bounds.max_y - bounds.min_y + 1);
      return static_cast<double>(bounds_width_cells * bounds_height_cells) * resolution *
             resolution;
    };
  AreaCandidate rejection_candidate;
  rejection_candidate.min_x = min_x;
  rejection_candidate.min_y = min_y;
  rejection_candidate.max_x = max_x;
  rejection_candidate.max_y = max_y;
  rejection_candidate.bbox_area_m2 = validation.area_m2;
  rejection_candidate.fill_ratio = validation.free_ratio;
  rejection_candidate.free_ratio = validation.free_ratio;
  const double known_area_m2 = bounds_area_m2(known_extent);
  const double free_area_m2 = bounds_area_m2(free_extent);
  rejection_candidate.known_extent_ratio = known_area_m2 > k_epsilon ?
    validation.area_m2 / known_area_m2 : 0.0;
  rejection_candidate.free_extent_ratio = free_area_m2 > k_epsilon ?
    validation.area_m2 / free_area_m2 : 0.0;
  std::string rejection_reason;
  if (reject_area_candidate(rejection_candidate, known_extent, free_extent, rejection_reason)) {
    validation.reason = rejection_reason;
    return validation;
  }

  if (params_.area_interior_validation_enabled) {
    if (validation.occupied_ratio > params_.area_max_occupied_ratio) {
      validation.reason = "occupied_ratio";
      return validation;
    }
    if (validation.unknown_ratio > params_.area_max_unknown_ratio) {
      validation.reason = "unknown_ratio";
      return validation;
    }
    if (validation.boundary_ratio > params_.area_max_boundary_ratio) {
      validation.reason = "boundary_ratio";
      return validation;
    }
  }
  if (validation.area_m2 < params_.area_min_area_m2 ||
    bbox_width_m < params_.area_min_side_m || bbox_height_m < params_.area_min_side_m)
  {
    validation.reason = "too_small";
    return validation;
  }
  if (validation.area_m2 > params_.area_max_bbox_area_m2) {
    validation.reason = "bbox_area_too_large";
    return validation;
  }
  if (aspect_ratio > params_.area_max_aspect_ratio) {
    validation.reason = "corridor_like";
    return validation;
  }
  if (validation.free_ratio < params_.area_min_free_ratio) {
    validation.reason = "low_free_ratio";
    return validation;
  }
  if (params_.area_reject_tiny_ambiguous &&
    validation.area_m2 < params_.area_tiny_area_threshold_m2 &&
    (validation.boundary_evidence < params_.area_tiny_area_min_boundary_evidence ||
    validation.free_ratio < params_.area_tiny_area_min_free_ratio))
  {
    validation.reason = "tiny_ambiguous";
    return validation;
  }
  if (validation.confidence < params_.area_min_confidence) {
    validation.reason = "low_confidence";
    return validation;
  }

  validation.accepted = true;
  validation.reason = "accepted";
  return validation;
}

/// @brief Find long occupied/unknown/boundary runs crossing an area bbox interior.
SegmentClassifier::BarrierInfo SegmentClassifier::detect_internal_barrier(
  const ScanlineBand & candidate,
  const OccupancyGridView & view,
  const std::vector<bool> & raw_free) const
{
  BarrierInfo best;
  if (!params_.area_internal_barrier_enabled) {
    return best;
  }
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int margin_cells = std::max(
    1, static_cast<int>(std::ceil(params_.area_internal_barrier_margin_m / resolution)));
  const int max_gap_cells = std::max(
    0, static_cast<int>(std::ceil(params_.area_internal_barrier_max_gap_m / resolution)));
  const int min_x = std::max(0, candidate.min_x);
  const int min_y = std::max(0, candidate.min_y);
  const int max_x = std::min(view.width() - 1, candidate.max_x);
  const int max_y = std::min(view.height() - 1, candidate.max_y);
  const int width_cells = max_x - min_x + 1;
  const int height_cells = max_y - min_y + 1;
  if (width_cells <= margin_cells * 2 || height_cells <= margin_cells * 2) {
    return best;
  }

  auto is_barrier_cell = [&view, &raw_free](const int x, const int y) {
      if (!view.in_bounds(x, y)) {
        return true;
      }
      const int index = view.to_index(x, y);
      return !raw_free[static_cast<std::size_t>(index)] &&
             (view.is_raw_occupied(index) || view.is_unknown(index) || view.is_occupied(index));
    };
  auto longest_run = [max_gap_cells](const std::vector<bool> & values) {
      int best_run = 0;
      int current = 0;
      int gap = 0;
      for (const bool value : values) {
        if (value) {
          ++current;
          gap = 0;
        } else if (current > 0 && gap < max_gap_cells) {
          ++current;
          ++gap;
        } else {
          best_run = std::max(best_run, current - gap);
          current = 0;
          gap = 0;
        }
      }
      best_run = std::max(best_run, current - gap);
      return best_run;
    };

  for (int x = min_x + margin_cells; x <= max_x - margin_cells; ++x) {
    std::vector<bool> values;
    values.reserve(static_cast<std::size_t>(height_cells));
    for (int y = min_y; y <= max_y; ++y) {
      values.push_back(is_barrier_cell(x, y));
    }
    const double span_ratio = static_cast<double>(longest_run(values)) /
      static_cast<double>(std::max(1, height_cells));
    if (span_ratio > best.span_ratio) {
      best.found = span_ratio >= params_.area_internal_barrier_min_span_ratio;
      best.vertical = true;
      best.line = x;
      best.span_ratio = span_ratio;
    }
  }
  for (int y = min_y + margin_cells; y <= max_y - margin_cells; ++y) {
    std::vector<bool> values;
    values.reserve(static_cast<std::size_t>(width_cells));
    for (int x = min_x; x <= max_x; ++x) {
      values.push_back(is_barrier_cell(x, y));
    }
    const double span_ratio = static_cast<double>(longest_run(values)) /
      static_cast<double>(std::max(1, width_cells));
    if (span_ratio > best.span_ratio) {
      best.found = span_ratio >= params_.area_internal_barrier_min_span_ratio;
      best.vertical = false;
      best.line = y;
      best.span_ratio = span_ratio;
    }
  }
  if (best.span_ratio < params_.area_internal_barrier_min_span_ratio) {
    best.found = false;
  }
  return best;
}

/// @brief Emit structured area validation diagnostics.
void SegmentClassifier::log_area_validation(
  const std::uint32_t id,
  const ScanlineBand & candidate,
  const AreaValidation & validation) const
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "spatial_segmenter"),
    "AMR_LOG schema=v1 component=spatial_segmenter event=area_validation id=%u area_m2=%.3f free_ratio=%.3f occupied_ratio=%.3f unknown_ratio=%.3f internal_barriers=%d boundary_evidence=%.3f confidence=%.3f result=%s reason=%s",
    id,
    validation.area_m2,
    validation.free_ratio,
    validation.occupied_ratio,
    validation.unknown_ratio,
    validation.internal_barriers,
    validation.boundary_evidence,
    validation.confidence,
    validation.accepted ? "accepted" : "rejected",
    validation.reason.c_str());
  if (validation.reason == "internal_barrier") {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "spatial_segmenter"),
      "AMR_LOG schema=v1 component=spatial_segmenter event=area_rejected reason=internal_barrier id=%u orientation=%s span_ratio=%.3f bbox=%d,%d,%d,%d",
      id,
      validation.barrier.vertical ? "vertical" : "horizontal",
      validation.barrier.span_ratio,
      candidate.min_x,
      candidate.min_y,
      candidate.max_x,
      candidate.max_y);
  }
}

/// @brief Expand a binary mask by a metric radius.
std::vector<bool> SegmentClassifier::expanded_mask(
  const OccupancyGridView & view,
  const std::vector<bool> & seeds,
  const double radius_m) const
{
  std::vector<bool> mask(static_cast<std::size_t>(view.cell_count()), false);
  if (!view.valid() || seeds.size() != static_cast<std::size_t>(view.cell_count())) {
    return mask;
  }

  const double resolution = std::max(view.resolution(), k_epsilon);
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(radius_m / resolution)));
  const double radius_with_margin = radius_m + (resolution * 0.5);
  for (int index = 0; index < view.cell_count(); ++index) {
    if (!seeds[static_cast<std::size_t>(index)]) {
      continue;
    }
    const GridIndex center = view.to_grid(index);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int x = center.x + dx;
        const int y = center.y + dy;
        if (!view.in_bounds(x, y)) {
          continue;
        }
        const double distance = std::hypot(
          static_cast<double>(dx) * resolution,
          static_cast<double>(dy) * resolution);
        if (distance <= radius_with_margin) {
          mask[static_cast<std::size_t>(view.to_index(x, y))] = true;
        }
      }
    }
  }
  return mask;
}

/// @brief Return grid-cell bounds for all true cells in a mask.
SegmentClassifier::GridBounds SegmentClassifier::mask_bounds(
  const OccupancyGridView & view,
  const std::vector<bool> & mask) const
{
  GridBounds bounds;
  if (!view.valid() || mask.size() != static_cast<std::size_t>(view.cell_count())) {
    return bounds;
  }

  for (int index = 0; index < view.cell_count(); ++index) {
    if (!mask[static_cast<std::size_t>(index)]) {
      continue;
    }
    const GridIndex grid = view.to_grid(index);
    if (!bounds.valid) {
      bounds.min_x = grid.x;
      bounds.max_x = grid.x;
      bounds.min_y = grid.y;
      bounds.max_y = grid.y;
      bounds.valid = true;
      continue;
    }
    bounds.min_x = std::min(bounds.min_x, grid.x);
    bounds.max_x = std::max(bounds.max_x, grid.x);
    bounds.min_y = std::min(bounds.min_y, grid.y);
    bounds.max_y = std::max(bounds.max_y, grid.y);
  }
  return bounds;
}

/// @brief Convert a raw component bbox into a rejection-checked diagnostic candidate.
SegmentClassifier::AreaCandidate SegmentClassifier::component_bbox_candidate(
  const std::uint32_t diagnostic_id,
  const Component & component,
  const ComponentStats & stats,
  const GridBounds & known_extent,
  const std::vector<bool> & raw_free,
  const OccupancyGridView & view,
  const bool largest_component_without_split) const
{
  AreaCandidate candidate;
  candidate.diagnostic_id = diagnostic_id;
  candidate.min_x = component.min_x;
  candidate.min_y = component.min_y;
  candidate.max_x = component.max_x;
  candidate.max_y = component.max_y;
  candidate.component_area_m2 = stats.area_m2;
  candidate.from_component_bbox = true;
  candidate.largest_component_without_split = largest_component_without_split;
  const double resolution = std::max(view.resolution(), k_epsilon);
  const int bbox_width_cells = std::max(0, component.max_x - component.min_x + 1);
  const int bbox_height_cells = std::max(0, component.max_y - component.min_y + 1);
  const int bbox_cells = bbox_width_cells * bbox_height_cells;
  candidate.bbox_area_m2 =
    static_cast<double>(bbox_cells) * resolution * resolution;
  int free_cells = 0;
  for (int y = component.min_y; y <= component.max_y; ++y) {
    for (int x = component.min_x; x <= component.max_x; ++x) {
      if (!view.in_bounds(x, y)) {
        continue;
      }
      const int index = view.to_index(x, y);
      if (raw_free[static_cast<std::size_t>(index)]) {
        ++free_cells;
      }
    }
  }
  candidate.fill_ratio = bbox_cells > 0 ?
    static_cast<double>(free_cells) / static_cast<double>(bbox_cells) : 0.0;
  candidate.free_ratio = candidate.fill_ratio;
  candidate.confidence = open_area_confidence(stats);

  auto bounds_area_m2 = [resolution](const GridBounds & bounds) {
      if (!bounds.valid) {
        return 0.0;
      }
      const int width_cells = std::max(0, bounds.max_x - bounds.min_x + 1);
      const int height_cells = std::max(0, bounds.max_y - bounds.min_y + 1);
      return static_cast<double>(width_cells * height_cells) * resolution * resolution;
    };
  const double known_area_m2 = bounds_area_m2(known_extent);
  const GridBounds free_extent = mask_bounds(view, raw_free);
  const double free_area_m2 = bounds_area_m2(free_extent);
  candidate.known_extent_ratio = known_area_m2 > k_epsilon ?
    candidate.bbox_area_m2 / known_area_m2 : 0.0;
  candidate.free_extent_ratio = free_area_m2 > k_epsilon ?
    candidate.bbox_area_m2 / free_area_m2 : 0.0;
  return candidate;
}

/// @brief Detect compact local open-area rectangles after corridor and portal exclusions.
std::vector<SegmentClassifier::AreaCandidate> SegmentClassifier::detect_local_area_candidates(
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
  int & rejected_count) const
{
  std::vector<AreaCandidate> candidates;
  local_candidate_count = 0;
  rejected_count = 0;
  if (!view.valid() || raw_free.size() != static_cast<std::size_t>(view.cell_count()) ||
    area_mask.size() != raw_free.size())
  {
    return candidates;
  }

  const int width = view.width();
  const int height = view.height();
  const double resolution = std::max(view.resolution(), k_epsilon);
  std::vector<bool> occupied(static_cast<std::size_t>(view.cell_count()), false);
  std::vector<bool> unknown(static_cast<std::size_t>(view.cell_count()), false);
  std::vector<bool> finite_obstacle_context(static_cast<std::size_t>(view.cell_count()), false);
  for (int index = 0; index < view.cell_count(); ++index) {
    occupied[static_cast<std::size_t>(index)] = view.is_raw_occupied(index);
    unknown[static_cast<std::size_t>(index)] = view.is_unknown(index);
    finite_obstacle_context[static_cast<std::size_t>(index)] =
      raw_obstacle_distance.valid() && std::isfinite(raw_obstacle_distance.at(index));
  }

  auto build_integral = [width, height, &view](const std::vector<bool> & mask) {
      std::vector<int> integral(static_cast<std::size_t>((width + 1) * (height + 1)), 0);
      for (int y = 0; y < height; ++y) {
        int row_sum = 0;
        for (int x = 0; x < width; ++x) {
          if (mask[static_cast<std::size_t>(view.to_index(x, y))]) {
            ++row_sum;
          }
          integral[static_cast<std::size_t>(((y + 1) * (width + 1)) + (x + 1))] =
            integral[static_cast<std::size_t>((y * (width + 1)) + (x + 1))] + row_sum;
        }
      }
      return integral;
    };
  auto query_integral = [width](
    const std::vector<int> & integral,
    const int x0,
    const int y0,
    const int x1,
    const int y1)
    {
      const int stride = width + 1;
      return integral[static_cast<std::size_t>((y1 * stride) + x1)] -
             integral[static_cast<std::size_t>((y0 * stride) + x1)] -
             integral[static_cast<std::size_t>((y1 * stride) + x0)] +
             integral[static_cast<std::size_t>((y0 * stride) + x0)];
    };

  const auto raw_free_integral = build_integral(raw_free);
  const auto area_mask_integral = build_integral(area_mask);
  const auto occupied_integral = build_integral(occupied);
  const auto unknown_integral = build_integral(unknown);
  const auto corridor_integral = build_integral(corridor_exclusion_mask);
  const auto portal_integral = build_integral(portal_exclusion_mask);
  const auto finite_context_integral = build_integral(finite_obstacle_context);
  const GridBounds free_extent = mask_bounds(view, raw_free);

  auto bounds_area_m2 = [resolution](const GridBounds & bounds) {
      if (!bounds.valid) {
        return 0.0;
      }
      const int width_cells = std::max(0, bounds.max_x - bounds.min_x + 1);
      const int height_cells = std::max(0, bounds.max_y - bounds.min_y + 1);
      return static_cast<double>(width_cells * height_cells) * resolution * resolution;
    };
  const double known_area_m2 = bounds_area_m2(known_extent);
  const double free_extent_area_m2 = bounds_area_m2(free_extent);
  const int stride_cells = std::max(
    1, static_cast<int>(std::round(params_.area_window_stride_m / resolution)));
  std::uint32_t diagnostic_id = 1U;

  auto source_component_for_window = [
    &view, &component_ids, &raw_free](
    const int min_x,
    const int min_y,
    const int max_x,
    const int max_y)
    {
      const int center_x = (min_x + max_x) / 2;
      const int center_y = (min_y + max_y) / 2;
      if (view.in_bounds(center_x, center_y)) {
        const int center_index = view.to_index(center_x, center_y);
        if (center_index >= 0 && center_index < static_cast<int>(component_ids.size()) &&
          component_ids[static_cast<std::size_t>(center_index)] >= 0)
        {
          return component_ids[static_cast<std::size_t>(center_index)];
        }
      }
      for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
          if (!view.in_bounds(x, y)) {
            continue;
          }
          const int index = view.to_index(x, y);
          if (raw_free[static_cast<std::size_t>(index)] &&
            index < static_cast<int>(component_ids.size()) &&
            component_ids[static_cast<std::size_t>(index)] >= 0)
          {
            return component_ids[static_cast<std::size_t>(index)];
          }
        }
      }
      return -1;
    };

  for (const double requested_width_m : params_.area_window_sizes_m) {
    for (const double requested_height_m : params_.area_window_sizes_m) {
      const int window_width_cells = std::max(
        1, static_cast<int>(std::round(requested_width_m / resolution)));
      const int window_height_cells = std::max(
        1, static_cast<int>(std::round(requested_height_m / resolution)));
      const double window_width_m = static_cast<double>(window_width_cells) * resolution;
      const double window_height_m = static_cast<double>(window_height_cells) * resolution;
      const double bbox_area_m2 = window_width_m * window_height_m;
      if (window_width_m < params_.area_min_width_m ||
        window_height_m < params_.area_min_height_m ||
        window_width_m > params_.area_max_width_m ||
        window_height_m > params_.area_max_height_m ||
        bbox_area_m2 < params_.area_min_area_m2 ||
        window_width_cells > width || window_height_cells > height)
      {
        continue;
      }

      for (int y0 = 0; y0 <= height - window_height_cells; y0 += stride_cells) {
        for (int x0 = 0; x0 <= width - window_width_cells; x0 += stride_cells) {
          const int x1 = x0 + window_width_cells;
          const int y1 = y0 + window_height_cells;
          const int total_cells = window_width_cells * window_height_cells;
          const int usable_free_cells = query_integral(area_mask_integral, x0, y0, x1, y1);
          const int raw_free_cells = query_integral(raw_free_integral, x0, y0, x1, y1);
          const int occupied_cells = query_integral(occupied_integral, x0, y0, x1, y1);
          const int unknown_cells = query_integral(unknown_integral, x0, y0, x1, y1);
          const int corridor_cells = query_integral(corridor_integral, x0, y0, x1, y1);
          const int portal_cells = query_integral(portal_integral, x0, y0, x1, y1);
          const int finite_context_cells = query_integral(
            finite_context_integral, x0, y0, x1, y1);
          const double inv_total = 1.0 / static_cast<double>(std::max(1, total_cells));
          const double free_ratio = static_cast<double>(usable_free_cells) * inv_total;
          const double fill_ratio = static_cast<double>(raw_free_cells) * inv_total;
          const double occupied_ratio = static_cast<double>(occupied_cells) * inv_total;
          const double unknown_ratio = static_cast<double>(unknown_cells) * inv_total;
          const double corridor_overlap_ratio = static_cast<double>(corridor_cells) * inv_total;
          const double portal_overlap_ratio = static_cast<double>(portal_cells) * inv_total;
          if (free_ratio < params_.area_min_local_free_ratio ||
            occupied_ratio > params_.area_max_local_occupied_ratio ||
            unknown_ratio > params_.area_max_local_unknown_ratio ||
            corridor_overlap_ratio > 0.20 ||
            portal_overlap_ratio > 0.01 ||
            finite_context_cells == 0)
          {
            continue;
          }

          AreaCandidate candidate;
          candidate.diagnostic_id = diagnostic_id++;
          candidate.min_x = x0;
          candidate.min_y = y0;
          candidate.max_x = x1 - 1;
          candidate.max_y = y1 - 1;
          candidate.bbox_area_m2 = bbox_area_m2;
          candidate.fill_ratio = fill_ratio;
          candidate.free_ratio = free_ratio;
          candidate.occupied_ratio = occupied_ratio;
          candidate.unknown_ratio = unknown_ratio;
          candidate.corridor_overlap_ratio = corridor_overlap_ratio;
          candidate.portal_overlap_ratio = portal_overlap_ratio;
          candidate.confidence = clamp01(
            free_ratio - (occupied_ratio * 2.0) - (unknown_ratio * 0.5) -
            corridor_overlap_ratio);
          candidate.source_component = source_component_for_window(
            candidate.min_x, candidate.min_y, candidate.max_x, candidate.max_y);
          if (candidate.source_component >= 0 &&
            candidate.source_component < static_cast<int>(component_areas_m2.size()))
          {
            candidate.component_area_m2 =
              component_areas_m2[static_cast<std::size_t>(candidate.source_component)];
          }
          candidate.known_extent_ratio = known_area_m2 > k_epsilon ?
            candidate.bbox_area_m2 / known_area_m2 : 0.0;
          candidate.free_extent_ratio = free_extent_area_m2 > k_epsilon ?
            candidate.bbox_area_m2 / free_extent_area_m2 : 0.0;
          if (candidate.confidence < params_.area_min_confidence) {
            continue;
          }

          ++local_candidate_count;
          std::string rejection_reason;
          if (reject_area_candidate(candidate, known_extent, free_extent, rejection_reason)) {
            ++rejected_count;
            log_area_rejected(candidate, rejection_reason);
            continue;
          }
          candidates.push_back(candidate);
        }
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(), [](const AreaCandidate & lhs, const AreaCandidate & rhs) {
      if (std::abs(lhs.confidence - rhs.confidence) > 1.0e-6) {
        return lhs.confidence > rhs.confidence;
      }
      return lhs.bbox_area_m2 > rhs.bbox_area_m2;
    });

  auto iou = [](const AreaCandidate & lhs, const AreaCandidate & rhs) {
      const int ix0 = std::max(lhs.min_x, rhs.min_x);
      const int iy0 = std::max(lhs.min_y, rhs.min_y);
      const int ix1 = std::min(lhs.max_x, rhs.max_x);
      const int iy1 = std::min(lhs.max_y, rhs.max_y);
      const int iw = std::max(0, ix1 - ix0 + 1);
      const int ih = std::max(0, iy1 - iy0 + 1);
      const double intersection = static_cast<double>(iw * ih);
      const double lhs_area = static_cast<double>(
        std::max(0, lhs.max_x - lhs.min_x + 1) *
        std::max(0, lhs.max_y - lhs.min_y + 1));
      const double rhs_area = static_cast<double>(
        std::max(0, rhs.max_x - rhs.min_x + 1) *
        std::max(0, rhs.max_y - rhs.min_y + 1));
      const double union_area = lhs_area + rhs_area - intersection;
      return union_area > k_epsilon ? intersection / union_area : 0.0;
    };

  std::vector<AreaCandidate> merged;
  for (const auto & candidate : candidates) {
    bool overlaps_existing = false;
    for (const auto & accepted : merged) {
      if (iou(candidate, accepted) > params_.area_merge_iou_threshold) {
        overlaps_existing = true;
        break;
      }
    }
    if (overlaps_existing) {
      continue;
    }
    merged.push_back(candidate);
    if (static_cast<int>(merged.size()) >= std::max(0, params_.area_max_areas)) {
      break;
    }
  }

  return merged;
}

/// @brief Return true when an area proposal should never be visualized.
bool SegmentClassifier::reject_area_candidate(
  const AreaCandidate & candidate,
  const GridBounds & known_extent,
  const GridBounds & free_extent,
  std::string & reason) const
{
  if (params_.area_reject_map_sized_bbox &&
    (candidate.known_extent_ratio >= params_.area_max_bbox_known_extent_ratio ||
    (candidate.free_extent_ratio >= params_.area_max_bbox_known_extent_ratio &&
    candidate.known_extent_ratio >= params_.area_max_bbox_known_extent_ratio * 0.75)))
  {
    reason = "map_sized_bbox";
    return true;
  }

  if (params_.area_reject_map_sized_bbox && known_extent.valid) {
    constexpr int near_cells = 1;
    if (candidate.min_x <= known_extent.min_x + near_cells ||
      candidate.min_y <= known_extent.min_y + near_cells ||
      candidate.max_x >= known_extent.max_x - near_cells ||
      candidate.max_y >= known_extent.max_y - near_cells)
    {
      reason = "touches_known_bounds";
      return true;
    }
  }

  if (free_extent.valid &&
    candidate.min_x <= free_extent.min_x && candidate.min_y <= free_extent.min_y &&
    candidate.max_x >= free_extent.max_x && candidate.max_y >= free_extent.max_y &&
    candidate.known_extent_ratio >= params_.area_max_bbox_known_extent_ratio * 0.75)
  {
    reason = "map_sized_bbox";
    return true;
  }

  if (candidate.bbox_area_m2 > params_.area_max_bbox_area_m2) {
    reason = "bbox_area_too_large";
    return true;
  }

  const double width_cells =
    static_cast<double>(std::max(1, candidate.max_x - candidate.min_x + 1));
  const double height_cells =
    static_cast<double>(std::max(1, candidate.max_y - candidate.min_y + 1));
  const double aspect_ratio = std::max(width_cells, height_cells) /
    std::max(std::min(width_cells, height_cells), 1.0);
  if (aspect_ratio > params_.area_max_bbox_aspect_ratio) {
    reason = "extreme_aspect_ratio";
    return true;
  }

  if (candidate.fill_ratio < params_.area_min_bbox_fill_ratio) {
    reason = "low_fill_ratio";
    return true;
  }

  if (candidate.from_component_bbox && params_.area_reject_largest_component_without_split &&
    candidate.component_area_m2 > params_.area_max_component_area_m2_without_split)
  {
    reason = candidate.largest_component_without_split ?
      "largest_component_without_split" : "huge_unresolved_component";
    return true;
  }

  return false;
}

/// @brief Emit a structured area rejection diagnostic.
void SegmentClassifier::log_area_rejected(
  const AreaCandidate & candidate,
  const std::string & reason) const
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "spatial_segmenter"),
    "AMR_LOG schema=v1 component=spatial_segmenter event=area_rejected reason=%s id=%u bbox_area_m2=%.3f known_extent_ratio=%.3f fill_ratio=%.3f",
    reason.c_str(),
    candidate.diagnostic_id,
    candidate.bbox_area_m2,
    candidate.known_extent_ratio,
    candidate.fill_ratio);
}

/// @brief Compute PCA, area, width, and bbox statistics for a component.
SegmentClassifier::ComponentStats SegmentClassifier::analyze_component(
  const Component & component,
  const OccupancyGridView & view) const
{
  ComponentStats stats;
  if (component.cells.empty()) {
    return stats;
  }

  const double resolution = view.resolution();
  stats.area_m2 = static_cast<double>(component.cells.size()) * resolution * resolution;

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const int cell : component.cells) {
    const auto point = view.cell_center(cell);
    sum_x += point.x;
    sum_y += point.y;
  }
  stats.centroid.x = sum_x / static_cast<double>(component.cells.size());
  stats.centroid.y = sum_y / static_cast<double>(component.cells.size());
  stats.centroid.z = params_.marker_height_z;

  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  for (const int cell : component.cells) {
    const auto point = view.cell_center(cell);
    const double dx = point.x - stats.centroid.x;
    const double dy = point.y - stats.centroid.y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  const double inv_count = 1.0 / static_cast<double>(component.cells.size());
  cov_xx *= inv_count;
  cov_xy *= inv_count;
  cov_yy *= inv_count;

  const double angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
  stats.axis_x = std::cos(angle);
  stats.axis_y = std::sin(angle);
  if (!std::isfinite(stats.axis_x) || !std::isfinite(stats.axis_y)) {
    stats.axis_x = 1.0;
    stats.axis_y = 0.0;
  }

  stats.min_projection = std::numeric_limits<double>::infinity();
  stats.max_projection = -std::numeric_limits<double>::infinity();
  for (const int cell : component.cells) {
    const auto point = view.cell_center(cell);
    const double projection =
      ((point.x - stats.centroid.x) * stats.axis_x) +
      ((point.y - stats.centroid.y) * stats.axis_y);
    stats.min_projection = std::min(stats.min_projection, projection);
    stats.max_projection = std::max(stats.max_projection, projection);
  }
  stats.length_m = std::max(
    resolution,
    (stats.max_projection - stats.min_projection) + resolution);
  stats.width_m = stats.area_m2 / std::max(stats.length_m, resolution);
  stats.aspect_ratio = stats.length_m / std::max(stats.width_m, resolution);

  const auto p0 = view.grid_corner(component.min_x, component.min_y);
  const auto p1 = view.grid_corner(component.max_x + 1, component.min_y);
  const auto p2 = view.grid_corner(component.max_x + 1, component.max_y + 1);
  const auto p3 = view.grid_corner(component.min_x, component.max_y + 1);
  stats.bbox_polygon = {p0, p1, p2, p3, p0};

  const double min_x = std::min({p0.x, p1.x, p2.x, p3.x});
  const double max_x = std::max({p0.x, p1.x, p2.x, p3.x});
  const double min_y = std::min({p0.y, p1.y, p2.y, p3.y});
  const double max_y = std::max({p0.y, p1.y, p2.y, p3.y});
  stats.bbox_min = make_point(min_x, min_y, params_.marker_height_z);
  stats.bbox_max = make_point(max_x, max_y, params_.marker_height_z);
  stats.bbox_width_m = std::hypot(p1.x - p0.x, p1.y - p0.y);
  stats.bbox_height_m = std::hypot(p3.x - p0.x, p3.y - p0.y);

  return stats;
}

/// @brief Return true when stats satisfy open-area rules.
bool SegmentClassifier::is_open_area(const ComponentStats & stats) const
{
  return stats.area_m2 >= params_.area_min_area_m2 &&
         stats.bbox_width_m >= params_.area_min_width_m &&
         stats.bbox_height_m >= params_.area_min_width_m &&
         open_area_confidence(stats) >= params_.area_min_confidence;
}

/// @brief Compute a bounded heuristic corridor confidence score.
double SegmentClassifier::corridor_confidence(const SkeletonLine & line) const
{
  const double length_score =
    clamp01(line.length_m / std::max(params_.corridor_min_length_m * 2.0, k_epsilon));
  const double width_mid = (params_.corridor_min_width_m + params_.corridor_max_width_m) * 0.5;
  const double width_half_span =
    std::max((params_.corridor_max_width_m - params_.corridor_min_width_m) * 0.5, k_epsilon);
  const double width_score = clamp01(1.0 - (std::abs(line.width_m - width_mid) / width_half_span));
  const double stability_score = clamp01(
    1.0 - (line.width_stddev_m / std::max(
      params_.corridor_skeleton_min_width_variance_m,
      k_epsilon)));
  return clamp01(
    (0.30 * length_score) +
    (0.30 * width_score) +
    (0.25 * stability_score) +
    (0.15 * line.traversable_ratio));
}

/// @brief Compute a bounded heuristic open-area confidence score.
double SegmentClassifier::open_area_confidence(const ComponentStats & stats) const
{
  const double area_score =
    clamp01(stats.area_m2 / std::max(params_.area_min_area_m2 * 2.0, k_epsilon));
  const double width_score =
    clamp01(
    std::min(
      stats.bbox_width_m,
      stats.bbox_height_m) / std::max(params_.area_min_width_m, k_epsilon));
  const double compactness_score =
    clamp01(
    std::min(stats.bbox_width_m, stats.bbox_height_m) /
    std::max(std::max(stats.bbox_width_m, stats.bbox_height_m), k_epsilon));
  return clamp01((0.45 * area_score) + (0.35 * width_score) + (0.20 * compactness_score));
}

/// @brief Clamp a value into [0, 1].
double SegmentClassifier::clamp01(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

}  // namespace amr::spatial_segmenter
