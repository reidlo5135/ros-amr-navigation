#ifndef AMR_GEOMETRY__FOOTPRINT_HPP_
#define AMR_GEOMETRY__FOOTPRINT_HPP_

/**
 * @file footprint.hpp
 * @brief Inline footprint geometry helpers shared by costmap and planner nodes.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace amr::geometry
{

/// @brief 2D point expressed in the robot footprint or world frame.
struct FootprintPoint
{
  /// @brief X coordinate in meters.
  double x;
  /// @brief Y coordinate in meters.
  double y;
};

/// @brief Ordered polygon describing a robot footprint boundary.
using FootprintPolygon = std::vector<FootprintPoint>;

/// @brief Convert a flat x/y parameter vector into a footprint polygon.
inline FootprintPolygon make_footprint_polygon(const std::vector<double> &flat_polygon)
{
  FootprintPolygon polygon;
  polygon.reserve(flat_polygon.size() / 2U);
  for (std::size_t index = 0; index + 1U < flat_polygon.size(); index += 2U) {
    polygon.push_back({flat_polygon[index], flat_polygon[index + 1U]});
  }
  return polygon;
}

/// @brief Compute the maximum distance from the footprint origin to any vertex.
inline double footprint_circumscribed_radius(const FootprintPolygon &polygon)
{
  double radius = 0.0;
  for (const auto &point : polygon) {
    radius = std::max(radius, std::hypot(point.x, point.y));
  }
  return radius;
}

/// @brief Transform a footprint polygon from robot coordinates into world coordinates.
inline FootprintPolygon transform_footprint(
  const FootprintPolygon &polygon,
  const double pose_x,
  const double pose_y,
  const double pose_yaw)
{
  FootprintPolygon transformed;
  transformed.reserve(polygon.size());
  const double cos_yaw = std::cos(pose_yaw);
  const double sin_yaw = std::sin(pose_yaw);
  for (const auto &point : polygon) {
    transformed.push_back({
      pose_x + (point.x * cos_yaw) - (point.y * sin_yaw),
      pose_y + (point.x * sin_yaw) + (point.y * cos_yaw)
    });
  }
  return transformed;
}

/// @brief Test whether a 2D point lies inside a polygon using ray casting.
inline bool point_in_polygon(
  const double x,
  const double y,
  const FootprintPolygon &polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }

  bool inside = false;
  std::size_t previous = polygon.size() - 1U;
  for (std::size_t current = 0; current < polygon.size(); ++current) {
    const auto &a = polygon[current];
    const auto &b = polygon[previous];
    const bool intersects =
      ((a.y > y) != (b.y > y)) &&
      (x < ((b.x - a.x) * (y - a.y) / ((b.y - a.y) + 1e-12)) + a.x);
    if (intersects) {
      inside = !inside;
    }
    previous = current;
  }
  return inside;
}

/// @brief Compute the signed orientation of three points.
inline double orientation(
  const FootprintPoint &a,
  const FootprintPoint &b,
  const FootprintPoint &c)
{
  return ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x));
}

/// @brief Test whether a point lies on the bounding box of a segment.
inline bool on_segment(
  const FootprintPoint &a,
  const FootprintPoint &b,
  const FootprintPoint &p)
{
  return
    p.x >= std::min(a.x, b.x) - 1e-9 &&
    p.x <= std::max(a.x, b.x) + 1e-9 &&
    p.y >= std::min(a.y, b.y) - 1e-9 &&
    p.y <= std::max(a.y, b.y) + 1e-9;
}

/// @brief Test whether two line segments intersect, including collinear overlap.
inline bool segments_intersect(
  const FootprintPoint &p1,
  const FootprintPoint &q1,
  const FootprintPoint &p2,
  const FootprintPoint &q2)
{
  const double o1 = orientation(p1, q1, p2);
  const double o2 = orientation(p1, q1, q2);
  const double o3 = orientation(p2, q2, p1);
  const double o4 = orientation(p2, q2, q1);

  if (((o1 > 0.0) != (o2 > 0.0)) &&((o3 > 0.0) != (o4 > 0.0))) {
    return true;
  }

  if (std::abs(o1) <= 1e-9 &&on_segment(p1, q1, p2)) {
    return true;
  }
  if (std::abs(o2) <= 1e-9 &&on_segment(p1, q1, q2)) {
    return true;
  }
  if (std::abs(o3) <= 1e-9 &&on_segment(p2, q2, p1)) {
    return true;
  }
  if (std::abs(o4) <= 1e-9 &&on_segment(p2, q2, q1)) {
    return true;
  }

  return false;
}

/// @brief Test whether a polygon overlaps an axis-aligned grid cell.
inline bool polygon_intersects_cell(
  const FootprintPolygon &polygon,
  const double cell_min_x,
  const double cell_min_y,
  const double cell_max_x,
  const double cell_max_y)
{
  if (polygon.empty()) {
    return false;
  }

  for (const auto &point : polygon) {
    if (
      point.x >= cell_min_x &&point.x <= cell_max_x &&
      point.y >= cell_min_y &&point.y <= cell_max_y)
    {
      return true;
    }
  }

  const FootprintPoint corners[4] = {
    {cell_min_x, cell_min_y},
    {cell_max_x, cell_min_y},
    {cell_max_x, cell_max_y},
    {cell_min_x, cell_max_y}
  };
  for (const auto &corner : corners) {
    if (point_in_polygon(corner.x, corner.y, polygon)) {
      return true;
    }
  }

  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const FootprintPoint &a = polygon[index];
    const FootprintPoint &b = polygon[(index + 1U) % polygon.size()];
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      const FootprintPoint &c = corners[corner_index];
      const FootprintPoint &d = corners[(corner_index + 1) % 4];
      if (segments_intersect(a, b, c, d)) {
        return true;
      }
    }
  }

  return false;
}

/// @brief Test whether a world-frame polygon collides with occupied grid cells.
inline bool polygon_collides_with_grid(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const double resolution,
  const double origin_x,
  const double origin_y,
  const FootprintPolygon &polygon,
  const int obstacle_threshold,
  const bool allow_unknown)
{
  if (
    width <= 0 || height <= 0 ||
    resolution <= 0.0 ||
    occupancy_grid.size() != static_cast<std::size_t>(width * height) ||
    polygon.size() < 3U)
  {
    return true;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto &point : polygon) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  const int min_cell_x = static_cast<int>(std::floor((min_x - origin_x) / resolution));
  const int min_cell_y = static_cast<int>(std::floor((min_y - origin_y) / resolution));
  const int max_cell_x = static_cast<int>(std::floor((max_x - origin_x) / resolution));
  const int max_cell_y = static_cast<int>(std::floor((max_y - origin_y) / resolution));

  for (int grid_y = min_cell_y; grid_y <= max_cell_y; ++grid_y) {
    for (int grid_x = min_cell_x; grid_x <= max_cell_x; ++grid_x) {
      if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
        return true;
      }

      const int8_t cell_value = occupancy_grid[static_cast<std::size_t>((grid_y * width) + grid_x)];
      const bool occupied = cell_value == -1 ? !allow_unknown : cell_value >= obstacle_threshold;
      if (!occupied) {
        continue;
      }

      const double cell_min_x = origin_x + (static_cast<double>(grid_x) * resolution);
      const double cell_min_y = origin_y + (static_cast<double>(grid_y) * resolution);
      const double cell_max_x = cell_min_x + resolution;
      const double cell_max_y = cell_min_y + resolution;
      if (polygon_intersects_cell(polygon, cell_min_x, cell_min_y, cell_max_x, cell_max_y)) {
        return true;
      }
    }
  }

  return false;
}

/// @brief Transform a robot footprint pose and test it against an occupancy grid.
inline bool footprint_pose_collides(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const double resolution,
  const double origin_x,
  const double origin_y,
  const FootprintPolygon &footprint,
  const double pose_x,
  const double pose_y,
  const double pose_yaw,
  const int obstacle_threshold,
  const bool allow_unknown)
{
  return polygon_collides_with_grid(
    occupancy_grid,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    transform_footprint(footprint, pose_x, pose_y, pose_yaw),
    obstacle_threshold,
    allow_unknown);
}

}  // namespace amr::geometry

#endif  // AMR_GEOMETRY__FOOTPRINT_HPP_
