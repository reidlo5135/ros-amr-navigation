#ifndef AMR_FRONTIER_NAVIGATOR__FRONTIER_UTILS_HPP_
#define AMR_FRONTIER_NAVIGATOR__FRONTIER_UTILS_HPP_

/**
 * @file frontier_utils.hpp
 * @brief OccupancyGrid coordinate conversion and classification helpers.
 */

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace amr::frontier_navigation
{

/// @brief Integer cell coordinate in an occupancy grid.
struct GridCell
{
  /// @brief Column index.
  int x{0};
  /// @brief Row index.
  int y{0};
};

/// @brief Coarse traversability state used by frontier goal resolution.
enum class CellState
{
  /// @brief Outside the map or explicitly unknown.
  Unknown,
  /// @brief Known traversable free cell.
  Free,
  /// @brief Known occupied or non-traversable cell.
  Occupied
};

/// @brief Extract planar yaw from a quaternion.
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quaternion);

/// @brief Build a planar quaternion from yaw.
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);

/// @brief Return planar Euclidean distance between two stamped poses.
double pose_distance(
  const geometry_msgs::msg::PoseStamped &lhs,
  const geometry_msgs::msg::PoseStamped &rhs);

/// @brief Estimate total path length in meters.
double path_length(const nav_msgs::msg::Path &path);

/// @brief Immutable view over a nav_msgs OccupancyGrid with rotated-origin transforms.
class OccupancyGridView
{
public:
  /// @brief Construct an empty invalid view.
  OccupancyGridView() = default;

  /// @brief Construct a view over a map message and classification thresholds.
  OccupancyGridView(
    const nav_msgs::msg::OccupancyGrid &map,
    int free_threshold,
    int occupied_threshold);

  /// @brief Return true when dimensions, resolution, and data size are valid.
  bool valid() const;

  /// @brief Return map width in cells.
  int width() const;

  /// @brief Return map height in cells.
  int height() const;

  /// @brief Return map resolution in meters/cell.
  double resolution() const;

  /// @brief Return true when a cell coordinate is inside map bounds.
  bool in_bounds(const GridCell &cell) const;

  /// @brief Convert world coordinates into an in-bounds grid cell.
  bool world_to_grid(double world_x, double world_y, GridCell &cell) const;

  /// @brief Convert world coordinates into a grid cell without clamping to map bounds.
  GridCell world_to_grid_unbounded(double world_x, double world_y) const;

  /// @brief Convert a grid cell center into a stamped world pose.
  geometry_msgs::msg::PoseStamped grid_to_pose(
    const GridCell &cell,
    const std::string &fallback_frame_id,
    double yaw) const;

  /// @brief Classify a cell as unknown, free, or occupied/non-traversable.
  CellState classify(const GridCell &cell) const;

  /// @brief Return true when the cell is known and traversable.
  bool is_free(const GridCell &cell) const;

  /// @brief Return true when the cell is unknown or outside the map.
  bool is_unknown(const GridCell &cell) const;

  /// @brief Return true when the cell has no occupied cell within the given radius.
  bool has_obstacle_clearance(const GridCell &cell, double clearance_m) const;

  /// @brief Return true when nearby cells contain unknown space or the map edge.
  bool has_unknown_neighbor(const GridCell &cell, int radius_cells = 1) const;

  /// @brief Return distance from a world point to the cell center.
  double distance_to_cell_center(
    const GridCell &cell,
    double world_x,
    double world_y) const;

private:
  const nav_msgs::msg::OccupancyGrid *map_{nullptr};
  int free_threshold_{25};
  int occupied_threshold_{65};
};

}  // namespace amr::frontier_navigation

#endif  // AMR_FRONTIER_NAVIGATOR__FRONTIER_UTILS_HPP_
