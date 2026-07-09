#ifndef AMR_SPATIAL_SEGMENTER__OCCUPANCY_GRID_VIEW_HPP_
#define AMR_SPATIAL_SEGMENTER__OCCUPANCY_GRID_VIEW_HPP_

/**
 * @file occupancy_grid_view.hpp
 * @brief Lightweight OccupancyGrid validation, classification, and coordinate conversion.
 */

#include <vector>

#include "amr_spatial_segmenter/grid_types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace amr::spatial_segmenter
{

/// @brief Read-only classified view over a nav_msgs OccupancyGrid.
class OccupancyGridView
{
public:
  /// @brief Construct an invalid view.
  OccupancyGridView() = default;
  /// @brief Construct and classify a grid.
  OccupancyGridView(
    const nav_msgs::msg::OccupancyGrid & grid,
    int free_threshold,
    int occupied_threshold);

  /// @brief Return true when dimensions, resolution, and data size are usable.
  bool valid() const;
  /// @brief Grid width in cells.
  int width() const;
  /// @brief Grid height in cells.
  int height() const;
  /// @brief Cell resolution in meters.
  double resolution() const;
  /// @brief Total number of cells.
  int cell_count() const;
  /// @brief Return true when the coordinate is inside the grid.
  bool in_bounds(int x, int y) const;
  /// @brief Convert coordinate to row-major index.
  int to_index(int x, int y) const;
  /// @brief Convert row-major index to coordinate.
  GridIndex to_grid(int index) const;
  /// @brief Return the classified state at a row-major index.
  CellState state(int index) const;
  /// @brief Return true when a cell is conservatively free.
  bool is_free(int index) const;
  /// @brief Return true when a cell is unknown.
  bool is_unknown(int index) const;
  /// @brief Return true when a cell is occupied or ambiguous.
  bool is_occupied(int index) const;
  /// @brief Return true when a cell is a raw SLAM occupied wall cell.
  bool is_raw_occupied(int index) const;
  /// @brief Return true when a cell is neither free nor definitely occupied.
  bool is_ambiguous(int index) const;
  /// @brief Return a mask where raw occupied wall cells are true.
  std::vector<bool> raw_occupied_mask() const;
  /// @brief Return a mask where raw free cells are true.
  std::vector<bool> raw_free_mask() const;
  /// @brief Return a mask where occupied, ambiguous, and unknown cells are true.
  std::vector<bool> blocked_mask() const;
  /// @brief Convert a cell center to world coordinates.
  geometry_msgs::msg::Point cell_center(int index) const;
  /// @brief Convert a cell corner to world coordinates.
  geometry_msgs::msg::Point grid_corner(int x, int y) const;

private:
  void classify(int free_threshold, int occupied_threshold);
  geometry_msgs::msg::Point grid_to_world(double grid_x, double grid_y) const;

  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<CellState> states_;
  bool valid_{false};
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__OCCUPANCY_GRID_VIEW_HPP_
