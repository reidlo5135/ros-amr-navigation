/**
 * @file occupancy_grid_view.cpp
 * @brief Implementation of OccupancyGrid classification helpers.
 */

#include "amr_spatial_segmenter/occupancy_grid_view.hpp"

#include <cmath>

namespace amr::spatial_segmenter
{

/// @copydoc OccupancyGridView::OccupancyGridView
OccupancyGridView::OccupancyGridView(
  const nav_msgs::msg::OccupancyGrid & grid,
  const int free_threshold,
  const int occupied_threshold)
: grid_(grid)
{
  const auto width_value = static_cast<std::size_t>(grid_.info.width);
  const auto height_value = static_cast<std::size_t>(grid_.info.height);
  valid_ = grid_.info.width > 0U &&
    grid_.info.height > 0U &&
    grid_.info.resolution > 0.0F &&
    grid_.data.size() == width_value * height_value;
  if (valid_) {
    classify(free_threshold, occupied_threshold);
  }
}

/// @copydoc OccupancyGridView::valid
bool OccupancyGridView::valid() const
{
  return valid_;
}

/// @copydoc OccupancyGridView::width
int OccupancyGridView::width() const
{
  return static_cast<int>(grid_.info.width);
}

/// @copydoc OccupancyGridView::height
int OccupancyGridView::height() const
{
  return static_cast<int>(grid_.info.height);
}

/// @copydoc OccupancyGridView::resolution
double OccupancyGridView::resolution() const
{
  return grid_.info.resolution;
}

/// @copydoc OccupancyGridView::cell_count
int OccupancyGridView::cell_count() const
{
  return static_cast<int>(states_.size());
}

/// @copydoc OccupancyGridView::in_bounds
bool OccupancyGridView::in_bounds(const int x, const int y) const
{
  return x >= 0 && y >= 0 && x < width() && y < height();
}

/// @copydoc OccupancyGridView::to_index
int OccupancyGridView::to_index(const int x, const int y) const
{
  return (y * width()) + x;
}

/// @copydoc OccupancyGridView::to_grid
GridIndex OccupancyGridView::to_grid(const int index) const
{
  GridIndex grid_index;
  if (width() <= 0) {
    return grid_index;
  }
  grid_index.x = index % width();
  grid_index.y = index / width();
  return grid_index;
}

/// @copydoc OccupancyGridView::state
CellState OccupancyGridView::state(const int index) const
{
  if (index < 0 || index >= static_cast<int>(states_.size())) {
    return CellState::Unknown;
  }
  return states_[index];
}

/// @copydoc OccupancyGridView::is_free
bool OccupancyGridView::is_free(const int index) const
{
  return state(index) == CellState::Free;
}

/// @copydoc OccupancyGridView::is_unknown
bool OccupancyGridView::is_unknown(const int index) const
{
  return state(index) == CellState::Unknown;
}

/// @copydoc OccupancyGridView::is_occupied
bool OccupancyGridView::is_occupied(const int index) const
{
  return state(index) == CellState::Occupied || state(index) == CellState::Ambiguous;
}

/// @copydoc OccupancyGridView::is_raw_occupied
bool OccupancyGridView::is_raw_occupied(const int index) const
{
  return state(index) == CellState::Occupied;
}

/// @copydoc OccupancyGridView::is_ambiguous
bool OccupancyGridView::is_ambiguous(const int index) const
{
  return state(index) == CellState::Ambiguous;
}

/// @copydoc OccupancyGridView::raw_occupied_mask
std::vector<bool> OccupancyGridView::raw_occupied_mask() const
{
  std::vector<bool> mask(states_.size(), false);
  for (std::size_t i = 0; i < states_.size(); ++i) {
    mask[i] = states_[i] == CellState::Occupied;
  }
  return mask;
}

/// @copydoc OccupancyGridView::raw_free_mask
std::vector<bool> OccupancyGridView::raw_free_mask() const
{
  std::vector<bool> mask(states_.size(), false);
  for (std::size_t i = 0; i < states_.size(); ++i) {
    mask[i] = states_[i] == CellState::Free;
  }
  return mask;
}

/// @copydoc OccupancyGridView::blocked_mask
std::vector<bool> OccupancyGridView::blocked_mask() const
{
  std::vector<bool> mask(states_.size(), true);
  for (std::size_t i = 0; i < states_.size(); ++i) {
    mask[i] = states_[i] != CellState::Free;
  }
  return mask;
}

/// @copydoc OccupancyGridView::cell_center
geometry_msgs::msg::Point OccupancyGridView::cell_center(const int index) const
{
  const GridIndex grid_index = to_grid(index);
  return grid_to_world(
    static_cast<double>(grid_index.x) + 0.5,
    static_cast<double>(grid_index.y) + 0.5);
}

/// @copydoc OccupancyGridView::grid_corner
geometry_msgs::msg::Point OccupancyGridView::grid_corner(const int x, const int y) const
{
  return grid_to_world(static_cast<double>(x), static_cast<double>(y));
}

/// @brief Classify raw occupancy values into unknown/free/occupied states.
void OccupancyGridView::classify(const int free_threshold, const int occupied_threshold)
{
  states_.clear();
  states_.reserve(grid_.data.size());
  for (const int8_t raw_value : grid_.data) {
    const int value = static_cast<int>(raw_value);
    if (value < 0) {
      states_.push_back(CellState::Unknown);
    } else if (value <= free_threshold) {
      states_.push_back(CellState::Free);
    } else if (value >= occupied_threshold) {
      states_.push_back(CellState::Occupied);
    } else {
      states_.push_back(CellState::Ambiguous);
    }
  }
}

/// @brief Convert grid coordinates in cells to world coordinates.
geometry_msgs::msg::Point OccupancyGridView::grid_to_world(
  const double grid_x,
  const double grid_y) const
{
  const double local_x = grid_x * resolution();
  const double local_y = grid_y * resolution();
  const auto & origin = grid_.info.origin;
  const double yaw = 2.0 * std::atan2(origin.orientation.z, origin.orientation.w);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  geometry_msgs::msg::Point point;
  point.x = origin.position.x + (c * local_x) - (s * local_y);
  point.y = origin.position.y + (s * local_x) + (c * local_y);
  point.z = origin.position.z;
  return point;
}

}  // namespace amr::spatial_segmenter
