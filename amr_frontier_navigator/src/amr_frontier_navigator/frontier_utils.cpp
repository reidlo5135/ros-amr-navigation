/**
 * @file frontier_utils.cpp
 * @brief OccupancyGrid coordinate conversion and classification helpers.
 */

#include "amr_frontier_navigator/frontier_utils.hpp"

#include <algorithm>
#include <cmath>

namespace amr::frontier_navigation
{

namespace
{

constexpr int8_t kUnknownCellValue = -1;

}  // namespace

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  return std::atan2(
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)),
    1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z))));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

double pose_distance(
  const geometry_msgs::msg::PoseStamped &lhs,
  const geometry_msgs::msg::PoseStamped &rhs)
{
  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

double path_length(const nav_msgs::msg::Path &path)
{
  double length = 0.0;
  if (path.poses.size() < 2U) {
    return length;
  }
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    length += pose_distance(path.poses[index - 1U], path.poses[index]);
  }
  return length;
}

OccupancyGridView::OccupancyGridView(
  const nav_msgs::msg::OccupancyGrid &map,
  const int free_threshold,
  const int occupied_threshold)
: map_(&map),
  free_threshold_(free_threshold),
  occupied_threshold_(occupied_threshold)
{
}

bool OccupancyGridView::valid() const
{
  if (!map_) {
    return false;
  }
  const auto expected_size =
    static_cast<std::size_t>(map_->info.width) * static_cast<std::size_t>(map_->info.height);
  return
    map_->info.width > 0U &&
    map_->info.height > 0U &&
    map_->info.resolution > 0.0F &&
    map_->data.size() == expected_size;
}

int OccupancyGridView::width() const
{
  return valid() ? static_cast<int>(map_->info.width) : 0;
}

int OccupancyGridView::height() const
{
  return valid() ? static_cast<int>(map_->info.height) : 0;
}

double OccupancyGridView::resolution() const
{
  return valid() ? static_cast<double>(map_->info.resolution) : 0.0;
}

bool OccupancyGridView::in_bounds(const GridCell &cell) const
{
  return
    valid() &&
    cell.x >= 0 &&
    cell.y >= 0 &&
    cell.x < static_cast<int>(map_->info.width) &&
    cell.y < static_cast<int>(map_->info.height);
}

GridCell OccupancyGridView::world_to_grid_unbounded(
  const double world_x,
  const double world_y) const
{
  if (!valid()) {
    return {};
  }

  const double yaw = yaw_from_quaternion(map_->info.origin.orientation);
  const double dx = world_x - map_->info.origin.position.x;
  const double dy = world_y - map_->info.origin.position.y;
  const double local_x = (std::cos(yaw) * dx) + (std::sin(yaw) * dy);
  const double local_y = (-std::sin(yaw) * dx) + (std::cos(yaw) * dy);
  const double resolution_m = resolution();

  return GridCell{
    static_cast<int>(std::floor(local_x / resolution_m)),
    static_cast<int>(std::floor(local_y / resolution_m))};
}

bool OccupancyGridView::world_to_grid(
  const double world_x,
  const double world_y,
  GridCell &cell) const
{
  cell = world_to_grid_unbounded(world_x, world_y);
  return in_bounds(cell);
}

geometry_msgs::msg::PoseStamped OccupancyGridView::grid_to_pose(
  const GridCell &cell,
  const std::string &fallback_frame_id,
  const double yaw) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!valid()) {
    pose.header.frame_id = fallback_frame_id;
    pose.pose.orientation = quaternion_from_yaw(yaw);
    return pose;
  }

  const double resolution_m = resolution();
  const double local_x = (static_cast<double>(cell.x) + 0.5) * resolution_m;
  const double local_y = (static_cast<double>(cell.y) + 0.5) * resolution_m;
  const double origin_yaw = yaw_from_quaternion(map_->info.origin.orientation);

  pose.header = map_->header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = fallback_frame_id;
  }
  pose.pose.position.x =
    map_->info.origin.position.x +
    (std::cos(origin_yaw) * local_x) -
    (std::sin(origin_yaw) * local_y);
  pose.pose.position.y =
    map_->info.origin.position.y +
    (std::sin(origin_yaw) * local_x) +
    (std::cos(origin_yaw) * local_y);
  pose.pose.orientation = quaternion_from_yaw(yaw);
  return pose;
}

CellState OccupancyGridView::classify(const GridCell &cell) const
{
  if (!in_bounds(cell)) {
    return CellState::Unknown;
  }

  const int index = (cell.y * width()) + cell.x;
  const int value = map_->data[static_cast<std::size_t>(index)];
  if (value == kUnknownCellValue) {
    return CellState::Unknown;
  }
  if (value >= 0 && value <= free_threshold_) {
    return CellState::Free;
  }
  if (value >= occupied_threshold_) {
    return CellState::Occupied;
  }
  return CellState::Occupied;
}

bool OccupancyGridView::is_free(const GridCell &cell) const
{
  return classify(cell) == CellState::Free;
}

bool OccupancyGridView::is_unknown(const GridCell &cell) const
{
  return classify(cell) == CellState::Unknown;
}

bool OccupancyGridView::has_obstacle_clearance(
  const GridCell &cell,
  const double clearance_m) const
{
  if (!is_free(cell)) {
    return false;
  }
  if (clearance_m <= 0.0 || resolution() <= 0.0) {
    return true;
  }

  const int radius_cells = static_cast<int>(std::ceil(clearance_m / resolution()));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance_m =
        std::sqrt(static_cast<double>((dx * dx) + (dy * dy))) * resolution();
      if (distance_m > clearance_m) {
        continue;
      }
      const GridCell neighbor{cell.x + dx, cell.y + dy};
      if (!in_bounds(neighbor)) {
        continue;
      }
      if (classify(neighbor) == CellState::Occupied) {
        return false;
      }
    }
  }
  return true;
}

bool OccupancyGridView::has_unknown_neighbor(
  const GridCell &cell,
  const int radius_cells) const
{
  const int radius = std::max(1, radius_cells);
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const GridCell neighbor{cell.x + dx, cell.y + dy};
      if (!in_bounds(neighbor) || is_unknown(neighbor)) {
        return true;
      }
    }
  }
  return false;
}

double OccupancyGridView::distance_to_cell_center(
  const GridCell &cell,
  const double world_x,
  const double world_y) const
{
  const auto pose = grid_to_pose(cell, "", 0.0);
  const double dx = pose.pose.position.x - world_x;
  const double dy = pose.pose.position.y - world_y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr::frontier_navigation
