/**
 * @file distance_map.cpp
 * @brief Implementation of a chamfer-style obstacle distance map.
 */

#include "amr_spatial_segmenter/distance_map.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace amr::spatial_segmenter
{

namespace
{

struct QueueEntry
{
  double distance{0.0};
  int index{0};
};

bool operator>(const QueueEntry & lhs, const QueueEntry & rhs)
{
  return lhs.distance > rhs.distance;
}

}  // namespace

/// @copydoc DistanceMap::compute
void DistanceMap::compute(
  const int width,
  const int height,
  const double resolution,
  const std::vector<bool> & blocked)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  const int cell_count = width_ * height_;
  distances_.assign(
    static_cast<std::size_t>(std::max(
      0,
      cell_count)),
    std::numeric_limits<double>::infinity());
  if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0 ||
    blocked.size() != static_cast<std::size_t>(cell_count))
  {
    return;
  }

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  for (int index = 0; index < cell_count; ++index) {
    if (blocked[static_cast<std::size_t>(index)]) {
      distances_[static_cast<std::size_t>(index)] = 0.0;
      queue.push(QueueEntry{0.0, index});
    }
  }

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double diagonal = std::sqrt(2.0) * resolution_;

  while (!queue.empty()) {
    const QueueEntry current = queue.top();
    queue.pop();
    if (current.distance > distances_[static_cast<std::size_t>(current.index)]) {
      continue;
    }
    const int x = current.index % width_;
    const int y = current.index / width_;
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + dx[direction];
      const int ny = y + dy[direction];
      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
        continue;
      }
      const int next_index = (ny * width_) + nx;
      const double step = direction < 4 ? resolution_ : diagonal;
      const double next_distance = current.distance + step;
      if (next_distance < distances_[static_cast<std::size_t>(next_index)]) {
        distances_[static_cast<std::size_t>(next_index)] = next_distance;
        queue.push(QueueEntry{next_distance, next_index});
      }
    }
  }
}

/// @copydoc DistanceMap::at
double DistanceMap::at(const int index) const
{
  if (index < 0 || index >= static_cast<int>(distances_.size())) {
    return 0.0;
  }
  return distances_[static_cast<std::size_t>(index)];
}

/// @copydoc DistanceMap::data
const std::vector<double> & DistanceMap::data() const
{
  return distances_;
}

/// @copydoc DistanceMap::valid
bool DistanceMap::valid() const
{
  return width_ > 0 && height_ > 0 && resolution_ > 0.0 &&
         distances_.size() == static_cast<std::size_t>(width_ * height_);
}

}  // namespace amr::spatial_segmenter
