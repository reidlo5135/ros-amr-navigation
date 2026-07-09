/**
 * @file connected_components.cpp
 * @brief Implementation of connected component extraction.
 */

#include "amr_spatial_segmenter/connected_components.hpp"

#include <algorithm>
#include <queue>

namespace amr::spatial_segmenter
{

/// @copydoc ConnectedComponents::extract
std::vector<Component> ConnectedComponents::extract(
  const std::vector<bool> & mask,
  const int width,
  const int height,
  const int min_cells)
{
  std::vector<Component> components;
  if (width <= 0 || height <= 0 || mask.size() != static_cast<std::size_t>(width * height)) {
    return components;
  }

  std::vector<bool> visited(mask.size(), false);
  std::queue<int> queue;
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int seed = 0; seed < width * height; ++seed) {
    if (!mask[static_cast<std::size_t>(seed)] || visited[static_cast<std::size_t>(seed)]) {
      continue;
    }

    Component component;
    const int seed_x = seed % width;
    const int seed_y = seed / width;
    component.min_x = seed_x;
    component.max_x = seed_x;
    component.min_y = seed_y;
    component.max_y = seed_y;
    visited[static_cast<std::size_t>(seed)] = true;
    queue.push(seed);

    while (!queue.empty()) {
      const int current = queue.front();
      queue.pop();
      component.cells.push_back(current);
      const int x = current % width;
      const int y = current / width;
      component.min_x = std::min(component.min_x, x);
      component.max_x = std::max(component.max_x, x);
      component.min_y = std::min(component.min_y, y);
      component.max_y = std::max(component.max_y, y);

      for (int direction = 0; direction < 8; ++direction) {
        const int nx = x + dx[direction];
        const int ny = y + dy[direction];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const int next = (ny * width) + nx;
        if (visited[static_cast<std::size_t>(next)] || !mask[static_cast<std::size_t>(next)]) {
          continue;
        }
        visited[static_cast<std::size_t>(next)] = true;
        queue.push(next);
      }
    }

    if (static_cast<int>(component.cells.size()) >= min_cells) {
      components.push_back(component);
    }
  }

  return components;
}

}  // namespace amr::spatial_segmenter
