#include "amr_global_planner/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace amr_global_planner::planner
{

namespace
{

constexpr int kUnknownCellValue = -1;

struct OpenSetEntry
{
  int index;
  double f_cost;
};

struct OpenSetEntryCompare
{
  bool operator()(const OpenSetEntry & lhs, const OpenSetEntry & rhs) const
  {
    return lhs.f_cost > rhs.f_cost;
  }
};

}  // namespace

bool GridCell::operator==(const GridCell & other) const
{
  return this->x == other.x && this->y == other.y;
}

AStarPlanner::AStarPlanner(
  const int obstacle_threshold,
  const bool allow_unknown,
  const AStarConnectivity connectivity,
  const double turn_penalty,
  const bool prevent_corner_cutting)
: obstacle_threshold_(obstacle_threshold),
  allow_unknown_(allow_unknown),
  turn_penalty_(turn_penalty),
  connectivity_(connectivity),
  prevent_corner_cutting_(prevent_corner_cutting)
{
}

AStarPlanner::~AStarPlanner() = default;

AStarPlanResult AStarPlanner::plan(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  const GridCell & start,
  const GridCell & goal) const
{
  if (width <= 0 || height <= 0) {
    return {false, {}, "Map size must be positive"};
  }

  if (occupancy_grid.size() != static_cast<std::size_t>(width * height)) {
    return {false, {}, "Occupancy grid size does not match width and height"};
  }

  if (
    !this->is_within_bounds(start, width, height) ||
    !this->is_within_bounds(goal, width, height))
  {
    return {false, {}, "Start or goal is outside map bounds"};
  }

  if (this->is_occupied(occupancy_grid, width, start)) {
    return {false, {}, "Start cell is occupied"};
  }

  if (this->is_occupied(occupancy_grid, width, goal)) {
    return {false, {}, "Goal cell is occupied"};
  }

  if (start == goal) {
    return {true, {start}, "Start and goal are identical"};
  }

  std::vector<Node> nodes(
    static_cast<std::size_t>(width * height),
    {std::numeric_limits<double>::infinity(), 0.0, -1, false, false});
  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, OpenSetEntryCompare> open_set;

  const int start_index = this->to_index(start, width);
  const int goal_index = this->to_index(goal, width);

  nodes[static_cast<std::size_t>(start_index)].g_cost = 0.0;
  nodes[static_cast<std::size_t>(start_index)].h_cost = this->heuristic(start, goal);
  nodes[static_cast<std::size_t>(start_index)].opened = true;
  open_set.push(
    {
      start_index,
      nodes[static_cast<std::size_t>(start_index)].g_cost +
      nodes[static_cast<std::size_t>(start_index)].h_cost
    });

  while (!open_set.empty()) {
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    Node & current_node = nodes[static_cast<std::size_t>(current_entry.index)];
    if (current_node.closed) {
      continue;
    }

    current_node.closed = true;
    if (current_entry.index == goal_index) {
      std::vector<GridCell> path;
      int path_index = goal_index;
      while (path_index >= 0) {
        path.push_back({path_index % width, path_index / width});
        path_index = nodes[static_cast<std::size_t>(path_index)].parent_index;
      }

      std::reverse(path.begin(), path.end());
      return {true, path, "Path found"};
    }

    const GridCell current_cell{current_entry.index % width, current_entry.index / width};
    for (const auto & neighbor : this->get_neighbors(current_cell)) {
      if (
        !this->is_within_bounds(neighbor, width, height) ||
        this->is_occupied(occupancy_grid, width, neighbor) ||
        this->is_diagonal_move_blocked(occupancy_grid, width, height, current_cell, neighbor))
      {
        continue;
      }

      const int neighbor_index = this->to_index(neighbor, width);
      Node & neighbor_node = nodes[static_cast<std::size_t>(neighbor_index)];
      if (neighbor_node.closed) {
        continue;
      }

      const bool is_diagonal = neighbor.x != current_cell.x && neighbor.y != current_cell.y;
      double tentative_g_cost = current_node.g_cost + (is_diagonal ? std::sqrt(2.0) : 1.0);
      if (current_node.parent_index >= 0) {
        const GridCell previous_cell{
          current_node.parent_index % width,
          current_node.parent_index / width};
        tentative_g_cost += this->turn_penalty(previous_cell, current_cell, neighbor);
      }

      if (!neighbor_node.opened || tentative_g_cost < neighbor_node.g_cost) {
        neighbor_node.g_cost = tentative_g_cost;
        neighbor_node.h_cost = this->heuristic(neighbor, goal);
        neighbor_node.parent_index = current_entry.index;
        neighbor_node.opened = true;
        open_set.push(
          {
            neighbor_index,
            neighbor_node.g_cost + neighbor_node.h_cost
          });
      }
    }
  }

  return {false, {}, "No path found"};
}

bool AStarPlanner::is_within_bounds(const GridCell & cell, const int width, const int height) const
{
  return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
}

bool AStarPlanner::is_occupied(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const GridCell & cell) const
{
  const int cell_value = occupancy_grid[static_cast<std::size_t>(this->to_index(cell, width))];
  if (cell_value == kUnknownCellValue) {
    return !this->allow_unknown_;
  }

  return cell_value >= this->obstacle_threshold_;
}

bool AStarPlanner::is_diagonal_move_blocked(
  const std::vector<int8_t> & occupancy_grid,
  const int width,
  const int height,
  const GridCell & current,
  const GridCell & next) const
{
  if (!this->prevent_corner_cutting_) {
    return false;
  }

  const bool is_diagonal = current.x != next.x && current.y != next.y;
  if (!is_diagonal) {
    return false;
  }

  const GridCell horizontal_neighbor{next.x, current.y};
  const GridCell vertical_neighbor{current.x, next.y};

  if (
    !this->is_within_bounds(horizontal_neighbor, width, height) ||
    !this->is_within_bounds(vertical_neighbor, width, height))
  {
    return true;
  }

  return
    this->is_occupied(occupancy_grid, width, horizontal_neighbor) ||
    this->is_occupied(occupancy_grid, width, vertical_neighbor);
}

int AStarPlanner::to_index(const GridCell & cell, const int width) const
{
  return cell.y * width + cell.x;
}

double AStarPlanner::heuristic(const GridCell & from, const GridCell & to) const
{
  const double dx = std::abs(from.x - to.x);
  const double dy = std::abs(from.y - to.y);

  if (this->connectivity_ == AStarConnectivity::Eight) {
    const double min_delta = std::min(dx, dy);
    const double max_delta = std::max(dx, dy);
    return (min_delta * std::sqrt(2.0)) + (max_delta - min_delta);
  }

  return dx + dy;
}

double AStarPlanner::turn_penalty(
  const GridCell & previous,
  const GridCell & current,
  const GridCell & next) const
{
  const int previous_dx = current.x - previous.x;
  const int previous_dy = current.y - previous.y;
  const int next_dx = next.x - current.x;
  const int next_dy = next.y - current.y;

  if (previous_dx == next_dx && previous_dy == next_dy) {
    return 0.0;
  }

  return this->turn_penalty_;
}

std::vector<GridCell> AStarPlanner::get_neighbors(const GridCell & cell) const
{
  std::vector<GridCell> neighbors{
    {cell.x + 1, cell.y},
    {cell.x - 1, cell.y},
    {cell.x, cell.y + 1},
    {cell.x, cell.y - 1}
  };

  if (this->connectivity_ == AStarConnectivity::Eight) {
    neighbors.push_back({cell.x + 1, cell.y + 1});
    neighbors.push_back({cell.x + 1, cell.y - 1});
    neighbors.push_back({cell.x - 1, cell.y + 1});
    neighbors.push_back({cell.x - 1, cell.y - 1});
  }

  return neighbors;
}

}  // namespace amr_global_planner::planner
