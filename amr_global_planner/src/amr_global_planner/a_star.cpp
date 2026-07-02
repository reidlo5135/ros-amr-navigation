/**
 * @file a_star.cpp
 * @brief Implementation of the footprint-aware grid A* planner.
 */

#include "amr_global_planner/a_star.hpp"

namespace amr::planner::global
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
  /// @brief Order priority-queue entries by lowest total A* cost.
  bool operator()(const OpenSetEntry &lhs, const OpenSetEntry &rhs) const
  {
    return lhs.f_cost > rhs.f_cost;
  }
};

}  // namespace

/// @copydoc GridCell::operator==()
bool GridCell::operator==(const GridCell &other) const
{
  return this->x == other.x &&this->y == other.y;
}

/// @copydoc AStarPlanner::AStarPlanner()
AStarPlanner::AStarPlanner(
  const int obstacle_threshold,
  const bool allow_unknown,
  const AStarConnectivity connectivity,
  const double turn_penalty,
  const bool prevent_corner_cutting,
  const double start_row_hold_penalty,
  const int goal_row_align_distance_cells,
  const double goal_row_align_penalty)
: obstacle_threshold_(obstacle_threshold),
  allow_unknown_(allow_unknown),
  turn_penalty_(turn_penalty),
  connectivity_(connectivity),
  prevent_corner_cutting_(prevent_corner_cutting),
  start_row_hold_penalty_(start_row_hold_penalty),
  goal_row_align_distance_cells_(std::max(0, goal_row_align_distance_cells)),
  goal_row_align_penalty_(goal_row_align_penalty),
  map_resolution_(0.0),
  map_origin_x_(0.0),
  map_origin_y_(0.0)
{
}

/// @copydoc AStarPlanner::~AStarPlanner()
AStarPlanner::~AStarPlanner() = default;

/// @copydoc AStarPlanner::set_collision_model()
void AStarPlanner::set_collision_model(
  amr::geometry::FootprintPolygon footprint,
  const double resolution,
  const double origin_x,
  const double origin_y)
{
  footprint_ = std::move(footprint);
  map_resolution_ = resolution;
  map_origin_x_ = origin_x;
  map_origin_y_ = origin_y;
}

/// @copydoc AStarPlanner::plan()
AStarPlanResult AStarPlanner::plan(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const GridCell &start,
  const GridCell &goal,
  const double start_yaw,
  const double goal_yaw) const
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

  if (this->is_occupied(occupancy_grid, height, width, start, start_yaw)) {
    return {false, {}, "Start cell is occupied"};
  }

  if (this->is_occupied(occupancy_grid, height, width, goal, goal_yaw)) {
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

    Node &current_node = nodes[static_cast<std::size_t>(current_entry.index)];
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
    for (const auto &neighbor : this->get_neighbors(current_cell)) {
      if (
        !this->is_within_bounds(neighbor, width, height) ||
        this->is_occupied(
          occupancy_grid,
          height,
          width,
          neighbor,
          std::atan2(
            static_cast<double>(neighbor.y - current_cell.y),
            static_cast<double>(neighbor.x - current_cell.x))) ||
        this->is_diagonal_move_blocked(occupancy_grid, width, height, current_cell, neighbor))
      {
        continue;
      }

      const int neighbor_index = this->to_index(neighbor, width);
      Node &neighbor_node = nodes[static_cast<std::size_t>(neighbor_index)];
      if (neighbor_node.closed) {
        continue;
      }

      const bool is_diagonal = neighbor.x != current_cell.x &&neighbor.y != current_cell.y;
      double tentative_g_cost = current_node.g_cost + (is_diagonal ? std::sqrt(2.0) : 1.0);
      if (current_node.parent_index >= 0) {
        const GridCell previous_cell{
          current_node.parent_index % width,
          current_node.parent_index / width};
        tentative_g_cost += this->turn_penalty(previous_cell, current_cell, neighbor);
      }
      tentative_g_cost += this->row_bias_penalty(neighbor, start, goal);

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

/// @copydoc AStarPlanner::is_within_bounds()
bool AStarPlanner::is_within_bounds(const GridCell &cell, const int width, const int height) const
{
  return cell.x >= 0 &&cell.x < width &&cell.y >= 0 &&cell.y < height;
}

/// @copydoc AStarPlanner::is_occupied()
bool AStarPlanner::is_occupied(
  const std::vector<int8_t> &occupancy_grid,
  const int height,
  const int width,
  const GridCell &cell,
  const double yaw) const
{
  if (!footprint_.empty() &&map_resolution_ > 0.0 &&height > 0) {
    const double pose_x = map_origin_x_ + (static_cast<double>(cell.x) + 0.5) * map_resolution_;
    const double pose_y = map_origin_y_ + (static_cast<double>(cell.y) + 0.5) * map_resolution_;
    return amr::geometry::footprint_pose_collides(
      occupancy_grid,
      width,
      height,
      map_resolution_,
      map_origin_x_,
      map_origin_y_,
      footprint_,
      pose_x,
      pose_y,
      yaw,
      obstacle_threshold_,
      allow_unknown_);
  }

  const int cell_value = occupancy_grid[static_cast<std::size_t>(this->to_index(cell, width))];
  if (cell_value == kUnknownCellValue) {
    return !this->allow_unknown_;
  }

  return cell_value >= this->obstacle_threshold_;
}

/// @copydoc AStarPlanner::is_diagonal_move_blocked()
bool AStarPlanner::is_diagonal_move_blocked(
  const std::vector<int8_t> &occupancy_grid,
  const int width,
  const int height,
  const GridCell &current,
  const GridCell &next) const
{
  if (!this->prevent_corner_cutting_) {
    return false;
  }

  const bool is_diagonal = current.x != next.x &&current.y != next.y;
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
    this->is_occupied(occupancy_grid, height, width, horizontal_neighbor, 0.0) ||
    this->is_occupied(occupancy_grid, height, width, vertical_neighbor, 0.0);
}

/// @copydoc AStarPlanner::to_index()
int AStarPlanner::to_index(const GridCell &cell, const int width) const
{
  return cell.y * width + cell.x;
}

/// @copydoc AStarPlanner::heuristic()
double AStarPlanner::heuristic(const GridCell &from, const GridCell &to) const
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

/// @copydoc AStarPlanner::turn_penalty()
double AStarPlanner::turn_penalty(
  const GridCell &previous,
  const GridCell &current,
  const GridCell &next) const
{
  const int previous_dx = current.x - previous.x;
  const int previous_dy = current.y - previous.y;
  const int next_dx = next.x - current.x;
  const int next_dy = next.y - current.y;

  if (previous_dx == next_dx &&previous_dy == next_dy) {
    return 0.0;
  }

  return this->turn_penalty_;
}

/// @copydoc AStarPlanner::row_bias_penalty()
double AStarPlanner::row_bias_penalty(
  const GridCell &next,
  const GridCell &start,
  const GridCell &goal) const
{
  const int x_delta = std::abs(goal.x - start.x);
  const int y_delta = std::abs(goal.y - start.y);
  if (x_delta <= y_delta) {
    return 0.0;
  }

  double penalty = 0.0;

  const bool in_goal_align_zone =
    std::abs(goal.x - next.x) <= this->goal_row_align_distance_cells_;

  if (!in_goal_align_zone &&this->start_row_hold_penalty_ > 0.0 &&next.y != start.y) {
    penalty += this->start_row_hold_penalty_ * static_cast<double>(std::abs(next.y - start.y));
  }

  if (in_goal_align_zone &&this->goal_row_align_penalty_ > 0.0 &&next.y != goal.y) {
    penalty += this->goal_row_align_penalty_ * static_cast<double>(std::abs(next.y - goal.y));
  }

  return penalty;
}

/// @copydoc AStarPlanner::get_neighbors()
std::vector<GridCell> AStarPlanner::get_neighbors(const GridCell &cell) const
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

}  // namespace amr::planner::global
