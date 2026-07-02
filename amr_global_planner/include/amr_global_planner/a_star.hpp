#ifndef AMR_GLOBAL_PLANNER__A_STAR_HPP_
#define AMR_GLOBAL_PLANNER__A_STAR_HPP_

/**
 * @file a_star.hpp
 * @brief Grid A* planner used by the AMR global planner lifecycle node.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "amr_geometry/footprint.hpp"

namespace amr::planner::global
{

/// @brief Integer grid coordinate in occupancy costmap space.
struct GridCell
{
  /// @brief Column index.
  int x;
  /// @brief Row index.
  int y;

  /// @brief Compare two grid cells by row and column.
  bool operator==(const GridCell &other) const;
};

/// @brief Result object returned by AStarPlanner::plan().
struct AStarPlanResult
{
  /// @brief True when a valid grid path was found.
  bool success;
  /// @brief Ordered grid path from start to goal when success is true.
  std::vector<GridCell> path;
  /// @brief Human-readable planning result or failure reason.
  std::string message;
};

/// @brief Allowed neighborhood connectivity for grid expansion.
enum class AStarConnectivity
{
  /// @brief Four-connected grid expansion.
  Four = 4,
  /// @brief Eight-connected grid expansion.
  Eight = 8
};

/// @brief Footprint-aware A* planner for occupancy-grid global planning.
class AStarPlanner final
{
public:
  /// @brief Shared pointer alias for planner ownership.
  using SharedPtr = std::shared_ptr<AStarPlanner>;
  /// @brief Unique pointer alias for planner ownership.
  using UniquePtr = std::unique_ptr<AStarPlanner>;

  /// @brief Construct an A* planner with cost and connectivity parameters.
  explicit AStarPlanner(
    int obstacle_threshold = 50,
    bool allow_unknown = false,
    AStarConnectivity connectivity = AStarConnectivity::Eight,
    double turn_penalty = 0.0,
    bool prevent_corner_cutting = true,
    double start_row_hold_penalty = 0.0,
    int goal_row_align_distance_cells = 0,
    double goal_row_align_penalty = 0.0);

  /// @brief Destroy the planner.
  virtual ~AStarPlanner();

  /// @brief Configure the footprint collision model used during planning.
  void set_collision_model(
    amr::geometry::FootprintPolygon footprint,
    double resolution,
    double origin_x,
    double origin_y);

  /// @brief Plan a path between start and goal cells on an occupancy grid.
  AStarPlanResult plan(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    const GridCell &start,
    const GridCell &goal,
    double start_yaw = 0.0,
    double goal_yaw = 0.0) const;

private:
  /// @brief Per-cell state stored during A* expansion.
  struct Node
  {
    /// @brief Accumulated cost from the start cell.
    double g_cost;
    /// @brief Heuristic cost to the goal cell.
    double h_cost;
    /// @brief Parent node index for path reconstruction.
    int parent_index;
    /// @brief True once the node has been inserted into the open set.
    bool opened;
    /// @brief True once the node has been permanently expanded.
    bool closed;
  };

  /// @brief Return true when a cell is inside the provided grid bounds.
  bool is_within_bounds(const GridCell &cell, int width, int height) const;
  /// @brief Return true when a cell collides with map occupancy or footprint constraints.
  bool is_occupied(
    const std::vector<int8_t> &occupancy_grid,
    int height,
    int width,
    const GridCell &cell,
    double yaw) const;
  /// @brief Return true when a diagonal step would cut through blocked adjacent cells.
  bool is_diagonal_move_blocked(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    const GridCell &current,
    const GridCell &next) const;
  /// @brief Convert a grid cell into a row-major vector index.
  int to_index(const GridCell &cell, int width) const;
  /// @brief Estimate remaining travel cost between two cells.
  double heuristic(const GridCell &from, const GridCell &to) const;
  /// @brief Compute extra cost for changing travel direction.
  double turn_penalty(
    const GridCell &previous,
    const GridCell &current,
    const GridCell &next) const;
  /// @brief Compute extra cost for leaving a desired start-goal row corridor.
  double row_bias_penalty(
    const GridCell &next,
    const GridCell &start,
    const GridCell &goal) const;
  /// @brief Return neighboring cells according to configured connectivity.
  std::vector<GridCell> get_neighbors(const GridCell &cell) const;

  int obstacle_threshold_;
  bool allow_unknown_;
  double turn_penalty_;
  AStarConnectivity connectivity_;
  bool prevent_corner_cutting_;
  double start_row_hold_penalty_;
  int goal_row_align_distance_cells_;
  double goal_row_align_penalty_;
  amr::geometry::FootprintPolygon footprint_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
};

}  // namespace amr::planner::global

#endif  // AMR_GLOBAL_PLANNER__A_STAR_HPP_
