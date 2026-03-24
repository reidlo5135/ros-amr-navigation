#ifndef AMR_GLOBAL_PLANNER__A_STAR_HPP_
#define AMR_GLOBAL_PLANNER__A_STAR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace amr_global_planner::planner
{

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell & other) const;
};

struct AStarPlanResult
{
  bool success;
  std::vector<GridCell> path;
  std::string message;
};

enum class AStarConnectivity
{
  Four = 4,
  Eight = 8
};

class AStarPlanner final
{
public:
  using SharedPtr = std::shared_ptr<AStarPlanner>;
  using UniquePtr = std::unique_ptr<AStarPlanner>;

  explicit AStarPlanner(
    int obstacle_threshold = 50,
    bool allow_unknown = false,
    AStarConnectivity connectivity = AStarConnectivity::Eight,
    double turn_penalty = 0.0,
    bool prevent_corner_cutting = true);

  ~AStarPlanner();

  AStarPlanResult plan(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    const GridCell & start,
    const GridCell & goal) const;

private:
  struct Node
  {
    double g_cost;
    double h_cost;
    int parent_index;
    bool opened;
    bool closed;
  };

  bool is_within_bounds(const GridCell & cell, int width, int height) const;
  bool is_occupied(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    const GridCell & cell) const;
  bool is_diagonal_move_blocked(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    const GridCell & current,
    const GridCell & next) const;
  int to_index(const GridCell & cell, int width) const;
  double heuristic(const GridCell & from, const GridCell & to) const;
  double turn_penalty(
    const GridCell & previous,
    const GridCell & current,
    const GridCell & next) const;
  std::vector<GridCell> get_neighbors(const GridCell & cell) const;

  int obstacle_threshold_;
  bool allow_unknown_;
  double turn_penalty_;
  AStarConnectivity connectivity_;
  bool prevent_corner_cutting_;
};

}  // namespace amr_global_planner::planner

#endif  // AMR_GLOBAL_PLANNER__A_STAR_HPP_
