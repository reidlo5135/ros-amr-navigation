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

#include "amr_geometry/footprint.hpp"

namespace amr::planner::global
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

  virtual ~AStarPlanner();

  void set_collision_model(
    amr::geometry::FootprintPolygon footprint,
    double resolution,
    double origin_x,
    double origin_y);

  AStarPlanResult plan(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    const GridCell & start,
    const GridCell & goal,
    double start_yaw = 0.0,
    double goal_yaw = 0.0) const;

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
    int height,
    int width,
    const GridCell & cell,
    double yaw) const;
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
  amr::geometry::FootprintPolygon footprint_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
};

}  // namespace amr::planner::global

#endif  // AMR_GLOBAL_PLANNER__A_STAR_HPP_
