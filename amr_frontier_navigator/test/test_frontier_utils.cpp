#include "amr_frontier_navigator/frontier_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

nav_msgs::msg::OccupancyGrid make_map(double yaw = 0.0)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.5F;
  map.info.origin.position.x = 1.0;
  map.info.origin.position.y = 2.0;
  map.info.origin.orientation = amr::frontier_navigation::quaternion_from_yaw(yaw);
  map.data.assign(static_cast<std::size_t>(map.info.width * map.info.height), 0);
  return map;
}

}  // namespace

TEST(FrontierUtils, WorldGridRoundTripHonorsOriginYaw)
{
  auto map = make_map(M_PI / 2.0);
  amr::frontier_navigation::OccupancyGridView view(map, 25, 65);

  amr::frontier_navigation::GridCell cell;
  ASSERT_TRUE(view.world_to_grid(0.75, 2.25, cell));
  EXPECT_EQ(cell.x, 0);
  EXPECT_EQ(cell.y, 0);

  const auto pose = view.grid_to_pose(cell, "map", 0.0);
  EXPECT_NEAR(pose.pose.position.x, 0.75, 1e-6);
  EXPECT_NEAR(pose.pose.position.y, 2.25, 1e-6);
}

TEST(FrontierUtils, ClassifiesUnknownFreeOccupiedAndOutOfMap)
{
  auto map = make_map();
  map.data[1] = -1;
  map.data[2] = 80;
  amr::frontier_navigation::OccupancyGridView view(map, 25, 65);

  EXPECT_EQ(view.classify({0, 0}), amr::frontier_navigation::CellState::Free);
  EXPECT_EQ(view.classify({1, 0}), amr::frontier_navigation::CellState::Unknown);
  EXPECT_EQ(view.classify({2, 0}), amr::frontier_navigation::CellState::Occupied);
  EXPECT_EQ(view.classify({100, 100}), amr::frontier_navigation::CellState::Unknown);
}

TEST(FrontierUtils, ObstacleClearanceRejectsNearbyOccupiedCells)
{
  auto map = make_map();
  map.data[5 + (5 * static_cast<int>(map.info.width))] = 80;
  amr::frontier_navigation::OccupancyGridView view(map, 25, 65);

  EXPECT_FALSE(view.has_obstacle_clearance({4, 5}, 0.55));
  EXPECT_TRUE(view.has_obstacle_clearance({0, 0}, 0.55));
}

TEST(FrontierUtils, UnknownNeighborTreatsMapEdgeAsFrontier)
{
  auto map = make_map();
  map.data[1 + (1 * static_cast<int>(map.info.width))] = -1;
  amr::frontier_navigation::OccupancyGridView view(map, 25, 65);

  EXPECT_TRUE(view.has_unknown_neighbor({0, 0}));
  EXPECT_TRUE(view.has_unknown_neighbor({2, 1}));
  EXPECT_FALSE(view.has_unknown_neighbor({5, 5}));
}
