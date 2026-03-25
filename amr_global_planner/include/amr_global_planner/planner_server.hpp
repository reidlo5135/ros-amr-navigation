#ifndef AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
#define AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "amr_geometry/footprint.hpp"
#include "amr_global_planner/a_star.hpp"
#include "amr_msgs/srv/plan_route.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_global_planner
{

class PlannerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_plan_segment(
    const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response);
  void handle_plan_route(
    const std::shared_ptr<amr_msgs::srv::PlanRoute::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanRoute::Response> response);
  bool compute_plan_between_poses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path,
    std::string & message) const;
  bool world_to_grid(
    const geometry_msgs::msg::PoseStamped & pose,
    planner::GridCell & cell) const;
  geometry_msgs::msg::PoseStamped grid_to_world(const planner::GridCell & cell) const;
  bool is_occupied_cell(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    const planner::GridCell & cell) const;
  bool is_cell_collision(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    const planner::GridCell & cell,
    double yaw) const;
  bool find_nearest_free_cell(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    planner::GridCell & cell,
    int max_radius,
    double yaw) const;
  std::vector<planner::GridCell> simplify_grid_path(
    const std::vector<planner::GridCell> & grid_path) const;
  nav_msgs::msg::Path create_path_message(
    const std::vector<planner::GridCell> & grid_path) const;
  nav_msgs::msg::Path merge_paths(const std::vector<nav_msgs::msg::Path> & paths) const;
  void costmap_subscription_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  rclcpp::Service<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_service_;
  rclcpp::Service<amr_msgs::srv::PlanRoute>::SharedPtr plan_route_service_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr computed_plan_publisher_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
  nav_msgs::msg::Path planned_path_;
  planner::AStarPlanner::UniquePtr a_star_planner_;
  std::string costmap_topic_;
  std::string computed_plan_topic_;
  std::string plan_segment_service_name_;
  std::string plan_route_service_name_;
  int obstacle_threshold_;
  int connectivity_;
  bool allow_unknown_;
  bool simplify_path_;
  bool prevent_corner_cutting_;
  double turn_penalty_;
  int nearest_free_search_radius_cells_;
  std::vector<double> footprint_polygon_param_;
  amr_geometry::FootprintPolygon footprint_polygon_;
};

}  // namespace amr_global_planner

#endif  // AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
