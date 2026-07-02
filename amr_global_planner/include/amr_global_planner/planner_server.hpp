#ifndef AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
#define AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_

/**
 * @file planner_server.hpp
 * @brief Lifecycle global planner node that serves segment and route planning requests.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "amr_geometry/footprint.hpp"
#include "amr_global_planner/a_star.hpp"
#include "amr_msgs/srv/plan_route.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace amr::planner::global
{

/// @brief Lifecycle node that plans global paths on the global occupancy costmap.
class PlannerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct the global planner node and declare ROS parameters.
  explicit PlannerServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the global planner node.
  virtual ~PlannerServer() = default;

private:
  /// @brief Axis classification for straight-line path replacement.
  enum class StraightAxis
  {
    /// @brief Start and goal are not considered axis aligned.
    None,
    /// @brief Start and goal are approximately on the same map row.
    Horizontal,
    /// @brief Start and goal are approximately on the same map column.
    Vertical
  };

  /// @brief Lifecycle callback return type alias.
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure subscriptions, publishers, services, and planner state.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate path publisher and service processing.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Deactivate lifecycle publishers.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release configured resources and cached costmap state.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Handle a single start-goal global planning request.
  void handle_plan_segment(
    const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response);
  /// @brief Handle a multi-waypoint route planning request.
  void handle_plan_route(
    const std::shared_ptr<amr_msgs::srv::PlanRoute::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanRoute::Response> response);
  /// @brief Compute a path between two poses using straight-line or A* planning.
  bool compute_plan_between_poses(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    nav_msgs::msg::Path &path,
    std::string &message) const;
  /// @brief Convert a world-frame pose into a costmap grid cell.
  bool world_to_grid(
    const geometry_msgs::msg::PoseStamped &pose,
    GridCell &cell) const;
  /// @brief Convert a costmap grid cell into a world-frame pose at cell center.
  geometry_msgs::msg::PoseStamped grid_to_world(const GridCell &cell) const;
  /// @brief Return true when a cell is occupied or unknown is disallowed.
  bool is_occupied_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    const GridCell &cell) const;
  /// @brief Return true when a grid cell collides with occupancy or footprint geometry.
  bool is_cell_collision(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    const GridCell &cell,
    double yaw) const;
  /// @brief Search around a requested cell for the nearest footprint-valid free cell.
  bool find_nearest_free_cell(
    const std::vector<int8_t> &occupancy_grid,
    int width,
    int height,
    GridCell &cell,
    int max_radius,
    double yaw) const;
  /// @brief Classify whether start and goal are close enough to a straight grid axis.
  StraightAxis classify_axis_aligned_straight_candidate(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    const GridCell &start_cell,
    const GridCell &goal_cell) const;
  /// @brief Check whether a direct world-frame segment is collision free.
  bool is_straight_line_collision_free(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;
  /// @brief Check whether the robot footprint is collision free at a world pose.
  bool is_world_pose_collision_free(
    const geometry_msgs::msg::PoseStamped &pose,
    double yaw) const;
  /// @brief Build an interpolated straight path between start and goal poses.
  nav_msgs::msg::Path create_straight_path_message(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;
  /// @brief Estimate maximum grid-row deviation of a path from the start-goal row span.
  int estimate_max_row_deviation(
    const std::vector<GridCell> &grid_path,
    const GridCell &start_cell,
    const GridCell &goal_cell) const;
  /// @brief Estimate maximum lateral world-frame deviation from the start-goal line.
  double estimate_path_lateral_deviation(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) const;
  /// @brief Remove redundant collinear grid points from a path when enabled.
  std::vector<GridCell> simplify_grid_path(const std::vector<GridCell> &grid_path) const;
  /// @brief Convert a grid path into a nav_msgs Path in the costmap frame.
  nav_msgs::msg::Path create_path_message(
    const std::vector<GridCell> &grid_path) const;
  /// @brief Concatenate route segment paths into one continuous global plan.
  nav_msgs::msg::Path merge_paths(const std::vector<nav_msgs::msg::Path> &paths) const;
  /// @brief Cache the latest global costmap subscription message.
  void costmap_subscription_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  rclcpp::Service<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_service_;
  rclcpp::Service<amr_msgs::srv::PlanRoute>::SharedPtr plan_route_service_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr computed_plan_publisher_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
  nav_msgs::msg::Path planned_path_;
  AStarPlanner::UniquePtr a_star_planner_;
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
  double start_row_hold_penalty_;
  int goal_row_align_distance_cells_;
  double goal_row_align_penalty_;
  int nearest_free_search_radius_cells_;
  bool axis_aligned_straightening_enabled_;
  int same_row_tolerance_cells_;
  int same_column_tolerance_cells_;
  double same_y_tolerance_m_;
  double same_x_tolerance_m_;
  int same_row_max_lateral_deviation_cells_;
  double axis_aligned_interpolation_distance_;
  bool axis_aligned_require_line_of_sight_;
  bool same_row_straightening_enabled_;
  double same_row_interpolation_distance_;
  bool same_row_require_line_of_sight_;
  bool structured_logging_enabled_;
  std::vector<double> footprint_polygon_param_;
  amr::geometry::FootprintPolygon footprint_polygon_;
};

}  // namespace amr::planner::global

#endif  // AMR_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
