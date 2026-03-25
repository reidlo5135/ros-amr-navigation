#ifndef AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "amr_geometry/footprint.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "amr_msgs/srv/plan_local_escape.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_local_planner
{

class LocalPlanner : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LocalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_plan_local_escape(
    const std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Request> request,
    std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response);
  void publish_local_plan();
  nav_msgs::msg::Path build_local_plan(
    const amr_msgs::msg::MotionCommand & command,
    const geometry_msgs::msg::PoseStamped & current_pose);
  nav_msgs::msg::Path build_inflated_local_plan(
    const nav_msgs::msg::Path & source_plan,
    const geometry_msgs::msg::PoseStamped & current_pose,
    std::size_t closest_index,
    double lookahead_distance);
  nav_msgs::msg::Path build_sliced_local_plan(
    const nav_msgs::msg::Path & source_plan,
    const geometry_msgs::msg::PoseStamped & current_pose,
    std::size_t closest_index) const;
  nav_msgs::msg::Path build_sliced_local_plan_with_lookahead(
    const nav_msgs::msg::Path & source_plan,
    const geometry_msgs::msg::PoseStamped & current_pose,
    std::size_t closest_index,
    double lookahead_distance) const;
  nav_msgs::msg::Path build_source_plan(const amr_msgs::msg::MotionCommand & command) const;
  std::size_t find_closest_pose_index(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & current_pose,
    std::size_t start_index) const;
  bool world_to_grid(
    const geometry_msgs::msg::Point & point,
    int & grid_x,
    int & grid_y) const;
  geometry_msgs::msg::PoseStamped grid_to_pose(
    int grid_x,
    int grid_y,
    const std::string & frame_id) const;
  bool is_occupied_cell(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y) const;
  bool is_grid_pose_collision(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    int grid_x,
    int grid_y,
    double yaw) const;
  bool find_nearest_free_cell(
    const std::vector<int8_t> & occupancy_grid,
    int width,
    int height,
    int & grid_x,
    int & grid_y,
    int max_radius,
    double yaw) const;
  geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double ratio) const;
  double pose_distance(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool find_first_blocked_pose_on_plan(
    const nav_msgs::msg::Path & plan,
    geometry_msgs::msg::PoseStamped & blocked_pose) const;
  double sample_lateral_occupancy(
    double origin_x,
    double origin_y,
    double heading,
    double lateral_sign) const;

  rclcpp::Subscription<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Service<amr_msgs::srv::PlanLocalEscape>::SharedPtr local_escape_service_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string command_topic_;
  std::string current_pose_topic_;
  std::string map_topic_;
  std::string local_plan_topic_;
  std::string local_escape_service_name_;
  int publish_period_ms_;
  double lookahead_distance_;
  double goal_tolerance_;
  int obstacle_threshold_;
  int connectivity_;
  bool allow_unknown_;
  bool prevent_corner_cutting_;
  double turn_penalty_;
  bool dynamic_obstacle_enabled_;
  double dynamic_obstacle_replan_lookahead_distance_;
  double dynamic_obstacle_escape_forward_distance_;
  double dynamic_obstacle_escape_lateral_distance_;
  double dynamic_obstacle_goal_proximity_disable_distance_;
  int nearest_free_search_radius_cells_;
  std::vector<double> footprint_polygon_param_;
  amr_geometry::FootprintPolygon footprint_polygon_;
  uint32_t last_command_id_;
  std::size_t last_progress_index_;
  amr_msgs::msg::MotionCommand latest_command_;
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  nav_msgs::msg::OccupancyGrid working_costmap_;
  bool has_command_;
  bool has_current_pose_;
  bool has_map_;
};

}  // namespace amr_local_planner

#endif  // AMR_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
