#ifndef AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_
#define AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_

/**
 * @file costmap_server.hpp
 * @brief Lifecycle node that builds global and local occupancy costmaps from map, scan, and TF.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "amr_msgs/srv/clear_costmap.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace amr::costmap::server
{

/// @brief Builds and publishes global/local costmaps for the AMR navigation stack.
class CostmapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct the costmap server node and declare ROS parameters.
  explicit CostmapServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the costmap server node.
  virtual ~CostmapServer() = default;

private:
  /// @brief Lifecycle callback return type alias.
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure subscriptions, publishers, services, TF, and costmap buffers.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /// @brief Activate lifecycle publishers and periodic publication.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /// @brief Deactivate lifecycle publishers.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /// @brief Release subscriptions, services, TF, and cached map data.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /// @brief Release resources during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /// @brief Store the latest static map and rebuild costmaps.
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  /// @brief Store the latest scan and update dynamic local obstacles.
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  /// @brief Clear dynamic costmap state through the clear-costmap service.
  void handle_clear_costmap(
    const std::shared_ptr<amr_msgs::srv::ClearCostmap::Request> request,
    std::shared_ptr<amr_msgs::srv::ClearCostmap::Response> response);
  /// @brief Update the robot pose from the configured map-to-base TF lookup.
  bool update_current_pose_from_tf();
  /// @brief Rebuild the inflated global costmap from the latest static map.
  void rebuild_global_costmap();
  /// @brief Rebuild the local dynamic costmap around the robot.
  void rebuild_local_costmap();
  /// @brief Publish the global costmap when the lifecycle publisher is active.
  void publish_global_costmap();
  /// @brief Publish the local costmap when throttling allows publication.
  void publish_local_costmap();
  /// @brief Publish all available costmap outputs.
  void publish_costmaps();
  /// @brief Recompute footprint radius and padding-derived metrics.
  void update_footprint_metrics();
  /// @brief Return true when the local costmap publish throttle has elapsed.
  bool should_publish_local_costmap();
  /// @brief Convert world coordinates into the static map grid.
  bool world_to_grid(double world_x, double world_y, int &grid_x, int &grid_y) const;
  /// @brief Convert world coordinates into an arbitrary occupancy grid.
  bool world_to_costmap_grid(
    const nav_msgs::msg::OccupancyGrid &costmap,
    double world_x,
    double world_y,
    int &grid_x,
    int &grid_y) const;
  /// @brief Check whether static obstacles exist near a grid cell within a clearance radius.
  bool has_static_obstacle_near(int grid_x, int grid_y, int clearance_cells) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Service<amr_msgs::srv::ClearCostmap>::SharedPtr clear_costmap_service_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_topic_;
  std::string scan_topic_;
  std::string global_costmap_topic_;
  std::string local_costmap_topic_;
  std::string clear_costmap_service_name_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double tf_lookup_timeout_sec_;
  int obstacle_threshold_;
  double global_inflation_radius_;
  int global_inflation_cost_;
  double local_dynamic_inflation_radius_;
  int local_dynamic_cost_;
  double dynamic_max_distance_;
  double dynamic_forward_angle_deg_;
  int dynamic_static_clearance_cells_;
  bool publish_global_on_scan_;
  int local_publish_min_period_ms_;
  bool structured_logging_enabled_;
  bool local_window_enabled_;
  double local_window_radius_;
  std::vector<double> footprint_polygon_;
  double footprint_padding_;
  double footprint_circumscribed_radius_;
  rclcpp::Time last_local_publish_time_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid global_costmap_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_map_;
  bool has_pose_;
  bool has_scan_;
};

}  // namespace amr::costmap::server

#endif  // AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_
