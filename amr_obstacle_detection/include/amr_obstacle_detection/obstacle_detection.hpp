#ifndef AMR_OBSTACLE_DETECTION__OBSTACLE_DETECTION_HPP_
#define AMR_OBSTACLE_DETECTION__OBSTACLE_DETECTION_HPP_

#include <string>

#include "amr_msgs/msg/obstacle_report.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_obstacle_detection
{

class ObstacleDetection : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ObstacleDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  amr_msgs::msg::ObstacleReport build_obstacle_report() const;
  void publish_report();
  bool world_to_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    double x,
    double y,
    int & grid_x,
    int & grid_y) const;
  bool has_static_obstacle_near(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalize_angle(double angle) const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::ObstacleReport>::SharedPtr report_publisher_;

  std::string scan_topic_;
  std::string pose_topic_;
  std::string map_topic_;
  std::string report_topic_;
  double max_distance_;
  double forward_angle_deg_;
  int minimum_points_;
  int static_clearance_cells_;
  double blocking_distance_;
  double blocking_lateral_distance_;
  double critical_distance_;
  double high_distance_;
  double medium_distance_;
  int obstacle_threshold_;
  sensor_msgs::msg::LaserScan latest_scan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool has_scan_;
  bool has_pose_;
  bool has_map_;
};

}  // namespace amr_obstacle_detection

#endif  // AMR_OBSTACLE_DETECTION__OBSTACLE_DETECTION_HPP_
