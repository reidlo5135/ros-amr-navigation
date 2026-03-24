#ifndef AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_
#define AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_

#include <string>
#include <vector>

#include "amr_msgs/msg/obstacle_report.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_costmap_server
{

class CostmapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CostmapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_obstacle_report(const amr_msgs::msg::ObstacleReport::SharedPtr message);
  void rebuild_costmaps();
  void publish_costmaps();
  void update_footprint_metrics();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<amr_msgs::msg::ObstacleReport>::SharedPtr obstacle_report_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_publisher_;

  std::string map_topic_;
  std::string obstacle_report_topic_;
  std::string global_costmap_topic_;
  std::string local_costmap_topic_;
  int obstacle_threshold_;
  double global_inflation_radius_;
  int global_inflation_cost_;
  double local_dynamic_inflation_radius_;
  int local_dynamic_cost_;
  std::vector<double> footprint_polygon_;
  double footprint_padding_;
  double footprint_circumscribed_radius_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid global_costmap_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
  amr_msgs::msg::ObstacleReport latest_obstacle_report_;
  bool has_map_;
  bool has_obstacle_report_;
};

}  // namespace amr_costmap_server

#endif  // AMR_COSTMAP_SERVER__COSTMAP_SERVER_HPP_
