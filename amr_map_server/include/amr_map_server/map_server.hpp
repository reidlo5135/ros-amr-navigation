#ifndef AMR_MAP_SERVER__MAP_SERVER_HPP_
#define AMR_MAP_SERVER__MAP_SERVER_HPP_

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

namespace amr_map_server
{

class MapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct MapQualityMetrics
  {
    double known_ratio{0.0};
    double free_ratio{0.0};
    double occupied_ratio{0.0};
    double inflated_free_ratio{0.0};
    bool corrected_pose_ready{false};
    bool passed{false};
    std::string summary;
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  bool load_static_map_from_files();
  void publish_official_map();
  std::string resolve_path(const std::string & configured_path) const;
  void set_quaternion_from_yaw(geometry_msgs::msg::Quaternion & orientation, double yaw) const;
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const;

  void handle_temporary_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_corrected_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_get_map(
    const nav_msgs::srv::GetMap::Request::SharedPtr request,
    nav_msgs::srv::GetMap::Response::SharedPtr response);
  void handle_freeze_temporary_map(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_evaluate_temporary_map(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_save_temporary_map(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  MapQualityMetrics evaluate_temporary_map_quality() const;
  bool save_map_to_files(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::string & image_path,
    const std::string & yaml_path) const;
  bool save_temporary_map_to_official_and_files(std::string & message);
  void maybe_auto_save_temporary_map();
  bool grid_index(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    std::size_t & index) const;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr official_map_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr temporary_map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr corrected_odometry_subscription_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_temporary_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr evaluate_temporary_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_temporary_map_service_;

  nav_msgs::msg::OccupancyGrid official_map_;
  nav_msgs::msg::OccupancyGrid temporary_map_;
  mutable std::mutex map_mutex_;

  std::string yaml_path_;
  std::string frame_id_;
  std::string map_topic_;
  std::string temporary_map_topic_;
  std::string corrected_odometry_topic_;
  std::string get_map_service_name_;
  std::string freeze_temporary_map_service_name_;
  std::string evaluate_temporary_map_service_name_;
  std::string save_temporary_map_service_name_;
  bool mapping_mode_;
  std::string save_directory_;
  std::string save_basename_;
  double quality_min_known_ratio_;
  double quality_min_free_ratio_;
  double quality_min_occupied_ratio_;
  int quality_inflation_radius_cells_;
  double quality_min_inflated_free_ratio_;
  bool quality_require_corrected_pose_;
  bool auto_save_enabled_;
  int auto_save_required_consecutive_passes_;
  int auto_save_consecutive_pass_count_;
  bool auto_save_completed_;
  bool official_map_loaded_;
  bool temporary_map_loaded_;
  bool has_latest_corrected_pose_;
};

}  // namespace amr_map_server

#endif  // AMR_MAP_SERVER__MAP_SERVER_HPP_
