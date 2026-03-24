#ifndef AMR_MAP_SERVER__MAP_SERVER_HPP_
#define AMR_MAP_SERVER__MAP_SERVER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "yaml-cpp/yaml.h"

namespace amr_map_server
{

class MapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Pose2D
  {
    double x;
    double y;
    double yaw;
  };

  struct MapQualityMetrics
  {
    double known_ratio;
    double free_ratio;
    double occupied_ratio;
    double inflated_free_ratio;
    bool corrected_pose_ready;
    bool passed;
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
  void initialize_mapping_map();
  void publish_maps();
  void publish_official_map();
  void publish_temporary_map();
  std::string resolve_path(const std::string & configured_path) const;
  void set_quaternion_from_yaw(geometry_msgs::msg::Quaternion & orientation, double yaw) const;
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
  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void update_map_from_scan(const sensor_msgs::msg::LaserScan & scan);
  Pose2D build_predicted_pose() const;
  Pose2D refine_pose_with_scan_matching(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & predicted_pose) const;
  double score_scan_candidate(
    const nav_msgs::msg::OccupancyGrid & map,
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & candidate_pose) const;
  bool has_nearby_occupied_cell(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    int radius_cells) const;
  bool world_to_grid(double x, double y, int & grid_x, int & grid_y) const;
  bool world_to_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    double x,
    double y,
    int & grid_x,
    int & grid_y) const;
  bool grid_index(int grid_x, int grid_y, std::size_t & index) const;
  bool grid_index(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    std::size_t & index) const;
  void mark_free_cell(int grid_x, int grid_y);
  void mark_occupied_cell(int grid_x, int grid_y);
  void raytrace_free_cells(int start_x, int start_y, int end_x, int end_y);
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalize_angle(double angle) const;
  MapQualityMetrics evaluate_temporary_map_quality() const;
  bool save_map_to_files(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::string & image_path,
    const std::string & yaml_path) const;
  bool save_temporary_map_to_official_and_files(std::string & message);
  void maybe_auto_save_temporary_map();
  void update_cell_score(int grid_x, int grid_y, int delta);
  void refresh_cell_from_score(std::size_t index);
  void decay_occupied_scores();
  void publish_map_to_odom_tf();

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr official_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr temporary_map_publisher_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_temporary_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr evaluate_temporary_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_temporary_map_service_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr mapping_publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  nav_msgs::msg::OccupancyGrid official_map_;
  nav_msgs::msg::OccupancyGrid temporary_map_;
  mutable std::mutex map_mutex_;
  std::vector<int16_t> occupancy_scores_;
  std::string yaml_path_;
  std::string frame_id_;
  std::string odom_frame_;
  std::string map_topic_;
  std::string temporary_map_topic_;
  std::string get_map_service_name_;
  std::string freeze_temporary_map_service_name_;
  std::string evaluate_temporary_map_service_name_;
  std::string save_temporary_map_service_name_;
  bool mapping_mode_;
  std::string mapping_pose_topic_;
  std::string mapping_imu_topic_;
  std::string mapping_scan_topic_;
  int mapping_publish_period_ms_;
  double mapping_resolution_;
  int mapping_width_;
  int mapping_height_;
  double mapping_origin_x_;
  double mapping_origin_y_;
  double mapping_origin_yaw_;
  double mapping_min_range_;
  double mapping_max_range_;
  bool mapping_publish_identity_tf_;
  std::string save_directory_;
  std::string save_basename_;
  bool scan_matching_enabled_;
  double scan_matching_linear_window_;
  double scan_matching_linear_step_;
  double scan_matching_angular_window_deg_;
  double scan_matching_angular_step_deg_;
  int scan_matching_max_beams_;
  int scan_matching_min_valid_beams_;
  int scan_matching_occupied_search_radius_cells_;
  int scan_matching_minimum_occupied_cells_;
  double scan_matching_occupied_match_score_;
  double scan_matching_free_space_penalty_;
  int mapping_hit_score_;
  int mapping_free_score_;
  int mapping_decay_score_;
  int mapping_occupied_score_threshold_;
  int mapping_free_score_threshold_;
  int mapping_score_min_;
  int mapping_score_max_;
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
  nav_msgs::msg::Odometry latest_odometry_;
  Pose2D latest_corrected_pose_;
  double start_odom_yaw_;
  double start_imu_yaw_;
  double latest_imu_yaw_;
  bool has_latest_odometry_;
  bool has_latest_corrected_pose_;
  bool has_start_odom_yaw_;
  bool has_latest_imu_;
  bool has_start_imu_yaw_;
};

}  // namespace amr_map_server

#endif  // AMR_MAP_SERVER__MAP_SERVER_HPP_
