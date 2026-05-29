#ifndef AMR_LOCALIZATION__LOCALIZATION_HPP_
#define AMR_LOCALIZATION__LOCALIZATION_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace amr::localization::estimator
{

class Localization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Localization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~Localization() = default;

private:
  struct Particle
  {
    double x;
    double y;
    double yaw;
    double weight;
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void publish_auto_initial_pose();
  void initialize_particles(const geometry_msgs::msg::PoseStamped &pose);
  void apply_motion_update(
    const geometry_msgs::msg::PoseStamped &previous_odom_pose,
    const geometry_msgs::msg::PoseStamped &current_odom_pose);
  void apply_measurement_update(const sensor_msgs::msg::LaserScan &scan);
  void resample_particles();
  void update_estimated_pose_from_particles(const rclcpp::Time &stamp);
  void publish_outputs(const rclcpp::Time &stamp);
  geometry_msgs::msg::TransformStamped build_map_to_odom_transform(const rclcpp::Time &stamp) const;
  geometry_msgs::msg::PoseStamped odometry_pose_to_pose_stamped(
    const nav_msgs::msg::Odometry &odometry) const;
  bool world_to_grid(double world_x, double world_y, int &grid_x, int &grid_y) const;
  bool is_occupied_cell(int grid_x, int grid_y) const;
  double nearest_obstacle_distance(double world_x, double world_y) const;
  double compute_particle_likelihood(
    const Particle &particle,
    const sensor_msgs::msg::LaserScan &scan) const;
  double sample_normal(double stddev);
  double normalize_angle(double angle) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation) const;
  void update_pose_orientation(geometry_msgs::msg::PoseStamped &pose, double yaw) const;
  void reset_state();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odometry_publisher_;
  rclcpp::TimerBase::SharedPtr auto_initial_pose_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string initial_pose_topic_;
  std::string estimated_pose_topic_;
  std::string estimated_odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  bool auto_initial_pose_enabled_;
  double auto_initial_pose_delay_sec_;
  double auto_initial_pose_covariance_x_;
  double auto_initial_pose_covariance_y_;
  double auto_initial_pose_covariance_yaw_;
  int particle_count_;
  double initial_particle_std_xy_;
  double initial_particle_std_yaw_;
  double motion_noise_linear_;
  double motion_noise_lateral_;
  double motion_noise_angular_;
  double measurement_sigma_;
  int measurement_search_radius_cells_;
  int max_beams_;
  double max_beam_range_;
  int occupied_threshold_;

  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  geometry_msgs::msg::PoseStamped initial_map_pose_;
  geometry_msgs::msg::PoseStamped previous_odom_pose_;
  geometry_msgs::msg::PoseStamped estimated_pose_;
  std::vector<Particle> particles_;
  std::mt19937 random_engine_;
  bool has_latest_odom_;
  bool has_latest_scan_;
  bool has_map_;
  bool has_previous_odom_;
  bool has_initial_pose_;
  bool particles_initialized_;
  bool auto_initial_pose_published_;
};

}  // namespace amr::localization::estimator

#endif  // AMR_LOCALIZATION__LOCALIZATION_HPP_
