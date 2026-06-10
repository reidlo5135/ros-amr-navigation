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

#include <amr_msgs/msg/motion_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

  struct MotionDelta
  {
    double local_x{0.0};
    double local_y{0.0};
    double yaw{0.0};
    double raw_translation_m{0.0};
    double raw_yaw_rad{0.0};
    double applied_translation_m{0.0};
    double applied_yaw_rad{0.0};
    bool limited{false};
    bool slipping{false};
    bool stall_suspected{false};
    std::string reason{"nominal"};
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
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);
  void handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message);
  void publish_auto_initial_pose();
  void initialize_particles(const geometry_msgs::msg::PoseStamped &pose);
  void apply_motion_update(
    const geometry_msgs::msg::PoseStamped &previous_odom_pose,
    const geometry_msgs::msg::PoseStamped &current_odom_pose);
  MotionDelta compute_guarded_motion_delta(
    const geometry_msgs::msg::PoseStamped &previous_odom_pose,
    const geometry_msgs::msg::PoseStamped &current_odom_pose);
  void update_wheel_slip_state(
    const MotionDelta &raw_delta,
    const rclcpp::Time &stamp);
  void apply_measurement_update(const sensor_msgs::msg::LaserScan &scan);
  void resample_particles();
  void update_estimated_pose_from_particles(const rclcpp::Time &stamp);
  void publish_outputs(const rclcpp::Time &stamp);
  geometry_msgs::msg::TransformStamped build_map_to_odom_transform(const rclcpp::Time &stamp);
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
  double transform_yaw(const geometry_msgs::msg::TransformStamped &transform) const;
  void update_pose_orientation(geometry_msgs::msg::PoseStamped &pose, double yaw) const;
  bool is_localization_guard_relaxed(const rclcpp::Time &stamp) const;
  bool is_recent(const rclcpp::Time &stamp, const rclcpp::Time &now, double timeout_sec) const;
  void reset_state();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
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
  std::string cmd_vel_topic_;
  std::string motion_status_topic_;
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
  bool localization_guard_enabled_;
  bool slip_detection_enabled_;
  double odom_translation_slip_threshold_m_;
  double odom_rotation_slip_threshold_rad_;
  double pose_translation_confirm_threshold_m_;
  double pose_rotation_confirm_threshold_rad_;
  double command_linear_threshold_;
  double command_angular_threshold_;
  int localization_guard_confirm_cycles_;
  int localization_guard_clear_cycles_;
  double initial_pose_grace_sec_;
  double goal_proximity_relax_distance_m_;
  double odom_translation_gain_when_slipping_;
  double odom_rotation_gain_when_slipping_;
  double max_odom_translation_delta_per_update_m_;
  double max_odom_rotation_delta_per_update_rad_;
  double measurement_likelihood_warn_threshold_;
  bool map_odom_guard_enabled_;
  double max_correction_translation_per_update_m_;
  double max_correction_rotation_per_update_rad_;
  double hard_jump_warn_translation_m_;
  double hard_jump_warn_rotation_rad_;
  bool map_odom_bypass_on_initial_pose_;
  bool map_odom_bypass_on_global_reset_;

  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  geometry_msgs::msg::Twist latest_cmd_vel_;
  amr_msgs::msg::MotionStatus latest_motion_status_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  geometry_msgs::msg::PoseStamped initial_map_pose_;
  geometry_msgs::msg::PoseStamped previous_odom_pose_;
  geometry_msgs::msg::PoseStamped estimated_pose_;
  geometry_msgs::msg::PoseStamped previous_estimated_pose_;
  geometry_msgs::msg::TransformStamped last_map_to_odom_transform_;
  std::vector<Particle> particles_;
  std::mt19937 random_engine_;
  rclcpp::Time latest_cmd_vel_time_;
  rclcpp::Time latest_motion_status_time_;
  rclcpp::Time last_initial_pose_time_;
  bool has_latest_odom_;
  bool has_latest_scan_;
  bool has_latest_cmd_vel_;
  bool has_latest_motion_status_;
  bool has_map_;
  bool has_previous_odom_;
  bool has_previous_estimated_pose_;
  bool has_map_to_odom_transform_;
  bool has_initial_pose_;
  bool particles_initialized_;
  bool auto_initial_pose_published_;
  bool structured_logging_enabled_;
  bool odom_slip_suspected_;
  bool physical_stall_suspected_;
  bool localization_guard_confirmed_;
  bool map_odom_bypass_next_update_;
  int localization_guard_suspect_streak_;
  int localization_guard_clear_streak_;
  double last_pose_delta_m_;
  double last_pose_delta_yaw_rad_;
  double last_measurement_likelihood_;
  bool last_measurement_update_success_;
};

}  // namespace amr::localization::estimator

#endif  // AMR_LOCALIZATION__LOCALIZATION_HPP_
