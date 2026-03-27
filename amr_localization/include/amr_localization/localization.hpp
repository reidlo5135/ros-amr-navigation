#ifndef AMR_LOCALIZATION__LOCALIZATION_HPP_
#define AMR_LOCALIZATION__LOCALIZATION_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <cstddef>
#include <vector>

#include "amr_msgs/msg/localization_candidate_array.hpp"
#include "amr_msgs/msg/localization_status.hpp"
#include "amr_msgs/srv/trigger_global_localization.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace amr_localization
{

class Localization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Localization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Particle
  {
    double x;
    double y;
    double yaw;
    double weight;
  };

  struct CandidateCluster
  {
    double x;
    double y;
    double yaw;
    double score;
    double cluster_weight;
    double dominance_ratio;
    double position_std;
    double yaw_std;
  };

  struct CandidateTrack
  {
    uint32_t id;
    double x;
    double y;
    double yaw;
    double score;
  };

  enum class LocalizationMode : uint8_t
  {
    kTracking = amr_msgs::msg::LocalizationStatus::MODE_TRACKING,
    kGlobalRelocalizing = amr_msgs::msg::LocalizationStatus::MODE_GLOBAL_RELOCALIZING,
    kFailed = amr_msgs::msg::LocalizationStatus::MODE_FAILED
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void publish_auto_initial_pose();
  void initialize_particles(const geometry_msgs::msg::PoseStamped & pose);
  bool initialize_particles_global();
  void apply_motion_update(
    const geometry_msgs::msg::PoseStamped & previous_odom_pose,
    const geometry_msgs::msg::PoseStamped & current_odom_pose);
  void apply_measurement_update(const sensor_msgs::msg::LaserScan & scan);
  void resample_particles();
  void update_relocalization_candidate_lock();
  void reset_relocalization_temp_submap();
  void update_relocalization_temp_submap(const sensor_msgs::msg::LaserScan & scan);
  CandidateCluster refine_candidate_cluster_scan_first(const CandidateCluster & seed_cluster) const;
  double score_candidate_with_relocalization_temp_submap(const CandidateCluster & cluster) const;
  std::vector<CandidateCluster> extract_candidate_clusters() const;
  amr_msgs::msg::LocalizationCandidateArray build_candidate_array_message(
    const rclcpp::Time & stamp,
    const std::vector<CandidateCluster> & clusters);
  void update_estimated_pose_from_particles(const rclcpp::Time & stamp);
  void publish_outputs(const rclcpp::Time & stamp);
  void publish_localization_status(const rclcpp::Time & stamp);
  void publish_localization_candidates(const rclcpp::Time & stamp);
  geometry_msgs::msg::TransformStamped build_map_to_odom_transform(const rclcpp::Time & stamp) const;
  geometry_msgs::msg::PoseStamped odometry_pose_to_pose_stamped(
    const nav_msgs::msg::Odometry & odometry) const;
  bool world_to_grid(double world_x, double world_y, int & grid_x, int & grid_y) const;
  bool grid_to_world(int grid_x, int grid_y, double & world_x, double & world_y) const;
  bool local_submap_index_to_local_point(int index, double & local_x, double & local_y) const;
  bool is_occupied_cell(int grid_x, int grid_y) const;
  bool is_free_cell(int grid_x, int grid_y) const;
  bool sample_random_free_pose(Particle & particle);
  double raycast_obstacle_range(double world_x, double world_y, double angle, double max_range) const;
  double nearest_obstacle_distance(double world_x, double world_y) const;
  double compute_particle_likelihood(
    const Particle & particle,
    const sensor_msgs::msg::LaserScan & scan) const;
  void start_global_relocalization(const std::string & reason);
  void update_localization_mode(const rclcpp::Time & stamp);
  void handle_trigger_global_localization(
    const std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Request> request,
    std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Response> response);
  bool startup_mode_is_manual_set_initial_pose() const;
  bool startup_mode_is_global_relocalization() const;
  bool startup_mode_is_active_relocalization() const;
  bool startup_mode_is_fixed_start_pose() const;
  double sample_normal(double stddev);
  double normalize_angle(double angle) const;
  double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  void update_pose_orientation(geometry_msgs::msg::PoseStamped & pose, double yaw) const;
  void reset_state();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odometry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::LocalizationStatus>::SharedPtr localization_status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::LocalizationCandidateArray>::SharedPtr localization_candidates_publisher_;
  rclcpp::Service<amr_msgs::srv::TriggerGlobalLocalization>::SharedPtr trigger_global_localization_service_;
  rclcpp::TimerBase::SharedPtr auto_initial_pose_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string initial_pose_topic_;
  std::string estimated_pose_topic_;
  std::string estimated_odom_topic_;
  std::string localization_status_topic_;
  std::string localization_candidates_topic_;
  std::string trigger_global_localization_service_name_;
  std::string startup_localization_mode_;
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
  int global_particle_count_;
  int global_particle_sample_attempts_;
  double global_resample_position_noise_;
  double global_resample_yaw_noise_;
  double relocalization_max_position_std_;
  double relocalization_max_yaw_std_;
  double relocalization_min_cluster_weight_;
  double relocalization_min_cluster_dominance_ratio_;
  double estimate_cluster_distance_;
  double estimate_cluster_yaw_;
  double relocalization_candidate_lock_confidence_threshold_;
  double relocalization_candidate_lock_cluster_weight_threshold_;
  double relocalization_candidate_lock_distance_;
  double relocalization_candidate_lock_yaw_;
  int relocalization_candidate_lock_min_updates_;
  int relocalization_max_candidates_;
  double relocalization_candidate_match_distance_;
  double relocalization_candidate_match_yaw_;
  double relocalization_candidate_refine_distance_;
  double relocalization_candidate_refine_yaw_;
  int relocalization_candidate_refine_xy_steps_;
  int relocalization_candidate_refine_yaw_steps_;
  double relocalization_candidate_min_score_ratio_;
  bool relocalization_temp_submap_enabled_;
  double relocalization_temp_submap_resolution_;
  double relocalization_temp_submap_size_m_;
  int relocalization_temp_submap_min_hits_;
  double relocalization_temp_submap_score_weight_;
  bool kidnapped_detection_enabled_;
  bool kidnapped_start_with_global_localization_;
  bool kidnapped_auto_trigger_enabled_;
  double kidnapped_low_confidence_threshold_;
  int kidnapped_low_confidence_updates_;
  double relocalization_success_confidence_threshold_;
  int relocalization_success_updates_;
  double relocalization_timeout_sec_;

  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_;
  geometry_msgs::msg::PoseStamped initial_map_pose_;
  geometry_msgs::msg::PoseStamped previous_odom_pose_;
  geometry_msgs::msg::PoseStamped estimated_pose_;
  std::vector<Particle> particles_;
  std::mt19937 random_engine_;
  LocalizationMode localization_mode_;
  bool kidnapped_suspected_;
  bool relocalization_requested_;
  uint32_t relocalization_count_;
  double localization_confidence_;
  double last_measurement_confidence_;
  double localization_cluster_weight_;
  double localization_cluster_dominance_ratio_;
  double localization_position_std_;
  double localization_yaw_std_;
  int low_confidence_update_count_;
  int relocalization_success_count_;
  int relocalization_observation_count_;
  rclcpp::Time relocalization_started_at_;
  bool relocalization_candidate_locked_;
  geometry_msgs::msg::PoseStamped relocalization_candidate_pose_;
  geometry_msgs::msg::PoseStamped relocalization_reference_odom_pose_;
  std::vector<CandidateTrack> previous_candidate_tracks_;
  std::vector<uint16_t> relocalization_temp_submap_counts_;
  std::size_t relocalization_temp_submap_marked_cells_;
  int relocalization_temp_submap_width_;
  int relocalization_temp_submap_height_;
  bool has_relocalization_reference_odom_pose_;
  uint32_t next_candidate_id_;
  bool has_latest_odom_;
  bool has_latest_scan_;
  bool has_map_;
  bool has_previous_odom_;
  bool has_initial_pose_;
  bool particles_initialized_;
  bool auto_initial_pose_published_;
  bool startup_global_relocalization_pending_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__LOCALIZATION_HPP_
