#include "amr_localization/localization.hpp"

namespace amr_localization
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumWeight = 1e-6;

}  // namespace

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("localization", options),
  odom_topic_(""),
  scan_topic_(""),
  map_topic_(""),
  initial_pose_topic_(""),
  estimated_pose_topic_(""),
  estimated_odom_topic_(""),
  localization_status_topic_(""),
  localization_candidates_topic_(""),
  trigger_global_localization_service_name_("/amr/localization/trigger_global_localization"),
  startup_localization_mode_("global_relocalization"),
  map_frame_("map"),
  odom_frame_("odom"),
  base_frame_("base_link"),
  initial_x_(0.0),
  initial_y_(0.0),
  initial_yaw_(0.0),
  auto_initial_pose_enabled_(true),
  auto_initial_pose_delay_sec_(3.0),
  auto_initial_pose_covariance_x_(0.25),
  auto_initial_pose_covariance_y_(0.25),
  auto_initial_pose_covariance_yaw_(0.06853891945200942),
  particle_count_(200),
  initial_particle_std_xy_(0.15),
  initial_particle_std_yaw_(0.15),
  motion_noise_linear_(0.02),
  motion_noise_lateral_(0.01),
  motion_noise_angular_(0.05),
  measurement_sigma_(0.15),
  measurement_search_radius_cells_(4),
  max_beams_(24),
  max_beam_range_(6.0),
  occupied_threshold_(50),
  global_particle_count_(800),
  global_particle_sample_attempts_(1000),
  global_resample_position_noise_(0.03),
  global_resample_yaw_noise_(0.08),
  relocalization_max_position_std_(0.25),
  relocalization_max_yaw_std_(0.45),
  relocalization_min_cluster_weight_(0.18),
  relocalization_min_cluster_dominance_ratio_(1.8),
  estimate_cluster_distance_(0.35),
  estimate_cluster_yaw_(0.75),
  relocalization_candidate_lock_confidence_threshold_(0.12),
  relocalization_candidate_lock_cluster_weight_threshold_(0.14),
  relocalization_candidate_lock_distance_(1.0),
  relocalization_candidate_lock_yaw_(1.2),
  relocalization_candidate_lock_min_updates_(2),
  relocalization_max_candidates_(4),
  relocalization_candidate_match_distance_(0.8),
  relocalization_candidate_match_yaw_(0.9),
  kidnapped_detection_enabled_(true),
  kidnapped_start_with_global_localization_(false),
  kidnapped_auto_trigger_enabled_(false),
  kidnapped_low_confidence_threshold_(0.22),
  kidnapped_low_confidence_updates_(6),
  relocalization_success_confidence_threshold_(0.40),
  relocalization_success_updates_(3),
  relocalization_timeout_sec_(8.0),
  map_occupancy_grid_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  random_engine_(std::random_device{}()),
  localization_mode_(LocalizationMode::kTracking),
  kidnapped_suspected_(false),
  relocalization_requested_(false),
  relocalization_count_(0U),
  localization_confidence_(0.0),
  last_measurement_confidence_(0.0),
  localization_cluster_weight_(0.0),
  localization_cluster_dominance_ratio_(0.0),
  localization_position_std_(std::numeric_limits<double>::infinity()),
  localization_yaw_std_(std::numeric_limits<double>::infinity()),
  low_confidence_update_count_(0),
  relocalization_success_count_(0),
  relocalization_observation_count_(0),
  relocalization_started_at_(0, 0, RCL_ROS_TIME),
  relocalization_candidate_locked_(false),
  next_candidate_id_(1U),
  has_latest_odom_(false),
  has_latest_scan_(false),
  has_map_(false),
  has_previous_odom_(false),
  has_initial_pose_(false),
  particles_initialized_(false),
  auto_initial_pose_published_(false)
{
  this->declare_parameter("topics.odom", this->odom_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.initial_pose", this->initial_pose_topic_);
  this->declare_parameter("topics.estimated_pose", this->estimated_pose_topic_);
  this->declare_parameter("topics.estimated_odometry", this->estimated_odom_topic_);
  this->declare_parameter("topics.status", this->localization_status_topic_);
  this->declare_parameter("topics.candidates", this->localization_candidates_topic_);
  this->declare_parameter(
    "services.trigger_global_localization",
    this->trigger_global_localization_service_name_);
  this->declare_parameter("startup.localization_mode", this->startup_localization_mode_);
  this->declare_parameter("frames.map", this->map_frame_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("start_pose.x", this->initial_x_);
  this->declare_parameter("start_pose.y", this->initial_y_);
  this->declare_parameter("start_pose.yaw", this->initial_yaw_);
  this->declare_parameter("auto_initial_pose.enabled", this->auto_initial_pose_enabled_);
  this->declare_parameter("auto_initial_pose.delay_sec", this->auto_initial_pose_delay_sec_);
  this->declare_parameter(
    "auto_initial_pose.covariance.x", this->auto_initial_pose_covariance_x_);
  this->declare_parameter(
    "auto_initial_pose.covariance.y", this->auto_initial_pose_covariance_y_);
  this->declare_parameter(
    "auto_initial_pose.covariance.yaw", this->auto_initial_pose_covariance_yaw_);
  this->declare_parameter("amcl.particle_count", this->particle_count_);
  this->declare_parameter("amcl.initial_particle_std_xy", this->initial_particle_std_xy_);
  this->declare_parameter("amcl.initial_particle_std_yaw", this->initial_particle_std_yaw_);
  this->declare_parameter("amcl.motion_noise_linear", this->motion_noise_linear_);
  this->declare_parameter("amcl.motion_noise_lateral", this->motion_noise_lateral_);
  this->declare_parameter("amcl.motion_noise_angular", this->motion_noise_angular_);
  this->declare_parameter("amcl.measurement_sigma", this->measurement_sigma_);
  this->declare_parameter(
    "amcl.measurement_search_radius_cells", this->measurement_search_radius_cells_);
  this->declare_parameter("amcl.max_beams", this->max_beams_);
  this->declare_parameter("amcl.max_beam_range", this->max_beam_range_);
  this->declare_parameter("amcl.occupied_threshold", this->occupied_threshold_);
  this->declare_parameter("amcl.global_particle_count", this->global_particle_count_);
  this->declare_parameter(
    "amcl.global_particle_sample_attempts", this->global_particle_sample_attempts_);
  this->declare_parameter(
    "amcl.global_resample_position_noise", this->global_resample_position_noise_);
  this->declare_parameter("amcl.global_resample_yaw_noise", this->global_resample_yaw_noise_);
  this->declare_parameter(
    "amcl.relocalization_max_position_std", this->relocalization_max_position_std_);
  this->declare_parameter(
    "amcl.relocalization_max_yaw_std", this->relocalization_max_yaw_std_);
  this->declare_parameter(
    "amcl.relocalization_min_cluster_weight", this->relocalization_min_cluster_weight_);
  this->declare_parameter(
    "amcl.relocalization_min_cluster_dominance_ratio",
    this->relocalization_min_cluster_dominance_ratio_);
  this->declare_parameter("amcl.estimate_cluster_distance", this->estimate_cluster_distance_);
  this->declare_parameter("amcl.estimate_cluster_yaw", this->estimate_cluster_yaw_);
  this->declare_parameter(
    "amcl.relocalization_candidate_lock_confidence_threshold",
    this->relocalization_candidate_lock_confidence_threshold_);
  this->declare_parameter(
    "amcl.relocalization_candidate_lock_cluster_weight_threshold",
    this->relocalization_candidate_lock_cluster_weight_threshold_);
  this->declare_parameter(
    "amcl.relocalization_candidate_lock_distance",
    this->relocalization_candidate_lock_distance_);
  this->declare_parameter(
    "amcl.relocalization_candidate_lock_yaw",
    this->relocalization_candidate_lock_yaw_);
  this->declare_parameter(
    "amcl.relocalization_candidate_lock_min_updates",
    this->relocalization_candidate_lock_min_updates_);
  this->declare_parameter("amcl.relocalization_max_candidates", this->relocalization_max_candidates_);
  this->declare_parameter(
    "amcl.relocalization_candidate_match_distance",
    this->relocalization_candidate_match_distance_);
  this->declare_parameter(
    "amcl.relocalization_candidate_match_yaw",
    this->relocalization_candidate_match_yaw_);
  this->declare_parameter("kidnapped.enabled", this->kidnapped_detection_enabled_);
  this->declare_parameter(
    "kidnapped.start_with_global_localization",
    this->kidnapped_start_with_global_localization_);
  this->declare_parameter("kidnapped.auto_trigger", this->kidnapped_auto_trigger_enabled_);
  this->declare_parameter(
    "kidnapped.low_confidence_threshold",
    this->kidnapped_low_confidence_threshold_);
  this->declare_parameter(
    "kidnapped.consecutive_low_confidence_updates",
    this->kidnapped_low_confidence_updates_);
  this->declare_parameter(
    "kidnapped.relocalization_success_threshold",
    this->relocalization_success_confidence_threshold_);
  this->declare_parameter(
    "kidnapped.relocalization_success_streak",
    this->relocalization_success_updates_);
  this->declare_parameter("kidnapped.relocalization_timeout_sec", this->relocalization_timeout_sec_);
}

Localization::CallbackReturn Localization::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.initial_pose", this->initial_pose_topic_);
  this->get_parameter("topics.estimated_pose", this->estimated_pose_topic_);
  this->get_parameter("topics.estimated_odometry", this->estimated_odom_topic_);
  this->get_parameter("topics.status", this->localization_status_topic_);
  this->get_parameter("topics.candidates", this->localization_candidates_topic_);
  this->get_parameter(
    "services.trigger_global_localization",
    this->trigger_global_localization_service_name_);
  this->get_parameter("startup.localization_mode", this->startup_localization_mode_);
  this->get_parameter("frames.map", this->map_frame_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("start_pose.x", this->initial_x_);
  this->get_parameter("start_pose.y", this->initial_y_);
  this->get_parameter("start_pose.yaw", this->initial_yaw_);
  this->get_parameter("auto_initial_pose.enabled", this->auto_initial_pose_enabled_);
  this->get_parameter("auto_initial_pose.delay_sec", this->auto_initial_pose_delay_sec_);
  this->get_parameter(
    "auto_initial_pose.covariance.x", this->auto_initial_pose_covariance_x_);
  this->get_parameter(
    "auto_initial_pose.covariance.y", this->auto_initial_pose_covariance_y_);
  this->get_parameter(
    "auto_initial_pose.covariance.yaw", this->auto_initial_pose_covariance_yaw_);
  this->get_parameter("amcl.particle_count", this->particle_count_);
  this->get_parameter("amcl.initial_particle_std_xy", this->initial_particle_std_xy_);
  this->get_parameter("amcl.initial_particle_std_yaw", this->initial_particle_std_yaw_);
  this->get_parameter("amcl.motion_noise_linear", this->motion_noise_linear_);
  this->get_parameter("amcl.motion_noise_lateral", this->motion_noise_lateral_);
  this->get_parameter("amcl.motion_noise_angular", this->motion_noise_angular_);
  this->get_parameter("amcl.measurement_sigma", this->measurement_sigma_);
  this->get_parameter(
    "amcl.measurement_search_radius_cells", this->measurement_search_radius_cells_);
  this->get_parameter("amcl.max_beams", this->max_beams_);
  this->get_parameter("amcl.max_beam_range", this->max_beam_range_);
  this->get_parameter("amcl.occupied_threshold", this->occupied_threshold_);
  this->get_parameter("amcl.global_particle_count", this->global_particle_count_);
  this->get_parameter(
    "amcl.global_particle_sample_attempts", this->global_particle_sample_attempts_);
  this->get_parameter(
    "amcl.global_resample_position_noise", this->global_resample_position_noise_);
  this->get_parameter("amcl.global_resample_yaw_noise", this->global_resample_yaw_noise_);
  this->get_parameter(
    "amcl.relocalization_max_position_std", this->relocalization_max_position_std_);
  this->get_parameter(
    "amcl.relocalization_max_yaw_std", this->relocalization_max_yaw_std_);
  this->get_parameter(
    "amcl.relocalization_min_cluster_weight", this->relocalization_min_cluster_weight_);
  this->get_parameter(
    "amcl.relocalization_min_cluster_dominance_ratio",
    this->relocalization_min_cluster_dominance_ratio_);
  this->get_parameter("amcl.estimate_cluster_distance", this->estimate_cluster_distance_);
  this->get_parameter("amcl.estimate_cluster_yaw", this->estimate_cluster_yaw_);
  this->get_parameter(
    "amcl.relocalization_candidate_lock_confidence_threshold",
    this->relocalization_candidate_lock_confidence_threshold_);
  this->get_parameter(
    "amcl.relocalization_candidate_lock_cluster_weight_threshold",
    this->relocalization_candidate_lock_cluster_weight_threshold_);
  this->get_parameter(
    "amcl.relocalization_candidate_lock_distance",
    this->relocalization_candidate_lock_distance_);
  this->get_parameter(
    "amcl.relocalization_candidate_lock_yaw",
    this->relocalization_candidate_lock_yaw_);
  this->get_parameter(
    "amcl.relocalization_candidate_lock_min_updates",
    this->relocalization_candidate_lock_min_updates_);
  this->get_parameter("amcl.relocalization_max_candidates", this->relocalization_max_candidates_);
  this->get_parameter(
    "amcl.relocalization_candidate_match_distance",
    this->relocalization_candidate_match_distance_);
  this->get_parameter(
    "amcl.relocalization_candidate_match_yaw",
    this->relocalization_candidate_match_yaw_);
  this->get_parameter("kidnapped.enabled", this->kidnapped_detection_enabled_);
  this->get_parameter(
    "kidnapped.start_with_global_localization",
    this->kidnapped_start_with_global_localization_);
  this->get_parameter("kidnapped.auto_trigger", this->kidnapped_auto_trigger_enabled_);
  this->get_parameter(
    "kidnapped.low_confidence_threshold",
    this->kidnapped_low_confidence_threshold_);
  this->get_parameter(
    "kidnapped.consecutive_low_confidence_updates",
    this->kidnapped_low_confidence_updates_);
  this->get_parameter(
    "kidnapped.relocalization_success_threshold",
    this->relocalization_success_confidence_threshold_);
  this->get_parameter(
    "kidnapped.relocalization_success_streak",
    this->relocalization_success_updates_);
  this->get_parameter("kidnapped.relocalization_timeout_sec", this->relocalization_timeout_sec_);

  if (
    !this->startup_mode_is_manual_set_initial_pose() &&
    !this->startup_mode_is_global_relocalization() &&
    !this->startup_mode_is_active_relocalization() &&
    !this->startup_mode_is_fixed_start_pose())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown startup.localization_mode '%s'; falling back to 'global_relocalization'",
      this->startup_localization_mode_.c_str());
    this->startup_localization_mode_ = "global_relocalization";
  }

  this->kidnapped_start_with_global_localization_ =
    this->startup_mode_is_global_relocalization() ||
    this->startup_mode_is_active_relocalization();

  if (!this->startup_mode_is_fixed_start_pose()) {
    this->auto_initial_pose_enabled_ = false;
  }

  if (
    this->odom_topic_.empty() || this->scan_topic_.empty() || this->map_topic_.empty() ||
    this->initial_pose_topic_.empty() || this->estimated_pose_topic_.empty() ||
    this->estimated_odom_topic_.empty() || this->localization_status_topic_.empty() ||
    this->localization_candidates_topic_.empty() ||
    this->trigger_global_localization_service_name_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Localization topics/services must not be empty: odom='%s' scan='%s' map='%s' initial_pose='%s' pose='%s' odometry='%s' status='%s' candidates='%s' trigger='%s'",
      this->odom_topic_.c_str(),
      this->scan_topic_.c_str(),
      this->map_topic_.c_str(),
      this->initial_pose_topic_.c_str(),
      this->estimated_pose_topic_.c_str(),
      this->estimated_odom_topic_.c_str(),
      this->localization_status_topic_.c_str(),
      this->localization_candidates_topic_.c_str(),
      this->trigger_global_localization_service_name_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->reset_state();

  this->initial_map_pose_.header.frame_id = this->map_frame_;
  this->initial_map_pose_.pose.position.x = this->initial_x_;
  this->initial_map_pose_.pose.position.y = this->initial_y_;
  this->initial_map_pose_.pose.position.z = 0.0;
  this->update_pose_orientation(this->initial_map_pose_, this->initial_yaw_);
  if (this->startup_mode_is_fixed_start_pose()) {
    this->has_initial_pose_ = true;
    this->initialize_particles(this->initial_map_pose_);
  }

  this->odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->odom_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      this->handle_odometry(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });
  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      this->handle_map(message);
    });
  this->initial_pose_subscription_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->initial_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      this->handle_initial_pose(message);
    });
  this->initial_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->initial_pose_topic_, rclcpp::SystemDefaultsQoS());
  this->estimated_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->estimated_pose_topic_, rclcpp::SystemDefaultsQoS());
  this->estimated_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->estimated_odom_topic_, rclcpp::SystemDefaultsQoS());
  this->localization_status_publisher_ =
    this->create_publisher<amr_msgs::msg::LocalizationStatus>(
    this->localization_status_topic_, rclcpp::SystemDefaultsQoS());
  this->localization_candidates_publisher_ =
    this->create_publisher<amr_msgs::msg::LocalizationCandidateArray>(
    this->localization_candidates_topic_, rclcpp::SystemDefaultsQoS());
  this->trigger_global_localization_service_ =
    this->create_service<amr_msgs::srv::TriggerGlobalLocalization>(
    this->trigger_global_localization_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Request> request,
      std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Response> response)
    {
      this->handle_trigger_global_localization(request, response);
    });
  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(
    this->get_logger(),
    "Configured AMCL-lite localization with odom='%s', scan='%s', map='%s', particles=%d, status='%s', candidates='%s', trigger='%s', startup_mode='%s', startup_global=%s'",
    this->odom_topic_.c_str(),
    this->scan_topic_.c_str(),
    this->map_topic_.c_str(),
    this->particle_count_,
    this->localization_status_topic_.c_str(),
    this->localization_candidates_topic_.c_str(),
    this->trigger_global_localization_service_name_.c_str(),
    this->startup_localization_mode_.c_str(),
    this->kidnapped_start_with_global_localization_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_activate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_activate();
  }
  if (this->localization_status_publisher_) {
    this->localization_status_publisher_->on_activate();
  }
  if (this->localization_candidates_publisher_) {
    this->localization_candidates_publisher_->on_activate();
  }

  if (this->has_initial_pose_) {
    this->update_estimated_pose_from_particles(this->now());
    this->publish_outputs(this->now());
  }

  if (this->auto_initial_pose_enabled_ && !this->auto_initial_pose_published_) {
    this->auto_initial_pose_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(std::max(0.0, this->auto_initial_pose_delay_sec_))),
      [this]() { this->publish_auto_initial_pose(); });
  }
  RCLCPP_INFO(this->get_logger(), "Activated localization");
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_deactivate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_deactivate();
  }
  if (this->localization_status_publisher_) {
    this->localization_status_publisher_->on_deactivate();
  }
  if (this->localization_candidates_publisher_) {
    this->localization_candidates_publisher_->on_deactivate();
  }
  if (this->auto_initial_pose_timer_) {
    this->auto_initial_pose_timer_->cancel();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated localization");
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->scan_subscription_.reset();
  this->map_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->initial_pose_publisher_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
  this->localization_status_publisher_.reset();
  this->localization_candidates_publisher_.reset();
  this->trigger_global_localization_service_.reset();
  this->auto_initial_pose_timer_.reset();
  this->transform_broadcaster_.reset();
  this->reset_state();
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->scan_subscription_.reset();
  this->map_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->initial_pose_publisher_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
  this->localization_status_publisher_.reset();
  this->localization_candidates_publisher_.reset();
  this->trigger_global_localization_service_.reset();
  this->auto_initial_pose_timer_.reset();
  this->transform_broadcaster_.reset();
  this->reset_state();
  return CallbackReturn::SUCCESS;
}

void Localization::handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  this->latest_odom_ = *message;
  this->has_latest_odom_ = true;

  const auto current_odom_pose = this->odometry_pose_to_pose_stamped(*message);
  if (!this->has_previous_odom_) {
    this->previous_odom_pose_ = current_odom_pose;
    this->has_previous_odom_ = true;
    this->update_estimated_pose_from_particles(message->header.stamp);
    this->publish_outputs(message->header.stamp);
    return;
  }

  if (this->particles_initialized_) {
    this->apply_motion_update(this->previous_odom_pose_, current_odom_pose);
    this->update_estimated_pose_from_particles(message->header.stamp);
    this->publish_outputs(message->header.stamp);
  }

  this->previous_odom_pose_ = current_odom_pose;
}

void Localization::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_latest_scan_ = true;

  if (!this->particles_initialized_ || !this->has_map_) {
    return;
  }

  this->apply_measurement_update(*message);
  const auto stamp = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U ?
    this->now() :
    rclcpp::Time(message->header.stamp);
  this->update_localization_mode(stamp);
  this->resample_particles();
  this->update_estimated_pose_from_particles(stamp);
  this->publish_outputs(stamp);
}

void Localization::handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  this->map_occupancy_grid_ = message;
  this->has_map_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Received map: frame='%s' size=%u x %u resolution=%.3f",
    message->header.frame_id.c_str(),
    message->info.width,
    message->info.height,
    message->info.resolution);

  if (
    (this->startup_mode_is_global_relocalization() || this->startup_mode_is_active_relocalization()) &&
    !this->particles_initialized_ &&
    this->localization_mode_ == LocalizationMode::kTracking)
  {
    this->start_global_relocalization("startup global localization");
  }
}

void Localization::handle_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  this->initial_map_pose_.header = message->header;
  if (this->initial_map_pose_.header.frame_id.empty()) {
    this->initial_map_pose_.header.frame_id = this->map_frame_;
  }
  this->initial_map_pose_.pose = message->pose.pose;
  this->has_initial_pose_ = true;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = this->initial_map_pose_.header;
  pose.pose = this->initial_map_pose_.pose;
  this->initialize_particles(pose);
  this->localization_mode_ = LocalizationMode::kTracking;
  this->kidnapped_suspected_ = false;
  this->relocalization_requested_ = false;
  this->low_confidence_update_count_ = 0;
  this->relocalization_success_count_ = 0;
  this->last_measurement_confidence_ = 0.0;
  this->previous_candidate_tracks_.clear();
  this->next_candidate_id_ = 1U;

  if (this->has_latest_odom_) {
    this->previous_odom_pose_ = this->odometry_pose_to_pose_stamped(this->latest_odom_);
    this->has_previous_odom_ = true;
  }

  const auto stamp = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U ?
    this->now() :
    rclcpp::Time(message->header.stamp);
  this->update_estimated_pose_from_particles(stamp);
  this->publish_outputs(stamp);

  RCLCPP_INFO(
    this->get_logger(),
    "Applied initial pose: frame='%s' x=%.3f y=%.3f yaw=%.3f",
    this->initial_map_pose_.header.frame_id.c_str(),
    this->initial_map_pose_.pose.position.x,
    this->initial_map_pose_.pose.position.y,
    this->quaternion_yaw(this->initial_map_pose_.pose.orientation));
}

void Localization::publish_auto_initial_pose()
{
  if (this->auto_initial_pose_timer_) {
    this->auto_initial_pose_timer_->cancel();
    this->auto_initial_pose_timer_.reset();
  }
  if (!this->initial_pose_publisher_) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = this->now();
  initial_pose.header.frame_id = this->map_frame_;
  initial_pose.pose.pose.position.x = this->initial_x_;
  initial_pose.pose.pose.position.y = this->initial_y_;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = std::sin(this->initial_yaw_ * 0.5);
  initial_pose.pose.pose.orientation.w = std::cos(this->initial_yaw_ * 0.5);
  initial_pose.pose.covariance.fill(0.0);
  initial_pose.pose.covariance[0] = this->auto_initial_pose_covariance_x_;
  initial_pose.pose.covariance[7] = this->auto_initial_pose_covariance_y_;
  initial_pose.pose.covariance[35] = this->auto_initial_pose_covariance_yaw_;

  this->initial_pose_publisher_->publish(initial_pose);
  this->auto_initial_pose_published_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Published auto initial pose: frame='%s' x=%.3f y=%.3f yaw=%.3f",
    initial_pose.header.frame_id.c_str(),
    initial_pose.pose.pose.position.x,
    initial_pose.pose.pose.position.y,
    this->initial_yaw_);
}

void Localization::initialize_particles(const geometry_msgs::msg::PoseStamped & pose)
{
  this->particles_.clear();
  this->particles_.reserve(static_cast<std::size_t>(std::max(1, this->particle_count_)));

  const auto base_yaw = this->quaternion_yaw(pose.pose.orientation);
  const auto particle_count = std::max(1, this->particle_count_);
  const double uniform_weight = 1.0 / static_cast<double>(particle_count);

  for (int index = 0; index < particle_count; ++index) {
    Particle particle{};
    particle.x = pose.pose.position.x + this->sample_normal(this->initial_particle_std_xy_);
    particle.y = pose.pose.position.y + this->sample_normal(this->initial_particle_std_xy_);
    particle.yaw = this->normalize_angle(base_yaw + this->sample_normal(this->initial_particle_std_yaw_));
    particle.weight = uniform_weight;
    this->particles_.push_back(particle);
  }

  this->particles_initialized_ = true;
  this->localization_confidence_ = 0.0;
  this->localization_cluster_weight_ = 0.0;
  this->localization_cluster_dominance_ratio_ = 0.0;
  this->localization_position_std_ = std::numeric_limits<double>::infinity();
  this->localization_yaw_std_ = std::numeric_limits<double>::infinity();
  this->relocalization_candidate_locked_ = false;
}

bool Localization::initialize_particles_global()
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  this->particles_.clear();
  this->particles_.reserve(static_cast<std::size_t>(std::max(1, this->global_particle_count_)));

  const auto particle_count = std::max(1, this->global_particle_count_);
  const double uniform_weight = 1.0 / static_cast<double>(particle_count);

  for (int index = 0; index < particle_count; ++index) {
    Particle particle{};
    if (!this->sample_random_free_pose(particle)) {
      this->particles_.clear();
      return false;
    }
    particle.weight = uniform_weight;
    this->particles_.push_back(particle);
  }

  this->particles_initialized_ = true;
  this->localization_confidence_ = 0.0;
  this->localization_cluster_weight_ = 0.0;
  this->localization_cluster_dominance_ratio_ = 0.0;
  this->localization_position_std_ = std::numeric_limits<double>::infinity();
  this->localization_yaw_std_ = std::numeric_limits<double>::infinity();
  this->relocalization_candidate_locked_ = false;
  return true;
}

void Localization::apply_motion_update(
  const geometry_msgs::msg::PoseStamped & previous_odom_pose,
  const geometry_msgs::msg::PoseStamped & current_odom_pose)
{
  if (this->particles_.empty()) {
    return;
  }

  const double previous_odom_yaw = this->quaternion_yaw(previous_odom_pose.pose.orientation);
  const double current_odom_yaw = this->quaternion_yaw(current_odom_pose.pose.orientation);
  const double delta_odom_x = current_odom_pose.pose.position.x - previous_odom_pose.pose.position.x;
  const double delta_odom_y = current_odom_pose.pose.position.y - previous_odom_pose.pose.position.y;
  const double local_delta_x =
    (std::cos(previous_odom_yaw) * delta_odom_x) + (std::sin(previous_odom_yaw) * delta_odom_y);
  const double local_delta_y =
    (-std::sin(previous_odom_yaw) * delta_odom_x) + (std::cos(previous_odom_yaw) * delta_odom_y);
  const double delta_yaw = this->normalize_angle(current_odom_yaw - previous_odom_yaw);

  for (auto & particle : this->particles_) {
    const double noisy_local_x = local_delta_x + this->sample_normal(this->motion_noise_linear_);
    const double noisy_local_y = local_delta_y + this->sample_normal(this->motion_noise_lateral_);
    const double noisy_delta_yaw = delta_yaw + this->sample_normal(this->motion_noise_angular_);

    particle.x +=
      (std::cos(particle.yaw) * noisy_local_x) - (std::sin(particle.yaw) * noisy_local_y);
    particle.y +=
      (std::sin(particle.yaw) * noisy_local_x) + (std::cos(particle.yaw) * noisy_local_y);
    particle.yaw = this->normalize_angle(particle.yaw + noisy_delta_yaw);
  }
}

void Localization::apply_measurement_update(const sensor_msgs::msg::LaserScan & scan)
{
  if (this->particles_.empty() || !this->has_map_) {
    return;
  }

  double total_weight = 0.0;
  double best_weight = 0.0;
  for (auto & particle : this->particles_) {
    particle.weight = std::max(kMinimumWeight, this->compute_particle_likelihood(particle, scan));
    total_weight += particle.weight;
    best_weight = std::max(best_weight, particle.weight);
  }

  this->last_measurement_confidence_ = best_weight;

  if (total_weight <= 0.0) {
    const double uniform_weight = 1.0 / static_cast<double>(this->particles_.size());
    for (auto & particle : this->particles_) {
      particle.weight = uniform_weight;
    }
    this->localization_confidence_ = 0.0;
    return;
  }

  for (auto & particle : this->particles_) {
    particle.weight /= total_weight;
  }
  this->localization_confidence_ = best_weight;
}

void Localization::resample_particles()
{
  if (this->particles_.empty()) {
    return;
  }

  std::vector<Particle> resampled_particles;
  resampled_particles.reserve(this->particles_.size());

  const double step = 1.0 / static_cast<double>(this->particles_.size());
  std::uniform_real_distribution<double> distribution(0.0, step);
  double target = distribution(this->random_engine_);
  double cumulative_weight = this->particles_.front().weight;
  std::size_t index = 0U;

  for (std::size_t sample = 0; sample < this->particles_.size(); ++sample) {
    while (target > cumulative_weight && index + 1U < this->particles_.size()) {
      ++index;
      cumulative_weight += this->particles_[index].weight;
    }

    Particle particle = this->particles_[index];
    particle.weight = step;
    resampled_particles.push_back(particle);
    target += step;
  }

  if (this->localization_mode_ == LocalizationMode::kGlobalRelocalizing) {
    for (auto & particle : resampled_particles) {
      particle.x += this->sample_normal(this->global_resample_position_noise_);
      particle.y += this->sample_normal(this->global_resample_position_noise_);
      particle.yaw = this->normalize_angle(
        particle.yaw + this->sample_normal(this->global_resample_yaw_noise_));

      if (this->relocalization_candidate_locked_) {
        const double dx = particle.x - this->relocalization_candidate_pose_.pose.position.x;
        const double dy = particle.y - this->relocalization_candidate_pose_.pose.position.y;
        const double distance = std::sqrt((dx * dx) + (dy * dy));
        const double candidate_yaw =
          this->quaternion_yaw(this->relocalization_candidate_pose_.pose.orientation);
        const double yaw_delta = std::fabs(this->normalize_angle(particle.yaw - candidate_yaw));
        if (
          distance > this->relocalization_candidate_lock_distance_ ||
          yaw_delta > this->relocalization_candidate_lock_yaw_)
        {
          particle.x =
            this->relocalization_candidate_pose_.pose.position.x +
            this->sample_normal(std::max(0.05, this->relocalization_candidate_lock_distance_ * 0.35));
          particle.y =
            this->relocalization_candidate_pose_.pose.position.y +
            this->sample_normal(std::max(0.05, this->relocalization_candidate_lock_distance_ * 0.35));
          particle.yaw = this->normalize_angle(
            candidate_yaw +
            this->sample_normal(std::max(0.10, this->relocalization_candidate_lock_yaw_ * 0.35)));
        }
      }
    }
  }

  this->particles_ = std::move(resampled_particles);
}

void Localization::update_relocalization_candidate_lock()
{
  if (this->localization_mode_ != LocalizationMode::kGlobalRelocalizing) {
    this->relocalization_candidate_locked_ = false;
    return;
  }

  if (this->relocalization_observation_count_ < this->relocalization_candidate_lock_min_updates_) {
    return;
  }

  const bool should_lock =
    this->last_measurement_confidence_ >= this->relocalization_candidate_lock_confidence_threshold_ &&
    this->localization_cluster_weight_ >= this->relocalization_candidate_lock_cluster_weight_threshold_;

  if (!should_lock) {
    return;
  }

  this->relocalization_candidate_pose_ = this->estimated_pose_;
  this->relocalization_candidate_locked_ = true;
}

std::vector<Localization::CandidateCluster> Localization::extract_candidate_clusters() const
{
  std::vector<CandidateCluster> clusters;
  if (this->particles_.empty()) {
    return clusters;
  }

  std::vector<Particle> sorted_particles = this->particles_;
  std::sort(
    sorted_particles.begin(), sorted_particles.end(),
    [](const Particle & lhs, const Particle & rhs) {
      return lhs.weight > rhs.weight;
    });

  const double cluster_distance_sq =
    this->estimate_cluster_distance_ * this->estimate_cluster_distance_;

  for (const auto & seed_particle : sorted_particles) {
    bool overlaps_existing_cluster = false;
    for (const auto & cluster : clusters) {
      const double dx = seed_particle.x - cluster.x;
      const double dy = seed_particle.y - cluster.y;
      const double yaw_delta = this->normalize_angle(seed_particle.yaw - cluster.yaw);
      if (
        ((dx * dx) + (dy * dy)) <= cluster_distance_sq &&
        std::fabs(yaw_delta) <= this->estimate_cluster_yaw_)
      {
        overlaps_existing_cluster = true;
        break;
      }
    }
    if (overlaps_existing_cluster) {
      continue;
    }

    CandidateCluster cluster{};
    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weighted_sin_yaw = 0.0;
    double weighted_cos_yaw = 0.0;
    double total_weight = 0.0;
    double max_score = 0.0;
    double position_variance_accumulator = 0.0;
    double yaw_variance_accumulator = 0.0;
    double secondary_weight = 0.0;

    for (const auto & particle : this->particles_) {
      const double dx = particle.x - seed_particle.x;
      const double dy = particle.y - seed_particle.y;
      const double yaw_delta = this->normalize_angle(particle.yaw - seed_particle.yaw);
      if (
        ((dx * dx) + (dy * dy)) > cluster_distance_sq ||
        std::fabs(yaw_delta) > this->estimate_cluster_yaw_)
      {
        secondary_weight = std::max(secondary_weight, particle.weight);
        continue;
      }

      weighted_x += particle.x * particle.weight;
      weighted_y += particle.y * particle.weight;
      weighted_sin_yaw += std::sin(particle.yaw) * particle.weight;
      weighted_cos_yaw += std::cos(particle.yaw) * particle.weight;
      total_weight += particle.weight;
      max_score = std::max(max_score, particle.weight);
    }

    if (total_weight <= 0.0) {
      continue;
    }

    cluster.x = weighted_x / total_weight;
    cluster.y = weighted_y / total_weight;
    cluster.yaw = std::atan2(weighted_sin_yaw / total_weight, weighted_cos_yaw / total_weight);
    cluster.score = max_score;
    cluster.cluster_weight = total_weight;
    cluster.dominance_ratio = total_weight / std::max(secondary_weight, 1e-6);

    for (const auto & particle : this->particles_) {
      const double dx = particle.x - seed_particle.x;
      const double dy = particle.y - seed_particle.y;
      const double yaw_delta = this->normalize_angle(particle.yaw - seed_particle.yaw);
      if (
        ((dx * dx) + (dy * dy)) > cluster_distance_sq ||
        std::fabs(yaw_delta) > this->estimate_cluster_yaw_)
      {
        continue;
      }

      const double centered_dx = particle.x - cluster.x;
      const double centered_dy = particle.y - cluster.y;
      const double centered_yaw = this->normalize_angle(particle.yaw - cluster.yaw);
      position_variance_accumulator +=
        ((centered_dx * centered_dx) + (centered_dy * centered_dy)) * particle.weight;
      yaw_variance_accumulator += (centered_yaw * centered_yaw) * particle.weight;
    }

    cluster.position_std = std::sqrt(std::max(0.0, position_variance_accumulator / total_weight));
    cluster.yaw_std = std::sqrt(std::max(0.0, yaw_variance_accumulator / total_weight));
    clusters.push_back(cluster);

    if (static_cast<int>(clusters.size()) >= std::max(1, this->relocalization_max_candidates_)) {
      break;
    }
  }

  return clusters;
}

amr_msgs::msg::LocalizationCandidateArray Localization::build_candidate_array_message(
  const rclcpp::Time & stamp,
  const std::vector<CandidateCluster> & clusters)
{
  amr_msgs::msg::LocalizationCandidateArray message;
  message.header.stamp = stamp;
  message.header.frame_id = this->map_frame_;
  message.primary_candidate_id = 0U;

  std::vector<CandidateTrack> updated_tracks;
  updated_tracks.reserve(clusters.size());
  std::vector<bool> previous_track_used(this->previous_candidate_tracks_.size(), false);

  for (const auto & cluster : clusters) {
    int matched_index = -1;
    double best_match_cost = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < this->previous_candidate_tracks_.size(); ++index) {
      if (previous_track_used[index]) {
        continue;
      }
      const auto & previous_track = this->previous_candidate_tracks_[index];
      const double dx = cluster.x - previous_track.x;
      const double dy = cluster.y - previous_track.y;
      const double distance = std::sqrt((dx * dx) + (dy * dy));
      const double yaw_delta = std::fabs(this->normalize_angle(cluster.yaw - previous_track.yaw));
      if (
        distance > this->relocalization_candidate_match_distance_ ||
        yaw_delta > this->relocalization_candidate_match_yaw_)
      {
        continue;
      }

      const double match_cost = distance + (0.25 * yaw_delta);
      if (match_cost < best_match_cost) {
        best_match_cost = match_cost;
        matched_index = static_cast<int>(index);
      }
    }

    CandidateTrack track{};
    if (matched_index >= 0) {
      previous_track_used[static_cast<std::size_t>(matched_index)] = true;
      track.id = this->previous_candidate_tracks_[static_cast<std::size_t>(matched_index)].id;
    } else {
      track.id = this->next_candidate_id_++;
    }

    track.x = cluster.x;
    track.y = cluster.y;
    track.yaw = cluster.yaw;
    track.score = cluster.score;
    updated_tracks.push_back(track);

    amr_msgs::msg::LocalizationCandidate candidate;
    candidate.candidate_id = track.id;
    candidate.pose.header = message.header;
    candidate.pose.pose.position.x = cluster.x;
    candidate.pose.pose.position.y = cluster.y;
    candidate.pose.pose.position.z = 0.0;
    candidate.pose.pose.orientation.x = 0.0;
    candidate.pose.pose.orientation.y = 0.0;
    candidate.pose.pose.orientation.z = std::sin(cluster.yaw * 0.5);
    candidate.pose.pose.orientation.w = std::cos(cluster.yaw * 0.5);
    candidate.score = cluster.score;
    candidate.cluster_weight = cluster.cluster_weight;
    candidate.dominance_ratio = cluster.dominance_ratio;
    candidate.position_std = cluster.position_std;
    candidate.yaw_std = cluster.yaw_std;
    message.candidates.push_back(candidate);
  }

  if (!message.candidates.empty()) {
    message.primary_candidate_id = message.candidates.front().candidate_id;
  }
  this->previous_candidate_tracks_ = std::move(updated_tracks);
  return message;
}

void Localization::update_estimated_pose_from_particles(const rclcpp::Time & stamp)
{
  if (this->particles_.empty()) {
    this->estimated_pose_ = this->initial_map_pose_;
    this->estimated_pose_.header.stamp = stamp;
    this->estimated_pose_.header.frame_id = this->map_frame_;
    return;
  }

  const auto best_particle_it = std::max_element(
    this->particles_.begin(), this->particles_.end(),
    [](const Particle & lhs, const Particle & rhs) {
      return lhs.weight < rhs.weight;
    });
  const Particle & best_particle = *best_particle_it;
  const double cluster_distance_sq = this->estimate_cluster_distance_ * this->estimate_cluster_distance_;
  double secondary_cluster_seed_weight = 0.0;
  Particle secondary_cluster_seed{};

  double weighted_x = 0.0;
  double weighted_y = 0.0;
  double weighted_sin_yaw = 0.0;
  double weighted_cos_yaw = 0.0;
  double total_weight = 0.0;
  double position_variance_accumulator = 0.0;
  double yaw_variance_accumulator = 0.0;

  for (const auto & particle : this->particles_) {
    const double dx = particle.x - best_particle.x;
    const double dy = particle.y - best_particle.y;
    const double distance_sq = (dx * dx) + (dy * dy);
    const double yaw_delta = this->normalize_angle(particle.yaw - best_particle.yaw);
    if (distance_sq > cluster_distance_sq || std::fabs(yaw_delta) > this->estimate_cluster_yaw_) {
      if (particle.weight > secondary_cluster_seed_weight) {
        secondary_cluster_seed_weight = particle.weight;
        secondary_cluster_seed = particle;
      }
      continue;
    }

    weighted_x += particle.x * particle.weight;
    weighted_y += particle.y * particle.weight;
    weighted_sin_yaw += std::sin(particle.yaw) * particle.weight;
    weighted_cos_yaw += std::cos(particle.yaw) * particle.weight;
    total_weight += particle.weight;
  }

  if (total_weight <= 0.0) {
    weighted_x = best_particle.x;
    weighted_y = best_particle.y;
    weighted_sin_yaw = std::sin(best_particle.yaw);
    weighted_cos_yaw = std::cos(best_particle.yaw);
    total_weight = 1.0;
  }

  const double mean_x = weighted_x / total_weight;
  const double mean_y = weighted_y / total_weight;
  const double mean_yaw = std::atan2(weighted_sin_yaw / total_weight, weighted_cos_yaw / total_weight);
  double secondary_cluster_weight = 0.0;

  if (secondary_cluster_seed_weight > 0.0) {
    for (const auto & particle : this->particles_) {
      const double dx = particle.x - secondary_cluster_seed.x;
      const double dy = particle.y - secondary_cluster_seed.y;
      const double distance_sq = (dx * dx) + (dy * dy);
      const double yaw_delta = this->normalize_angle(particle.yaw - secondary_cluster_seed.yaw);
      if (distance_sq > cluster_distance_sq || std::fabs(yaw_delta) > this->estimate_cluster_yaw_) {
        continue;
      }
      secondary_cluster_weight += particle.weight;
    }
  }

  for (const auto & particle : this->particles_) {
    const double dx = particle.x - best_particle.x;
    const double dy = particle.y - best_particle.y;
    const double distance_sq = (dx * dx) + (dy * dy);
    const double yaw_delta = this->normalize_angle(particle.yaw - best_particle.yaw);
    if (distance_sq > cluster_distance_sq || std::fabs(yaw_delta) > this->estimate_cluster_yaw_) {
      continue;
    }

    const double centered_dx = particle.x - mean_x;
    const double centered_dy = particle.y - mean_y;
    const double centered_yaw = this->normalize_angle(particle.yaw - mean_yaw);
    position_variance_accumulator +=
      ((centered_dx * centered_dx) + (centered_dy * centered_dy)) * particle.weight;
    yaw_variance_accumulator += (centered_yaw * centered_yaw) * particle.weight;
  }

  this->localization_cluster_weight_ = total_weight;
  this->localization_cluster_dominance_ratio_ =
    total_weight / std::max(secondary_cluster_weight, 1e-6);
  this->localization_position_std_ = std::sqrt(std::max(0.0, position_variance_accumulator / total_weight));
  this->localization_yaw_std_ = std::sqrt(std::max(0.0, yaw_variance_accumulator / total_weight));

  this->estimated_pose_.header.stamp = stamp;
  this->estimated_pose_.header.frame_id = this->map_frame_;
  this->estimated_pose_.pose.position.x = mean_x;
  this->estimated_pose_.pose.position.y = mean_y;
  this->estimated_pose_.pose.position.z = 0.0;
  this->update_pose_orientation(this->estimated_pose_, mean_yaw);
}

void Localization::publish_outputs(const rclcpp::Time & stamp)
{
  this->publish_localization_status(stamp);
  this->publish_localization_candidates(stamp);

  if (
    !this->estimated_pose_publisher_ || !this->estimated_pose_publisher_->is_activated() ||
    !this->estimated_odometry_publisher_ || !this->estimated_odometry_publisher_->is_activated())
  {
    return;
  }

  this->estimated_pose_publisher_->publish(this->estimated_pose_);

  nav_msgs::msg::Odometry estimated_odometry;
  estimated_odometry.header = this->estimated_pose_.header;
  estimated_odometry.child_frame_id = this->base_frame_;
  estimated_odometry.pose.pose = this->estimated_pose_.pose;
  estimated_odometry.twist = this->latest_odom_.twist;
  this->estimated_odometry_publisher_->publish(estimated_odometry);

  if (this->transform_broadcaster_ && this->has_latest_odom_) {
    this->transform_broadcaster_->sendTransform(this->build_map_to_odom_transform(stamp));
  }
}

void Localization::publish_localization_status(const rclcpp::Time & stamp)
{
  if (!this->localization_status_publisher_ || !this->localization_status_publisher_->is_activated()) {
    return;
  }

  amr_msgs::msg::LocalizationStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = this->map_frame_;
  status.active = this->particles_initialized_;
  status.mode = static_cast<uint8_t>(this->localization_mode_);
  status.kidnapped_suspected = this->kidnapped_suspected_;
  status.relocalization_requested = this->relocalization_requested_;
  status.relocalization_count = this->relocalization_count_;
  status.confidence = std::clamp(this->localization_confidence_, 0.0, 1.0);
  this->localization_status_publisher_->publish(status);
}

void Localization::publish_localization_candidates(const rclcpp::Time & stamp)
{
  if (
    !this->localization_candidates_publisher_ ||
    !this->localization_candidates_publisher_->is_activated())
  {
    return;
  }

  const auto clusters = this->extract_candidate_clusters();
  auto message = this->build_candidate_array_message(stamp, clusters);
  this->localization_candidates_publisher_->publish(message);
}

geometry_msgs::msg::TransformStamped Localization::build_map_to_odom_transform(
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = this->map_frame_;
  transform.child_frame_id = this->odom_frame_;

  const auto current_odom_pose = this->odometry_pose_to_pose_stamped(this->latest_odom_);
  const double map_yaw = this->quaternion_yaw(this->estimated_pose_.pose.orientation);
  const double odom_yaw = this->quaternion_yaw(current_odom_pose.pose.orientation);
  const double yaw_delta = this->normalize_angle(map_yaw - odom_yaw);
  const double odom_x = current_odom_pose.pose.position.x;
  const double odom_y = current_odom_pose.pose.position.y;

  transform.transform.translation.x =
    this->estimated_pose_.pose.position.x -
    ((std::cos(yaw_delta) * odom_x) - (std::sin(yaw_delta) * odom_y));
  transform.transform.translation.y =
    this->estimated_pose_.pose.position.y -
    ((std::sin(yaw_delta) * odom_x) + (std::cos(yaw_delta) * odom_y));
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = std::sin(yaw_delta * 0.5);
  transform.transform.rotation.w = std::cos(yaw_delta * 0.5);
  return transform;
}

geometry_msgs::msg::PoseStamped Localization::odometry_pose_to_pose_stamped(
  const nav_msgs::msg::Odometry & odometry) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = this->odom_frame_;
  }
  pose.pose = odometry.pose.pose;
  return pose;
}

bool Localization::world_to_grid(double world_x, double world_y, int & grid_x, int & grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  const auto & info = this->map_occupancy_grid_->info;
  const double origin_yaw = this->quaternion_yaw(info.origin.orientation);
  const double relative_x = world_x - info.origin.position.x;
  const double relative_y = world_y - info.origin.position.y;
  const double local_x =
    (std::cos(origin_yaw) * relative_x) + (std::sin(origin_yaw) * relative_y);
  const double local_y =
    (-std::sin(origin_yaw) * relative_x) + (std::cos(origin_yaw) * relative_y);

  grid_x = static_cast<int>(std::floor(local_x / static_cast<double>(info.resolution)));
  grid_y = static_cast<int>(std::floor(local_y / static_cast<double>(info.resolution)));

  return
    grid_x >= 0 &&
    grid_y >= 0 &&
    grid_x < static_cast<int>(info.width) &&
    grid_y < static_cast<int>(info.height);
}

bool Localization::grid_to_world(
  const int grid_x,
  const int grid_y,
  double & world_x,
  double & world_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  const auto & info = this->map_occupancy_grid_->info;
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(info.width) ||
    grid_y >= static_cast<int>(info.height))
  {
    return false;
  }

  const double local_x = (static_cast<double>(grid_x) + 0.5) * static_cast<double>(info.resolution);
  const double local_y = (static_cast<double>(grid_y) + 0.5) * static_cast<double>(info.resolution);
  const double origin_yaw = this->quaternion_yaw(info.origin.orientation);
  world_x =
    info.origin.position.x +
    (std::cos(origin_yaw) * local_x) -
    (std::sin(origin_yaw) * local_y);
  world_y =
    info.origin.position.y +
    (std::sin(origin_yaw) * local_x) +
    (std::cos(origin_yaw) * local_y);
  return true;
}

bool Localization::is_occupied_cell(const int grid_x, const int grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(this->map_occupancy_grid_->info.width) ||
    grid_y >= static_cast<int>(this->map_occupancy_grid_->info.height))
  {
    return false;
  }

  const auto index =
    static_cast<std::size_t>((grid_y * static_cast<int>(this->map_occupancy_grid_->info.width)) + grid_x);
  return this->map_occupancy_grid_->data[index] >= this->occupied_threshold_;
}

bool Localization::is_free_cell(const int grid_x, const int grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(this->map_occupancy_grid_->info.width) ||
    grid_y >= static_cast<int>(this->map_occupancy_grid_->info.height))
  {
    return false;
  }

  const auto index =
    static_cast<std::size_t>((grid_y * static_cast<int>(this->map_occupancy_grid_->info.width)) + grid_x);
  const int8_t value = this->map_occupancy_grid_->data[index];
  return value >= 0 && value < this->occupied_threshold_;
}

bool Localization::sample_random_free_pose(Particle & particle)
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  const auto width = static_cast<int>(this->map_occupancy_grid_->info.width);
  const auto height = static_cast<int>(this->map_occupancy_grid_->info.height);
  if (width <= 0 || height <= 0) {
    return false;
  }

  std::uniform_int_distribution<int> x_distribution(0, width - 1);
  std::uniform_int_distribution<int> y_distribution(0, height - 1);
  std::uniform_real_distribution<double> yaw_distribution(-kPi, kPi);

  for (int attempt = 0; attempt < std::max(1, this->global_particle_sample_attempts_); ++attempt) {
    const int grid_x = x_distribution(this->random_engine_);
    const int grid_y = y_distribution(this->random_engine_);
    if (!this->is_free_cell(grid_x, grid_y)) {
      continue;
    }

    double world_x = 0.0;
    double world_y = 0.0;
    if (!this->grid_to_world(grid_x, grid_y, world_x, world_y)) {
      continue;
    }

    particle.x = world_x;
    particle.y = world_y;
    particle.yaw = yaw_distribution(this->random_engine_);
    return true;
  }

  return false;
}

double Localization::raycast_obstacle_range(
  const double world_x,
  const double world_y,
  const double angle,
  const double max_range) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return max_range;
  }

  const double step = std::max(
    0.01,
    static_cast<double>(this->map_occupancy_grid_->info.resolution) * 0.5);
  double traveled = 0.0;

  while (traveled <= max_range) {
    const double sample_x = world_x + (traveled * std::cos(angle));
    const double sample_y = world_y + (traveled * std::sin(angle));
    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_grid(sample_x, sample_y, grid_x, grid_y)) {
      return std::min(traveled, max_range);
    }
    if (this->is_occupied_cell(grid_x, grid_y)) {
      return traveled;
    }
    traveled += step;
  }

  return max_range;
}

double Localization::nearest_obstacle_distance(const double world_x, const double world_y) const
{
  int center_x = 0;
  int center_y = 0;
  if (!this->world_to_grid(world_x, world_y, center_x, center_y)) {
    return this->max_beam_range_;
  }

  double minimum_distance = std::numeric_limits<double>::max();
  for (int offset_y = -this->measurement_search_radius_cells_;
    offset_y <= this->measurement_search_radius_cells_;
    ++offset_y)
  {
    for (int offset_x = -this->measurement_search_radius_cells_;
      offset_x <= this->measurement_search_radius_cells_;
      ++offset_x)
    {
      const int grid_x = center_x + offset_x;
      const int grid_y = center_y + offset_y;
      if (!this->is_occupied_cell(grid_x, grid_y)) {
        continue;
      }

      const double distance_cells =
        std::sqrt(static_cast<double>((offset_x * offset_x) + (offset_y * offset_y)));
      minimum_distance = std::min(
        minimum_distance,
        distance_cells * static_cast<double>(this->map_occupancy_grid_->info.resolution));
    }
  }

  if (minimum_distance == std::numeric_limits<double>::max()) {
    return this->max_beam_range_;
  }

  return minimum_distance;
}

double Localization::compute_particle_likelihood(
  const Particle & particle,
  const sensor_msgs::msg::LaserScan & scan) const
{
  if (!this->has_map_ || scan.ranges.empty()) {
    return kMinimumWeight;
  }

  const auto beam_count = static_cast<int>(scan.ranges.size());
  const auto sampled_beam_count = std::max(1, std::min(this->max_beams_, beam_count));
  const auto stride = std::max(1, beam_count / sampled_beam_count);
  const double effective_max_range = std::min(this->max_beam_range_, static_cast<double>(scan.range_max));
  double accumulated_score = 0.0;
  int used_beams = 0;

  for (int beam_index = 0; beam_index < beam_count; beam_index += stride) {
    const double beam_range = static_cast<double>(scan.ranges[beam_index]);
    if (
      !std::isfinite(beam_range) ||
      beam_range < static_cast<double>(scan.range_min) ||
      beam_range > effective_max_range)
    {
      continue;
    }

    const double beam_angle =
      static_cast<double>(scan.angle_min) +
      (static_cast<double>(beam_index) * static_cast<double>(scan.angle_increment)) +
      particle.yaw;
    const double expected_range =
      this->raycast_obstacle_range(particle.x, particle.y, beam_angle, effective_max_range);
    const double range_error = beam_range - expected_range;
    const double normalized_distance = range_error / std::max(this->measurement_sigma_, 1e-3);
    const double score = std::exp(-0.5 * normalized_distance * normalized_distance);

    accumulated_score += score;
    ++used_beams;
  }

  if (used_beams == 0) {
    return kMinimumWeight;
  }

  return std::max(kMinimumWeight, accumulated_score / static_cast<double>(used_beams));
}

void Localization::start_global_relocalization(const std::string & reason)
{
  if (!this->kidnapped_detection_enabled_) {
    return;
  }

  if (!this->initialize_particles_global()) {
    this->localization_mode_ = LocalizationMode::kFailed;
    this->kidnapped_suspected_ = true;
    this->relocalization_requested_ = false;
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to start global relocalization because no free-space particle set could be initialized.");
    return;
  }

  this->localization_mode_ = LocalizationMode::kGlobalRelocalizing;
  this->kidnapped_suspected_ = true;
  this->relocalization_requested_ = true;
  this->relocalization_count_ += 1U;
  this->relocalization_started_at_ = this->now();
  this->low_confidence_update_count_ = 0;
  this->relocalization_success_count_ = 0;
  this->relocalization_observation_count_ = 0;
  this->last_measurement_confidence_ = 0.0;
  this->relocalization_candidate_locked_ = false;
  this->previous_candidate_tracks_.clear();
  this->next_candidate_id_ = 1U;

  RCLCPP_WARN(
    this->get_logger(),
    "Starting global relocalization attempt %u (%s)",
    this->relocalization_count_,
    reason.empty() ? "unspecified" : reason.c_str());
}

void Localization::update_localization_mode(const rclcpp::Time & stamp)
{
  if (!this->kidnapped_detection_enabled_) {
    this->localization_mode_ = LocalizationMode::kTracking;
    this->kidnapped_suspected_ = false;
    this->relocalization_requested_ = false;
    return;
  }

  const double confidence = std::clamp(this->last_measurement_confidence_, 0.0, 1.0);

  if (this->localization_mode_ == LocalizationMode::kTracking) {
    if (confidence < this->kidnapped_low_confidence_threshold_) {
      this->low_confidence_update_count_ += 1;
    } else {
      this->low_confidence_update_count_ = 0;
      this->kidnapped_suspected_ = false;
    }

    if (
      this->kidnapped_auto_trigger_enabled_ &&
      this->low_confidence_update_count_ >= this->kidnapped_low_confidence_updates_)
    {
      this->start_global_relocalization("low measurement confidence");
    }
    return;
  }

  if (this->localization_mode_ == LocalizationMode::kGlobalRelocalizing) {
    this->relocalization_observation_count_ += 1;
    this->update_relocalization_candidate_lock();
    const bool relocalization_converged =
      confidence >= this->relocalization_success_confidence_threshold_ &&
      this->localization_cluster_weight_ >= this->relocalization_min_cluster_weight_ &&
      this->localization_cluster_dominance_ratio_ >=
      this->relocalization_min_cluster_dominance_ratio_ &&
      this->localization_position_std_ <= this->relocalization_max_position_std_ &&
      this->localization_yaw_std_ <= this->relocalization_max_yaw_std_;
    if (relocalization_converged) {
      this->relocalization_success_count_ += 1;
    } else {
      this->relocalization_success_count_ = 0;
    }

    if (this->relocalization_success_count_ >= this->relocalization_success_updates_) {
      this->localization_mode_ = LocalizationMode::kTracking;
      this->kidnapped_suspected_ = false;
      this->relocalization_requested_ = false;
      this->low_confidence_update_count_ = 0;
      this->relocalization_success_count_ = 0;
      RCLCPP_INFO(
        this->get_logger(),
        "Global relocalization succeeded with confidence %.3f, cluster_weight %.3f, dominance %.3f, position_std %.3f, yaw_std %.3f",
        confidence,
        this->localization_cluster_weight_,
        this->localization_cluster_dominance_ratio_,
        this->localization_position_std_,
        this->localization_yaw_std_);
      return;
    }

    if (
      this->relocalization_started_at_.nanoseconds() > 0 &&
      (stamp - this->relocalization_started_at_).seconds() >= this->relocalization_timeout_sec_)
    {
      this->localization_mode_ = LocalizationMode::kFailed;
      this->relocalization_requested_ = false;
      this->relocalization_success_count_ = 0;
      RCLCPP_ERROR(
        this->get_logger(),
        "Global relocalization timed out after %.2f sec",
        this->relocalization_timeout_sec_);
    }
    return;
  }
}

void Localization::handle_trigger_global_localization(
  const std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Request> request,
  std::shared_ptr<amr_msgs::srv::TriggerGlobalLocalization::Response> response)
{
  if (!this->has_map_) {
    response->accepted = false;
    response->message = "Map is not available yet.";
    return;
  }

  if (!this->kidnapped_detection_enabled_) {
    response->accepted = false;
    response->message = "Kidnapped/global relocalization is disabled.";
    return;
  }

  if (this->localization_mode_ == LocalizationMode::kGlobalRelocalizing) {
    response->accepted = false;
    response->message = "Global relocalization is already in progress.";
    return;
  }

  this->start_global_relocalization(request->reason);
  response->accepted = this->localization_mode_ == LocalizationMode::kGlobalRelocalizing;
  response->message = response->accepted ?
    std::string("Global relocalization started.") :
    std::string("Failed to start global relocalization.");
}

bool Localization::startup_mode_is_manual_set_initial_pose() const
{
  return this->startup_localization_mode_ == "manual_set_initial_pose";
}

bool Localization::startup_mode_is_global_relocalization() const
{
  return this->startup_localization_mode_ == "global_relocalization";
}

bool Localization::startup_mode_is_active_relocalization() const
{
  return this->startup_localization_mode_ == "active_relocalization";
}

bool Localization::startup_mode_is_fixed_start_pose() const
{
  return this->startup_localization_mode_ == "fixed_start_pose";
}

double Localization::sample_normal(const double stddev)
{
  if (stddev <= 0.0) {
    return 0.0;
  }

  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(this->random_engine_);
}

double Localization::normalize_angle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double Localization::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp =
    2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp =
    1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

void Localization::update_pose_orientation(
  geometry_msgs::msg::PoseStamped & pose,
  const double yaw) const
{
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
}

void Localization::reset_state()
{
  this->latest_odom_ = nav_msgs::msg::Odometry();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  this->initial_map_pose_ = geometry_msgs::msg::PoseStamped();
  this->previous_odom_pose_ = geometry_msgs::msg::PoseStamped();
  this->estimated_pose_ = geometry_msgs::msg::PoseStamped();
  this->particles_.clear();
  this->localization_mode_ = LocalizationMode::kTracking;
  this->kidnapped_suspected_ = false;
  this->relocalization_requested_ = false;
  this->relocalization_count_ = 0U;
  this->localization_confidence_ = 0.0;
  this->last_measurement_confidence_ = 0.0;
  this->localization_cluster_weight_ = 0.0;
  this->localization_cluster_dominance_ratio_ = 0.0;
  this->localization_position_std_ = std::numeric_limits<double>::infinity();
  this->localization_yaw_std_ = std::numeric_limits<double>::infinity();
  this->low_confidence_update_count_ = 0;
  this->relocalization_success_count_ = 0;
  this->relocalization_observation_count_ = 0;
  this->relocalization_started_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  this->relocalization_candidate_locked_ = false;
  this->relocalization_candidate_pose_ = geometry_msgs::msg::PoseStamped();
  this->previous_candidate_tracks_.clear();
  this->next_candidate_id_ = 1U;
  this->has_latest_odom_ = false;
  this->has_latest_scan_ = false;
  this->has_map_ = false;
  this->has_previous_odom_ = false;
  this->has_initial_pose_ = false;
  this->particles_initialized_ = false;
  this->auto_initial_pose_published_ = false;
}

}  // namespace amr_localization
