#include "amr_localization/localization.hpp"

namespace amr::localization::estimator
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumWeight = 1e-6;

}  // namespace

Localization::Localization(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("localization", options),
  odom_topic_(""),
  scan_topic_(""),
  map_topic_(""),
  initial_pose_topic_(""),
  estimated_pose_topic_(""),
  estimated_odom_topic_(""),
  map_frame_("map"),
  odom_frame_("odom"),
  base_frame_("base_link"),
  initial_x_(0.0),
  initial_y_(0.0),
  initial_yaw_(0.0),
  start_pose_enabled_(true),
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
  map_occupancy_grid_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  random_engine_(std::random_device{}()),
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
  this->declare_parameter("frames.map", this->map_frame_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("start_pose.x", this->initial_x_);
  this->declare_parameter("start_pose.y", this->initial_y_);
  this->declare_parameter("start_pose.yaw", this->initial_yaw_);
  this->declare_parameter("start_pose.enabled", this->start_pose_enabled_);
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
}

Localization::CallbackReturn Localization::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.initial_pose", this->initial_pose_topic_);
  this->get_parameter("topics.estimated_pose", this->estimated_pose_topic_);
  this->get_parameter("topics.estimated_odometry", this->estimated_odom_topic_);
  this->get_parameter("frames.map", this->map_frame_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("start_pose.x", this->initial_x_);
  this->get_parameter("start_pose.y", this->initial_y_);
  this->get_parameter("start_pose.yaw", this->initial_yaw_);
  this->get_parameter("start_pose.enabled", this->start_pose_enabled_);
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

  if (
    this->odom_topic_.empty() || this->scan_topic_.empty() || this->map_topic_.empty() ||
    this->initial_pose_topic_.empty() || this->estimated_pose_topic_.empty() ||
    this->estimated_odom_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Localization topics must not be empty: odom='%s' scan='%s' map='%s' initial_pose='%s' pose='%s' odometry='%s'",
      this->odom_topic_.c_str(),
      this->scan_topic_.c_str(),
      this->map_topic_.c_str(),
      this->initial_pose_topic_.c_str(),
      this->estimated_pose_topic_.c_str(),
      this->estimated_odom_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->reset_state();

  this->initial_map_pose_.header.frame_id = this->map_frame_;
  this->initial_map_pose_.pose.position.x = this->initial_x_;
  this->initial_map_pose_.pose.position.y = this->initial_y_;
  this->initial_map_pose_.pose.position.z = 0.0;
  this->update_pose_orientation(this->initial_map_pose_, this->initial_yaw_);
  if (this->start_pose_enabled_) {
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
  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(
    this->get_logger(),
    "Configured AMCL-lite localization with odom='%s', scan='%s', map='%s', particles=%d, start_pose_enabled=%s",
    this->odom_topic_.c_str(),
    this->scan_topic_.c_str(),
    this->map_topic_.c_str(),
    this->particle_count_,
    this->start_pose_enabled_ ? "true" : "false");
  if (!this->start_pose_enabled_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Localization will not publish map->odom until an explicit initial pose is received on '%s'",
      this->initial_pose_topic_.c_str());
  }
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_activate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_activate();
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

Localization::CallbackReturn Localization::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->estimated_pose_publisher_) {
    this->estimated_pose_publisher_->on_deactivate();
  }
  if (this->estimated_odometry_publisher_) {
    this->estimated_odometry_publisher_->on_deactivate();
  }
  if (this->auto_initial_pose_timer_) {
    this->auto_initial_pose_timer_->cancel();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated localization");
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->scan_subscription_.reset();
  this->map_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->initial_pose_publisher_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
  this->auto_initial_pose_timer_.reset();
  this->transform_broadcaster_.reset();
  this->reset_state();
  return CallbackReturn::SUCCESS;
}

Localization::CallbackReturn Localization::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->odometry_subscription_.reset();
  this->scan_subscription_.reset();
  this->map_subscription_.reset();
  this->initial_pose_subscription_.reset();
  this->initial_pose_publisher_.reset();
  this->estimated_pose_publisher_.reset();
  this->estimated_odometry_publisher_.reset();
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
    if (!this->particles_initialized_) {
      return;
    }
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
  this->resample_particles();

  const auto stamp = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U ?
    this->now() :
    rclcpp::Time(message->header.stamp);
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

void Localization::initialize_particles(const geometry_msgs::msg::PoseStamped &pose)
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
}

void Localization::apply_motion_update(
  const geometry_msgs::msg::PoseStamped &previous_odom_pose,
  const geometry_msgs::msg::PoseStamped &current_odom_pose)
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

  for (auto &particle : this->particles_) {
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

void Localization::apply_measurement_update(const sensor_msgs::msg::LaserScan &scan)
{
  if (this->particles_.empty() || !this->has_map_) {
    return;
  }

  double total_weight = 0.0;
  for (auto &particle : this->particles_) {
    particle.weight = std::max(kMinimumWeight, this->compute_particle_likelihood(particle, scan));
    total_weight += particle.weight;
  }

  if (total_weight <= 0.0) {
    const double uniform_weight = 1.0 / static_cast<double>(this->particles_.size());
    for (auto &particle : this->particles_) {
      particle.weight = uniform_weight;
    }
    return;
  }

  for (auto &particle : this->particles_) {
    particle.weight /= total_weight;
  }
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

  this->particles_ = std::move(resampled_particles);
}

void Localization::update_estimated_pose_from_particles(const rclcpp::Time &stamp)
{
  if (this->particles_.empty()) {
    this->estimated_pose_ = this->initial_map_pose_;
    this->estimated_pose_.header.stamp = stamp;
    this->estimated_pose_.header.frame_id = this->map_frame_;
    return;
  }

  double weighted_x = 0.0;
  double weighted_y = 0.0;
  double weighted_sin_yaw = 0.0;
  double weighted_cos_yaw = 0.0;
  double total_weight = 0.0;

  for (const auto &particle : this->particles_) {
    weighted_x += particle.x * particle.weight;
    weighted_y += particle.y * particle.weight;
    weighted_sin_yaw += std::sin(particle.yaw) * particle.weight;
    weighted_cos_yaw += std::cos(particle.yaw) * particle.weight;
    total_weight += particle.weight;
  }

  if (total_weight <= 0.0) {
    total_weight = 1.0;
  }

  this->estimated_pose_.header.stamp = stamp;
  this->estimated_pose_.header.frame_id = this->map_frame_;
  this->estimated_pose_.pose.position.x = weighted_x / total_weight;
  this->estimated_pose_.pose.position.y = weighted_y / total_weight;
  this->estimated_pose_.pose.position.z = 0.0;
  this->update_pose_orientation(
    this->estimated_pose_,
    std::atan2(weighted_sin_yaw / total_weight, weighted_cos_yaw / total_weight));
}

void Localization::publish_outputs(const rclcpp::Time &stamp)
{
  if (!this->particles_initialized_) {
    return;
  }

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

geometry_msgs::msg::TransformStamped Localization::build_map_to_odom_transform(
  const rclcpp::Time &stamp) const
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
  const nav_msgs::msg::Odometry &odometry) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = this->odom_frame_;
  }
  pose.pose = odometry.pose.pose;
  return pose;
}

bool Localization::world_to_grid(double world_x, double world_y, int &grid_x, int &grid_y) const
{
  if (!this->has_map_ || !this->map_occupancy_grid_) {
    return false;
  }

  const auto &info = this->map_occupancy_grid_->info;
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
  const Particle &particle,
  const sensor_msgs::msg::LaserScan &scan) const
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
    const double endpoint_x = particle.x + (beam_range * std::cos(beam_angle));
    const double endpoint_y = particle.y + (beam_range * std::sin(beam_angle));
    const double obstacle_distance = this->nearest_obstacle_distance(endpoint_x, endpoint_y);
    const double normalized_distance = obstacle_distance / std::max(this->measurement_sigma_, 1e-3);
    const double score = std::exp(-0.5 * normalized_distance * normalized_distance);

    accumulated_score += score;
    ++used_beams;
  }

  if (used_beams == 0) {
    return kMinimumWeight;
  }

  return std::max(kMinimumWeight, accumulated_score / static_cast<double>(used_beams));
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

double Localization::quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation) const
{
  const double siny_cosp =
    2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp =
    1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

void Localization::update_pose_orientation(
  geometry_msgs::msg::PoseStamped &pose,
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
  this->has_latest_odom_ = false;
  this->has_latest_scan_ = false;
  this->has_map_ = false;
  this->has_previous_odom_ = false;
  this->has_initial_pose_ = false;
  this->particles_initialized_ = false;
  this->auto_initial_pose_published_ = false;
}

}  // namespace amr::localization::estimator
