#include "amr_recovery_server/recovery_server.hpp"

namespace amr_recovery_server
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

RecoveryServer::RecoveryServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("recovery_server", options),
  plan_recovery_service_name_("/amr/recovery_server/plan_recovery"),
  scan_topic_("/scan"),
  default_node_id_("recovery"),
  wait_duration_sec_(1.0),
  backup_distance_(0.20),
  backup_speed_(0.06),
  spin_angle_rad_(1.5707963267948966),
  arl_wait_duration_sec_(0.5),
  arl_spin_angle_rad_(0.7853981633974483),
  arl_probe_distance_(0.12),
  arl_probe_long_distance_(0.45),
  arl_probe_speed_(0.04),
  arl_probe_safety_margin_(0.18),
  arl_probe_sector_half_width_rad_(0.30),
  arl_probe_spin_heading_threshold_rad_(0.30),
  has_latest_scan_(false)
{
  this->declare_parameter("services.plan_recovery", this->plan_recovery_service_name_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("defaults.node_id", this->default_node_id_);
  this->declare_parameter("recovery.wait_duration_sec", this->wait_duration_sec_);
  this->declare_parameter("recovery.backup_distance", this->backup_distance_);
  this->declare_parameter("recovery.backup_speed", this->backup_speed_);
  this->declare_parameter("recovery.spin_angle_rad", this->spin_angle_rad_);
  this->declare_parameter("arl.wait_duration_sec", this->arl_wait_duration_sec_);
  this->declare_parameter("arl.spin_angle_rad", this->arl_spin_angle_rad_);
  this->declare_parameter("arl.probe_distance", this->arl_probe_distance_);
  this->declare_parameter("arl.probe_long_distance", this->arl_probe_long_distance_);
  this->declare_parameter("arl.probe_speed", this->arl_probe_speed_);
  this->declare_parameter("arl.probe_safety_margin", this->arl_probe_safety_margin_);
  this->declare_parameter("arl.probe_sector_half_width_rad", this->arl_probe_sector_half_width_rad_);
  this->declare_parameter(
    "arl.probe_spin_heading_threshold_rad",
    this->arl_probe_spin_heading_threshold_rad_);
}

RecoveryServer::CallbackReturn RecoveryServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("services.plan_recovery", this->plan_recovery_service_name_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("defaults.node_id", this->default_node_id_);
  this->get_parameter("recovery.wait_duration_sec", this->wait_duration_sec_);
  this->get_parameter("recovery.backup_distance", this->backup_distance_);
  this->get_parameter("recovery.backup_speed", this->backup_speed_);
  this->get_parameter("recovery.spin_angle_rad", this->spin_angle_rad_);
  this->get_parameter("arl.wait_duration_sec", this->arl_wait_duration_sec_);
  this->get_parameter("arl.spin_angle_rad", this->arl_spin_angle_rad_);
  this->get_parameter("arl.probe_distance", this->arl_probe_distance_);
  this->get_parameter("arl.probe_long_distance", this->arl_probe_long_distance_);
  this->get_parameter("arl.probe_speed", this->arl_probe_speed_);
  this->get_parameter("arl.probe_safety_margin", this->arl_probe_safety_margin_);
  this->get_parameter("arl.probe_sector_half_width_rad", this->arl_probe_sector_half_width_rad_);
  this->get_parameter(
    "arl.probe_spin_heading_threshold_rad",
    this->arl_probe_spin_heading_threshold_rad_);

  if (this->plan_recovery_service_name_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Recovery server service name must not be empty");
    return CallbackReturn::FAILURE;
  }

  if (this->scan_topic_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Recovery server scan topic must not be empty");
    return CallbackReturn::FAILURE;
  }

  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });

  this->plan_recovery_service_ = this->create_service<amr_msgs::srv::PlanRecovery>(
    this->plan_recovery_service_name_,
    [this](
      const std::shared_ptr<amr_msgs::srv::PlanRecovery::Request> request,
      std::shared_ptr<amr_msgs::srv::PlanRecovery::Response> response)
    {
      this->handle_plan_recovery(request, response);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured recovery server with service='%s'",
    this->plan_recovery_service_name_.c_str());
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Activated recovery server");
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->plan_recovery_service_.reset();
  this->scan_subscription_.reset();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->has_latest_scan_ = false;
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return this->on_cleanup(state);
}

void RecoveryServer::handle_plan_recovery(
  const std::shared_ptr<amr_msgs::srv::PlanRecovery::Request> request,
  std::shared_ptr<amr_msgs::srv::PlanRecovery::Response> response)
{
  if (request->behavior == "wait") {
    response->command = this->build_wait_command(request->current_pose);
    response->success = true;
    response->message = "Planned wait recovery.";
    return;
  }
  if (request->behavior == "backup") {
    response->command = this->build_backup_command(request->current_pose);
    response->success = true;
    response->message = "Planned backup recovery.";
    return;
  }
  if (request->behavior == "spin") {
    response->command = this->build_spin_command(request->current_pose);
    response->success = true;
    response->message = "Planned spin recovery.";
    return;
  }
  if (request->behavior == "arl_spin") {
    response->command = this->build_arl_spin_command(request->current_pose);
    response->success = true;
    response->message = "Planned active relocalization spin.";
    return;
  }
  if (request->behavior == "arl_wait") {
    response->command = this->build_arl_wait_command(request->current_pose);
    response->success = true;
    response->message = "Planned active relocalization wait.";
    return;
  }
  if (request->behavior == "probe_forward") {
    response->command = this->build_probe_forward_command(request->current_pose);
    response->success = true;
    response->message = "Planned active relocalization forward probe.";
    return;
  }
  if (request->behavior == "probe_forward_long") {
    response->command = this->build_probe_forward_long_command(request->current_pose);
    response->success = true;
    response->message = "Planned active relocalization long forward probe.";
    return;
  }
  if (request->behavior == "arl_probe_auto") {
    response->command = this->build_arl_probe_auto_command(request->current_pose);
    response->success = true;
    response->message = "Planned scan-driven active relocalization probe.";
    return;
  }

  response->success = false;
  response->message = "Unknown recovery behavior: " + request->behavior;
}

void RecoveryServer::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_latest_scan_ = true;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_backup_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  command.mode = amr_msgs::msg::MotionCommand::MODE_BACKUP;
  command.route_id = "recovery_backup";
  command.node_id = this->default_node_id_;
  command.goal_pose = current_pose;
  command.align_heading_at_goal = false;
  command.recovery_distance = std::max(0.0, this->backup_distance_);
  command.recovery_speed = std::max(0.01, this->backup_speed_);
  command.recovery_duration =
    command.recovery_speed > 1e-6 ? command.recovery_distance / command.recovery_speed : 0.0;
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_spin_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  command.mode = amr_msgs::msg::MotionCommand::MODE_SPIN;
  command.route_id = "recovery_spin";
  command.node_id = this->default_node_id_;
  command.goal_pose = current_pose;
  const double current_yaw = quaternion_yaw(current_pose.pose.orientation);
  command.goal_pose.pose.orientation = yaw_to_quaternion(current_yaw + this->spin_angle_rad_);
  command.align_heading_at_goal = true;
  command.recovery_angle = this->spin_angle_rad_;
  command.recovery_duration = 0.0;
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_wait_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  command.mode = amr_msgs::msg::MotionCommand::MODE_WAIT;
  command.route_id = "recovery_wait";
  command.node_id = this->default_node_id_;
  command.goal_pose = current_pose;
  command.align_heading_at_goal = false;
  command.recovery_duration = std::max(0.0, this->wait_duration_sec_);
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_arl_spin_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command = this->build_spin_command(current_pose);
  command.route_id = "arl_spin";
  const double current_yaw = quaternion_yaw(current_pose.pose.orientation);
  command.goal_pose.pose.orientation = yaw_to_quaternion(current_yaw + this->arl_spin_angle_rad_);
  command.recovery_angle = this->arl_spin_angle_rad_;
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_arl_wait_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command = this->build_wait_command(current_pose);
  command.route_id = "arl_wait";
  command.recovery_duration = std::max(0.0, this->arl_wait_duration_sec_);
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_probe_forward_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  command.mode = amr_msgs::msg::MotionCommand::MODE_PROBE;
  command.route_id = "arl_probe_forward";
  command.node_id = this->default_node_id_;
  command.goal_pose = current_pose;
  command.align_heading_at_goal = false;
  command.recovery_distance = std::max(0.0, this->arl_probe_distance_);
  command.recovery_speed = std::max(0.01, this->arl_probe_speed_);
  command.recovery_duration =
    command.recovery_speed > 1e-6 ? command.recovery_distance / command.recovery_speed : 0.0;
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_probe_forward_long_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  auto command = this->build_probe_forward_command(current_pose);
  command.route_id = "arl_probe_forward_long";
  command.recovery_distance = std::max(command.recovery_distance, this->arl_probe_long_distance_);
  command.recovery_duration =
    command.recovery_speed > 1e-6 ? command.recovery_distance / command.recovery_speed : 0.0;
  return command;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_arl_probe_auto_command(
  const geometry_msgs::msg::PoseStamped & current_pose) const
{
  if (!this->has_latest_scan_ || this->latest_scan_.ranges.empty()) {
    return this->build_arl_wait_command(current_pose);
  }

  static constexpr double kCandidateHeadings[] = {
    -2.356194490192345,
    -1.5707963267948966,
    -1.0471975511965976,
    -0.5235987755982988,
    0.0,
    0.5235987755982988,
    1.0471975511965976,
    1.5707963267948966,
    2.356194490192345
  };

  double best_heading = 0.0;
  double best_clearance = -1.0;
  for (const double candidate_heading : kCandidateHeadings) {
    double min_range = std::numeric_limits<double>::infinity();
    bool has_valid_range = false;
    for (std::size_t index = 0; index < this->latest_scan_.ranges.size(); ++index) {
      const double range = static_cast<double>(this->latest_scan_.ranges[index]);
      if (
        !std::isfinite(range) ||
        range < static_cast<double>(this->latest_scan_.range_min))
      {
        continue;
      }

      const double beam_angle =
        static_cast<double>(this->latest_scan_.angle_min) +
        (static_cast<double>(index) * static_cast<double>(this->latest_scan_.angle_increment));
      double angle_delta = beam_angle - candidate_heading;
      while (angle_delta > kPi) {
        angle_delta -= 2.0 * kPi;
      }
      while (angle_delta < -kPi) {
        angle_delta += 2.0 * kPi;
      }
      if (std::fabs(angle_delta) > this->arl_probe_sector_half_width_rad_) {
        continue;
      }

      min_range = std::min(min_range, range);
      has_valid_range = true;
    }

    if (!has_valid_range) {
      continue;
    }

    if (min_range > best_clearance) {
      best_clearance = min_range;
      best_heading = candidate_heading;
    }
  }

  if (best_clearance <= 0.0) {
    return this->build_arl_wait_command(current_pose);
  }

  if (std::fabs(best_heading) > this->arl_probe_spin_heading_threshold_rad_) {
    auto command = this->build_arl_spin_command(current_pose);
    const double current_yaw = quaternion_yaw(current_pose.pose.orientation);
    command.route_id = "arl_scan_spin";
    command.goal_pose.pose.orientation = yaw_to_quaternion(current_yaw + best_heading);
    command.recovery_angle = best_heading;
    return command;
  }

  const double usable_distance = std::max(0.0, best_clearance - this->arl_probe_safety_margin_);
  if (usable_distance < std::max(0.08, this->arl_probe_distance_ * 0.5)) {
    auto command = this->build_arl_spin_command(current_pose);
    command.route_id = "arl_scan_spin";
    command.recovery_angle = best_heading >= 0.0 ? this->arl_spin_angle_rad_ : -this->arl_spin_angle_rad_;
    const double current_yaw = quaternion_yaw(current_pose.pose.orientation);
    command.goal_pose.pose.orientation = yaw_to_quaternion(current_yaw + command.recovery_angle);
    return command;
  }

  auto command = this->build_probe_forward_command(current_pose);
  command.route_id = "arl_probe_auto";
  command.recovery_distance = std::clamp(
    usable_distance,
    std::max(0.10, this->arl_probe_distance_),
    std::max(this->arl_probe_long_distance_, this->arl_probe_distance_));
  command.recovery_duration =
    command.recovery_speed > 1e-6 ? command.recovery_distance / command.recovery_speed : 0.0;
  return command;
}

double RecoveryServer::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (
      orientation.w * orientation.z +
      orientation.x * orientation.y),
    1.0 - 2.0 * (
      orientation.y * orientation.y +
      orientation.z * orientation.z));
}

geometry_msgs::msg::Quaternion RecoveryServer::yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(0.5 * yaw);
  orientation.w = std::cos(0.5 * yaw);
  return orientation;
}

}  // namespace amr_recovery_server
