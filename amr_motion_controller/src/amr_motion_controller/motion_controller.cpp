#include "amr_motion_controller/motion_controller.hpp"

namespace amr::motion::controller
{

MotionController::MotionController(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("motion_controller", options),
  command_topic_(""),
  local_plan_topic_(""),
  current_pose_topic_(""),
  scan_topic_(""),
  status_topic_(""),
  cmd_vel_topic_(""),
  control_frequency_(10.0),
  linear_speed_(0.07),
  min_linear_speed_(0.05),
  tracking_lookahead_distance_(0.25),
  angular_gain_(1.5),
  max_angular_speed_(0.8),
  distance_tolerance_(0.15),
  goal_heading_tolerance_(0.20),
  goal_reach_heading_tolerance_(0.35),
  rotate_in_place_threshold_(0.6),
  rotate_in_place_goal_distance_(0.35),
  heading_slowdown_threshold_(0.2),
  min_heading_motion_scale_(0.15),
  max_linear_accel_(0.08),
  max_angular_accel_(0.8),
  progress_required_movement_radius_(0.05),
  progress_time_allowance_sec_(2.0),
  safety_gate_enabled_(true),
  safety_gate_allow_rotate_in_place_(true),
  safety_gate_stop_distance_(3.0),
  safety_gate_forward_angle_deg_(25.0),
  safety_gate_rotate_heading_threshold_(0.20),
  safety_gate_min_points_(3),
  velocity_control_mode_(VelocityControlMode::PID),
  recovery_start_yaw_(0.0),
  has_command_(false),
  has_local_plan_(false),
  has_current_pose_(false),
  has_latest_scan_(false),
  has_progress_reference_(false),
  has_recovery_reference_(false)
{
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.plan", this->local_plan_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.status", this->status_topic_);
  this->declare_parameter("topics.velocity", this->cmd_vel_topic_);

  this->declare_parameter("control.frequency", this->control_frequency_);
  this->declare_parameter("control.linear_speed", this->linear_speed_);
  this->declare_parameter("control.min_linear_speed", this->min_linear_speed_);
  this->declare_parameter("control.tracking_lookahead_distance", this->tracking_lookahead_distance_);
  this->declare_parameter("control.angular_gain", this->angular_gain_);
  this->declare_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->declare_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->declare_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->declare_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->declare_parameter(
    "control.rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->declare_parameter(
    "control.rotate_in_place_goal_distance", this->rotate_in_place_goal_distance_);
  this->declare_parameter(
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->declare_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->declare_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->declare_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);

  this->declare_parameter("safety_gate.enabled", this->safety_gate_enabled_);
  this->declare_parameter(
    "safety_gate.allow_rotate_in_place", this->safety_gate_allow_rotate_in_place_);
  this->declare_parameter(
    "safety_gate.stop_distance", this->safety_gate_stop_distance_);
  this->declare_parameter(
    "safety_gate.forward_angle_deg", this->safety_gate_forward_angle_deg_);
  this->declare_parameter(
    "safety_gate.rotate_heading_threshold", this->safety_gate_rotate_heading_threshold_);
  this->declare_parameter(
    "safety_gate.minimum_points", this->safety_gate_min_points_);

  this->declare_parameter("velocity_controller.mode", std::string("pid"));
  this->declare_parameter("velocity_controller.linear.kp", 0.35);
  this->declare_parameter("velocity_controller.linear.ki", 0.0);
  this->declare_parameter("velocity_controller.linear.kd", 0.04);
  this->declare_parameter("velocity_controller.linear.integral_limit", 0.20);
  this->declare_parameter("velocity_controller.angular.kp", 0.45);
  this->declare_parameter("velocity_controller.angular.ki", 0.0);
  this->declare_parameter("velocity_controller.angular.kd", 0.02);
  this->declare_parameter("velocity_controller.angular.integral_limit", 0.30);
  this->declare_parameter("velocity_controller.max_linear_accel", this->max_linear_accel_);
  this->declare_parameter("velocity_controller.max_angular_accel", this->max_angular_accel_);
}

MotionController::CallbackReturn MotionController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.plan", this->local_plan_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.status", this->status_topic_);
  this->get_parameter("topics.velocity", this->cmd_vel_topic_);

  this->get_parameter("control.frequency", this->control_frequency_);
  this->get_parameter("control.linear_speed", this->linear_speed_);
  this->get_parameter("control.min_linear_speed", this->min_linear_speed_);
  this->get_parameter("control.tracking_lookahead_distance", this->tracking_lookahead_distance_);
  this->get_parameter("control.angular_gain", this->angular_gain_);
  this->get_parameter("control.max_angular_speed", this->max_angular_speed_);
  this->get_parameter("control.distance_tolerance", this->distance_tolerance_);
  this->get_parameter("control.goal_heading_tolerance", this->goal_heading_tolerance_);
  this->get_parameter(
    "control.goal_reach_heading_tolerance", this->goal_reach_heading_tolerance_);
  this->get_parameter(
    "control.rotate_in_place_threshold", this->rotate_in_place_threshold_);
  this->get_parameter(
    "control.rotate_in_place_goal_distance", this->rotate_in_place_goal_distance_);
  this->get_parameter(
    "control.heading_slowdown_threshold", this->heading_slowdown_threshold_);
  this->get_parameter(
    "control.min_heading_motion_scale", this->min_heading_motion_scale_);
  this->get_parameter(
    "progress_checker.required_movement_radius", this->progress_required_movement_radius_);
  this->get_parameter(
    "progress_checker.time_allowance_sec", this->progress_time_allowance_sec_);

  this->get_parameter("safety_gate.enabled", this->safety_gate_enabled_);
  this->get_parameter(
    "safety_gate.allow_rotate_in_place", this->safety_gate_allow_rotate_in_place_);
  this->get_parameter(
    "safety_gate.stop_distance", this->safety_gate_stop_distance_);
  this->get_parameter(
    "safety_gate.forward_angle_deg", this->safety_gate_forward_angle_deg_);
  this->get_parameter(
    "safety_gate.rotate_heading_threshold", this->safety_gate_rotate_heading_threshold_);
  this->get_parameter(
    "safety_gate.minimum_points", this->safety_gate_min_points_);

  this->velocity_control_mode_ = this->parse_velocity_control_mode(
    this->get_parameter("velocity_controller.mode").as_string());
  this->linear_controller_config_.kp =
    this->get_parameter("velocity_controller.linear.kp").as_double();
  this->linear_controller_config_.ki =
    this->get_parameter("velocity_controller.linear.ki").as_double();
  this->linear_controller_config_.kd =
    this->get_parameter("velocity_controller.linear.kd").as_double();
  this->linear_controller_config_.integral_limit =
    this->get_parameter("velocity_controller.linear.integral_limit").as_double();
  this->angular_controller_config_.kp =
    this->get_parameter("velocity_controller.angular.kp").as_double();
  this->angular_controller_config_.ki =
    this->get_parameter("velocity_controller.angular.ki").as_double();
  this->angular_controller_config_.kd =
    this->get_parameter("velocity_controller.angular.kd").as_double();
  this->angular_controller_config_.integral_limit =
    this->get_parameter("velocity_controller.angular.integral_limit").as_double();
  this->max_linear_accel_ =
    this->get_parameter("velocity_controller.max_linear_accel").as_double();
  this->max_angular_accel_ =
    this->get_parameter("velocity_controller.max_angular_accel").as_double();

  if (
    this->command_topic_.empty() || this->local_plan_topic_.empty() ||
    this->current_pose_topic_.empty() || this->scan_topic_.empty() ||
    this->status_topic_.empty() ||
    this->cmd_vel_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Motion controller topics must not be empty: command='%s' local_plan='%s' pose='%s' scan='%s' status='%s' cmd_vel='%s'",
      this->command_topic_.c_str(),
      this->local_plan_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->scan_topic_.c_str(),
      this->status_topic_.c_str(),
      this->cmd_vel_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->reset_velocity_controller_state();

  this->motion_command_subscription_ = this->create_subscription<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionCommand::SharedPtr message) {
      this->handle_motion_command(message);
    });
  this->local_plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    this->local_plan_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Path::SharedPtr message) {
      this->handle_local_plan(message);
    });
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });
  this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    this->cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  this->motion_status_publisher_ = this->create_publisher<amr_msgs::msg::MotionStatus>(
    this->status_topic_, rclcpp::SystemDefaultsQoS());

  const auto control_period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(this->control_frequency_, 1.0)));
  this->timer_ = this->create_wall_timer(
    control_period,
    [this]() { this->publish_control(); });
  this->timer_->cancel();
  const auto velocity_control_mode = this->get_parameter("velocity_controller.mode").as_string();

  RCLCPP_INFO(
    this->get_logger(),
    "Configured motion controller with command='%s', plan='%s', pose='%s', cmd_vel='%s', mode='%s'",
    this->command_topic_.c_str(),
    this->local_plan_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->cmd_vel_topic_.c_str(),
    velocity_control_mode.c_str());

  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->cmd_vel_publisher_->on_activate();
  this->motion_status_publisher_->on_activate();
  this->timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Activated motion controller");
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->timer_) {
    this->timer_->cancel();
  }
  this->publish_zero_twist();
  if (this->cmd_vel_publisher_) {
    this->cmd_vel_publisher_->on_deactivate();
  }
  if (this->motion_status_publisher_) {
    this->motion_status_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated motion controller");
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->publish_zero_twist();
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->scan_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  return CallbackReturn::SUCCESS;
}

MotionController::CallbackReturn MotionController::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->publish_zero_twist();
  this->motion_command_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->current_pose_subscription_.reset();
  this->scan_subscription_.reset();
  this->cmd_vel_publisher_.reset();
  this->motion_status_publisher_.reset();
  this->timer_.reset();
  this->latest_command_ = amr_msgs::msg::MotionCommand();
  this->latest_local_plan_ = nav_msgs::msg::Path();
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_scan_ = sensor_msgs::msg::LaserScan();
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->has_command_ = false;
  this->has_local_plan_ = false;
  this->has_current_pose_ = false;
  this->has_latest_scan_ = false;
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  return CallbackReturn::SUCCESS;
}

void MotionController::handle_motion_command(const amr_msgs::msg::MotionCommand::SharedPtr message)
{
  this->latest_command_ = *message;
  this->has_command_ = true;
  if (message->mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE) {
    // Drop the previous command's plan immediately. A stale empty plan can otherwise
    // make the new command look invalid before the local planner republishes.
    this->latest_local_plan_ = nav_msgs::msg::Path();
    this->has_local_plan_ = false;
  }
  this->current_twist_ = geometry_msgs::msg::Twist();
  this->reset_velocity_controller_state();
  this->reset_progress_checker_state();
  this->has_recovery_reference_ = false;
  this->recovery_start_time_ = this->now();
  this->recovery_start_yaw_ = 0.0;
  RCLCPP_INFO(
    this->get_logger(),
    "Received motion command %u mode=%u with goal x=%.3f y=%.3f",
    message->command_id,
    message->mode,
    message->goal_pose.pose.position.x,
    message->goal_pose.pose.position.y);
}

void MotionController::handle_local_plan(const nav_msgs::msg::Path::SharedPtr message)
{
  this->latest_local_plan_ = *message;
  this->has_local_plan_ = true;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Updated local plan with %zu poses",
    message->poses.size());
}

void MotionController::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void MotionController::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  this->latest_scan_ = *message;
  this->has_latest_scan_ = true;
}

void MotionController::publish_control()
{
  if (
    !this->cmd_vel_publisher_ || !this->cmd_vel_publisher_->is_activated() ||
    !this->motion_status_publisher_ || !this->motion_status_publisher_->is_activated())
  {
    return;
  }

  geometry_msgs::msg::Twist desired_twist;
  geometry_msgs::msg::Twist output_twist;
  double debug_remaining_distance = 0.0;
  amr_msgs::msg::MotionStatus status;
  status.header.stamp = this->now();
  status.header.frame_id =
    this->current_pose_.header.frame_id.empty() ? "map" : this->current_pose_.header.frame_id;
  status.current_pose = this->current_pose_;
  status.command_id = this->has_command_ ? this->latest_command_.command_id : 0U;
  status.mode = this->has_command_ ? this->latest_command_.mode :
    amr_msgs::msg::MotionCommand::MODE_NAVIGATE;
  status.active = false;
  status.command_completed = false;
  status.goal_reached = false;
  status.obstacle_detected = false;
  status.blocked = false;
  status.stalled = false;
  status.local_plan_valid = false;
  status.costmap_blocked = false;
  status.safety_gate_blocked = false;
  status.has_blocked_pose = false;
  status.blocked_pose = geometry_msgs::msg::PoseStamped();
  status.remaining_distance = 0.0;
  status.heading_error = 0.0;

  const bool has_navigation_inputs =
    this->has_command_ && this->has_current_pose_ &&
    (this->latest_command_.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE || this->has_local_plan_);

  if (has_navigation_inputs) {
    status.active = true;
    const auto current_yaw = this->quaternion_yaw(this->current_pose_.pose.orientation);

    if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_NAVIGATE) {
      const auto tracking_target = this->select_tracking_target();
      const auto local_plan_remaining_distance =
        this->estimate_remaining_distance(this->latest_local_plan_);
      const auto goal_distance = this->pose_distance(
        this->current_pose_, this->latest_command_.goal_pose);
      debug_remaining_distance = std::max(local_plan_remaining_distance, goal_distance);
      const auto goal_yaw = this->quaternion_yaw(this->latest_command_.goal_pose.pose.orientation);
      const auto target_dx =
        tracking_target.pose.position.x - this->current_pose_.pose.position.x;
      const auto target_dy =
        tracking_target.pose.position.y - this->current_pose_.pose.position.y;
      double target_heading = goal_yaw;
      if (goal_distance > this->rotate_in_place_goal_distance_) {
        if ((target_dx * target_dx) + (target_dy * target_dy) > 1e-6) {
          target_heading = std::atan2(target_dy, target_dx);
        }
      }
      const auto heading_error = this->normalize_angle(target_heading - current_yaw);
      const auto abs_heading_error = std::abs(heading_error);

      const bool safety_gate_blocked = this->is_safety_gate_triggered();
      status.local_plan_valid = this->has_local_plan_ && !this->latest_local_plan_.poses.empty();
      status.costmap_blocked = false;
      status.safety_gate_blocked = safety_gate_blocked;
      status.obstacle_detected = safety_gate_blocked;
      status.blocked = status.obstacle_detected;
      status.has_blocked_pose = false;
      status.blocked_pose = geometry_msgs::msg::PoseStamped();
      status.goal_reached =
        goal_distance <= this->distance_tolerance_ &&
        abs_heading_error <= this->goal_reach_heading_tolerance_;
      status.remaining_distance = goal_distance;
      status.heading_error = heading_error;

      if (!this->has_progress_reference_) {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
        this->has_progress_reference_ = true;
      } else if (
        this->pose_distance(this->current_pose_, this->progress_reference_pose_) >=
        this->progress_required_movement_radius_)
      {
        this->progress_reference_pose_ = this->current_pose_;
        this->progress_reference_time_ = this->now();
      } else if (
        !status.blocked &&
        (this->now() - this->progress_reference_time_).seconds() >=
        this->progress_time_allowance_sec_)
      {
        status.stalled = true;
      }

      if (status.goal_reached) {
        status.command_completed = true;
        this->has_command_ = false;
        this->has_local_plan_ = false;
        this->current_twist_ = geometry_msgs::msg::Twist();
        this->reset_velocity_controller_state();
        this->reset_progress_checker_state();
        RCLCPP_INFO(
          this->get_logger(),
          "Goal reached for command %u",
          status.command_id);
      } else if (status.blocked || status.stalled) {
        desired_twist = geometry_msgs::msg::Twist();
        if (
          status.blocked &&
          this->safety_gate_allow_rotate_in_place_ &&
          abs_heading_error > this->safety_gate_rotate_heading_threshold_)
        {
          desired_twist.angular.z = this->clamp(
            this->angular_gain_ * heading_error,
            -this->max_angular_speed_,
            this->max_angular_speed_);
        }
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Navigation hold: blocked=%s stalled=%s",
          status.blocked ? "true" : "false",
          status.stalled ? "true" : "false");
      } else {
        desired_twist.angular.z = this->clamp(
          this->angular_gain_ * heading_error,
          -this->max_angular_speed_,
          this->max_angular_speed_);

        const bool rotate_in_place_only =
          goal_distance <= this->rotate_in_place_goal_distance_ &&
          abs_heading_error > this->rotate_in_place_threshold_;
        if (!rotate_in_place_only) {
          const double base_linear_speed = std::max(
            0.0, std::min(this->linear_speed_, goal_distance));
          double scale = 1.0;
          if (abs_heading_error > this->heading_slowdown_threshold_) {
            const double scale_window = std::max(
              3.14159265358979323846 - this->heading_slowdown_threshold_,
              1e-6);
            scale = 1.0 - (
              (abs_heading_error - this->heading_slowdown_threshold_) /
              scale_window);
          }

          scale = this->clamp(scale, this->min_heading_motion_scale_, 1.0);
          desired_twist.linear.x = base_linear_speed * scale;
          if (goal_distance > this->rotate_in_place_goal_distance_) {
            desired_twist.linear.x = std::max(
              std::min(this->min_linear_speed_, base_linear_speed),
              desired_twist.linear.x);
          }
        }
      }
    } else {
      this->ensure_recovery_reference_initialized();
      const double elapsed_sec = (this->now() - this->recovery_start_time_).seconds();

      if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_BACKUP) {
        const double traveled = this->pose_distance(this->current_pose_, this->recovery_reference_pose_);
        const double remaining = std::max(0.0, this->latest_command_.recovery_distance - traveled);
        status.remaining_distance = remaining;
        debug_remaining_distance = remaining;
        if (
          remaining <= this->distance_tolerance_ ||
          (this->latest_command_.recovery_duration > 0.0 &&
          elapsed_sec >= this->latest_command_.recovery_duration))
        {
          status.command_completed = true;
        } else {
          desired_twist.linear.x =
            -std::max(this->latest_command_.recovery_speed, this->min_linear_speed_);
        }
      } else if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_SPIN) {
        const double target_yaw = this->recovery_start_yaw_ + this->latest_command_.recovery_angle;
        const double heading_error = this->normalize_angle(target_yaw - current_yaw);
        status.heading_error = heading_error;
        status.remaining_distance = std::abs(heading_error);
        debug_remaining_distance = status.remaining_distance;
        if (std::abs(heading_error) <= this->goal_heading_tolerance_) {
          status.command_completed = true;
        } else {
          desired_twist.angular.z = this->clamp(
            this->angular_gain_ * heading_error,
            -this->max_angular_speed_,
            this->max_angular_speed_);
        }
      } else if (this->latest_command_.mode == amr_msgs::msg::MotionCommand::MODE_WAIT) {
        const double remaining = std::max(0.0, this->latest_command_.recovery_duration - elapsed_sec);
        status.remaining_distance = remaining;
        debug_remaining_distance = remaining;
        status.command_completed = remaining <= 1e-3;
      }

      if (status.command_completed) {
        this->has_command_ = false;
        this->current_twist_ = geometry_msgs::msg::Twist();
        this->reset_velocity_controller_state();
        this->reset_progress_checker_state();
        RCLCPP_INFO(
          this->get_logger(),
          "Recovery command %u completed",
          status.command_id);
      }
    }
  } else {
    status.goal_reached = true;
  }

  this->current_twist_ = this->apply_velocity_controller(this->current_twist_, desired_twist);
  output_twist = this->current_twist_;

  this->cmd_vel_publisher_->publish(output_twist);
  this->motion_status_publisher_->publish(status);

  if (status.active) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Control cmd=%u target(v=%.3f,w=%.3f) output(v=%.3f,w=%.3f) remaining=%.3f heading=%.3f",
      status.command_id,
      desired_twist.linear.x,
      desired_twist.angular.z,
      output_twist.linear.x,
      output_twist.angular.z,
      debug_remaining_distance,
      status.heading_error);
  }
}

void MotionController::reset_velocity_controller_state()
{
  this->linear_controller_state_ = AxisControllerState{};
  this->angular_controller_state_ = AxisControllerState{};
}

void MotionController::reset_progress_checker_state()
{
  this->progress_reference_pose_ = geometry_msgs::msg::PoseStamped();
  this->recovery_reference_pose_ = geometry_msgs::msg::PoseStamped();
  this->progress_reference_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->recovery_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->recovery_start_yaw_ = 0.0;
  this->has_progress_reference_ = false;
  this->has_recovery_reference_ = false;
}

void MotionController::publish_zero_twist()
{
  this->current_twist_ = geometry_msgs::msg::Twist();
  if (!this->cmd_vel_publisher_ || !this->cmd_vel_publisher_->is_activated()) {
    return;
  }

  this->cmd_vel_publisher_->publish(this->current_twist_);
}

void MotionController::ensure_recovery_reference_initialized()
{
  if (this->has_recovery_reference_) {
    return;
  }

  this->recovery_reference_pose_ = this->current_pose_;
  this->recovery_start_time_ = this->now();
  this->recovery_start_yaw_ = this->quaternion_yaw(this->current_pose_.pose.orientation);
  this->has_recovery_reference_ = true;
}

MotionController::VelocityControlMode MotionController::parse_velocity_control_mode(
  const std::string & mode) const
{
  std::string normalized = mode;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });

  if (normalized == "p") {
    return VelocityControlMode::P;
  }
  if (normalized == "pi") {
    return VelocityControlMode::PI;
  }
  if (normalized == "pid") {
    return VelocityControlMode::PID;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "Unknown velocity_controller.mode '%s'; falling back to pid",
    mode.c_str());
  return VelocityControlMode::PID;
}

double MotionController::apply_axis_controller(
  const double current,
  const double target,
  AxisControllerState & state,
  const AxisControllerConfig & config,
  const double max_step,
  const double dt) const
{
  const double error = target - current;
  double derivative = 0.0;

  if (
    this->velocity_control_mode_ == VelocityControlMode::PI ||
    this->velocity_control_mode_ == VelocityControlMode::PID)
  {
    state.integral = this->clamp(
      state.integral + (error * dt),
      -config.integral_limit,
      config.integral_limit);
  } else {
    state.integral = 0.0;
  }

  if (
    !state.first_update &&
    this->velocity_control_mode_ == VelocityControlMode::PID &&
    dt > 1e-6)
  {
    derivative = (error - state.previous_error) / dt;
  }

  double control_delta = config.kp * error;
  if (
    this->velocity_control_mode_ == VelocityControlMode::PI ||
    this->velocity_control_mode_ == VelocityControlMode::PID)
  {
    control_delta += config.ki * state.integral;
  }
  if (this->velocity_control_mode_ == VelocityControlMode::PID) {
    control_delta += config.kd * derivative;
  }

  control_delta = this->clamp(control_delta, -max_step, max_step);
  double next = current + control_delta;

  if (target >= current) {
    next = std::min(next, target);
  } else {
    next = std::max(next, target);
  }

  if (std::abs(target) <= 1e-6 && std::abs(error) <= max_step) {
    next = 0.0;
    state.integral = 0.0;
  }

  state.previous_error = error;
  state.first_update = false;
  return next;
}

geometry_msgs::msg::Twist MotionController::apply_velocity_controller(
  const geometry_msgs::msg::Twist & current,
  const geometry_msgs::msg::Twist & target)
{
  geometry_msgs::msg::Twist controlled = current;
  const double dt = 1.0 / std::max(this->control_frequency_, 1.0);
  const double max_linear_step = this->max_linear_accel_ * dt;
  const double max_angular_step = this->max_angular_accel_ * dt;

  controlled.linear.x = this->apply_axis_controller(
    current.linear.x,
    target.linear.x,
    this->linear_controller_state_,
    this->linear_controller_config_,
    max_linear_step,
    dt);
  controlled.angular.z = this->apply_axis_controller(
    current.angular.z,
    target.angular.z,
    this->angular_controller_state_,
    this->angular_controller_config_,
    max_angular_step,
    dt);
  return controlled;
}

double MotionController::estimate_remaining_distance(const nav_msgs::msg::Path & path) const
{
  double distance = 0.0;
  if (path.poses.size() < 2U) {
    return distance;
  }

  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto dx = current.x - previous.x;
    const auto dy = current.y - previous.y;
    distance += std::sqrt((dx * dx) + (dy * dy));
  }

  return distance;
}

double MotionController::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MotionController::normalize_angle(double angle) const
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MotionController::clamp(
  const double value,
  const double min_value,
  const double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

geometry_msgs::msg::PoseStamped MotionController::select_tracking_target() const
{
  if (this->latest_local_plan_.poses.empty()) {
    return this->latest_command_.goal_pose;
  }

  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < this->latest_local_plan_.poses.size(); ++index) {
    const double distance = this->pose_distance(this->current_pose_, this->latest_local_plan_.poses[index]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  double accumulated_distance = 0.0;
  for (std::size_t index = nearest_index + 1U; index < this->latest_local_plan_.poses.size(); ++index) {
    const auto & previous = this->latest_local_plan_.poses[index - 1U];
    const auto & current = this->latest_local_plan_.poses[index];
    accumulated_distance += this->pose_distance(previous, current);
    if (accumulated_distance >= this->tracking_lookahead_distance_) {
      return current;
    }
  }

  return this->latest_local_plan_.poses.back();
}

bool MotionController::is_safety_gate_triggered() const
{
  if (!this->safety_gate_enabled_ || !this->has_latest_scan_) {
    return false;
  }

  const double half_angle_rad =
    (this->safety_gate_forward_angle_deg_ * 3.14159265358979323846 / 180.0) * 0.5;
  int hit_count = 0;

  for (std::size_t index = 0; index < this->latest_scan_.ranges.size(); ++index) {
    const double angle =
      this->latest_scan_.angle_min +
      (static_cast<double>(index) * this->latest_scan_.angle_increment);
    if (std::abs(angle) > half_angle_rad) {
      continue;
    }

    const double range = this->latest_scan_.ranges[index];
    if (!std::isfinite(range)) {
      continue;
    }
    if (
      range < this->latest_scan_.range_min ||
      range > this->latest_scan_.range_max)
    {
      continue;
    }
    if (range <= this->safety_gate_stop_distance_) {
      ++hit_count;
      if (hit_count >= this->safety_gate_min_points_) {
        return true;
      }
    }
  }

  return false;
}

double MotionController::pose_distance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace amr::motion::controller
