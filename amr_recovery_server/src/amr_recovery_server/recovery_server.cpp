#include "amr_recovery_server/recovery_server.hpp"

namespace amr::recovery::server
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

RecoveryServer::RecoveryServer(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("recovery_server", options),
  plan_recovery_service_name_("/amr/recovery_server/plan_recovery"),
  default_node_id_("recovery"),
  wait_duration_sec_(1.0),
  backup_distance_(0.20),
  backup_speed_(0.06),
  spin_angle_rad_(1.5707963267948966)
{
  this->declare_parameter("services.plan_recovery", this->plan_recovery_service_name_);
  this->declare_parameter("defaults.node_id", this->default_node_id_);
  this->declare_parameter("recovery.wait_duration_sec", this->wait_duration_sec_);
  this->declare_parameter("recovery.backup_distance", this->backup_distance_);
  this->declare_parameter("recovery.backup_speed", this->backup_speed_);
  this->declare_parameter("recovery.spin_angle_rad", this->spin_angle_rad_);
}

RecoveryServer::CallbackReturn RecoveryServer::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("services.plan_recovery", this->plan_recovery_service_name_);
  this->get_parameter("defaults.node_id", this->default_node_id_);
  this->get_parameter("recovery.wait_duration_sec", this->wait_duration_sec_);
  this->get_parameter("recovery.backup_distance", this->backup_distance_);
  this->get_parameter("recovery.backup_speed", this->backup_speed_);
  this->get_parameter("recovery.spin_angle_rad", this->spin_angle_rad_);

  if (this->plan_recovery_service_name_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Recovery server service name must not be empty");
    return CallbackReturn::FAILURE;
  }

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

RecoveryServer::CallbackReturn RecoveryServer::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Activated recovery server");
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->plan_recovery_service_.reset();
  return CallbackReturn::SUCCESS;
}

RecoveryServer::CallbackReturn RecoveryServer::on_shutdown(const rclcpp_lifecycle::State &state)
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

  response->success = false;
  response->message = "Unknown recovery behavior: " + request->behavior;
}

amr_msgs::msg::MotionCommand RecoveryServer::build_backup_command(
  const geometry_msgs::msg::PoseStamped &current_pose) const
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
  const geometry_msgs::msg::PoseStamped &current_pose) const
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
  const geometry_msgs::msg::PoseStamped &current_pose) const
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

double RecoveryServer::quaternion_yaw(const geometry_msgs::msg::Quaternion &orientation)
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

}  // namespace amr::recovery::server
