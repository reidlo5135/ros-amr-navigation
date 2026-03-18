#include "amr_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <cmath>

namespace amr_lifecycle_manager
{

using namespace std::chrono_literals;

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("lifecycle_manager", options),
  autostart_(true),
  service_timeout_ms_(5000),
  state_poll_interval_ms_(200),
  initial_pose_enabled_(false),
  initial_pose_topic_("/amr/localization/initial_pose"),
  initial_pose_frame_id_("map"),
  initial_pose_delay_sec_(0.5),
  initial_pose_x_(0.0),
  initial_pose_y_(0.0),
  initial_pose_yaw_(0.0),
  initial_pose_covariance_x_(0.25),
  initial_pose_covariance_y_(0.25),
  initial_pose_covariance_yaw_(0.06853891945200942),
  shutdown_requested_(false)
{
  this->declare_parameter("managed_nodes", this->managed_node_names_);
  this->declare_parameter("autostart", this->autostart_);
  this->declare_parameter("service_timeout_ms", this->service_timeout_ms_);
  this->declare_parameter("state_poll_interval_ms", this->state_poll_interval_ms_);
  this->declare_parameter("initial_pose.enabled", this->initial_pose_enabled_);
  this->declare_parameter("initial_pose.topic", this->initial_pose_topic_);
  this->declare_parameter("initial_pose.frame_id", this->initial_pose_frame_id_);
  this->declare_parameter("initial_pose.delay_sec", this->initial_pose_delay_sec_);
  this->declare_parameter("initial_pose.x", this->initial_pose_x_);
  this->declare_parameter("initial_pose.y", this->initial_pose_y_);
  this->declare_parameter("initial_pose.yaw", this->initial_pose_yaw_);
  this->declare_parameter("initial_pose.covariance.x", this->initial_pose_covariance_x_);
  this->declare_parameter("initial_pose.covariance.y", this->initial_pose_covariance_y_);
  this->declare_parameter("initial_pose.covariance.yaw", this->initial_pose_covariance_yaw_);

  this->get_parameter("managed_nodes", this->managed_node_names_);
  this->get_parameter("autostart", this->autostart_);
  this->get_parameter("service_timeout_ms", this->service_timeout_ms_);
  this->get_parameter("state_poll_interval_ms", this->state_poll_interval_ms_);
  this->get_parameter("initial_pose.enabled", this->initial_pose_enabled_);
  this->get_parameter("initial_pose.topic", this->initial_pose_topic_);
  this->get_parameter("initial_pose.frame_id", this->initial_pose_frame_id_);
  this->get_parameter("initial_pose.delay_sec", this->initial_pose_delay_sec_);
  this->get_parameter("initial_pose.x", this->initial_pose_x_);
  this->get_parameter("initial_pose.y", this->initial_pose_y_);
  this->get_parameter("initial_pose.yaw", this->initial_pose_yaw_);
  this->get_parameter("initial_pose.covariance.x", this->initial_pose_covariance_x_);
  this->get_parameter("initial_pose.covariance.y", this->initial_pose_covariance_y_);
  this->get_parameter("initial_pose.covariance.yaw", this->initial_pose_covariance_yaw_);

  if (this->initial_pose_enabled_) {
    this->initial_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      this->initial_pose_topic_,
      rclcpp::SystemDefaultsQoS());
  }

  for (const auto & node_name : this->managed_node_names_) {
    ManagedNode managed_node;
    managed_node.name = node_name;
    managed_node.get_state_client =
      this->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");
    managed_node.change_state_client =
      this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
    this->managed_nodes_.push_back(managed_node);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Configured lifecycle manager '%s' with %zu managed nodes",
    this->get_fully_qualified_name(),
    this->managed_nodes_.size());

  if (this->autostart_) {
    this->bringup_thread_ = std::thread([this]() { this->run_bringup(); });
  }
}

LifecycleManager::~LifecycleManager()
{
  this->shutdown_requested_.store(true);
  if (this->bringup_thread_.joinable()) {
    this->bringup_thread_.join();
  }
}

void LifecycleManager::run_bringup()
{
  const auto service_timeout = std::chrono::milliseconds(this->service_timeout_ms_);
  for (const auto & managed_node : this->managed_nodes_) {
    if (this->shutdown_requested_.load()) {
      return;
    }

    if (!this->wait_for_service_clients(managed_node)) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Configuring managed node '%s'",
      managed_node.name.c_str());
    if (!this->request_transition(
        managed_node,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        service_timeout) ||
      !this->wait_for_state(
        managed_node,
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
        service_timeout))
    {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Activating managed node '%s'",
      managed_node.name.c_str());
    if (!this->request_transition(
        managed_node,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        service_timeout) ||
      !this->wait_for_state(
        managed_node,
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        service_timeout))
    {
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "All managed nodes are active");

  if (this->initial_pose_enabled_) {
    const auto delay = std::chrono::duration<double>(std::max(0.0, this->initial_pose_delay_sec_));
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(delay));
    if (!this->shutdown_requested_.load()) {
      this->publish_initial_pose();
    }
  }
}

bool LifecycleManager::wait_for_service_clients(const ManagedNode & managed_node) const
{
  const auto timeout = std::chrono::milliseconds(this->service_timeout_ms_);
  if (!managed_node.get_state_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timed out waiting for get_state service of '%s'",
      managed_node.name.c_str());
    return false;
  }
  if (!managed_node.change_state_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timed out waiting for change_state service of '%s'",
      managed_node.name.c_str());
    return false;
  }
  return true;
}

bool LifecycleManager::request_transition(
  const ManagedNode & managed_node,
  const std::uint8_t transition_id,
  const std::chrono::milliseconds timeout) const
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;
  auto future = managed_node.change_state_client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Timed out requesting transition %u for '%s'",
      static_cast<unsigned int>(transition_id),
      managed_node.name.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Transition %u was rejected by '%s'",
      static_cast<unsigned int>(transition_id),
      managed_node.name.c_str());
    return false;
  }

  return true;
}

bool LifecycleManager::wait_for_state(
  const ManagedNode & managed_node,
  const std::uint8_t target_state_id,
  const std::chrono::milliseconds timeout) const
{
  const auto start_time = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start_time) < timeout) {
    if (this->shutdown_requested_.load()) {
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = managed_node.get_state_client->async_send_request(request);
    if (future.wait_for(timeout) == std::future_status::ready) {
      const auto response = future.get();
      if (response->current_state.id == target_state_id) {
        return true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(this->state_poll_interval_ms_));
  }

  RCLCPP_ERROR(
    this->get_logger(),
    "Timed out waiting for '%s' to reach state id %u",
    managed_node.name.c_str(),
    static_cast<unsigned int>(target_state_id));
  return false;
}

void LifecycleManager::publish_initial_pose()
{
  if (!this->initial_pose_publisher_) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = this->now();
  initial_pose.header.frame_id = this->initial_pose_frame_id_;
  initial_pose.pose.pose.position.x = this->initial_pose_x_;
  initial_pose.pose.pose.position.y = this->initial_pose_y_;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = std::sin(this->initial_pose_yaw_ * 0.5);
  initial_pose.pose.pose.orientation.w = std::cos(this->initial_pose_yaw_ * 0.5);
  initial_pose.pose.covariance.fill(0.0);
  initial_pose.pose.covariance[0] = this->initial_pose_covariance_x_;
  initial_pose.pose.covariance[7] = this->initial_pose_covariance_y_;
  initial_pose.pose.covariance[35] = this->initial_pose_covariance_yaw_;
  this->initial_pose_publisher_->publish(initial_pose);

  RCLCPP_INFO(
    this->get_logger(),
    "Published managed initial pose on '%s': x=%.3f y=%.3f yaw=%.3f",
    this->initial_pose_topic_.c_str(),
    this->initial_pose_x_,
    this->initial_pose_y_,
    this->initial_pose_yaw_);
}

}  // namespace amr_lifecycle_manager
