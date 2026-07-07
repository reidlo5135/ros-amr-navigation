/**
 * @file frontier_navigator.cpp
 * @brief Lifecycle action orchestrator for unknown-goal frontier navigation.
 */

#include "amr_frontier_navigator/frontier_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <set>
#include <sstream>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr::frontier_navigation
{

namespace
{

using namespace std::chrono_literals;

/// @brief Return a stable true/false label for structured logs.
const char *bool_label(const bool value)
{
  return value ? "true" : "false";
}

/// @brief Escape whitespace in log values for structured logging.
std::string log_value(std::string value)
{
  if (value.empty()) {
    return "none";
  }
  for (char &character : value) {
    if (character == ' ' || character == '\t' || character == '\n' || character == '\r' || character == '=') {
      character = '_';
    }
  }
  return value;
}

/// @brief Return bounded positive goal parameter or node default.
uint16_t resolve_uint16_goal_value(const uint16_t requested, const int fallback)
{
  return requested > 0U ? requested : static_cast<uint16_t>(std::max(1, fallback));
}

/// @brief Return bounded positive floating goal parameter or node default.
double resolve_positive_goal_value(const float requested, const double fallback)
{
  return std::isfinite(requested) && requested > 0.0F ? static_cast<double>(requested) : fallback;
}

}  // namespace

FrontierNavigator::FrontierNavigator(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("ft_navigator", options)
{
  this->declare_parameter("actions.navigate_to_unknown_pose", this->unknown_action_name_);
  this->declare_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.local_plan", this->local_plan_topic_);
  this->declare_parameter("topics.frontier_unknown_goal", this->unknown_goal_topic_);
  this->declare_parameter("topics.frontier_known_goal", this->known_goal_topic_);
  this->declare_parameter("topics.frontier_global_plan", this->frontier_global_plan_topic_);
  this->declare_parameter("topics.frontier_local_plan", this->frontier_local_plan_topic_);
  this->declare_parameter("topics.frontier_status", this->status_topic_);
  this->declare_parameter("services.segment", this->plan_segment_service_);
  this->declare_parameter("frames.map", this->map_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("frames.base_fallback", this->base_fallback_frame_);
  this->declare_parameter("tf.lookup_timeout_sec", this->tf_lookup_timeout_sec_);
  this->declare_parameter("resolver.free_threshold", this->free_threshold_);
  this->declare_parameter("resolver.occupied_threshold", this->occupied_threshold_);
  this->declare_parameter("resolver.candidate_step_cells", this->candidate_step_cells_);
  this->declare_parameter("resolver.min_obstacle_clearance_m", this->min_obstacle_clearance_m_);
  this->declare_parameter("resolver.max_staging_search_radius_m", this->max_staging_search_radius_m_);
  this->declare_parameter("resolver.goal_tolerance_m", this->goal_tolerance_m_);
  this->declare_parameter("resolver.map_wait_timeout_sec", this->map_wait_timeout_sec_);
  this->declare_parameter("resolver.goal_known_wait_timeout_sec", this->goal_known_wait_timeout_sec_);
  this->declare_parameter("resolver.max_iterations", this->max_iterations_);
  this->declare_parameter("resolver.min_staging_progress_m", this->min_staging_progress_m_);
  this->declare_parameter("resolver.max_candidate_checks", this->max_candidate_checks_);
  this->declare_parameter("resolver.frontier_neighbor_radius_cells", this->frontier_neighbor_radius_cells_);
  this->declare_parameter("resolver.use_plan_segment_validation", this->use_plan_segment_validation_);
  this->declare_parameter("execution.action_server_wait_timeout_ms", this->action_server_wait_timeout_ms_);
  this->declare_parameter("execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->declare_parameter("execution.feedback_period_ms", this->feedback_period_ms_);
  this->declare_parameter("logging.structured_enabled", this->structured_logging_enabled_);
}

FrontierNavigator::CallbackReturn FrontierNavigator::on_configure(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("actions.navigate_to_unknown_pose", this->unknown_action_name_);
  this->get_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.local_plan", this->local_plan_topic_);
  this->get_parameter("topics.frontier_unknown_goal", this->unknown_goal_topic_);
  this->get_parameter("topics.frontier_known_goal", this->known_goal_topic_);
  this->get_parameter("topics.frontier_global_plan", this->frontier_global_plan_topic_);
  this->get_parameter("topics.frontier_local_plan", this->frontier_local_plan_topic_);
  this->get_parameter("topics.frontier_status", this->status_topic_);
  this->get_parameter("services.segment", this->plan_segment_service_);
  this->get_parameter("frames.map", this->map_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("frames.base_fallback", this->base_fallback_frame_);
  this->get_parameter("tf.lookup_timeout_sec", this->tf_lookup_timeout_sec_);
  this->get_parameter("resolver.free_threshold", this->free_threshold_);
  this->get_parameter("resolver.occupied_threshold", this->occupied_threshold_);
  this->get_parameter("resolver.candidate_step_cells", this->candidate_step_cells_);
  this->get_parameter("resolver.min_obstacle_clearance_m", this->min_obstacle_clearance_m_);
  this->get_parameter("resolver.max_staging_search_radius_m", this->max_staging_search_radius_m_);
  this->get_parameter("resolver.goal_tolerance_m", this->goal_tolerance_m_);
  this->get_parameter("resolver.map_wait_timeout_sec", this->map_wait_timeout_sec_);
  this->get_parameter("resolver.goal_known_wait_timeout_sec", this->goal_known_wait_timeout_sec_);
  this->get_parameter("resolver.max_iterations", this->max_iterations_);
  this->get_parameter("resolver.min_staging_progress_m", this->min_staging_progress_m_);
  this->get_parameter("resolver.max_candidate_checks", this->max_candidate_checks_);
  this->get_parameter("resolver.frontier_neighbor_radius_cells", this->frontier_neighbor_radius_cells_);
  this->get_parameter("resolver.use_plan_segment_validation", this->use_plan_segment_validation_);
  this->get_parameter("execution.action_server_wait_timeout_ms", this->action_server_wait_timeout_ms_);
  this->get_parameter("execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->get_parameter("execution.feedback_period_ms", this->feedback_period_ms_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);

  this->candidate_step_cells_ = std::max(1, this->candidate_step_cells_);
  this->max_candidate_checks_ = std::max(1, this->max_candidate_checks_);
  this->frontier_neighbor_radius_cells_ = std::max(1, this->frontier_neighbor_radius_cells_);
  this->max_iterations_ = std::max(1, this->max_iterations_);
  this->feedback_period_ms_ = std::max(20, this->feedback_period_ms_);

  if (
    this->unknown_action_name_.empty() || this->navigate_action_name_.empty() ||
    this->map_topic_.empty() || this->local_plan_topic_.empty() ||
    this->unknown_goal_topic_.empty() || this->known_goal_topic_.empty() ||
    this->frontier_global_plan_topic_.empty() || this->frontier_local_plan_topic_.empty() ||
    this->status_topic_.empty() || this->plan_segment_service_.empty() ||
    this->map_frame_.empty() || this->base_frame_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Frontier navigator action/topic/service/frame parameters must not be empty");
    return CallbackReturn::FAILURE;
  }

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer_);
  this->map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      this->map_callback(message);
    });
  this->local_plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    this->local_plan_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Path::SharedPtr message) {
      this->local_plan_callback(message);
    });

  this->unknown_goal_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(this->unknown_goal_topic_, rclcpp::SystemDefaultsQoS());
  this->known_goal_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(this->known_goal_topic_, rclcpp::SystemDefaultsQoS());
  this->frontier_global_plan_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>(this->frontier_global_plan_topic_, rclcpp::SystemDefaultsQoS());
  this->frontier_local_plan_publisher_ =
    this->create_publisher<nav_msgs::msg::Path>(this->frontier_local_plan_topic_, rclcpp::SystemDefaultsQoS());
  this->status_publisher_ =
    this->create_publisher<amr_msgs::msg::FrontierNavigationStatus>(this->status_topic_, rclcpp::SystemDefaultsQoS());
  this->plan_segment_client_ =
    this->create_client<amr_msgs::srv::PlanSegment>(this->plan_segment_service_);
  this->navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->navigate_action_name_);
  this->action_server_ = rclcpp_action::create_server<NavigateToUnknownPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->unknown_action_name_,
    [this](
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToUnknownPose::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleUnknown> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleUnknown> goal_handle) {
      this->handle_accepted(goal_handle);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured frontier navigator action='%s' delegate='%s' map='%s' planner='%s' overlays='%s,%s,%s,%s,%s'",
    this->unknown_action_name_.c_str(),
    this->navigate_action_name_.c_str(),
    this->map_topic_.c_str(),
    this->plan_segment_service_.c_str(),
    this->unknown_goal_topic_.c_str(),
    this->known_goal_topic_.c_str(),
    this->frontier_global_plan_topic_.c_str(),
    this->frontier_local_plan_topic_.c_str(),
    this->status_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

FrontierNavigator::CallbackReturn FrontierNavigator::on_activate(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->unknown_goal_publisher_) {
    this->unknown_goal_publisher_->on_activate();
  }
  if (this->known_goal_publisher_) {
    this->known_goal_publisher_->on_activate();
  }
  if (this->frontier_global_plan_publisher_) {
    this->frontier_global_plan_publisher_->on_activate();
  }
  if (this->frontier_local_plan_publisher_) {
    this->frontier_local_plan_publisher_->on_activate();
  }
  if (this->status_publisher_) {
    this->status_publisher_->on_activate();
  }
  publish_empty_overlays("IDLE", "Frontier navigation inactive.");
  RCLCPP_INFO(this->get_logger(), "Activated frontier navigator");
  return CallbackReturn::SUCCESS;
}

FrontierNavigator::CallbackReturn FrontierNavigator::on_deactivate(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  cancel_nested_goal();
  publish_empty_overlays("IDLE", "Frontier navigator deactivated.");
  if (this->unknown_goal_publisher_) {
    this->unknown_goal_publisher_->on_deactivate();
  }
  if (this->known_goal_publisher_) {
    this->known_goal_publisher_->on_deactivate();
  }
  if (this->frontier_global_plan_publisher_) {
    this->frontier_global_plan_publisher_->on_deactivate();
  }
  if (this->frontier_local_plan_publisher_) {
    this->frontier_local_plan_publisher_->on_deactivate();
  }
  if (this->status_publisher_) {
    this->status_publisher_->on_deactivate();
  }
  this->frontier_active_.store(false);
  return CallbackReturn::SUCCESS;
}

FrontierNavigator::CallbackReturn FrontierNavigator::on_cleanup(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  cancel_nested_goal();
  this->action_server_.reset();
  this->navigate_to_pose_client_.reset();
  this->plan_segment_client_.reset();
  this->map_subscription_.reset();
  this->local_plan_subscription_.reset();
  this->unknown_goal_publisher_.reset();
  this->known_goal_publisher_.reset();
  this->frontier_global_plan_publisher_.reset();
  this->frontier_local_plan_publisher_.reset();
  this->status_publisher_.reset();
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
  {
    std::scoped_lock lock(this->map_mutex_);
    this->latest_map_ = nav_msgs::msg::OccupancyGrid();
    this->has_map_ = false;
  }
  {
    std::scoped_lock lock(this->active_goal_mutex_);
    this->active_goal_handle_.reset();
  }
  this->frontier_active_.store(false);
  return CallbackReturn::SUCCESS;
}

FrontierNavigator::CallbackReturn FrontierNavigator::on_shutdown(
  const rclcpp_lifecycle::State &state)
{
  (void)state;
  cancel_nested_goal();
  this->frontier_active_.store(false);
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse FrontierNavigator::handle_goal(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToUnknownPose::Goal> goal)
{
  (void)uuid;
  std::scoped_lock lock(this->active_goal_mutex_);
  if (this->active_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting unknown goal because another frontier goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting unknown goal because frontier navigator is inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal || goal->unknown_goal.header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejecting unknown goal with empty frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FrontierNavigator::handle_cancel(
  const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  (void)goal_handle;
  cancel_nested_goal();
  RCLCPP_INFO(this->get_logger(), "Accepted frontier navigation cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FrontierNavigator::handle_accepted(const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  {
    std::scoped_lock lock(this->active_goal_mutex_);
    this->active_goal_handle_ = goal_handle;
  }
  std::thread(
    [this, goal_handle]() {
      try {
        this->execute(goal_handle);
      } catch (const std::exception &error) {
        auto result = std::make_shared<NavigateToUnknownPose::Result>();
        result->error_code = NavigateToUnknownPose::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled frontier navigator exception: ") + error.what();
        this->finalize_result(goal_handle, result, "aborted");
        this->clear_active_goal(goal_handle);
        this->frontier_active_.store(false);
        this->publish_empty_overlays("FAILED", result->error_msg);
      } catch (...) {
        auto result = std::make_shared<NavigateToUnknownPose::Result>();
        result->error_code = NavigateToUnknownPose::Result::UNKNOWN;
        result->error_msg = "Unhandled unknown frontier navigator exception.";
        this->finalize_result(goal_handle, result, "aborted");
        this->clear_active_goal(goal_handle);
        this->frontier_active_.store(false);
        this->publish_empty_overlays("FAILED", result->error_msg);
      }
    }).detach();
}

void FrontierNavigator::execute(const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateToUnknownPose::Result>();
  result->error_code = NavigateToUnknownPose::Result::UNKNOWN;
  result->error_msg.clear();
  result->iterations = 0U;
  result->original_goal_became_known = false;
  result->final_goal_reached = false;
  (void)goal->allow_final_unknown_retry;

  geometry_msgs::msg::PoseStamped original_goal;
  std::string error_message;
  if (!transform_goal_to_map(goal->unknown_goal, original_goal, error_message)) {
    result->error_code = NavigateToUnknownPose::Result::TF_ERROR;
    result->error_msg = error_message;
    this->finalize_result(goal_handle, result, "aborted");
    this->clear_active_goal(goal_handle);
    return;
  }
  result->original_goal = original_goal;

  this->frontier_active_.store(true);
  publish_pose(this->unknown_goal_publisher_, original_goal);
  publish_status_and_feedback(
    "WAITING_FOR_MAP", original_goal, geometry_msgs::msg::PoseStamped(), 0U, true,
    "Waiting for SLAM map and current pose.", goal_handle);

  const auto is_cancel_requested = [goal_handle]() {
      return goal_handle->is_canceling();
    };
  if (!wait_for_map(this->map_wait_timeout_sec_, is_cancel_requested)) {
    result->error_code = goal_handle->is_canceling() ?
      NavigateToUnknownPose::Result::CANCELED :
      NavigateToUnknownPose::Result::MAP_NOT_READY;
    result->error_msg = goal_handle->is_canceling() ?
      "Frontier navigation canceled while waiting for map." :
      "SLAM map was not received before timeout.";
    this->frontier_active_.store(false);
    this->publish_empty_overlays(goal_handle->is_canceling() ? "CANCELED" : "FAILED", result->error_msg);
    this->finalize_result(goal_handle, result, goal_handle->is_canceling() ? "canceled" : "aborted");
    this->clear_active_goal(goal_handle);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!lookup_current_pose(current_pose, error_message)) {
    result->error_code = NavigateToUnknownPose::Result::TF_ERROR;
    result->error_msg = error_message;
    this->frontier_active_.store(false);
    this->publish_empty_overlays("FAILED", result->error_msg);
    this->finalize_result(goal_handle, result, "aborted");
    this->clear_active_goal(goal_handle);
    return;
  }

  const uint16_t max_iterations =
    resolve_uint16_goal_value(goal->max_iterations, this->max_iterations_);
  const double known_wait_timeout =
    resolve_positive_goal_value(goal->goal_known_wait_timeout_sec, this->goal_known_wait_timeout_sec_);
  const double max_search_radius =
    resolve_positive_goal_value(goal->max_staging_search_radius_m, this->max_staging_search_radius_m_);
  const double min_progress =
    resolve_positive_goal_value(goal->min_staging_progress_m, this->min_staging_progress_m_);
  std::optional<double> previous_distance_to_original;

  for (uint16_t iteration = 0U; iteration <= max_iterations; ++iteration) {
    if (goal_handle->is_canceling()) {
      cancel_nested_goal();
      result->error_code = NavigateToUnknownPose::Result::CANCELED;
      result->error_msg = "Frontier navigation canceled.";
      this->frontier_active_.store(false);
      this->publish_empty_overlays("CANCELED", result->error_msg);
      this->finalize_result(goal_handle, result, "canceled");
      this->clear_active_goal(goal_handle);
      return;
    }

    nav_msgs::msg::OccupancyGrid map;
    if (!get_latest_map(map)) {
      result->error_code = NavigateToUnknownPose::Result::MAP_NOT_READY;
      result->error_msg = "SLAM map became unavailable.";
      this->frontier_active_.store(false);
      this->publish_empty_overlays("FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    const CellState original_state = classify_goal(original_goal, map);
    if (original_state == CellState::Occupied) {
      result->error_code = NavigateToUnknownPose::Result::OCCUPIED_GOAL;
      result->error_msg = "Original goal is occupied or non-traversable in the current map.";
      this->frontier_active_.store(false);
      this->publish_empty_overlays("FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    if (original_state == CellState::Free) {
      result->original_goal_became_known = iteration > 0U;
      if (!lookup_current_pose(current_pose, error_message)) {
        result->error_code = NavigateToUnknownPose::Result::TF_ERROR;
        result->error_msg = error_message;
        this->frontier_active_.store(false);
        this->publish_empty_overlays("FAILED", result->error_msg);
        this->finalize_result(goal_handle, result, "aborted");
        this->clear_active_goal(goal_handle);
        return;
      }

      nav_msgs::msg::Path final_plan;
      std::string plan_message;
      if (request_plan(current_pose, original_goal, final_plan, plan_message, is_cancel_requested)) {
        publish_path(final_plan);
      }
      publish_pose(this->known_goal_publisher_, original_goal);
      publish_status_and_feedback(
        iteration == 0U ? "GOAL_ALREADY_KNOWN" : "NAVIGATING_TO_ORIGINAL_GOAL",
        original_goal, original_goal, iteration, true,
        iteration == 0U ?
        "Original goal is already known/free; delegating to NavigateToPose." :
        "Original goal became known/free; navigating to the original goal.",
        goal_handle);
      const auto navigation_result = navigate_to_pose(
        original_goal, original_goal, "NAVIGATING_TO_ORIGINAL_GOAL", iteration, false, goal_handle);
      if (navigation_result.canceled) {
        result->error_code = NavigateToUnknownPose::Result::CANCELED;
        result->error_msg = navigation_result.message;
        this->frontier_active_.store(false);
        this->publish_empty_overlays("CANCELED", result->error_msg);
        this->finalize_result(goal_handle, result, "canceled");
        this->clear_active_goal(goal_handle);
        return;
      }
      if (!navigation_result.success) {
        result->error_code = NavigateToUnknownPose::Result::FINAL_NAVIGATION_FAILED;
        result->error_msg = navigation_result.message;
        this->frontier_active_.store(false);
        this->publish_empty_overlays("FAILED", result->error_msg);
        this->finalize_result(goal_handle, result, "aborted");
        this->clear_active_goal(goal_handle);
        return;
      }

      result->error_code = NavigateToUnknownPose::Result::NONE;
      result->error_msg = "Unknown-goal navigation succeeded.";
      result->last_known_goal = original_goal;
      result->iterations = iteration;
      result->final_goal_reached = true;
      this->publish_status_and_feedback(
        "SUCCEEDED", original_goal, original_goal, iteration, false, result->error_msg, goal_handle);
      this->frontier_active_.store(false);
      this->publish_empty_overlays("SUCCEEDED", result->error_msg);
      this->finalize_result(goal_handle, result, "succeeded");
      this->clear_active_goal(goal_handle);
      return;
    }

    if (iteration >= max_iterations) {
      result->error_code = NavigateToUnknownPose::Result::TIMEOUT;
      result->error_msg =
        "Original goal is still unknown after " + std::to_string(max_iterations) +
        " staging iteration(s).";
      this->frontier_active_.store(false);
      this->publish_empty_overlays("FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    if (!lookup_current_pose(current_pose, error_message)) {
      result->error_code = NavigateToUnknownPose::Result::TF_ERROR;
      result->error_msg = error_message;
      this->frontier_active_.store(false);
      this->publish_empty_overlays("FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    publish_status_and_feedback(
      "RESOLVING_STAGING_GOAL", original_goal, geometry_msgs::msg::PoseStamped(),
      static_cast<uint16_t>(iteration + 1U), true,
      "Resolving reachable known/free staging goal.", goal_handle);
    const auto staging_candidate = resolve_staging_goal(
      current_pose, original_goal, max_search_radius, min_progress,
      previous_distance_to_original, error_message, is_cancel_requested);
    if (!staging_candidate) {
      result->error_code = goal_handle->is_canceling() ?
        NavigateToUnknownPose::Result::CANCELED :
        NavigateToUnknownPose::Result::NO_REACHABLE_STAGING_GOAL;
      result->error_msg = goal_handle->is_canceling() ?
        "Frontier navigation canceled while resolving staging goal." :
        error_message;
      this->frontier_active_.store(false);
      this->publish_empty_overlays(goal_handle->is_canceling() ? "CANCELED" : "FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, goal_handle->is_canceling() ? "canceled" : "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    result->iterations = static_cast<uint16_t>(iteration + 1U);
    result->last_known_goal = staging_candidate->pose;
    publish_pose(this->known_goal_publisher_, staging_candidate->pose);
    publish_path(staging_candidate->plan);
    publish_status_and_feedback(
      "NAVIGATING_TO_STAGING_GOAL", original_goal, staging_candidate->pose,
      static_cast<uint16_t>(iteration + 1U), true,
      "Navigating to reachable known/free staging goal.", goal_handle);

    if (this->structured_logging_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "AMR_LOG schema=v1 component=ft_navigator event=staging_goal_selected iteration=%u target_x=%.3f target_y=%.3f distance_to_original=%.3f plan_length=%.3f frontier_adjacent=%s score=%.3f",
        static_cast<unsigned>(iteration + 1U),
        staging_candidate->pose.pose.position.x,
        staging_candidate->pose.pose.position.y,
        staging_candidate->distance_to_original,
        staging_candidate->plan_length,
        bool_label(staging_candidate->frontier_adjacent),
        staging_candidate->score);
    }

    const auto navigation_result = navigate_to_pose(
      staging_candidate->pose, original_goal, "NAVIGATING_TO_STAGING_GOAL",
      static_cast<uint16_t>(iteration + 1U), true, goal_handle);
    if (navigation_result.preempted_for_original) {
      result->original_goal_became_known = true;
      previous_distance_to_original = staging_candidate->distance_to_original;
      continue;
    }
    if (navigation_result.canceled) {
      result->error_code = NavigateToUnknownPose::Result::CANCELED;
      result->error_msg = navigation_result.message;
      this->frontier_active_.store(false);
      this->publish_empty_overlays("CANCELED", result->error_msg);
      this->finalize_result(goal_handle, result, "canceled");
      this->clear_active_goal(goal_handle);
      return;
    }
    if (!navigation_result.success) {
      result->error_code = NavigateToUnknownPose::Result::STAGING_NAVIGATION_FAILED;
      result->error_msg = navigation_result.message;
      this->frontier_active_.store(false);
      this->publish_empty_overlays("FAILED", result->error_msg);
      this->finalize_result(goal_handle, result, "aborted");
      this->clear_active_goal(goal_handle);
      return;
    }

    publish_status_and_feedback(
      "WAITING_FOR_MAP_EXPANSION", original_goal, staging_candidate->pose,
      static_cast<uint16_t>(iteration + 1U), true,
      "Staging goal reached; waiting for SLAM map expansion.", goal_handle);
    if (wait_for_original_goal_known(
        original_goal, known_wait_timeout, static_cast<uint16_t>(iteration + 1U), goal_handle))
    {
      result->original_goal_became_known = true;
    }
    previous_distance_to_original = staging_candidate->distance_to_original;
  }

  result->error_code = NavigateToUnknownPose::Result::TIMEOUT;
  result->error_msg = "Unknown-goal navigation stopped after exhausting iterations.";
  this->frontier_active_.store(false);
  this->publish_empty_overlays("FAILED", result->error_msg);
  this->finalize_result(goal_handle, result, "aborted");
  this->clear_active_goal(goal_handle);
}

void FrontierNavigator::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::scoped_lock lock(this->map_mutex_);
  this->latest_map_ = *message;
  this->has_map_ = true;
}

void FrontierNavigator::local_plan_callback(const nav_msgs::msg::Path::SharedPtr message)
{
  if (!message || !this->frontier_active_.load()) {
    return;
  }
  if (this->frontier_local_plan_publisher_ && this->frontier_local_plan_publisher_->is_activated()) {
    this->frontier_local_plan_publisher_->publish(*message);
  }
}

bool FrontierNavigator::wait_for_map(
  const double timeout_sec,
  const std::function<bool()> &is_cancel_requested)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, timeout_sec)));
  while (rclcpp::ok() && std::chrono::steady_clock::now() <= deadline) {
    if (is_cancel_requested && is_cancel_requested()) {
      return false;
    }
    nav_msgs::msg::OccupancyGrid map;
    if (get_latest_map(map)) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool FrontierNavigator::get_latest_map(nav_msgs::msg::OccupancyGrid &map) const
{
  std::scoped_lock lock(this->map_mutex_);
  if (!this->has_map_) {
    return false;
  }
  const OccupancyGridView view(this->latest_map_, this->free_threshold_, this->occupied_threshold_);
  if (!view.valid()) {
    return false;
  }
  map = this->latest_map_;
  return true;
}

bool FrontierNavigator::transform_goal_to_map(
  const geometry_msgs::msg::PoseStamped &input,
  geometry_msgs::msg::PoseStamped &output,
  std::string &error_message) const
{
  if (input.header.frame_id.empty()) {
    error_message = "Unknown goal frame_id is empty.";
    return false;
  }
  if (input.header.frame_id == this->map_frame_) {
    output = input;
    output.header.frame_id = this->map_frame_;
    return true;
  }
  if (!this->tf_buffer_) {
    error_message = "TF buffer is not configured.";
    return false;
  }
  try {
    output = this->tf_buffer_->transform(
      input, this->map_frame_, tf2::durationFromSec(this->tf_lookup_timeout_sec_));
    return true;
  } catch (const tf2::TransformException &error) {
    error_message = std::string("Failed to transform unknown goal to map frame: ") + error.what();
    return false;
  }
}

bool FrontierNavigator::lookup_current_pose(
  geometry_msgs::msg::PoseStamped &pose,
  std::string &error_message) const
{
  if (!this->tf_buffer_) {
    error_message = "TF buffer is not configured.";
    return false;
  }

  const auto lookup_frame = [this, &pose](const std::string &frame) {
      const auto transform = this->tf_buffer_->lookupTransform(
        this->map_frame_,
        frame,
        tf2::TimePointZero,
        tf2::durationFromSec(this->tf_lookup_timeout_sec_));
      pose.header = transform.header;
      pose.header.frame_id = this->map_frame_;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
    };

  try {
    lookup_frame(this->base_frame_);
    return true;
  } catch (const tf2::TransformException &primary_error) {
    if (this->base_fallback_frame_.empty() || this->base_fallback_frame_ == this->base_frame_) {
      error_message = std::string("Failed to lookup current pose: ") + primary_error.what();
      return false;
    }
    try {
      lookup_frame(this->base_fallback_frame_);
      return true;
    } catch (const tf2::TransformException &fallback_error) {
      error_message =
        std::string("Failed to lookup current pose from '") + this->base_frame_ +
        "' or fallback '" + this->base_fallback_frame_ + "': " + fallback_error.what();
      return false;
    }
  }
}

CellState FrontierNavigator::classify_goal(
  const geometry_msgs::msg::PoseStamped &goal,
  const nav_msgs::msg::OccupancyGrid &map) const
{
  const OccupancyGridView view(map, this->free_threshold_, this->occupied_threshold_);
  GridCell cell;
  if (!view.world_to_grid(goal.pose.position.x, goal.pose.position.y, cell)) {
    return CellState::Unknown;
  }
  return view.classify(cell);
}

bool FrontierNavigator::is_goal_known_free(const geometry_msgs::msg::PoseStamped &goal) const
{
  nav_msgs::msg::OccupancyGrid map;
  if (!get_latest_map(map)) {
    return false;
  }
  return classify_goal(goal, map) == CellState::Free;
}

bool FrontierNavigator::wait_for_original_goal_known(
  const geometry_msgs::msg::PoseStamped &original_goal,
  const double timeout_sec,
  const uint16_t iteration,
  const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, timeout_sec)));
  while (rclcpp::ok() && std::chrono::steady_clock::now() <= deadline) {
    if (goal_handle->is_canceling()) {
      return false;
    }
    if (is_goal_known_free(original_goal)) {
      return true;
    }
    publish_status_and_feedback(
      "WAITING_FOR_MAP_EXPANSION", original_goal, geometry_msgs::msg::PoseStamped(),
      iteration, true, "Waiting for original goal cell to become known/free.", goal_handle);
    std::this_thread::sleep_for(std::chrono::milliseconds(this->feedback_period_ms_));
  }
  return is_goal_known_free(original_goal);
}

std::optional<FrontierNavigator::StagingCandidate> FrontierNavigator::resolve_staging_goal(
  const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &original_goal,
  const double max_search_radius_m,
  const double min_progress_m,
  const std::optional<double> previous_distance_to_original,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  nav_msgs::msg::OccupancyGrid map;
  if (!get_latest_map(map)) {
    error_message = "SLAM map is not available.";
    return std::nullopt;
  }

  const OccupancyGridView view(map, this->free_threshold_, this->occupied_threshold_);
  if (!view.valid()) {
    error_message = "SLAM map is invalid.";
    return std::nullopt;
  }

  const double radius_m = std::max(view.resolution(), max_search_radius_m);
  const int max_radius_cells = static_cast<int>(std::ceil(radius_m / view.resolution()));
  const GridCell center = view.world_to_grid_unbounded(
    original_goal.pose.position.x,
    original_goal.pose.position.y);
  std::vector<GridCell> candidates;
  candidates.reserve(static_cast<std::size_t>((max_radius_cells * 2 + 1) * (max_radius_cells * 2 + 1)));
  std::set<std::pair<int, int>> local_candidate_keys;

  const auto progress_allowed = [&](const GridCell &cell) {
      const double distance_to_original = view.distance_to_cell_center(
        cell, original_goal.pose.position.x, original_goal.pose.position.y);
      return
        !previous_distance_to_original ||
        distance_to_original <= (*previous_distance_to_original - min_progress_m);
    };

  const auto add_candidate = [&](std::vector<GridCell> &target, const GridCell &cell) {
      if (!view.is_free(cell)) {
        return;
      }
      if (!view.has_obstacle_clearance(cell, this->min_obstacle_clearance_m_)) {
        return;
      }
      if (!progress_allowed(cell)) {
        return;
      }
      target.push_back(cell);
    };

  for (int dy = -max_radius_cells; dy <= max_radius_cells; dy += this->candidate_step_cells_) {
    for (int dx = -max_radius_cells; dx <= max_radius_cells; dx += this->candidate_step_cells_) {
      const double offset_m =
        std::sqrt(static_cast<double>((dx * dx) + (dy * dy))) * view.resolution();
      if (offset_m > radius_m) {
        continue;
      }
      const GridCell cell{center.x + dx, center.y + dy};
      local_candidate_keys.insert({cell.x, cell.y});
      add_candidate(candidates, cell);
    }
  }

  const auto sort_by_goal_distance = [&view, &original_goal](std::vector<GridCell> &cells) {
      std::sort(
        cells.begin(), cells.end(),
        [&view, &original_goal](const GridCell &lhs, const GridCell &rhs) {
          return view.distance_to_cell_center(
            lhs, original_goal.pose.position.x, original_goal.pose.position.y) <
            view.distance_to_cell_center(
            rhs, original_goal.pose.position.x, original_goal.pose.position.y);
        });
    };

  const auto evaluate_candidates =
    [&](std::vector<GridCell> &source, int &checked_candidates) -> std::optional<StagingCandidate> {
      sort_by_goal_distance(source);
      std::optional<StagingCandidate> best_candidate;
      checked_candidates = 0;
      for (const GridCell &cell : source) {
        if (is_cancel_requested && is_cancel_requested()) {
          error_message = "Canceled while checking staging candidates.";
          return std::nullopt;
        }
        if (checked_candidates >= this->max_candidate_checks_) {
          break;
        }
        ++checked_candidates;

        const auto cell_pose = view.grid_to_pose(cell, this->map_frame_, 0.0);
        const double yaw_to_original = std::atan2(
          original_goal.pose.position.y - cell_pose.pose.position.y,
          original_goal.pose.position.x - cell_pose.pose.position.x);
        auto candidate_pose = cell_pose;
        candidate_pose.header.frame_id = this->map_frame_;
        candidate_pose.header.stamp = this->now();
        candidate_pose.pose.orientation = quaternion_from_yaw(yaw_to_original);

        nav_msgs::msg::Path plan;
        std::string plan_message;
        if (this->use_plan_segment_validation_) {
          if (!request_plan(current_pose, candidate_pose, plan, plan_message, is_cancel_requested)) {
            continue;
          }
        } else {
          plan.header.frame_id = this->map_frame_;
          plan.header.stamp = this->now();
          plan.poses.push_back(current_pose);
          plan.poses.push_back(candidate_pose);
        }

        StagingCandidate candidate;
        candidate.pose = candidate_pose;
        candidate.plan = plan;
        candidate.distance_to_original = view.distance_to_cell_center(
          cell, original_goal.pose.position.x, original_goal.pose.position.y);
        candidate.plan_length = path_length(plan);
        candidate.frontier_adjacent = view.has_unknown_neighbor(cell, this->frontier_neighbor_radius_cells_);
        candidate.score =
          candidate.distance_to_original +
          (candidate.plan_length * 0.20) +
          (candidate.frontier_adjacent ? 0.0 : 0.75);

        if (!best_candidate || candidate.score < best_candidate->score) {
          best_candidate = candidate;
        }
      }
      return best_candidate;
    };

  int checked_local_candidates = 0;
  std::optional<StagingCandidate> best_candidate =
    evaluate_candidates(candidates, checked_local_candidates);

  std::vector<GridCell> frontier_candidates;
  int checked_frontier_candidates = 0;
  if (!best_candidate) {
    frontier_candidates.reserve(
      static_cast<std::size_t>(view.width()) * static_cast<std::size_t>(view.height()) / 8U);
    for (int y = 0; y < view.height(); y += this->candidate_step_cells_) {
      for (int x = 0; x < view.width(); x += this->candidate_step_cells_) {
        const GridCell cell{x, y};
        if (local_candidate_keys.count({cell.x, cell.y}) > 0U) {
          continue;
        }
        if (!view.has_unknown_neighbor(cell, this->frontier_neighbor_radius_cells_)) {
          continue;
        }
        add_candidate(frontier_candidates, cell);
      }
    }
    best_candidate = evaluate_candidates(frontier_candidates, checked_frontier_candidates);
  }

  if (!best_candidate) {
    std::ostringstream stream;
    stream << "No reachable staging goal found. Checked " << checked_local_candidates
           << " local candidate(s) within " << radius_m
           << " m around the original goal";
    if (previous_distance_to_original) {
      stream << " with at least " << min_progress_m << " m progress";
    }
    if (!frontier_candidates.empty() || checked_frontier_candidates > 0) {
      stream << " and " << checked_frontier_candidates << " map-frontier candidate(s)";
    }
    stream << ".";
    error_message = stream.str();
    return std::nullopt;
  }

  error_message.clear();
  return best_candidate;
}

bool FrontierNavigator::request_plan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav_msgs::msg::Path &plan,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  if (!this->plan_segment_client_) {
    error_message = "PlanSegment client is not configured.";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(std::max(1, this->planner_wait_timeout_ms_));
  while (!this->plan_segment_client_->service_is_ready()) {
    if (is_cancel_requested && is_cancel_requested()) {
      error_message = "Canceled while waiting for PlanSegment service.";
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      error_message = "PlanSegment service is not ready.";
      return false;
    }
    this->plan_segment_client_->wait_for_service(50ms);
  }

  auto request = std::make_shared<amr_msgs::srv::PlanSegment::Request>();
  request->start = start;
  request->goal = goal;
  auto future = this->plan_segment_client_->async_send_request(request);
  while (future.wait_for(50ms) != std::future_status::ready) {
    if (is_cancel_requested && is_cancel_requested()) {
      error_message = "Canceled while waiting for PlanSegment response.";
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      error_message = "PlanSegment response timed out.";
      return false;
    }
  }

  const auto response = future.get();
  if (!response || !response->success) {
    error_message = response ? response->message : "PlanSegment response was empty.";
    return false;
  }
  plan = response->plan;
  return true;
}

FrontierNavigator::NavigationOutcome FrontierNavigator::navigate_to_pose(
  const geometry_msgs::msg::PoseStamped &goal,
  const geometry_msgs::msg::PoseStamped &original_goal,
  const std::string &phase,
  const uint16_t iteration,
  const bool monitor_original_goal,
  const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  NavigationOutcome outcome;
  if (!this->navigate_to_pose_client_) {
    outcome.message = "NavigateToPose action client is not configured.";
    return outcome;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(std::max(1, this->action_server_wait_timeout_ms_));
  while (!this->navigate_to_pose_client_->action_server_is_ready()) {
    if (goal_handle->is_canceling()) {
      outcome.canceled = true;
      outcome.error_code = NavigateToUnknownPose::Result::CANCELED;
      outcome.message = "Canceled while waiting for NavigateToPose action server.";
      return outcome;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      outcome.message = "NavigateToPose action server is not ready.";
      return outcome;
    }
    this->navigate_to_pose_client_->wait_for_action_server(50ms);
  }

  NavigateToPose::Goal nested_goal;
  nested_goal.goal_pose = goal;
  typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [this, original_goal, goal, phase, iteration, goal_handle](
      typename NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      (void)feedback;
      this->publish_status_and_feedback(
        phase, original_goal, goal, iteration, true,
        "NavigateToPose is executing delegated goal.", goal_handle);
    };

  const auto goal_future = this->navigate_to_pose_client_->async_send_goal(nested_goal, options);
  while (goal_future.wait_for(50ms) != std::future_status::ready) {
    if (goal_handle->is_canceling()) {
      outcome.canceled = true;
      outcome.error_code = NavigateToUnknownPose::Result::CANCELED;
      outcome.message = "Canceled while sending delegated NavigateToPose goal.";
      return outcome;
    }
  }

  auto nested_handle = goal_future.get();
  if (!nested_handle) {
    outcome.message = "NavigateToPose rejected delegated goal.";
    return outcome;
  }
  {
    std::scoped_lock lock(this->nested_goal_mutex_);
    this->nested_goal_handle_ = nested_handle;
  }

  const auto result_future = this->navigate_to_pose_client_->async_get_result(nested_handle);
  while (result_future.wait_for(std::chrono::milliseconds(this->feedback_period_ms_)) !=
    std::future_status::ready)
  {
    if (goal_handle->is_canceling()) {
      this->navigate_to_pose_client_->async_cancel_goal(nested_handle);
      outcome.canceled = true;
      outcome.error_code = NavigateToUnknownPose::Result::CANCELED;
      outcome.message = "Frontier navigation canceled delegated NavigateToPose goal.";
      {
        std::scoped_lock lock(this->nested_goal_mutex_);
        this->nested_goal_handle_.reset();
      }
      return outcome;
    }
    if (monitor_original_goal && is_goal_known_free(original_goal)) {
      this->navigate_to_pose_client_->async_cancel_goal(nested_handle);
      outcome.preempted_for_original = true;
      outcome.message = "Original goal became known/free during staging navigation.";
      {
        std::scoped_lock lock(this->nested_goal_mutex_);
        this->nested_goal_handle_.reset();
      }
      return outcome;
    }
    publish_status_and_feedback(
      phase, original_goal, goal, iteration, true,
      "NavigateToPose is executing delegated goal.", goal_handle);
  }

  const auto wrapped_result = result_future.get();
  {
    std::scoped_lock lock(this->nested_goal_mutex_);
    if (this->nested_goal_handle_ == nested_handle) {
      this->nested_goal_handle_.reset();
    }
  }

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    outcome.success = true;
    outcome.error_code = NavigateToUnknownPose::Result::NONE;
    outcome.message = "Delegated NavigateToPose goal succeeded.";
    return outcome;
  }
  if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
    outcome.canceled = true;
    outcome.error_code = NavigateToUnknownPose::Result::CANCELED;
    outcome.message = "Delegated NavigateToPose goal was canceled.";
    return outcome;
  }
  outcome.error_code = NavigateToUnknownPose::Result::UNKNOWN;
  outcome.message = wrapped_result.result ?
    wrapped_result.result->error_msg :
    "Delegated NavigateToPose goal failed without a result message.";
  if (outcome.message.empty()) {
    outcome.message = "Delegated NavigateToPose goal failed.";
  }
  return outcome;
}

void FrontierNavigator::cancel_nested_goal()
{
  std::shared_ptr<NavigateGoalHandle> handle;
  {
    std::scoped_lock lock(this->nested_goal_mutex_);
    handle = this->nested_goal_handle_;
    this->nested_goal_handle_.reset();
  }
  if (handle && this->navigate_to_pose_client_) {
    this->navigate_to_pose_client_->async_cancel_goal(handle);
  }
}

void FrontierNavigator::clear_active_goal(const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  std::scoped_lock lock(this->active_goal_mutex_);
  if (this->active_goal_handle_ == goal_handle) {
    this->active_goal_handle_.reset();
  }
}

void FrontierNavigator::publish_pose(
  const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher,
  const geometry_msgs::msg::PoseStamped &pose) const
{
  if (publisher && publisher->is_activated()) {
    publisher->publish(pose);
  }
}

void FrontierNavigator::publish_path(const nav_msgs::msg::Path &path) const
{
  if (this->frontier_global_plan_publisher_ && this->frontier_global_plan_publisher_->is_activated()) {
    this->frontier_global_plan_publisher_->publish(path);
  }
}

void FrontierNavigator::publish_empty_overlays(
  const std::string &phase,
  const std::string &message)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = this->map_frame_;
  empty_path.header.stamp = this->now();
  if (this->frontier_global_plan_publisher_ && this->frontier_global_plan_publisher_->is_activated()) {
    this->frontier_global_plan_publisher_->publish(empty_path);
  }
  if (this->frontier_local_plan_publisher_ && this->frontier_local_plan_publisher_->is_activated()) {
    this->frontier_local_plan_publisher_->publish(empty_path);
  }

  geometry_msgs::msg::PoseStamped empty_pose;
  empty_pose.header.stamp = this->now();
  publish_pose(this->unknown_goal_publisher_, empty_pose);
  publish_pose(this->known_goal_publisher_, empty_pose);

  amr_msgs::msg::FrontierNavigationStatus status;
  status.header.frame_id = this->map_frame_;
  status.header.stamp = this->now();
  status.phase = phase;
  status.active = false;
  status.original_goal_known = false;
  status.distance_to_original_goal = -1.0F;
  status.message = message;
  if (this->status_publisher_ && this->status_publisher_->is_activated()) {
    this->status_publisher_->publish(status);
  }
}

void FrontierNavigator::publish_status_and_feedback(
  const std::string &phase,
  const geometry_msgs::msg::PoseStamped &original_goal,
  const geometry_msgs::msg::PoseStamped &active_known_goal,
  const uint16_t iteration,
  const bool active,
  const std::string &message,
  const std::shared_ptr<GoalHandleUnknown> goal_handle)
{
  geometry_msgs::msg::PoseStamped current_pose;
  std::string tf_error;
  const bool has_pose = lookup_current_pose(current_pose, tf_error);
  const bool original_known = is_goal_known_free(original_goal);
  const float distance_to_original = has_pose ?
    static_cast<float>(pose_distance(current_pose, original_goal)) :
    -1.0F;

  amr_msgs::msg::FrontierNavigationStatus status;
  status.header.frame_id = this->map_frame_;
  status.header.stamp = this->now();
  status.phase = phase;
  status.original_goal = original_goal;
  status.active_known_goal = active_known_goal;
  status.iteration = iteration;
  status.active = active;
  status.original_goal_known = original_known;
  status.distance_to_original_goal = distance_to_original;
  status.message = message;
  if (this->status_publisher_ && this->status_publisher_->is_activated()) {
    this->status_publisher_->publish(status);
  }

  if (goal_handle && goal_handle->is_active()) {
    auto feedback = std::make_shared<NavigateToUnknownPose::Feedback>();
    feedback->current_pose = current_pose;
    feedback->original_goal = original_goal;
    feedback->active_known_goal = active_known_goal;
    feedback->iteration = iteration;
    feedback->phase = phase;
    feedback->distance_to_original_goal = distance_to_original;
    feedback->original_goal_known = original_known;
    goal_handle->publish_feedback(feedback);
  }
}

void FrontierNavigator::finalize_result(
  const std::shared_ptr<GoalHandleUnknown> goal_handle,
  const std::shared_ptr<NavigateToUnknownPose::Result> result,
  const std::string &state)
{
  if (!goal_handle || !goal_handle->is_active()) {
    return;
  }
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=ft_navigator event=goal_finished result=%s error_code=%u iterations=%u final_reached=%s reason=%s",
      state.c_str(),
      result->error_code,
      result->iterations,
      bool_label(result->final_goal_reached),
      log_value(result->error_msg).c_str());
  }
  if (state == "succeeded") {
    goal_handle->succeed(result);
  } else if (state == "canceled") {
    goal_handle->canceled(result);
  } else {
    goal_handle->abort(result);
  }
}

}  // namespace amr::frontier_navigation
