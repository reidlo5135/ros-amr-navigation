/**
 * @file bt_navigator.cpp
 * @brief Implementation of AMR navigation action orchestration and recovery policy.
 */

#include "amr_bt_navigator/bt_navigator.hpp"

#include <iomanip>
#include <sstream>

namespace amr::bt::navigator
{

namespace
{

enum class BtOutcome
{
  kRunning,
  kSucceeded,
  kCanceled,
  kStopped
};

/// @brief Translate local planner decision codes into structured log labels.
const char *local_plan_decision_label(const uint8_t decision)
{
  switch (decision) {
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
      return "ok";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
      return "goal_proximity_blocked";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
      return "global_replan_required";
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return "hard_blocked";
    default:
      return "unknown";
  }
}

/// @brief Return a stable true/false label for structured logs.
const char *bool_label(const bool value)
{
  return value ? "true" : "false";
}

/// @brief Sanitize a string for single-token structured log fields.
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

/// @brief Resolve the default behavior-tree XML path from the package share directory.
std::string get_default_behavior_tree_xml_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("amr_bt_navigator") +
           "/config/navigate_to_pose.xml";
  } catch (const std::exception &) {
    return "config/navigate_to_pose.xml";
  }
}

/// @brief Build a planar quaternion from a yaw angle.
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

/// @brief Copy a route waypoint and align its heading toward the next waypoint when available.
geometry_msgs::msg::PoseStamped make_route_goal_pose(
  const std::vector<geometry_msgs::msg::PoseStamped> &goal_poses,
  std::size_t index)
{
  geometry_msgs::msg::PoseStamped goal_pose = goal_poses[index];
  if (index + 1U >= goal_poses.size()) {
    return goal_pose;
  }

  const auto &next_goal = goal_poses[index + 1U];
  const double dx = next_goal.pose.position.x - goal_pose.pose.position.x;
  const double dy = next_goal.pose.position.y - goal_pose.pose.position.y;
  if ((dx * dx) + (dy * dy) <= 1e-8) {
    return goal_pose;
  }

  goal_pose.pose.orientation = yaw_to_quaternion(std::atan2(dy, dx));
  return goal_pose;
}

}  // namespace

/// @copydoc Btnavigator::Btnavigator
Btnavigator::Btnavigator(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("bt_navigator", options),
  navigate_action_name_("/navigate_to_pose"),
  navigate_poses_action_name_("/navigate_to_poses"),
  command_topic_("/motion_command"),
  motion_status_topic_("/motion_status"),
  local_plan_status_topic_("/local_plan_status"),
  plan_recovery_service_("/plan_recovery"),
  plan_local_escape_service_("/plan_local_escape"),
  clear_costmap_service_("/clear_costmap"),
  plan_segment_service_("/plan_segment"),
  map_frame_("map"),
  odom_frame_("odom"),
  base_frame_("base_footprint"),
  tf_lookup_timeout_sec_(0.05),
  behavior_tree_xml_path_(""),
  default_node_id_("start"),
  planner_wait_timeout_ms_(2000),
  feedback_period_ms_(100),
  recovery_max_retries_(3),
  recovery_retry_delay_ms_(700),
  recovery_reacquire_settle_ms_(900),
  align_heading_at_goal_(true),
  structured_logging_enabled_(true),
  recovery_decision_logging_enabled_(true),
  nominal_speed_(0.075),
  next_command_id_(1U),
  has_current_pose_(false),
  has_motion_status_(false),
  has_local_plan_status_(false),
  last_terminal_command_id_(0U),
  last_terminal_status_time_(0, 0, this->get_clock()->get_clock_type()),
  latched_goal_reached_(false),
  latched_command_completed_(false)
{
  this->behavior_tree_xml_path_ = get_default_behavior_tree_xml_path();
  this->declare_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->declare_parameter("actions.navigate_to_poses", this->navigate_poses_action_name_);
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.status", this->motion_status_topic_);
  this->declare_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->declare_parameter("services.plan_recovery", this->plan_recovery_service_);
  this->declare_parameter("services.local_escape", this->plan_local_escape_service_);
  this->declare_parameter("services.clear_costmap", this->clear_costmap_service_);
  this->declare_parameter("services.segment", this->plan_segment_service_);
  this->declare_parameter("frames.map", this->map_frame_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("tf.lookup_timeout_sec", this->tf_lookup_timeout_sec_);
  this->declare_parameter("behavior_tree.xml_path", this->behavior_tree_xml_path_);
  this->declare_parameter("behavior_tree_xml_path", this->behavior_tree_xml_path_);
  this->declare_parameter("defaults.node_id", this->default_node_id_);
  this->declare_parameter(
    "execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->declare_parameter("execution.feedback_period_ms", this->feedback_period_ms_);
  this->declare_parameter("recovery.max_retries", this->recovery_max_retries_);
  this->declare_parameter("recovery.retry_delay_ms", this->recovery_retry_delay_ms_);
  this->declare_parameter(
    "recovery.reacquire_settle_ms", this->recovery_reacquire_settle_ms_);
  this->declare_parameter("execution.align_heading_at_goal", this->align_heading_at_goal_);
  this->declare_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->declare_parameter(
    "logging.recovery_decision_logging", this->recovery_decision_logging_enabled_);
  this->declare_parameter("execution.nominal_linear_speed", this->nominal_speed_);
}

/// @copydoc Btnavigator::on_configure
Btnavigator::CallbackReturn Btnavigator::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->get_parameter("actions.navigate_to_poses", this->navigate_poses_action_name_);
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.status", this->motion_status_topic_);
  this->get_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->get_parameter("services.plan_recovery", this->plan_recovery_service_);
  this->get_parameter("services.local_escape", this->plan_local_escape_service_);
  this->get_parameter("services.clear_costmap", this->clear_costmap_service_);
  this->get_parameter("services.segment", this->plan_segment_service_);
  this->get_parameter("frames.map", this->map_frame_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("tf.lookup_timeout_sec", this->tf_lookup_timeout_sec_);
  this->get_parameter("behavior_tree.xml_path", this->behavior_tree_xml_path_);
  {
    std::string legacy_behavior_tree_xml_path;
    if (this->get_parameter("behavior_tree_xml_path", legacy_behavior_tree_xml_path) &&
      !legacy_behavior_tree_xml_path.empty())
    {
      this->behavior_tree_xml_path_ = legacy_behavior_tree_xml_path;
    }
  }
  this->get_parameter("defaults.node_id", this->default_node_id_);
  this->get_parameter(
    "execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->get_parameter("execution.feedback_period_ms", this->feedback_period_ms_);
  this->get_parameter("recovery.max_retries", this->recovery_max_retries_);
  this->get_parameter("recovery.retry_delay_ms", this->recovery_retry_delay_ms_);
  this->get_parameter(
    "recovery.reacquire_settle_ms", this->recovery_reacquire_settle_ms_);
  this->get_parameter("execution.align_heading_at_goal", this->align_heading_at_goal_);
  this->get_parameter("logging.structured_enabled", this->structured_logging_enabled_);
  this->get_parameter(
    "logging.recovery_decision_logging", this->recovery_decision_logging_enabled_);
  this->get_parameter("execution.nominal_linear_speed", this->nominal_speed_);

  if (this->behavior_tree_xml_path_.empty()) {
    this->behavior_tree_xml_path_ = get_default_behavior_tree_xml_path();
  }

  if (
    this->command_topic_.empty() || this->motion_status_topic_.empty() ||
    this->local_plan_status_topic_.empty() || this->map_frame_.empty() ||
    this->base_frame_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigator topics/frames must not be empty: command='%s' status='%s' local_plan_status='%s' map_frame='%s' base_frame='%s'",
      this->command_topic_.c_str(),
      this->motion_status_topic_.c_str(),
      this->local_plan_status_topic_.c_str(),
      this->map_frame_.c_str(),
      this->base_frame_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer_);
  this->motion_command_publisher_ = this->create_publisher<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS());
  this->plan_recovery_client_ = this->create_client<amr_msgs::srv::PlanRecovery>(
    this->plan_recovery_service_);
  this->plan_local_escape_client_ = this->create_client<amr_msgs::srv::PlanLocalEscape>(
    this->plan_local_escape_service_);
  this->clear_costmap_client_ = this->create_client<amr_msgs::srv::ClearCostmap>(
    this->clear_costmap_service_);
  this->plan_segment_client_ = this->create_client<amr_msgs::srv::PlanSegment>(
    this->plan_segment_service_);
  this->motion_status_subscription_ = this->create_subscription<amr_msgs::msg::MotionStatus>(
    this->motion_status_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::MotionStatus::SharedPtr message) {
      this->handle_motion_status(message);
    });
  this->local_plan_status_subscription_ = this->create_subscription<amr_msgs::msg::LocalPlanStatus>(
    this->local_plan_status_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::LocalPlanStatus::SharedPtr message) {
      this->handle_local_plan_status(message);
    });
  this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->navigate_action_name_,
    [this](
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      this->handle_accepted(goal_handle);
    });
  this->action_server_poses_ = rclcpp_action::create_server<NavigateToPoses>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->navigate_poses_action_name_,
    [this](
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPoses::Goal> goal) {
      return this->handle_goals(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle) {
      return this->handle_cancel_goals(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle) {
      this->handle_accepted_goals(goal_handle);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured navigator with actions='%s'/'%s', command='%s', status='%s', local_plan_status='%s', map_frame='%s', odom_frame='%s', base_frame='%s', recovery='%s', local_escape='%s', clear_costmap='%s', planner='%s', bt_xml='%s'",
    this->navigate_action_name_.c_str(),
    this->navigate_poses_action_name_.c_str(),
    this->command_topic_.c_str(),
    this->motion_status_topic_.c_str(),
    this->local_plan_status_topic_.c_str(),
    this->map_frame_.c_str(),
    this->odom_frame_.c_str(),
    this->base_frame_.c_str(),
    this->plan_recovery_service_.c_str(),
    this->plan_local_escape_service_.c_str(),
    this->clear_costmap_service_.c_str(),
    this->plan_segment_service_.c_str(),
    this->behavior_tree_xml_path_.c_str());

  return CallbackReturn::SUCCESS;
}

/// @copydoc Btnavigator::on_activate
Btnavigator::CallbackReturn Btnavigator::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated navigator");
  return CallbackReturn::SUCCESS;
}

/// @copydoc Btnavigator::on_deactivate
Btnavigator::CallbackReturn Btnavigator::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated navigator");
  return CallbackReturn::SUCCESS;
}

/// @copydoc Btnavigator::on_cleanup
Btnavigator::CallbackReturn Btnavigator::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->action_server_.reset();
  this->action_server_poses_.reset();
  this->plan_recovery_client_.reset();
  this->plan_local_escape_client_.reset();
  this->clear_costmap_client_.reset();
  this->plan_segment_client_.reset();
  this->motion_status_subscription_.reset();
  this->local_plan_status_subscription_.reset();
  this->motion_command_publisher_.reset();
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->latest_local_plan_status_ = amr_msgs::msg::LocalPlanStatus();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  this->has_local_plan_status_ = false;
  this->active_goal_handle_.reset();
  this->active_goals_handle_.reset();
  return CallbackReturn::SUCCESS;
}

/// @copydoc Btnavigator::on_shutdown
Btnavigator::CallbackReturn Btnavigator::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->action_server_.reset();
  this->action_server_poses_.reset();
  this->plan_recovery_client_.reset();
  this->plan_local_escape_client_.reset();
  this->clear_costmap_client_.reset();
  this->plan_segment_client_.reset();
  this->motion_status_subscription_.reset();
  this->local_plan_status_subscription_.reset();
  this->motion_command_publisher_.reset();
  this->tf_listener_.reset();
  this->tf_buffer_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->latest_local_plan_status_ = amr_msgs::msg::LocalPlanStatus();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  this->has_local_plan_status_ = false;
  this->active_goal_handle_.reset();
  this->active_goals_handle_.reset();
  return CallbackReturn::SUCCESS;
}

/// @copydoc Btnavigator::handle_goal
rclcpp_action::GoalResponse Btnavigator::handle_goal(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  if (this->has_active_goal()) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_pose result=rejected reason=active_goal_exists");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_pose result=rejected reason=navigator_inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->goal_pose.header.frame_id.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_pose result=rejected reason=empty_frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_pose result=accepted goal_id=%u frame=%s target_x=%.3f target_y=%.3f",
      this->next_command_id_,
      goal->goal_pose.header.frame_id.c_str(),
      goal->goal_pose.pose.position.x,
      goal->goal_pose.pose.position.y);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/// @copydoc Btnavigator::handle_goals
rclcpp_action::GoalResponse Btnavigator::handle_goals(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToPoses::Goal> goal)
{
  (void)uuid;
  if (this->has_active_goal()) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_poses result=rejected reason=active_goal_exists");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_poses result=rejected reason=navigator_inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->goal_poses.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_poses result=rejected reason=empty_goal_poses");
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (std::size_t index = 0; index < goal->goal_poses.size(); ++index) {
    if (goal->goal_poses[index].header.frame_id.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_poses result=rejected reason=empty_frame_id goal_index=%zu",
        index);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  const auto &first_goal = goal->goal_poses.front();
  const auto &last_goal = goal->goal_poses.back();
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_received route_id=navigate_to_poses result=accepted goal_id=%u goal_count=%zu start_x=%.3f start_y=%.3f target_x=%.3f target_y=%.3f",
      this->next_command_id_,
      goal->goal_poses.size(),
      first_goal.pose.position.x,
      first_goal.pose.position.y,
      last_goal.pose.position.x,
      last_goal.pose.position.y);
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/// @copydoc Btnavigator::handle_cancel
rclcpp_action::CancelResponse Btnavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_canceled route_id=navigate_to_pose reason=cancel_requested result=accepted");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

/// @copydoc Btnavigator::handle_cancel_goals
rclcpp_action::CancelResponse Btnavigator::handle_cancel_goals(
  const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  (void)goal_handle;
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_canceled route_id=navigate_to_poses reason=cancel_requested result=accepted");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

/// @copydoc Btnavigator::handle_accepted
void Btnavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    this->active_goal_handle_ = goal_handle;
  }
  std::thread(
    [this, goal_handle]() {
      try {
        this->execute(goal_handle);
      } catch (const BT::RuntimeError &error) {
        this->log_exception("execute_thread_navigate_to_pose", "BT::RuntimeError", error, true);
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled BT runtime exception: ") + error.what();
        this->finalize_goal(goal_handle, result, "aborted", "thread_bt_runtime_error");
        this->clear_active_goal(goal_handle);
      } catch (const std::future_error &error) {
        this->log_exception("execute_thread_navigate_to_pose", "std::future_error", error, true);
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled future exception: ") + error.what();
        this->finalize_goal(goal_handle, result, "aborted", "thread_future_error");
        this->clear_active_goal(goal_handle);
      } catch (const rclcpp::exceptions::RCLError &error) {
        this->log_exception("execute_thread_navigate_to_pose", "rclcpp::exceptions::RCLError", error, true);
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled ROS client exception: ") + error.what();
        this->finalize_goal(goal_handle, result, "aborted", "thread_rclcpp_error");
        this->clear_active_goal(goal_handle);
      } catch (const std::exception &error) {
        this->log_exception("execute_thread_navigate_to_pose", "std::exception", error, true);
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled exception: ") + error.what();
        this->finalize_goal(goal_handle, result, "aborted", "thread_exception");
        this->clear_active_goal(goal_handle);
      } catch (...) {
        this->log_unknown_exception("execute_thread_navigate_to_pose", true);
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::UNKNOWN;
        result->error_msg = "Unhandled unknown exception.";
        this->finalize_goal(goal_handle, result, "aborted", "thread_unknown_exception");
        this->clear_active_goal(goal_handle);
      }
    }).detach();
}

/// @copydoc Btnavigator::handle_accepted_goals
void Btnavigator::handle_accepted_goals(
  const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    this->active_goals_handle_ = goal_handle;
  }
  std::thread(
    [this, goal_handle]() {
      try {
        this->execute_goals(goal_handle);
      } catch (const BT::RuntimeError &error) {
        this->log_exception("execute_thread_navigate_to_poses", "BT::RuntimeError", error, true);
        auto result = std::make_shared<NavigateToPoses::Result>();
        result->error_code = NavigateToPoses::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled BT runtime exception: ") + error.what();
        result->completed_goals = 0U;
        this->finalize_goal(goal_handle, result, "aborted", "thread_bt_runtime_error");
        this->clear_active_goal(goal_handle);
      } catch (const std::future_error &error) {
        this->log_exception("execute_thread_navigate_to_poses", "std::future_error", error, true);
        auto result = std::make_shared<NavigateToPoses::Result>();
        result->error_code = NavigateToPoses::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled future exception: ") + error.what();
        result->completed_goals = 0U;
        this->finalize_goal(goal_handle, result, "aborted", "thread_future_error");
        this->clear_active_goal(goal_handle);
      } catch (const rclcpp::exceptions::RCLError &error) {
        this->log_exception("execute_thread_navigate_to_poses", "rclcpp::exceptions::RCLError", error, true);
        auto result = std::make_shared<NavigateToPoses::Result>();
        result->error_code = NavigateToPoses::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled ROS client exception: ") + error.what();
        result->completed_goals = 0U;
        this->finalize_goal(goal_handle, result, "aborted", "thread_rclcpp_error");
        this->clear_active_goal(goal_handle);
      } catch (const std::exception &error) {
        this->log_exception("execute_thread_navigate_to_poses", "std::exception", error, true);
        auto result = std::make_shared<NavigateToPoses::Result>();
        result->error_code = NavigateToPoses::Result::UNKNOWN;
        result->error_msg = std::string("Unhandled exception: ") + error.what();
        result->completed_goals = 0U;
        this->finalize_goal(goal_handle, result, "aborted", "thread_exception");
        this->clear_active_goal(goal_handle);
      } catch (...) {
        this->log_unknown_exception("execute_thread_navigate_to_poses", true);
        auto result = std::make_shared<NavigateToPoses::Result>();
        result->error_code = NavigateToPoses::Result::UNKNOWN;
        result->error_msg = "Unhandled unknown exception.";
        result->completed_goals = 0U;
        this->finalize_goal(goal_handle, result, "aborted", "thread_unknown_exception");
        this->clear_active_goal(goal_handle);
      }
    }).detach();
}

/// @copydoc Btnavigator::execute
void Btnavigator::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto result = this->execute_goal_pose(
    goal->goal_pose,
    "navigate_to_pose",
    0U,
    1U,
    this->align_heading_at_goal_,
    [goal_handle]() { return goal_handle->is_canceling(); },
    [goal_handle, this](
      const geometry_msgs::msg::PoseStamped &pose,
      const amr_msgs::msg::MotionStatus &status,
      int32_t recovery_count,
      const rclcpp::Duration &nav_time) {
      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      feedback->current_pose = pose;
      feedback->distance_remaining = static_cast<float>(status.remaining_distance);
      feedback->number_of_recoveries = static_cast<int16_t>(recovery_count);
      const auto nav_ns = nav_time.nanoseconds();
      feedback->navigation_time.sec = static_cast<int32_t>(nav_ns / 1000000000LL);
      feedback->navigation_time.nanosec = static_cast<uint32_t>(nav_ns % 1000000000LL);
      if (this->nominal_speed_ > 0.0 && status.remaining_distance > 0.0) {
        const double est_sec =
          static_cast<double>(status.remaining_distance) / this->nominal_speed_;
        feedback->estimated_time_remaining.sec = static_cast<int32_t>(est_sec);
        feedback->estimated_time_remaining.nanosec =
          static_cast<uint32_t>((est_sec - std::floor(est_sec)) * 1e9);
      }
      goal_handle->publish_feedback(feedback);
    });

  auto action_result = std::make_shared<NavigateToPose::Result>();
  action_result->error_msg = result.message;
  if (result.canceled) {
    action_result->error_code = NavigateToPose::Result::NONE;
    this->finalize_goal(goal_handle, action_result, "canceled", "execute_result_canceled");
  } else if (result.success) {
    action_result->error_code = NavigateToPose::Result::NONE;
    this->finalize_goal(goal_handle, action_result, "succeeded", "execute_result_succeeded");
  } else {
    action_result->error_code = result.error_code;
    this->finalize_goal(goal_handle, action_result, "aborted", "execute_result_failed");
  }

  this->clear_active_goal(goal_handle);
}

/// @copydoc Btnavigator::execute_goals
void Btnavigator::execute_goals(const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  uint32_t completed_goals = 0U;
  const auto goal_count = static_cast<uint32_t>(goal->goal_poses.size());

  if (goal->goal_poses.empty()) {
    auto result = std::make_shared<NavigateToPoses::Result>();
    result->error_code = NavigateToPoses::Result::UNKNOWN;
    result->error_msg = "Waypoint route is empty.";
    result->completed_goals = 0U;
    this->finalize_goal(goal_handle, result, "aborted", "empty_route");
    this->clear_active_goal(goal_handle);
    return;
  }

  for (std::size_t index = 0; index < goal->goal_poses.size(); ++index) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<NavigateToPoses::Result>();
      result->error_code = NavigateToPoses::Result::NONE;
      result->error_msg = "Waypoint route canceled.";
      result->completed_goals = completed_goals;
      this->finalize_goal(goal_handle, result, "canceled", "route_cancel_requested");
      this->clear_active_goal(goal_handle);
      return;
    }

    const auto route_goal_pose = make_route_goal_pose(goal->goal_poses, index);
    const bool align_heading_at_goal =
      this->align_heading_at_goal_ && (index + 1U) >= goal->goal_poses.size();
    const auto waypoint_result = this->execute_goal_pose(
      route_goal_pose,
      "navigate_to_poses",
      static_cast<uint32_t>(index),
      goal_count,
      align_heading_at_goal,
      [goal_handle]() { return goal_handle->is_canceling(); },
      [goal_handle, index, goal_count, this](
        const geometry_msgs::msg::PoseStamped &pose,
        const amr_msgs::msg::MotionStatus &status,
        int32_t recovery_count,
        const rclcpp::Duration &nav_time) {
        auto feedback = std::make_shared<NavigateToPoses::Feedback>();
        feedback->current_pose = pose;
        feedback->current_goal_index = static_cast<uint32_t>(index);
        feedback->goal_count = goal_count;
        feedback->distance_remaining = static_cast<float>(status.remaining_distance);
        feedback->number_of_recoveries = static_cast<int16_t>(recovery_count);
        const auto nav_ns = nav_time.nanoseconds();
        feedback->navigation_time.sec = static_cast<int32_t>(nav_ns / 1000000000LL);
        feedback->navigation_time.nanosec = static_cast<uint32_t>(nav_ns % 1000000000LL);
        if (this->nominal_speed_ > 0.0 && status.remaining_distance > 0.0) {
          const double est_sec =
            static_cast<double>(status.remaining_distance) / this->nominal_speed_;
          feedback->estimated_time_remaining.sec = static_cast<int32_t>(est_sec);
          feedback->estimated_time_remaining.nanosec =
            static_cast<uint32_t>((est_sec - std::floor(est_sec)) * 1e9);
        }
        goal_handle->publish_feedback(feedback);
      });

    if (waypoint_result.canceled) {
      auto result = std::make_shared<NavigateToPoses::Result>();
      result->error_code = NavigateToPoses::Result::NONE;
      result->error_msg = waypoint_result.message;
      result->completed_goals = completed_goals;
      this->finalize_goal(goal_handle, result, "canceled", "waypoint_canceled");
      this->clear_active_goal(goal_handle);
      return;
    }

    if (!waypoint_result.success) {
      auto result = std::make_shared<NavigateToPoses::Result>();
      result->error_code = waypoint_result.error_code;
      result->error_msg =
        "Waypoint " + std::to_string(index + 1) + " failed: " + waypoint_result.message;
      result->completed_goals = completed_goals;
      this->finalize_goal(goal_handle, result, "aborted", "waypoint_failed");
      this->clear_active_goal(goal_handle);
      return;
    }

    completed_goals += 1U;
    if (this->structured_logging_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=waypoint_completed route_id=navigate_to_poses current_goal_index=%zu goal_count=%u completed_goals=%u result=success reason=goal_reached",
        index,
        goal_count,
        completed_goals);
    }
  }

  auto result = std::make_shared<NavigateToPoses::Result>();
  result->error_code = NavigateToPoses::Result::NONE;
  result->error_msg =
    "Completed all " + std::to_string(completed_goals) + " waypoint goals.";
  result->completed_goals = completed_goals;
  this->finalize_goal(goal_handle, result, "succeeded", "route_completed");
  this->clear_active_goal(goal_handle);
}

/// @copydoc Btnavigator::execute_goal_pose
Btnavigator::ExecutionResult Btnavigator::execute_goal_pose(
  const geometry_msgs::msg::PoseStamped &goal_pose,
  const std::string &route_id,
  const uint32_t current_goal_index,
  const uint32_t goal_count,
  const bool align_heading_at_goal,
  const std::function<bool()> &is_cancel_requested,
  const std::function<void(
    const geometry_msgs::msg::PoseStamped &,
    const amr_msgs::msg::MotionStatus &,
    int32_t,
    const rclcpp::Duration &)> &publish_feedback)
{
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=goal_started route_id=%s goal_id=%u frame=%s target_x=%.3f target_y=%.3f align_heading=%s",
      route_id.c_str(),
      this->next_command_id_,
      goal_pose.header.frame_id.c_str(),
      goal_pose.pose.position.x,
      goal_pose.pose.position.y,
      bool_label(align_heading_at_goal));
  }

  BT::BehaviorTreeFactory factory;
  auto blackboard = BT::Blackboard::create();
  blackboard->set("navigator", this);
  blackboard->set("goal_pose", goal_pose);
  blackboard->set("route_id", route_id);
  blackboard->set("current_goal_index", current_goal_index);
  blackboard->set("goal_count", goal_count);
  blackboard->set("align_heading_at_goal", align_heading_at_goal);
  blackboard->set("is_cancel_requested", is_cancel_requested);
  blackboard->set("planned_path", nav_msgs::msg::Path());
  blackboard->set("active_command", amr_msgs::msg::MotionCommand());
  blackboard->set("active_command_dispatch_ns", static_cast<int64_t>(0));
  blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
  blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
  blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
  blackboard->set("local_escape_dispatched", false);
  blackboard->set("status_message", std::string("Behavior tree is running."));
  blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kRunning));
  blackboard->set("recovery_attempts", 0);

  factory.registerSimpleCondition(
    "CheckNavigatorReady",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      std::string error_message;
      if (navigator->is_navigator_ready(error_message)) {
        return BT::NodeStatus::SUCCESS;
      }
      blackboard->set("status_message", error_message);
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleCondition(
    "CheckCurrentPose",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto current_pose = navigator->get_current_pose_copy();
      if (current_pose.header.frame_id.empty()) {
        blackboard->set("status_message", std::string("Current pose is not available yet."));
        return BT::NodeStatus::FAILURE;
      }
      blackboard->set("current_pose", current_pose);
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "WaitForPlannerService",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto cancel_requested =
        blackboard->get<std::function<bool()>>("is_cancel_requested");
      std::string error_message;
      if (navigator->wait_for_planner_service(error_message, cancel_requested)) {
        return BT::NodeStatus::SUCCESS;
      }
      blackboard->set("status_message", error_message);
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleAction(
    "RequestGlobalPlan",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto cancel_requested =
        blackboard->get<std::function<bool()>>("is_cancel_requested");
      const auto current_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("current_pose");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      nav_msgs::msg::Path plan;
      std::string error_message;
      if (navigator->structured_logging_enabled_) {
        RCLCPP_INFO(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan start_x=%.3f start_y=%.3f goal_x=%.3f goal_y=%.3f",
          current_pose.pose.position.x,
          current_pose.pose.position.y,
          goal_pose.pose.position.x,
          goal_pose.pose.position.y);
      }
      if (!navigator->request_global_plan(
          current_pose, goal_pose, plan, error_message, cancel_requested))
      {
        RCLCPP_WARN(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan result=failed reason=%s",
          log_value(error_message).c_str());
        blackboard->set("status_message", error_message);
        return BT::NodeStatus::FAILURE;
      }
      if (navigator->structured_logging_enabled_) {
        RCLCPP_INFO(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan result=success path_points=%zu",
          plan.poses.size());
      }
      blackboard->set("planned_path", plan);
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "PublishMotionCommand",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      const auto route_id = blackboard->get<std::string>("route_id");
      const auto current_goal_index = blackboard->get<uint32_t>("current_goal_index");
      const auto goal_count = blackboard->get<uint32_t>("goal_count");
      const bool align_heading_at_goal = blackboard->get<bool>("align_heading_at_goal");
      const auto plan = blackboard->get<nav_msgs::msg::Path>("planned_path");
      auto command = navigator->build_motion_command(goal_pose, route_id, plan, align_heading_at_goal);
      navigator->publish_motion_command(command, current_goal_index, goal_count, "navigate");
      if (navigator->structured_logging_enabled_) {
        RCLCPP_INFO(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=publish_motion_command goal_id=%u route_id=%s current_goal_index=%u goal_count=%u path_points=%zu result=dispatched",
          command.command_id,
          route_id.c_str(),
          current_goal_index,
          goal_count,
          plan.poses.size());
      }
      blackboard->set("active_command", command);
      blackboard->set("active_command_dispatch_ns", navigator->now().nanoseconds());
      blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
      blackboard->set("local_escape_dispatched", false);
      blackboard->set("recovery_attempts", 0);
      blackboard->set("status_message", std::string("Motion command dispatched."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleCondition(
    "CheckCancelRequested",
    [blackboard](BT::TreeNode &) {
      const auto cancel_requested =
        blackboard->get<std::function<bool()>>("is_cancel_requested");
      return cancel_requested() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleCondition(
    "CheckGoalReached",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto status = navigator->get_motion_status_copy();
      const auto command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      navigator->latch_terminal_status_if_matches(status, command.command_id);
      if (navigator->has_latched_terminal_status(command.command_id)) {
        if (navigator->structured_logging_enabled_) {
          const auto current_goal_index = blackboard->get<uint32_t>("current_goal_index");
          const auto goal_count = blackboard->get<uint32_t>("goal_count");
          RCLCPP_INFO(
            navigator->get_logger(),
            "AMR_LOG schema=v1 component=bt_navigator event=terminal_latch_observed phase=check_goal_reached route_goal_id=%u active_command_id=%u motion_command_id=%u current_goal_index=%u goal_count=%u goal_reached=%s command_completed=%s result=success",
            command.command_id,
            command.command_id,
            status.command_id,
            current_goal_index,
            goal_count,
            bool_label(status.goal_reached),
            bool_label(status.command_completed));
        }
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleCondition(
    "CheckRecoveryNeeded",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto status = navigator->get_motion_status_copy();
      const auto local_plan_status = navigator->get_local_plan_status_copy();
      const auto command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      const auto dispatch_ns = blackboard->get<int64_t>("active_command_dispatch_ns");
      auto recovery_condition_since_ns = blackboard->get<int64_t>("recovery_condition_since_ns");
      const auto recovery_reacquire_until_ns =
        blackboard->get<int64_t>("recovery_reacquire_until_ns");
      const auto recovery_reacquire_command_id =
        blackboard->get<uint32_t>("recovery_reacquire_command_id");
      const auto settle_ns =
        static_cast<int64_t>(std::max(250, navigator->feedback_period_ms_ * 5)) * 1000000LL;
      const auto debounce_ns =
        static_cast<int64_t>(std::max(300, navigator->feedback_period_ms_ * 3)) * 1000000LL;
      const auto now_ns = navigator->now().nanoseconds();

      if (recovery_reacquire_until_ns > 0 && recovery_reacquire_command_id != 0U) {
        const bool reacquired =
          status.command_id == recovery_reacquire_command_id && 
          navigator->has_reacquired_navigation(command, status, local_plan_status);
        if (reacquired) {
          blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
          blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
          blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
          blackboard->set("local_escape_dispatched", false);
          if (navigator->structured_logging_enabled_) {
            RCLCPP_INFO(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_finished goal_id=%u result=reacquired recovery_skipped=true reason=navigation_reacquired planner_ok=%s controller_ok=%s local_plan_valid=%s planner_decision=%s",
              command.command_id,
              bool_label(!local_plan_status.recovery_required),
              bool_label(!status.blocked && !status.stalled),
              bool_label(status.local_plan_valid),
              local_plan_decision_label(local_plan_status.decision));
          }
          blackboard->set(
            "status_message",
            std::string("Recovery exit confirmed after a stable navigation reacquire."));
          return BT::NodeStatus::FAILURE;
        }

        if (now_ns < recovery_reacquire_until_ns) {
          blackboard->set(
            "status_message",
            std::string("Waiting for a stable post-recovery navigation reacquire."));
          return BT::NodeStatus::FAILURE;
        }

        if (navigator->structured_logging_enabled_) {
          RCLCPP_WARN(
            navigator->get_logger(),
            "AMR_LOG schema=v1 component=bt_navigator event=recovery_finished goal_id=%u result=reacquire_timeout recovery_skipped=false reason=navigation_reacquire_timeout planner_ok=%s controller_ok=%s local_plan_valid=%s planner_decision=%s",
            recovery_reacquire_command_id,
            bool_label(!local_plan_status.recovery_required),
            bool_label(!status.blocked && !status.stalled),
            bool_label(status.local_plan_valid),
            local_plan_decision_label(local_plan_status.decision));
        }
        blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
        blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
        blackboard->set("recovery_condition_since_ns", now_ns);
        blackboard->set(
          "status_message",
          std::string("Post-recovery reacquire window expired; blocked evaluation resumed."));
        return BT::NodeStatus::FAILURE;
      }

      if (status.command_id != command.command_id || !status.active) {
        blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
        return BT::NodeStatus::FAILURE;
      }

      if (dispatch_ns > 0 && (navigator->now().nanoseconds() - dispatch_ns) < settle_ns) {
        blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
        return BT::NodeStatus::FAILURE;
      }

      const bool planner_recovery_needed =
        local_plan_status.command_id == command.command_id && 
        local_plan_status.active && 
        local_plan_status.recovery_required;
      const bool recovery_needed = status.blocked || status.stalled || planner_recovery_needed;
      if (!recovery_needed) {
        blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
        return BT::NodeStatus::FAILURE;
      }

      if (recovery_condition_since_ns == 0) {
        blackboard->set("recovery_condition_since_ns", now_ns);
        return BT::NodeStatus::FAILURE;
      }

      if ((now_ns - recovery_condition_since_ns) < debounce_ns) {
        return BT::NodeStatus::FAILURE;
      }

      if (recovery_needed) {
        const std::string recovery_reason = planner_recovery_needed ?
          local_plan_decision_label(local_plan_status.decision) :
          (status.blocked ? std::string("controller_blocked") : std::string("controller_stalled"));
        if (navigator->structured_logging_enabled_ && navigator->recovery_decision_logging_enabled_) {
          RCLCPP_WARN_THROTTLE(
            navigator->get_logger(),
            *navigator->get_clock(),
            1000,
            "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=%s recovery_type=pending recovery_skipped=false planner_ok=%s controller_ok=%s blocked=%s safety_blocked=%s local_plan_valid=%s planner_decision=%s blocked_distance_m=%.3f",
            command.command_id,
            recovery_reason.c_str(),
            bool_label(!planner_recovery_needed),
            bool_label(!status.blocked && !status.stalled),
            bool_label(status.blocked || status.stalled || local_plan_status.recovery_required),
            bool_label(status.safety_gate_blocked),
            bool_label(status.local_plan_valid),
            local_plan_decision_label(local_plan_status.decision),
            local_plan_status.blocked_distance);
        }
        if (planner_recovery_needed) {
          switch (local_plan_status.decision) {
            case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
              blackboard->set(
                "status_message",
                std::string(
                  "Recovery requested because the planner reports a near-goal blocked approach."));
              break;
            case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
              blackboard->set(
                "status_message",
                std::string(
                  "Recovery requested because the planner recommends a global replan."));
              break;
            case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
              blackboard->set(
                "status_message",
                std::string(
                  "Recovery requested because the planner reports a hard blocked corridor."));
              break;
            default:
              blackboard->set(
                "status_message",
                std::string("Recovery requested because the local planner reported a blocked path."));
              break;
          }
        } else if (status.blocked) {
          blackboard->set(
            "status_message",
            std::string("Recovery requested because the motion controller reported a blocked path."));
        } else if (status.stalled) {
          blackboard->set(
            "status_message",
            std::string("Recovery requested because the motion controller reported stalled progress."));
        }
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleAction(
    "HandleRecoveryCycle",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto cancel_requested =
        blackboard->get<std::function<bool()>>("is_cancel_requested");
      const auto current_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("current_pose");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      const auto route_id = blackboard->get<std::string>("route_id");
      const auto current_goal_index = blackboard->get<uint32_t>("current_goal_index");
      const auto goal_count = blackboard->get<uint32_t>("goal_count");
      const bool align_heading_at_goal = blackboard->get<bool>("align_heading_at_goal");
      const auto active_command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      const auto planned_path = blackboard->get<nav_msgs::msg::Path>("planned_path");
      const bool local_escape_dispatched = blackboard->get<bool>("local_escape_dispatched");
      const auto local_plan_status = navigator->get_local_plan_status_copy();
      auto attempts = blackboard->get<int>("recovery_attempts");
      std::string error_message;
      amr_msgs::msg::MotionCommand recovery_command;
      const auto arm_recovery_reacquire =
        [&](const amr_msgs::msg::MotionCommand &dispatched_command) {
          const auto reacquire_until_ns =
            navigator->now().nanoseconds() +
            (static_cast<int64_t>(std::max(250, navigator->recovery_reacquire_settle_ms_)) * 1000000LL);
          blackboard->set("active_command", dispatched_command);
          blackboard->set("active_command_dispatch_ns", navigator->now().nanoseconds());
          blackboard->set("recovery_reacquire_until_ns", reacquire_until_ns);
          blackboard->set("recovery_reacquire_command_id", dispatched_command.command_id);
        };
      const auto finish_canceled = [&]() {
        navigator->publish_stop_command(current_goal_index, goal_count, "cancel_requested");
        blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
        blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
        blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
        blackboard->set("status_message", std::string("Navigation canceled."));
        blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kCanceled));
        return BT::NodeStatus::SUCCESS;
      };

      if (navigator->structured_logging_enabled_) {
        RCLCPP_WARN(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=recovery_started goal_id=%u attempt=%d reason=%s planner_decision=%s path_points=%zu",
          active_command.command_id,
          attempts + 1,
          local_plan_decision_label(local_plan_status.decision),
          local_plan_decision_label(local_plan_status.decision),
          planned_path.poses.size());
      }

      if (cancel_requested()) {
        return finish_canceled();
      }

      {
        const auto refreshed_motion_status = navigator->get_motion_status_copy();
        const auto refreshed_local_plan_status = navigator->get_local_plan_status_copy();
        if (navigator->has_reacquired_navigation(
            active_command,
            refreshed_motion_status,
            refreshed_local_plan_status))
        {
          blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
          blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
          blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
          blackboard->set("local_escape_dispatched", false);
          blackboard->set(
            "status_message",
            std::string("Recovery skipped because navigation had already reacquired tracking."));
          if (navigator->structured_logging_enabled_) {
            RCLCPP_INFO(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_skipped goal_id=%u skipped=true recovery_skipped=true reason=already_reacquired planner_ok=%s controller_ok=%s local_plan_valid=%s planner_decision=%s",
              active_command.command_id,
              bool_label(!refreshed_local_plan_status.recovery_required),
              bool_label(!refreshed_motion_status.blocked && !refreshed_motion_status.stalled),
              bool_label(refreshed_motion_status.local_plan_valid),
              local_plan_decision_label(refreshed_local_plan_status.decision));
          }
          return BT::NodeStatus::SUCCESS;
        }
      }

      navigator->publish_stop_command(current_goal_index, goal_count, "recovery_start_stop_active_command");

      if (!navigator->wait_for_recovery_services(error_message, cancel_requested)) {
        if (cancel_requested()) {
          return finish_canceled();
        }
        blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
        blackboard->set("status_message", error_message);
        return BT::NodeStatus::SUCCESS;
      }

      const int attempt_index = attempts % 3;
      const uint8_t planner_decision = local_plan_status.decision;
      const bool planner_owned_recovery =
        navigator->has_planner_owned_recovery(active_command, local_plan_status);
      if (!navigator->clear_local_costmap(error_message, cancel_requested)) {
        if (cancel_requested()) {
          return finish_canceled();
        }
        RCLCPP_WARN(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=clear_local_costmap result=failed reason=%s",
          log_value(error_message).c_str());
      }

      const bool local_escape_candidate =
        navigator->should_try_local_escape(
          active_command,
          local_plan_status,
          planned_path,
          local_escape_dispatched);
      if (local_escape_candidate) {
        nav_msgs::msg::Path escape_plan;
        if (navigator->request_local_escape_plan(
            current_pose, planned_path, escape_plan, error_message, cancel_requested))
        {
          auto command = navigator->build_motion_command(
            goal_pose, route_id, escape_plan, align_heading_at_goal);
          navigator->publish_motion_command(command, current_goal_index, goal_count, "local_escape");
          arm_recovery_reacquire(command);
          blackboard->set("local_escape_dispatched", true);
          blackboard->set("planned_path", escape_plan);
          if (navigator->structured_logging_enabled_) {
            RCLCPP_INFO(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=planner_hard_blocked recovery_type=local_escape recovery_skipped=false planner_ok=false controller_ok=true path_points=%zu result=dispatched",
              command.command_id,
              escape_plan.poses.size());
          }
          blackboard->set(
            "status_message",
            std::string("Local escape plan dispatched; waiting for a stable navigation reacquire."));
          return BT::NodeStatus::SUCCESS;
        }

        if (cancel_requested()) {
          return finish_canceled();
        }
        RCLCPP_WARN(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=local_escape_failed recovery_type=local_escape recovery_skipped=true planner_ok=false controller_ok=false result=failed detail=%s",
          active_command.command_id,
          log_value(error_message).c_str());
        blackboard->set(
          "status_message",
          std::string("Local escape was rejected; falling back to heavier recovery behaviors. ") +
          error_message);
      }

      if (
        planner_owned_recovery && 
        planner_decision == amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED)
      {
        nav_msgs::msg::Path replanned_path;
        if (!navigator->wait_for_planner_service(error_message, cancel_requested) ||
          !navigator->request_global_plan(
            current_pose, goal_pose, replanned_path, error_message, cancel_requested))
        {
          if (cancel_requested())
          {
            return finish_canceled();
          }
          blackboard->set("status_message", error_message);
        } else {
          attempts += 1;
          blackboard->set("recovery_attempts", attempts);
          if (attempts >= navigator->recovery_max_retries_) {
            navigator->publish_stop_command(current_goal_index, goal_count, "global_replan_retry_exceeded");
            blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
            blackboard->set(
              "status_message",
              std::string("Recovery retries exceeded before another global replan could be dispatched."));
            return BT::NodeStatus::SUCCESS;
          }
          auto command = navigator->build_motion_command(
            goal_pose, route_id, replanned_path, align_heading_at_goal);
          navigator->publish_motion_command(command, current_goal_index, goal_count, "global_replan");
          arm_recovery_reacquire(command);
          blackboard->set("planned_path", replanned_path);
          if (navigator->structured_logging_enabled_) {
            RCLCPP_INFO(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=global_replan_required recovery_type=global_replan recovery_skipped=false planner_ok=false controller_ok=true path_points=%zu result=dispatched",
              command.command_id,
              replanned_path.poses.size());
          }
          blackboard->set(
            "status_message",
            std::string("Planner requested global replanning; waiting for a stable navigation reacquire."));
          return BT::NodeStatus::SUCCESS;
        }
      }
      else
      {
        const std::string recovery_behavior =
          navigator->select_recovery_behavior(active_command, local_plan_status, attempt_index);
        if (recovery_behavior.empty())
        {
          blackboard->set(
            "status_message",
            std::string("Recovery policy selected direct replanning without a recovery command."));
        }
        else if (!navigator->request_recovery_command(
            recovery_behavior, current_pose, goal_pose, recovery_command, error_message,
            cancel_requested))
        {
          if (cancel_requested())
          {
            return finish_canceled();
          }
          blackboard->set("status_message", error_message);
        }
        else
        {
          if (navigator->structured_logging_enabled_) {
            RCLCPP_WARN(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_started goal_id=%u recovery_type=%s reason=%s attempt=%d",
              recovery_command.command_id,
              recovery_behavior.c_str(),
              local_plan_decision_label(local_plan_status.decision),
              attempts + 1);
          }
          navigator->publish_motion_command(recovery_command, current_goal_index, goal_count, recovery_behavior);
          if (!navigator->wait_for_command_completion(
              recovery_command.command_id,
              navigator->recovery_behavior_timeout_ms(recovery_behavior),
              error_message,
              cancel_requested))
          {
            if (cancel_requested())
            {
              return finish_canceled();
            }
            RCLCPP_WARN(
              navigator->get_logger(),
              "AMR_LOG schema=v1 component=bt_navigator event=recovery_finished goal_id=%u recovery_type=%s result=failed reason=command_timeout detail=%s",
              recovery_command.command_id,
              recovery_behavior.c_str(),
              log_value(error_message).c_str());
            blackboard->set("status_message", error_message);
          }
          else
          {
            if (navigator->structured_logging_enabled_) {
              RCLCPP_INFO(
                navigator->get_logger(),
                "AMR_LOG schema=v1 component=bt_navigator event=recovery_finished goal_id=%u recovery_type=%s result=completed reason=command_completed",
                recovery_command.command_id,
                recovery_behavior.c_str());
            }
            const bool redispatch_existing_plan =
              navigator->should_redispatch_existing_plan_after_recovery(
              active_command,
              local_plan_status,
              recovery_behavior);
            if (redispatch_existing_plan)
            {
              attempts += 1;
              blackboard->set("recovery_attempts", attempts);
              if (attempts >= navigator->recovery_max_retries_) {
                navigator->publish_stop_command(current_goal_index, goal_count, "redispatch_retry_exceeded");
                blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
                blackboard->set(
                  "status_message",
                  std::string("Recovery retries exceeded before the current plan could be re-dispatched."));
                return BT::NodeStatus::SUCCESS;
              }
              auto command = navigator->build_motion_command(
                goal_pose, route_id, planned_path, align_heading_at_goal);
              navigator->publish_motion_command(command, current_goal_index, goal_count, "redispatch_existing_plan");
              arm_recovery_reacquire(command);
              if (navigator->structured_logging_enabled_) {
                RCLCPP_INFO(
                  navigator->get_logger(),
                  "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=redispatch_existing_plan recovery_type=%s recovery_skipped=false planner_ok=false controller_ok=true path_points=%zu result=dispatched",
                  command.command_id,
                  recovery_behavior.c_str(),
                  planned_path.poses.size());
              }
              blackboard->set(
                "status_message",
                navigator->describe_recovery_policy(
                  active_command,
                  local_plan_status,
                  recovery_behavior,
                  true) + " Waiting for a stable navigation reacquire.");
              return BT::NodeStatus::SUCCESS;
            }

            blackboard->set(
              "status_message",
              navigator->describe_recovery_policy(
                active_command,
                local_plan_status,
                recovery_behavior,
                false));
          }
        }
      }

      attempts += 1;
      blackboard->set("recovery_attempts", attempts);

      if (attempts >= navigator->recovery_max_retries_) {
        navigator->publish_stop_command(current_goal_index, goal_count, "recovery_retry_exceeded");
        blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
        blackboard->set(
          "status_message",
          std::string("Recovery retries exceeded. Navigator aborted."));
        return BT::NodeStatus::SUCCESS;
      }

      nav_msgs::msg::Path replanned_path;
      if (!navigator->wait_for_planner_service(error_message, cancel_requested) ||
        !navigator->request_global_plan(
          current_pose, goal_pose, replanned_path, error_message, cancel_requested))
      {
        if (cancel_requested()) {
          return finish_canceled();
        }
        blackboard->set("status_message", error_message);
        std::this_thread::sleep_for(std::chrono::milliseconds(navigator->recovery_retry_delay_ms_));
        return BT::NodeStatus::SUCCESS;
      }

      auto command = navigator->build_motion_command(
        goal_pose, route_id, replanned_path, align_heading_at_goal);
      navigator->publish_motion_command(command, current_goal_index, goal_count, "fresh_global_replan");
      arm_recovery_reacquire(command);
      blackboard->set("planned_path", replanned_path);
      if (navigator->structured_logging_enabled_) {
        RCLCPP_INFO(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=%u reason=fresh_global_replan recovery_type=global_replan recovery_skipped=false planner_ok=%s controller_ok=%s path_points=%zu result=dispatched",
          command.command_id,
          bool_label(!planner_owned_recovery),
          bool_label(planner_owned_recovery),
          replanned_path.poses.size());
      }
      blackboard->set(
        "status_message",
        planner_owned_recovery ?
        std::string("Planner-owned recovery policy completed; a fresh global plan was dispatched and is reacquiring.") :
        std::string("Controller-owned recovery policy completed; a fresh global plan was dispatched and is reacquiring."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "PublishStopCommand",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      const auto current_goal_index = blackboard->get<uint32_t>("current_goal_index");
      const auto goal_count = blackboard->get<uint32_t>("goal_count");
      const auto motion_status = navigator->get_motion_status_copy();
      const auto local_plan_status = navigator->get_local_plan_status_copy();
      if (navigator->structured_logging_enabled_) {
        RCLCPP_INFO(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=stale_command_clear phase=publish_stop_command route_goal_id=%u active_command_id=%u motion_command_id=%u local_plan_command_id=%u current_goal_index=%u goal_count=%u reason=bt_terminal_or_cancel",
          command.command_id,
          command.command_id,
          motion_status.command_id,
          local_plan_status.command_id,
          current_goal_index,
          goal_count);
      }
      navigator->publish_stop_command(current_goal_index, goal_count, "bt_terminal_or_cancel");
      blackboard->set("status_message", std::string("Stop command dispatched."));
      if (navigator->structured_logging_enabled_) {
        RCLCPP_WARN(
          navigator->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=publish_stop_command result=dispatched");
      }
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "MarkCanceled",
    [blackboard](BT::TreeNode &) {
      blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kCanceled));
      blackboard->set("status_message", std::string("Navigation canceled."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "MarkSucceeded",
    [blackboard](BT::TreeNode &) {
      blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kSucceeded));
      blackboard->set("status_message", std::string("Goal reached."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "WaitFeedbackPeriod",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      std::this_thread::sleep_for(std::chrono::milliseconds(navigator->feedback_period_ms_));
      return BT::NodeStatus::SUCCESS;
    });

  BT::Tree plan_tree;
  BT::Tree monitor_tree;
  try {
    factory.registerBehaviorTreeFromFile(this->behavior_tree_xml_path_);
    plan_tree = factory.createTree("PlanAndDispatch", blackboard);
    monitor_tree = factory.createTree("MonitorExecution", blackboard);
  } catch (const std::exception &error) {
    return ExecutionResult{
      false,
      false,
      std::string("Failed to initialize behavior tree: ") + error.what(),
      9001U};  // FAILED_TO_LOAD_BEHAVIOR_TREE
  }

  BT::NodeStatus plan_status = BT::NodeStatus::IDLE;
  try {
    plan_status = plan_tree.tickRoot();
  } catch (const BT::RuntimeError &error) {
    this->log_exception("plan_tree.tickRoot", "BT::RuntimeError", error, false);
    return ExecutionResult{false, false, std::string("Behavior tree plan tick failed: ") + error.what(), 9000U};
  } catch (const std::future_error &error) {
    this->log_exception("plan_tree.tickRoot", "std::future_error", error, false);
    return ExecutionResult{false, false, std::string("Behavior tree plan future failed: ") + error.what(), 9000U};
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("plan_tree.tickRoot", "rclcpp::exceptions::RCLError", error, false);
    return ExecutionResult{false, false, std::string("Behavior tree plan ROS client failed: ") + error.what(), 9000U};
  } catch (const std::exception &error) {
    this->log_exception("plan_tree.tickRoot", "std::exception", error, false);
    return ExecutionResult{false, false, std::string("Behavior tree plan tick failed: ") + error.what(), 9000U};
  } catch (...) {
    this->log_unknown_exception("plan_tree.tickRoot", false);
    return ExecutionResult{false, false, "Behavior tree plan tick failed with an unknown exception.", 9000U};
  }

  if (plan_status != BT::NodeStatus::SUCCESS) {
    if (is_cancel_requested()) {
      const auto message = blackboard->get<std::string>("status_message");
      return ExecutionResult{
        false,
        true,
        message.empty() ? std::string("Navigation canceled.") : message,
        0U};
    }
    return ExecutionResult{
      false,
      false,
      blackboard->get<std::string>("status_message")};
  }

  const auto goal_start = this->now();

  while (rclcpp::ok()) {
    const auto status = this->get_motion_status_copy();
    const auto pose = this->get_current_pose_copy();

    try {
      if (publish_feedback) {
        const auto recovery_count = blackboard->get<int>("recovery_attempts");
        publish_feedback(pose, status, recovery_count, this->now() - goal_start);
      }
    } catch (const std::exception &error) {
      this->log_exception("publish_feedback", "std::exception", error, false);
      return ExecutionResult{false, false, std::string("Failed to publish action feedback: ") + error.what(), 9000U};
    } catch (...) {
      this->log_unknown_exception("publish_feedback", false);
      return ExecutionResult{false, false, "Failed to publish action feedback with an unknown exception.", 9000U};
    }

    BT::NodeStatus monitor_status = BT::NodeStatus::IDLE;
    BtOutcome outcome = BtOutcome::kRunning;
    std::string message;
    try {
      blackboard->set("current_pose", pose);
      monitor_status = monitor_tree.tickRoot();
      outcome = static_cast<BtOutcome>(blackboard->get<int>("bt_outcome"));
      message = blackboard->get<std::string>("status_message");
    } catch (const BT::RuntimeError &error) {
      this->log_exception("monitor_tree.tickRoot", "BT::RuntimeError", error, false);
      return ExecutionResult{false, false, std::string("Behavior tree monitor tick failed: ") + error.what(), 9000U};
    } catch (const std::future_error &error) {
      this->log_exception("monitor_tree.tickRoot", "std::future_error", error, false);
      return ExecutionResult{false, false, std::string("Behavior tree monitor future failed: ") + error.what(), 9000U};
    } catch (const rclcpp::exceptions::RCLError &error) {
      this->log_exception("monitor_tree.tickRoot", "rclcpp::exceptions::RCLError", error, false);
      return ExecutionResult{false, false, std::string("Behavior tree monitor ROS client failed: ") + error.what(), 9000U};
    } catch (const std::exception &error) {
      this->log_exception("monitor_tree.tickRoot", "std::exception", error, false);
      return ExecutionResult{false, false, std::string("Behavior tree monitor tick failed: ") + error.what(), 9000U};
    } catch (...) {
      this->log_unknown_exception("monitor_tree.tickRoot", false);
      return ExecutionResult{false, false, "Behavior tree monitor tick failed with an unknown exception.", 9000U};
    }

    if (this->structured_logging_enabled_) {
      const auto active_command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      const auto local_plan_status = this->get_local_plan_status_copy();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "AMR_LOG schema=v1 component=bt_navigator event=bt_monitor_tick route_id=%s route_goal_id=%u active_command_id=%u motion_command_id=%u local_plan_command_id=%u current_goal_index=%u goal_count=%u monitor_status=%d bt_outcome=%d motion_goal_reached=%s motion_command_completed=%s latched_terminal=%s recovery_count=%d status_message=%s",
        route_id.c_str(),
        active_command.command_id,
        active_command.command_id,
        status.command_id,
        local_plan_status.command_id,
        current_goal_index,
        goal_count,
        static_cast<int>(monitor_status),
        static_cast<int>(outcome),
        bool_label(status.goal_reached),
        bool_label(status.command_completed),
        bool_label(this->has_latched_terminal_status(active_command.command_id)),
        blackboard->get<int>("recovery_attempts"),
        log_value(message).c_str());
    }
    if (outcome == BtOutcome::kSucceeded) {
      if (this->structured_logging_enabled_) {
        RCLCPP_INFO(
          this->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=goal_succeeded route_id=%s result=success reason=goal_reached recovery_count=%d duration_sec=%.3f",
          route_id.c_str(),
          blackboard->get<int>("recovery_attempts"),
          (this->now() - goal_start).seconds());
      }
      return ExecutionResult{true, false, message, 0U};
    }
    if (outcome == BtOutcome::kCanceled) {
      if (this->structured_logging_enabled_) {
        RCLCPP_INFO(
          this->get_logger(),
          "AMR_LOG schema=v1 component=bt_navigator event=goal_canceled route_id=%s result=canceled reason=cancel_requested recovery_count=%d duration_sec=%.3f",
          route_id.c_str(),
          blackboard->get<int>("recovery_attempts"),
          (this->now() - goal_start).seconds());
      }
      return ExecutionResult{false, true, message, 0U};
    }
    if (outcome == BtOutcome::kStopped) {
      RCLCPP_ERROR(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=goal_failed route_id=%s result=failed reason=%s recovery_count=%d duration_sec=%.3f",
        route_id.c_str(),
        log_value(message).c_str(),
        blackboard->get<int>("recovery_attempts"),
        (this->now() - goal_start).seconds());
      return ExecutionResult{false, false, message, 9000U};
    }
  }

  return ExecutionResult{
    false,
    false,
    "Navigator stopped because ROS is shutting down.",
    9000U};
}

/// @copydoc Btnavigator::handle_motion_status
void Btnavigator::handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_motion_status_ = *message;
  this->has_motion_status_ = true;
  if (message->command_id != 0U && (message->goal_reached || message->command_completed)) {
    this->last_terminal_command_id_ = message->command_id;
    this->last_terminal_status_time_ = this->now();
    this->latched_goal_reached_ = message->goal_reached;
    this->latched_command_completed_ = message->command_completed;
  }
}

/// @copydoc Btnavigator::handle_local_plan_status
void Btnavigator::handle_local_plan_status(
  const amr_msgs::msg::LocalPlanStatus::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_local_plan_status_ = *message;
  this->has_local_plan_status_ = true;
}

/// @copydoc Btnavigator::update_current_pose_from_tf
bool Btnavigator::update_current_pose_from_tf()
{
  if (!this->tf_buffer_) {
    std::scoped_lock lock(this->navigator_mutex_);
    this->has_current_pose_ = false;
    return false;
  }

  try {
    const auto transform = this->tf_buffer_->lookupTransform(
      this->map_frame_,
      this->base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(std::max(0.0, this->tf_lookup_timeout_sec_)));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    std::scoped_lock lock(this->navigator_mutex_);
    this->current_pose_ = pose;
    this->has_current_pose_ = true;
    return true;
  } catch (const tf2::TransformException &error) {
    std::scoped_lock lock(this->navigator_mutex_);
    this->has_current_pose_ = false;
    this->current_pose_ = geometry_msgs::msg::PoseStamped();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for TF %s -> %s before navigation planning: %s",
      this->map_frame_.c_str(),
      this->base_frame_.c_str(),
      error.what());
    return false;
  }
}

/// @copydoc Btnavigator::get_current_pose_copy
geometry_msgs::msg::PoseStamped Btnavigator::get_current_pose_copy()
{
  this->update_current_pose_from_tf();
  std::scoped_lock lock(this->navigator_mutex_);
  return this->current_pose_;
}

/// @copydoc Btnavigator::get_motion_status_copy
amr_msgs::msg::MotionStatus Btnavigator::get_motion_status_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_motion_status_;
}

/// @copydoc Btnavigator::get_local_plan_status_copy
amr_msgs::msg::LocalPlanStatus Btnavigator::get_local_plan_status_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_local_plan_status_;
}

/// @copydoc Btnavigator::is_navigator_ready
bool Btnavigator::is_navigator_ready(std::string &error_message) const
{
  if (
    !this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated() ||
    !this->plan_segment_client_ || !this->plan_recovery_client_ || !this->plan_local_escape_client_ ||
    !this->clear_costmap_client_)
  {
    error_message = "Navigator is not active.";
    return false;
  }
  error_message.clear();
  return true;
}

/// @copydoc Btnavigator::wait_for_planner_service
bool Btnavigator::wait_for_planner_service(
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  if (!this->plan_segment_client_) {
    error_message = "Global planner client is not configured.";
    return false;
  }

  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    const int wait_ms = std::min(100, this->planner_wait_timeout_ms_ - elapsed_ms);
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (this->plan_segment_client_->wait_for_service(std::chrono::milliseconds(wait_ms))) {
      error_message.clear();
      return true;
    }
    elapsed_ms += wait_ms;
  }

  error_message = "Global planner service is not available.";
  return false;
}

/// @copydoc Btnavigator::wait_for_recovery_services
bool Btnavigator::wait_for_recovery_services(
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  if (!this->plan_recovery_client_ || !this->plan_local_escape_client_ || !this->clear_costmap_client_) {
    error_message = "Recovery clients are not configured.";
    return false;
  }

  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    const int wait_ms = std::min(100, this->planner_wait_timeout_ms_ - elapsed_ms);
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (this->plan_recovery_client_->wait_for_service(std::chrono::milliseconds(wait_ms))) {
      break;
    }
    elapsed_ms += wait_ms;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_) {
    error_message = "Recovery planner service is not available.";
    return false;
  }

  elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    const int wait_ms = std::min(100, this->planner_wait_timeout_ms_ - elapsed_ms);
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (this->plan_local_escape_client_->wait_for_service(std::chrono::milliseconds(wait_ms))) {
      break;
    }
    elapsed_ms += wait_ms;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_) {
    error_message = "Local escape planner service is not available.";
    return false;
  }

  elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    const int wait_ms = std::min(100, this->planner_wait_timeout_ms_ - elapsed_ms);
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (this->clear_costmap_client_->wait_for_service(std::chrono::milliseconds(wait_ms))) {
      error_message.clear();
      return true;
    }
    elapsed_ms += wait_ms;
  }

  error_message = "Clear costmap service is not available.";
  return false;
}

/// @copydoc Btnavigator::request_global_plan
bool Btnavigator::request_global_plan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav_msgs::msg::Path &plan,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto request = std::make_shared<amr_msgs::srv::PlanSegment::Request>();
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  request->start = start;
  request->goal = goal;

  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan start_x=%.3f start_y=%.3f goal_x=%.3f goal_y=%.3f",
      request->start.pose.position.x,
      request->start.pose.position.y,
      request->goal.pose.position.x,
      request->goal.pose.position.y);
  }

  auto future = this->plan_segment_client_->async_send_request(request);
  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    elapsed_ms += 50;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_ && 
    future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    error_message = "Timed out while waiting for a global plan.";
    if (this->structured_logging_enabled_) {
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan result=failed reason=%s",
        log_value(error_message).c_str());
    }
    return false;
  }

  std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response;
  try {
    response = future.get();
  } catch (const std::future_error &error) {
    this->log_exception("request_global_plan.future.get", "std::future_error", error, false);
    error_message = std::string("Global planner future failed: ") + error.what();
    return false;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("request_global_plan.future.get", "rclcpp::exceptions::RCLError", error, false);
    error_message = std::string("Global planner ROS client failed: ") + error.what();
    return false;
  } catch (const std::exception &error) {
    this->log_exception("request_global_plan.future.get", "std::exception", error, false);
    error_message = std::string("Global planner request failed: ") + error.what();
    return false;
  } catch (...) {
    this->log_unknown_exception("request_global_plan.future.get", false);
    error_message = "Global planner request failed with an unknown exception.";
    return false;
  }
  if (!response->success) {
    error_message = response->message;
    if (this->structured_logging_enabled_) {
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan result=failed reason=%s",
        log_value(error_message).c_str());
    }
    return false;
  }

  plan = response->plan;
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=request_global_plan result=success path_points=%zu",
      plan.poses.size());
  }
  error_message.clear();
  return true;
}

/// @copydoc Btnavigator::request_recovery_command
bool Btnavigator::request_recovery_command(
  const std::string &behavior,
  const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &goal_pose,
  amr_msgs::msg::MotionCommand &command,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto request = std::make_shared<amr_msgs::srv::PlanRecovery::Request>();
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  request->behavior = behavior;
  request->current_pose = current_pose;
  request->goal_pose = goal_pose;

  auto future = this->plan_recovery_client_->async_send_request(request);
  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    elapsed_ms += 50;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_ && 
    future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    error_message = "Timed out while waiting for a recovery command.";
    return false;
  }

  std::shared_ptr<amr_msgs::srv::PlanRecovery::Response> response;
  try {
    response = future.get();
  } catch (const std::future_error &error) {
    this->log_exception("request_recovery_command.future.get", "std::future_error", error, false);
    error_message = std::string("Recovery planner future failed: ") + error.what();
    return false;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("request_recovery_command.future.get", "rclcpp::exceptions::RCLError", error, false);
    error_message = std::string("Recovery planner ROS client failed: ") + error.what();
    return false;
  } catch (const std::exception &error) {
    this->log_exception("request_recovery_command.future.get", "std::exception", error, false);
    error_message = std::string("Recovery planner request failed: ") + error.what();
    return false;
  } catch (...) {
    this->log_unknown_exception("request_recovery_command.future.get", false);
    error_message = "Recovery planner request failed with an unknown exception.";
    return false;
  }
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  command = response->command;
  command.header.stamp = this->now();
  command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  command.command_id = this->next_command_id_++;
  error_message.clear();
  return true;
}

/// @copydoc Btnavigator::request_local_escape_plan
bool Btnavigator::request_local_escape_plan(
  const geometry_msgs::msg::PoseStamped &current_pose,
  const nav_msgs::msg::Path &source_plan,
  nav_msgs::msg::Path &escape_plan,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto request = std::make_shared<amr_msgs::srv::PlanLocalEscape::Request>();
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  request->current_pose = current_pose;
  request->source_plan = source_plan;

  auto future = this->plan_local_escape_client_->async_send_request(request);
  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    elapsed_ms += 50;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_ && 
    future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    error_message = "Timed out while waiting for a local escape plan.";
    return false;
  }

  std::shared_ptr<amr_msgs::srv::PlanLocalEscape::Response> response;
  try {
    response = future.get();
  } catch (const std::future_error &error) {
    this->log_exception("request_local_escape_plan.future.get", "std::future_error", error, false);
    error_message = std::string("Local escape planner future failed: ") + error.what();
    return false;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("request_local_escape_plan.future.get", "rclcpp::exceptions::RCLError", error, false);
    error_message = std::string("Local escape planner ROS client failed: ") + error.what();
    return false;
  } catch (const std::exception &error) {
    this->log_exception("request_local_escape_plan.future.get", "std::exception", error, false);
    error_message = std::string("Local escape planner request failed: ") + error.what();
    return false;
  } catch (...) {
    this->log_unknown_exception("request_local_escape_plan.future.get", false);
    error_message = "Local escape planner request failed with an unknown exception.";
    return false;
  }
  if (!response->success) {
    error_message = this->describe_local_escape_failure(response->message);
    return false;
  }

  escape_plan = response->plan;
  error_message.clear();
  return true;
}

/// @copydoc Btnavigator::should_try_local_escape
bool Btnavigator::should_try_local_escape(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status,
  const nav_msgs::msg::Path &planned_path,
  bool local_escape_dispatched) const
{
  if (local_escape_dispatched || planned_path.poses.empty()) {
    return false;
  }

  if (!this->has_planner_owned_recovery(active_command, local_plan_status)) {
    return false;
  }

  switch (local_plan_status.decision) {
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return true;
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
      return false;
    default:
      return true;
  }
}

/// @copydoc Btnavigator::has_reacquired_navigation
bool Btnavigator::has_reacquired_navigation(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::MotionStatus &motion_status,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status) const
{
  if (
    motion_status.command_id != active_command.command_id ||
    !motion_status.active ||
    motion_status.mode != amr_msgs::msg::MotionCommand::MODE_NAVIGATE ||
    motion_status.blocked ||
    motion_status.stalled ||
    !motion_status.local_plan_valid)
  {
    return false;
  }

  return
    local_plan_status.command_id == active_command.command_id &&
    local_plan_status.active &&
    local_plan_status.local_plan_valid && 
    !local_plan_status.recovery_required;
}

/// @copydoc Btnavigator::has_planner_owned_recovery
bool Btnavigator::has_planner_owned_recovery(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status) const
{
  return
    local_plan_status.command_id == active_command.command_id && 
    local_plan_status.active && 
    local_plan_status.recovery_required;
}

/// @copydoc Btnavigator::select_recovery_behavior
std::string Btnavigator::select_recovery_behavior(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status,
  int attempt_index) const
{
  if (!this->has_planner_owned_recovery(active_command, local_plan_status)) {
    switch (attempt_index) {
      case 0:
        return "wait";
      case 1:
        return "backup";
      default:
        return "spin";
    }
  }

  switch (local_plan_status.decision) {
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
      return attempt_index < 2 ? "wait" : "backup";
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return attempt_index == 0 ? "backup" : "spin";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
      return "";
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
    default:
      switch (attempt_index) {
        case 0:
          return "wait";
        case 1:
          return "backup";
        default:
          return "spin";
      }
  }
}

/// @copydoc Btnavigator::should_redispatch_existing_plan_after_recovery
bool Btnavigator::should_redispatch_existing_plan_after_recovery(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status,
  const std::string &executed_behavior) const
{
  if (!this->has_planner_owned_recovery(active_command, local_plan_status)) {
    return false;
  }

  return
    local_plan_status.decision == amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED && 
    executed_behavior == "wait";
}

/// @copydoc Btnavigator::recovery_behavior_timeout_ms
int Btnavigator::recovery_behavior_timeout_ms(const std::string &behavior) const
{
  if (behavior == "wait") {
    return std::max(this->feedback_period_ms_ * 10, this->recovery_retry_delay_ms_);
  }
  if (behavior == "backup") {
    return std::max(2000, this->recovery_retry_delay_ms_ + 1000);
  }
  if (behavior == "spin") {
    return std::max(2500, this->recovery_retry_delay_ms_ + 1200);
  }
  return std::max(this->feedback_period_ms_ * 10, this->recovery_retry_delay_ms_);
}

/// @copydoc Btnavigator::describe_local_escape_failure
std::string Btnavigator::describe_local_escape_failure(const std::string &planner_message) const
{
  if (planner_message.empty()) {
    return "Local escape planner failed without a reason label.";
  }

  return "Local escape planner failed: " + planner_message;
}

/// @copydoc Btnavigator::describe_recovery_policy
std::string Btnavigator::describe_recovery_policy(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status,
  const std::string &behavior,
  bool redispatch_existing_plan) const
{
  if (redispatch_existing_plan) {
    return "Near-goal blocked policy: waited briefly, then re-dispatched the current plan.";
  }

  if (!this->has_planner_owned_recovery(active_command, local_plan_status)) {
    return "Controller-owned blocked policy executed " + behavior + " before a fresh global replan.";
  }

  switch (local_plan_status.decision) {
    case amr_msgs::msg::LocalPlanStatus::DECISION_GOAL_PROXIMITY_BLOCKED:
      return "Near-goal blocked policy executed " + behavior + " before continuing recovery.";
    case amr_msgs::msg::LocalPlanStatus::DECISION_HARD_BLOCKED:
      return "Hard-blocked corridor policy executed " + behavior + " before a fresh global replan.";
    case amr_msgs::msg::LocalPlanStatus::DECISION_GLOBAL_REPLAN_REQUIRED:
      return "Planner requested a direct global replan without an intermediate recovery command.";
    case amr_msgs::msg::LocalPlanStatus::DECISION_OK:
    default:
      return "Fallback recovery policy executed " + behavior + " before a fresh global replan.";
  }
}

/// @copydoc Btnavigator::clear_local_costmap
bool Btnavigator::clear_local_costmap(
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto request = std::make_shared<amr_msgs::srv::ClearCostmap::Request>();
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };

  request->local_only = true;

  auto future = this->clear_costmap_client_->async_send_request(request);
  int elapsed_ms = 0;
  while (elapsed_ms < this->planner_wait_timeout_ms_) {
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    elapsed_ms += 50;
  }
  if (elapsed_ms >= this->planner_wait_timeout_ms_ && 
    future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    error_message = "Timed out while clearing the local costmap.";
    return false;
  }

  std::shared_ptr<amr_msgs::srv::ClearCostmap::Response> response;
  try {
    response = future.get();
  } catch (const std::future_error &error) {
    this->log_exception("clear_local_costmap.future.get", "std::future_error", error, false);
    error_message = std::string("Clear costmap future failed: ") + error.what();
    return false;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("clear_local_costmap.future.get", "rclcpp::exceptions::RCLError", error, false);
    error_message = std::string("Clear costmap ROS client failed: ") + error.what();
    return false;
  } catch (const std::exception &error) {
    this->log_exception("clear_local_costmap.future.get", "std::exception", error, false);
    error_message = std::string("Clear costmap request failed: ") + error.what();
    return false;
  } catch (...) {
    this->log_unknown_exception("clear_local_costmap.future.get", false);
    error_message = "Clear costmap request failed with an unknown exception.";
    return false;
  }
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  error_message.clear();
  return true;
}

/// @copydoc Btnavigator::wait_for_command_completion
bool Btnavigator::wait_for_command_completion(
  const uint32_t command_id,
  const int timeout_ms,
  std::string &error_message,
  const std::function<bool()> &is_cancel_requested)
{
  auto cancel_requested = [&is_cancel_requested]() {
    return static_cast<bool>(is_cancel_requested) && is_cancel_requested();
  };
  const auto start_time = this->now();
  while (rclcpp::ok()) {
    if (cancel_requested()) {
      error_message = "Navigation canceled.";
      return false;
    }
    const auto status = this->get_motion_status_copy();
    if (status.command_id == command_id && status.command_completed) {
      error_message.clear();
      return true;
    }
    if ((this->now() - start_time).nanoseconds() / 1000000LL >= timeout_ms) {
      error_message = "Timed out while waiting for recovery motion completion.";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  error_message = "ROS shutdown while waiting for recovery motion completion.";
  return false;
}

/// @copydoc Btnavigator::has_active_goal
bool Btnavigator::has_active_goal() const
{
  std::scoped_lock active_goal_lock(this->active_goal_mutex_);
  return !this->active_goal_handle_.expired() || !this->active_goals_handle_.expired();
}

/// @copydoc Btnavigator::clear_active_goal
void Btnavigator::clear_active_goal(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::scoped_lock active_goal_lock(this->active_goal_mutex_);
  const auto active_goal = this->active_goal_handle_.lock();
  if (active_goal == goal_handle) {
    this->active_goal_handle_.reset();
  }
}

/// @copydoc Btnavigator::clear_active_goal
void Btnavigator::clear_active_goal(const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  std::scoped_lock active_goal_lock(this->active_goal_mutex_);
  const auto active_goal = this->active_goals_handle_.lock();
  if (active_goal == goal_handle) {
    this->active_goals_handle_.reset();
  }
}

/// @copydoc Btnavigator::finalize_goal
bool Btnavigator::finalize_goal(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
  const std::shared_ptr<NavigateToPose::Result> result,
  const std::string &requested_status,
  const std::string &reason)
{
  const std::string goal_key = this->goal_uuid_key(goal_handle->get_goal_id());
  const bool active = goal_handle->is_active();
  const bool canceling = goal_handle->is_canceling();
  {
    std::scoped_lock result_lock(this->result_mutex_);
    if (this->finalized_goal_ids_.find(goal_key) != this->finalized_goal_ids_.end()) {
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=action_result_duplicate route_id=navigate_to_pose goal_uuid=%s requested_status=%s reason=%s result=ignored",
        goal_key.c_str(),
        requested_status.c_str(),
        log_value(reason).c_str());
      return false;
    }
    if (!active && !canceling) {
      this->finalized_goal_ids_.insert(goal_key);
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=action_result_inactive route_id=navigate_to_pose goal_uuid=%s requested_status=%s reason=%s is_active=false is_canceling=false result=ignored",
        goal_key.c_str(),
        requested_status.c_str(),
        log_value(reason).c_str());
      return false;
    }
    this->finalized_goal_ids_.insert(goal_key);
  }

  const std::string final_status = canceling ? std::string("canceled") : requested_status;
  try {
    if (final_status == "succeeded") {
      goal_handle->succeed(result);
    } else if (final_status == "canceled") {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=action_result_finalized route_id=navigate_to_pose goal_uuid=%s requested_status=%s final_status=%s reason=%s is_active=%s is_canceling=%s result=sent",
      goal_key.c_str(),
      requested_status.c_str(),
      final_status.c_str(),
      log_value(reason).c_str(),
      bool_label(active),
      bool_label(canceling));
    return true;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("finalize_goal.navigate_to_pose", "rclcpp::exceptions::RCLError", error, true);
  } catch (const std::exception &error) {
    this->log_exception("finalize_goal.navigate_to_pose", "std::exception", error, true);
  } catch (...) {
    this->log_unknown_exception("finalize_goal.navigate_to_pose", true);
  }
  return false;
}

/// @copydoc Btnavigator::finalize_goal
bool Btnavigator::finalize_goal(
  const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle,
  const std::shared_ptr<NavigateToPoses::Result> result,
  const std::string &requested_status,
  const std::string &reason)
{
  const std::string goal_key = this->goal_uuid_key(goal_handle->get_goal_id());
  const bool active = goal_handle->is_active();
  const bool canceling = goal_handle->is_canceling();
  {
    std::scoped_lock result_lock(this->result_mutex_);
    if (this->finalized_goal_ids_.find(goal_key) != this->finalized_goal_ids_.end()) {
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=action_result_duplicate route_id=navigate_to_poses goal_uuid=%s requested_status=%s reason=%s completed_goals=%u result=ignored",
        goal_key.c_str(),
        requested_status.c_str(),
        log_value(reason).c_str(),
        result->completed_goals);
      return false;
    }
    if (!active && !canceling) {
      this->finalized_goal_ids_.insert(goal_key);
      RCLCPP_WARN(
        this->get_logger(),
        "AMR_LOG schema=v1 component=bt_navigator event=action_result_inactive route_id=navigate_to_poses goal_uuid=%s requested_status=%s reason=%s completed_goals=%u is_active=false is_canceling=false result=ignored",
        goal_key.c_str(),
        requested_status.c_str(),
        log_value(reason).c_str(),
        result->completed_goals);
      return false;
    }
    this->finalized_goal_ids_.insert(goal_key);
  }

  const std::string final_status = canceling ? std::string("canceled") : requested_status;
  try {
    if (final_status == "succeeded") {
      goal_handle->succeed(result);
    } else if (final_status == "canceled") {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=action_result_finalized route_id=navigate_to_poses goal_uuid=%s requested_status=%s final_status=%s reason=%s completed_goals=%u is_active=%s is_canceling=%s result=sent",
      goal_key.c_str(),
      requested_status.c_str(),
      final_status.c_str(),
      log_value(reason).c_str(),
      result->completed_goals,
      bool_label(active),
      bool_label(canceling));
    return true;
  } catch (const rclcpp::exceptions::RCLError &error) {
    this->log_exception("finalize_goal.navigate_to_poses", "rclcpp::exceptions::RCLError", error, true);
  } catch (const std::exception &error) {
    this->log_exception("finalize_goal.navigate_to_poses", "std::exception", error, true);
  } catch (...) {
    this->log_unknown_exception("finalize_goal.navigate_to_poses", true);
  }
  return false;
}

/// @copydoc Btnavigator::reset_terminal_latch
void Btnavigator::reset_terminal_latch(const uint32_t next_command_id)
{
  std::scoped_lock lock(this->navigator_mutex_);
  if (this->latched_goal_reached_ || this->latched_command_completed_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=terminal_latch_reset previous_command_id=%u next_command_id=%u goal_reached=%s command_completed=%s result=cleared",
      this->last_terminal_command_id_,
      next_command_id,
      bool_label(this->latched_goal_reached_),
      bool_label(this->latched_command_completed_));
  }
  this->last_terminal_command_id_ = 0U;
  this->last_terminal_status_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->latched_goal_reached_ = false;
  this->latched_command_completed_ = false;
}

/// @copydoc Btnavigator::latch_terminal_status_if_matches
void Btnavigator::latch_terminal_status_if_matches(
  const amr_msgs::msg::MotionStatus &status,
  const uint32_t active_command_id)
{
  if (
    active_command_id == 0U || status.command_id != active_command_id ||
    (!status.goal_reached && !status.command_completed))
  {
    return;
  }
  std::scoped_lock lock(this->navigator_mutex_);
  this->last_terminal_command_id_ = active_command_id;
  this->last_terminal_status_time_ = this->now();
  this->latched_goal_reached_ = this->latched_goal_reached_ || status.goal_reached;
  this->latched_command_completed_ = this->latched_command_completed_ || status.command_completed;
}

/// @copydoc Btnavigator::has_latched_terminal_status
bool Btnavigator::has_latched_terminal_status(const uint32_t active_command_id) const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return
    active_command_id != 0U &&
    this->last_terminal_command_id_ == active_command_id &&
    (this->latched_goal_reached_ || this->latched_command_completed_);
}

/// @copydoc Btnavigator::goal_uuid_key
std::string Btnavigator::goal_uuid_key(const rclcpp_action::GoalUUID &uuid) const
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    stream << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}

/// @copydoc Btnavigator::log_exception
void Btnavigator::log_exception(
  const std::string &phase,
  const std::string &exception_type,
  const std::exception &error,
  const bool fatal) const
{
  if (fatal) {
    RCLCPP_FATAL(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=fatal_exception phase=%s exception_type=%s reason=%s",
      phase.c_str(),
      exception_type.c_str(),
      log_value(error.what()).c_str());
    return;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "AMR_LOG schema=v1 component=bt_navigator event=caught_exception phase=%s exception_type=%s reason=%s",
    phase.c_str(),
    exception_type.c_str(),
    log_value(error.what()).c_str());
}

/// @copydoc Btnavigator::log_unknown_exception
void Btnavigator::log_unknown_exception(const std::string &phase, const bool fatal) const
{
  if (fatal) {
    RCLCPP_FATAL(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=fatal_exception phase=%s exception_type=unknown reason=unknown_exception",
      phase.c_str());
    return;
  }
  RCLCPP_ERROR(
    this->get_logger(),
    "AMR_LOG schema=v1 component=bt_navigator event=caught_exception phase=%s exception_type=unknown reason=unknown_exception",
    phase.c_str());
}

/// @copydoc Btnavigator::build_motion_command
amr_msgs::msg::MotionCommand Btnavigator::build_motion_command(
  const geometry_msgs::msg::PoseStamped &goal_pose,
  const std::string &route_id,
  const nav_msgs::msg::Path &plan,
  const bool align_heading_at_goal)
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    goal_pose.header.frame_id.empty() ? std::string("map") : goal_pose.header.frame_id;
  command.command_id = this->next_command_id_++;
  command.mode = amr_msgs::msg::MotionCommand::MODE_NAVIGATE;
  command.route_id = route_id.empty() ? std::string("navigate_to_pose") : route_id;
  command.node_id = this->default_node_id_;
  command.plan = plan;
  command.goal_pose = goal_pose;
  command.align_heading_at_goal = align_heading_at_goal;
  return command;
}

/// @copydoc Btnavigator::publish_motion_command
void Btnavigator::publish_motion_command(
  const amr_msgs::msg::MotionCommand &command,
  const uint32_t current_goal_index,
  const uint32_t goal_count,
  const std::string &command_type)
{
  if (!this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated()) {
    return;
  }
  this->reset_terminal_latch(command.command_id);
  this->motion_command_publisher_->publish(command);
  if (this->structured_logging_enabled_) {
    const bool has_plan_last_pose = !command.plan.poses.empty();
    const auto &plan_last_pose = has_plan_last_pose ? command.plan.poses.back() : command.goal_pose;
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=publish_motion_command command_id=%u goal_id=%u route_goal_id=%u current_goal_index=%u goal_count=%u command_type=%s route_id=%s mode=%u plan_size=%zu goal_x=%.3f goal_y=%.3f plan_last_x=%.3f plan_last_y=%.3f align_heading_at_goal=%s result=published",
      command.command_id,
      command.command_id,
      command.command_id,
      current_goal_index,
      goal_count,
      command_type.c_str(),
      command.route_id.empty() ? "none" : command.route_id.c_str(),
      command.mode,
      command.plan.poses.size(),
      command.goal_pose.pose.position.x,
      command.goal_pose.pose.position.y,
      plan_last_pose.pose.position.x,
      plan_last_pose.pose.position.y,
      bool_label(command.align_heading_at_goal));
  }
}

/// @copydoc Btnavigator::publish_stop_command
void Btnavigator::publish_stop_command(
  const uint32_t current_goal_index,
  const uint32_t goal_count,
  const std::string &reason)
{
  if (!this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated()) {
    return;
  }

  const auto current_pose = this->get_current_pose_copy();
  amr_msgs::msg::MotionCommand stop_command;
  stop_command.header.stamp = this->now();
  stop_command.header.frame_id =
    current_pose.header.frame_id.empty() ? std::string("map") : current_pose.header.frame_id;
  stop_command.command_id = this->next_command_id_++;
  stop_command.mode = amr_msgs::msg::MotionCommand::MODE_WAIT;
  stop_command.route_id = "navigate_to_pose";
  stop_command.node_id = this->default_node_id_;
  stop_command.goal_pose = current_pose;
  stop_command.align_heading_at_goal = false;
  stop_command.recovery_duration = 0.0;
  this->reset_terminal_latch(stop_command.command_id);
  this->motion_command_publisher_->publish(stop_command);
  if (this->structured_logging_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "AMR_LOG schema=v1 component=bt_navigator event=bt_phase_transition phase=publish_stop_command command_id=%u goal_id=%u route_goal_id=%u current_goal_index=%u goal_count=%u command_type=stop_clear_reset plan_size=0 goal_x=%.3f goal_y=%.3f plan_last_x=%.3f plan_last_y=%.3f reason=%s result=published",
      stop_command.command_id,
      stop_command.command_id,
      stop_command.command_id,
      current_goal_index,
      goal_count,
      stop_command.goal_pose.pose.position.x,
      stop_command.goal_pose.pose.position.y,
      stop_command.goal_pose.pose.position.x,
      stop_command.goal_pose.pose.position.y,
      log_value(reason).c_str());
  }
}

}  // namespace amr::bt::navigator
