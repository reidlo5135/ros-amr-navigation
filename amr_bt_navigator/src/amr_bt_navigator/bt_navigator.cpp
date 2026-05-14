#include "amr_bt_navigator/bt_navigator.hpp"

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

std::string get_default_behavior_tree_xml_path()
{
  try {
    return ament_index_cpp::get_package_share_directory("amr_bt_navigator") +
           "/config/navigate_to_pose.xml";
  } catch (const std::exception &) {
    return "config/navigate_to_pose.xml";
  }
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

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

Btnavigator::Btnavigator(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("navigator", options),
  navigate_action_name_("/amr/navigator/navigate_to_pose"),
  navigate_poses_action_name_("/amr/navigator/navigate_to_poses"),
  command_topic_(""),
  current_pose_topic_(""),
  motion_status_topic_(""),
  local_plan_status_topic_(""),
  plan_recovery_service_("/amr/recovery_server/plan_recovery"),
  plan_local_escape_service_("/amr/local_planner/plan_local_escape"),
  clear_costmap_service_("/amr/costmap_server/clear_costmap"),
  plan_segment_service_("/amr/global_planner/plan_segment"),
  behavior_tree_xml_path_(""),
  default_node_id_("start"),
  planner_wait_timeout_ms_(2000),
  feedback_period_ms_(100),
  recovery_max_retries_(3),
  recovery_retry_delay_ms_(700),
  recovery_reacquire_settle_ms_(700),
  nominal_speed_(0.075),
  next_command_id_(1U),
  has_current_pose_(false),
  has_motion_status_(false),
  has_local_plan_status_(false)
{
  this->behavior_tree_xml_path_ = get_default_behavior_tree_xml_path();
  this->declare_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->declare_parameter("actions.navigate_to_poses", this->navigate_poses_action_name_);
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.status", this->motion_status_topic_);
  this->declare_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->declare_parameter("services.plan_recovery", this->plan_recovery_service_);
  this->declare_parameter("services.local_escape", this->plan_local_escape_service_);
  this->declare_parameter("services.clear_costmap", this->clear_costmap_service_);
  this->declare_parameter("services.segment", this->plan_segment_service_);
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
  this->declare_parameter("execution.nominal_linear_speed", this->nominal_speed_);
}

Btnavigator::CallbackReturn Btnavigator::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->get_parameter("actions.navigate_to_poses", this->navigate_poses_action_name_);
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.status", this->motion_status_topic_);
  this->get_parameter("topics.local_plan_status", this->local_plan_status_topic_);
  this->get_parameter("services.plan_recovery", this->plan_recovery_service_);
  this->get_parameter("services.local_escape", this->plan_local_escape_service_);
  this->get_parameter("services.clear_costmap", this->clear_costmap_service_);
  this->get_parameter("services.segment", this->plan_segment_service_);
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
  this->get_parameter("execution.nominal_linear_speed", this->nominal_speed_);

  if (this->behavior_tree_xml_path_.empty()) {
    this->behavior_tree_xml_path_ = get_default_behavior_tree_xml_path();
  }

  if (
    this->command_topic_.empty() || this->current_pose_topic_.empty() ||
    this->motion_status_topic_.empty() || this->local_plan_status_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigator topics must not be empty: command='%s' pose='%s' status='%s' local_plan_status='%s'",
      this->command_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->motion_status_topic_.c_str(),
      this->local_plan_status_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

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
  this->current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->current_pose_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      this->handle_current_pose(message);
    });
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
    "Configured navigator with actions='%s'/'%s', command='%s', pose='%s', status='%s', local_plan_status='%s', recovery='%s', local_escape='%s', clear_costmap='%s', planner='%s', bt_xml='%s'",
    this->navigate_action_name_.c_str(),
    this->navigate_poses_action_name_.c_str(),
    this->command_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->motion_status_topic_.c_str(),
    this->local_plan_status_topic_.c_str(),
    this->plan_recovery_service_.c_str(),
    this->plan_local_escape_service_.c_str(),
    this->clear_costmap_service_.c_str(),
    this->plan_segment_service_.c_str(),
    this->behavior_tree_xml_path_.c_str());

  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated navigator");
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated navigator");
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->action_server_.reset();
  this->action_server_poses_.reset();
  this->plan_recovery_client_.reset();
  this->plan_local_escape_client_.reset();
  this->clear_costmap_client_.reset();
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->local_plan_status_subscription_.reset();
  this->motion_command_publisher_.reset();
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

Btnavigator::CallbackReturn Btnavigator::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->action_server_.reset();
  this->action_server_poses_.reset();
  this->plan_recovery_client_.reset();
  this->plan_local_escape_client_.reset();
  this->clear_costmap_client_.reset();
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->local_plan_status_subscription_.reset();
  this->motion_command_publisher_.reset();
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

rclcpp_action::GoalResponse Btnavigator::handle_goal(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  if (this->has_active_goal()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Rejecting goal while another navigate goal is still active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal while navigator is inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->goal_pose.header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal with empty frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted navigate goal: frame='%s' x=%.3f y=%.3f",
    goal->goal_pose.header.frame_id.c_str(),
    goal->goal_pose.pose.position.x,
    goal->goal_pose.pose.position.y);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse Btnavigator::handle_goals(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const NavigateToPoses::Goal> goal)
{
  (void)uuid;
  if (this->has_active_goal()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Rejecting waypoint route while another navigation goal is still active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting waypoint route while navigator is inactive");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->goal_poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejecting waypoint route with no goal poses");
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (std::size_t index = 0; index < goal->goal_poses.size(); ++index) {
    if (goal->goal_poses[index].header.frame_id.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejecting waypoint route because goal %zu has empty frame_id",
        index);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  const auto &first_goal = goal->goal_poses.front();
  const auto &last_goal = goal->goal_poses.back();
  RCLCPP_INFO(
    this->get_logger(),
    "Accepted waypoint route with %zu goals: first=(%.3f, %.3f) last=(%.3f, %.3f)",
    goal->goal_poses.size(),
    first_goal.pose.position.x,
    first_goal.pose.position.y,
    last_goal.pose.position.x,
    last_goal.pose.position.y);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Btnavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Cancel requested for active navigate goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse Btnavigator::handle_cancel_goals(
  const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Cancel requested for active waypoint route");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Btnavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    this->active_goal_handle_ = goal_handle;
  }
  std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void Btnavigator::handle_accepted_goals(
  const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    this->active_goals_handle_ = goal_handle;
  }
  std::thread([this, goal_handle]() { this->execute_goals(goal_handle); }).detach();
}

void Btnavigator::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  const auto clear_active_goal = [this, &goal_handle]() {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    const auto active_goal = this->active_goal_handle_.lock();
    if (active_goal == goal_handle) {
      this->active_goal_handle_.reset();
    }
  };

  const auto goal = goal_handle->get_goal();
  const auto result = this->execute_goal_pose(
    goal->goal_pose,
    "navigate_to_pose",
    true,
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
    goal_handle->canceled(action_result);
  } else if (result.success) {
    action_result->error_code = NavigateToPose::Result::NONE;
    goal_handle->succeed(action_result);
  } else {
    action_result->error_code = result.error_code;
    goal_handle->abort(action_result);
  }

  clear_active_goal();
}

void Btnavigator::execute_goals(const std::shared_ptr<GoalHandleNavigateToPoses> goal_handle)
{
  const auto clear_active_goal = [this, &goal_handle]() {
    std::scoped_lock active_goal_lock(this->active_goal_mutex_);
    const auto active_goal = this->active_goals_handle_.lock();
    if (active_goal == goal_handle) {
      this->active_goals_handle_.reset();
    }
  };

  const auto goal = goal_handle->get_goal();
  uint32_t completed_goals = 0U;
  const auto goal_count = static_cast<uint32_t>(goal->goal_poses.size());

  if (goal->goal_poses.empty()) {
    auto result = std::make_shared<NavigateToPoses::Result>();
    result->error_code = NavigateToPoses::Result::UNKNOWN;
    result->error_msg = "Waypoint route is empty.";
    result->completed_goals = 0U;
    goal_handle->abort(result);
    clear_active_goal();
    return;
  }

  for (std::size_t index = 0; index < goal->goal_poses.size(); ++index) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<NavigateToPoses::Result>();
      result->error_code = NavigateToPoses::Result::NONE;
      result->error_msg = "Waypoint route canceled.";
      result->completed_goals = completed_goals;
      goal_handle->canceled(result);
      clear_active_goal();
      return;
    }

    const auto route_goal_pose = make_route_goal_pose(goal->goal_poses, index);
    const bool align_heading_at_goal = (index + 1U) >= goal->goal_poses.size();
    const auto waypoint_result = this->execute_goal_pose(
      route_goal_pose,
      "navigate_to_poses",
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
      goal_handle->canceled(result);
      clear_active_goal();
      return;
    }

    if (!waypoint_result.success) {
      auto result = std::make_shared<NavigateToPoses::Result>();
      result->error_code = waypoint_result.error_code;
      result->error_msg =
        "Waypoint " + std::to_string(index + 1) + " failed: " + waypoint_result.message;
      result->completed_goals = completed_goals;
      goal_handle->abort(result);
      clear_active_goal();
      return;
    }

    completed_goals += 1U;
  }

  auto result = std::make_shared<NavigateToPoses::Result>();
  result->error_code = NavigateToPoses::Result::NONE;
  result->error_msg =
    "Completed all " + std::to_string(completed_goals) + " waypoint goals.";
  result->completed_goals = completed_goals;
  goal_handle->succeed(result);
  clear_active_goal();
}

Btnavigator::ExecutionResult Btnavigator::execute_goal_pose(
  const geometry_msgs::msg::PoseStamped &goal_pose,
  const std::string &route_id,
  const bool align_heading_at_goal,
  const std::function<bool()> &is_cancel_requested,
  const std::function<void(
    const geometry_msgs::msg::PoseStamped &,
    const amr_msgs::msg::MotionStatus &,
    int32_t,
    const rclcpp::Duration &)> &publish_feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Starting BT navigation execution: route='%s' frame='%s' x=%.3f y=%.3f",
    route_id.c_str(),
    goal_pose.header.frame_id.c_str(),
    goal_pose.pose.position.x,
    goal_pose.pose.position.y);

  BT::BehaviorTreeFactory factory;
  auto blackboard = BT::Blackboard::create();
  blackboard->set("navigator", this);
  blackboard->set("goal_pose", goal_pose);
  blackboard->set("route_id", route_id);
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
      RCLCPP_INFO(
        navigator->get_logger(),
        "BT: requesting global plan from (%.3f, %.3f) to (%.3f, %.3f)",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        goal_pose.pose.position.x,
        goal_pose.pose.position.y);
      if (!navigator->request_global_plan(
          current_pose, goal_pose, plan, error_message, cancel_requested))
      {
        RCLCPP_WARN(
          navigator->get_logger(),
          "BT: global plan request failed: %s",
          error_message.c_str());
        blackboard->set("status_message", error_message);
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(
        navigator->get_logger(),
        "BT: global plan ready with %zu poses",
        plan.poses.size());
      blackboard->set("planned_path", plan);
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "PublishMotionCommand",
    [blackboard](BT::TreeNode &) {
      auto *navigator = blackboard->get<Btnavigator *>("navigator");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      const auto route_id = blackboard->get<std::string>("route_id");
      const bool align_heading_at_goal = blackboard->get<bool>("align_heading_at_goal");
      const auto plan = blackboard->get<nav_msgs::msg::Path>("planned_path");
      auto command = navigator->build_motion_command(goal_pose, route_id, plan, align_heading_at_goal);
      navigator->publish_motion_command(command);
      RCLCPP_INFO(
        navigator->get_logger(),
        "BT: motion command %u dispatched with %zu poses",
        command.command_id,
        plan.poses.size());
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
      if (status.command_id == command.command_id && status.goal_reached) {
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
        navigator->publish_stop_command();
        blackboard->set("recovery_condition_since_ns", static_cast<int64_t>(0));
        blackboard->set("recovery_reacquire_until_ns", static_cast<int64_t>(0));
        blackboard->set("recovery_reacquire_command_id", static_cast<uint32_t>(0U));
        blackboard->set("status_message", std::string("Navigation canceled."));
        blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kCanceled));
        return BT::NodeStatus::SUCCESS;
      };

      RCLCPP_WARN(
        navigator->get_logger(),
        "BT: recovery needed; starting attempt %d",
        attempts + 1);

      if (cancel_requested()) {
        return finish_canceled();
      }

      navigator->publish_stop_command();

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
          "BT: clear local costmap failed: %s",
          error_message.c_str());
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
          navigator->publish_motion_command(command);
          arm_recovery_reacquire(command);
          blackboard->set("local_escape_dispatched", true);
          blackboard->set("planned_path", escape_plan);
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
          "BT: local escape planning failed, falling back to recovery behaviors: %s",
          error_message.c_str());
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
            navigator->publish_stop_command();
            blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
            blackboard->set(
              "status_message",
              std::string("Recovery retries exceeded before another global replan could be dispatched."));
            return BT::NodeStatus::SUCCESS;
          }
          auto command = navigator->build_motion_command(
            goal_pose, route_id, replanned_path, align_heading_at_goal);
          navigator->publish_motion_command(command);
          arm_recovery_reacquire(command);
          blackboard->set("planned_path", replanned_path);
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
          navigator->publish_motion_command(recovery_command);
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
            blackboard->set("status_message", error_message);
          }
          else
          {
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
                navigator->publish_stop_command();
                blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
                blackboard->set(
                  "status_message",
                  std::string("Recovery retries exceeded before the current plan could be re-dispatched."));
                return BT::NodeStatus::SUCCESS;
              }
              auto command = navigator->build_motion_command(
                goal_pose, route_id, planned_path, align_heading_at_goal);
              navigator->publish_motion_command(command);
              arm_recovery_reacquire(command);
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
        navigator->publish_stop_command();
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
      navigator->publish_motion_command(command);
      arm_recovery_reacquire(command);
      blackboard->set("planned_path", replanned_path);
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
      navigator->publish_stop_command();
      blackboard->set("status_message", std::string("Stop command dispatched."));
      RCLCPP_WARN(
        navigator->get_logger(),
        "BT: stop command dispatched");
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

  if (plan_tree.tickRoot() != BT::NodeStatus::SUCCESS) {
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

    if (publish_feedback) {
      const auto recovery_count = blackboard->get<int>("recovery_attempts");
      publish_feedback(pose, status, recovery_count, this->now() - goal_start);
    }

    blackboard->set("current_pose", pose);
    const auto monitor_status = monitor_tree.tickRoot();
    (void)monitor_status;

    const auto outcome = static_cast<BtOutcome>(blackboard->get<int>("bt_outcome"));
    const auto message = blackboard->get<std::string>("status_message");
    if (outcome == BtOutcome::kSucceeded) {
      RCLCPP_INFO(this->get_logger(), "BT: goal succeeded: %s", message.c_str());
      return ExecutionResult{true, false, message, 0U};
    }
    if (outcome == BtOutcome::kCanceled) {
      RCLCPP_INFO(this->get_logger(), "BT: goal canceled: %s", message.c_str());
      return ExecutionResult{false, true, message, 0U};
    }
    if (outcome == BtOutcome::kStopped) {
      RCLCPP_ERROR(this->get_logger(), "BT: goal aborted: %s", message.c_str());
      return ExecutionResult{false, false, message, 9000U};
    }
  }

  return ExecutionResult{
    false,
    false,
    "Navigator stopped because ROS is shutting down.",
    9000U};
}

void Btnavigator::handle_current_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = *message;
  this->has_current_pose_ = true;
}

void Btnavigator::handle_motion_status(const amr_msgs::msg::MotionStatus::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_motion_status_ = *message;
  this->has_motion_status_ = true;
}

void Btnavigator::handle_local_plan_status(
  const amr_msgs::msg::LocalPlanStatus::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_local_plan_status_ = *message;
  this->has_local_plan_status_ = true;
}

geometry_msgs::msg::PoseStamped Btnavigator::get_current_pose_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->current_pose_;
}

amr_msgs::msg::MotionStatus Btnavigator::get_motion_status_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_motion_status_;
}

amr_msgs::msg::LocalPlanStatus Btnavigator::get_local_plan_status_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_local_plan_status_;
}

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

  RCLCPP_INFO(
    this->get_logger(),
    "Requesting global plan: start=(%.3f, %.3f) goal=(%.3f, %.3f)",
    request->start.pose.position.x,
    request->start.pose.position.y,
    request->goal.pose.position.x,
    request->goal.pose.position.y);

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
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  plan = response->plan;
  RCLCPP_INFO(this->get_logger(), "Received global plan with %zu poses", plan.poses.size());
  error_message.clear();
  return true;
}

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

  const auto response = future.get();
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

  const auto response = future.get();
  if (!response->success) {
    error_message = this->describe_local_escape_failure(response->message);
    return false;
  }

  escape_plan = response->plan;
  error_message.clear();
  return true;
}

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

  if (local_plan_status.command_id != active_command.command_id || !local_plan_status.active)
  {
    return true;
  }

  return
    local_plan_status.local_plan_valid && 
    !local_plan_status.recovery_required;
}

bool Btnavigator::has_planner_owned_recovery(
  const amr_msgs::msg::MotionCommand &active_command,
  const amr_msgs::msg::LocalPlanStatus &local_plan_status) const
{
  return
    local_plan_status.command_id == active_command.command_id && 
    local_plan_status.active && 
    local_plan_status.recovery_required;
}

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

std::string Btnavigator::describe_local_escape_failure(const std::string &planner_message) const
{
  if (planner_message.empty()) {
    return "Local escape planner failed without a reason label.";
  }

  return "Local escape planner failed: " + planner_message;
}

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

  const auto response = future.get();
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  error_message.clear();
  return true;
}

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

bool Btnavigator::has_active_goal() const
{
  std::scoped_lock active_goal_lock(this->active_goal_mutex_);
  return !this->active_goal_handle_.expired() || !this->active_goals_handle_.expired();
}

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

void Btnavigator::publish_motion_command(const amr_msgs::msg::MotionCommand &command)
{
  if (!this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated()) {
    return;
  }
  this->motion_command_publisher_->publish(command);
  RCLCPP_INFO(
    this->get_logger(),
    "Published motion command %u toward goal x=%.3f y=%.3f",
    command.command_id,
    command.goal_pose.pose.position.x,
    command.goal_pose.pose.position.y);
}

void Btnavigator::publish_stop_command()
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
  this->motion_command_publisher_->publish(stop_command);
  RCLCPP_INFO(
    this->get_logger(),
    "Published stop command %u",
    stop_command.command_id);
}

}  // namespace amr::bt::navigator
