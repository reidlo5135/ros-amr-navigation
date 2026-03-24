#include "amr_bt_navigator/bt_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace amr_bt_navigator
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

}  // namespace

Btnavigator::Btnavigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", options),
  navigate_action_name_("/amr/navigator/navigate_to_pose"),
  command_topic_(""),
  current_pose_topic_(""),
  motion_status_topic_(""),
  obstacle_report_topic_(""),
  plan_segment_service_("/amr/global_planner/plan_segment"),
  behavior_tree_xml_path_(""),
  default_node_id_("start"),
  planner_wait_timeout_ms_(2000),
  feedback_period_ms_(100),
  next_command_id_(1U),
  has_current_pose_(false),
  has_motion_status_(false),
  has_obstacle_report_(false)
{
  try {
    this->behavior_tree_xml_path_ =
      ament_index_cpp::get_package_share_directory("amr_bt_navigator") + "/config/navigate_to_pose.xml";
  } catch (const std::exception &) {
    this->behavior_tree_xml_path_ = "config/navigate_to_pose.xml";
  }
  this->declare_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->declare_parameter("topics.command", this->command_topic_);
  this->declare_parameter("topics.pose", this->current_pose_topic_);
  this->declare_parameter("topics.status", this->motion_status_topic_);
  this->declare_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->declare_parameter("services.segment", this->plan_segment_service_);
  this->declare_parameter("behavior_tree.xml_path", this->behavior_tree_xml_path_);
  this->declare_parameter("defaults.node_id", this->default_node_id_);
  this->declare_parameter(
    "execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->declare_parameter("execution.feedback_period_ms", this->feedback_period_ms_);
}

Btnavigator::CallbackReturn Btnavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("actions.navigate_to_pose", this->navigate_action_name_);
  this->get_parameter("topics.command", this->command_topic_);
  this->get_parameter("topics.pose", this->current_pose_topic_);
  this->get_parameter("topics.status", this->motion_status_topic_);
  this->get_parameter("topics.obstacle_report", this->obstacle_report_topic_);
  this->get_parameter("services.segment", this->plan_segment_service_);
  this->get_parameter("behavior_tree.xml_path", this->behavior_tree_xml_path_);
  this->get_parameter("defaults.node_id", this->default_node_id_);
  this->get_parameter(
    "execution.planner_wait_timeout_ms", this->planner_wait_timeout_ms_);
  this->get_parameter("execution.feedback_period_ms", this->feedback_period_ms_);

  if (
    this->command_topic_.empty() || this->current_pose_topic_.empty() ||
    this->motion_status_topic_.empty() || this->obstacle_report_topic_.empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigator topics must not be empty: command='%s' pose='%s' status='%s' obstacle_report='%s'",
      this->command_topic_.c_str(),
      this->current_pose_topic_.c_str(),
      this->motion_status_topic_.c_str(),
      this->obstacle_report_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  this->motion_command_publisher_ = this->create_publisher<amr_msgs::msg::MotionCommand>(
    this->command_topic_, rclcpp::SystemDefaultsQoS());
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
  this->obstacle_report_subscription_ = this->create_subscription<amr_msgs::msg::ObstacleReport>(
    this->obstacle_report_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const amr_msgs::msg::ObstacleReport::SharedPtr message) {
      this->handle_obstacle_report(message);
    });
  this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    this->navigate_action_name_,
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      return this->handle_cancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
      this->handle_accepted(goal_handle);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured navigator with action='%s', command='%s', pose='%s', status='%s', obstacle_report='%s', planner='%s', bt_xml='%s'",
    this->navigate_action_name_.c_str(),
    this->command_topic_.c_str(),
    this->current_pose_topic_.c_str(),
    this->motion_status_topic_.c_str(),
    this->obstacle_report_topic_.c_str(),
    this->plan_segment_service_.c_str(),
    this->behavior_tree_xml_path_.c_str());

  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated navigator");
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->motion_command_publisher_) {
    this->motion_command_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated navigator");
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->action_server_.reset();
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->obstacle_report_subscription_.reset();
  this->motion_command_publisher_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->latest_obstacle_report_ = amr_msgs::msg::ObstacleReport();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  this->has_obstacle_report_ = false;
  return CallbackReturn::SUCCESS;
}

Btnavigator::CallbackReturn Btnavigator::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->action_server_.reset();
  this->plan_segment_client_.reset();
  this->current_pose_subscription_.reset();
  this->motion_status_subscription_.reset();
  this->obstacle_report_subscription_.reset();
  this->motion_command_publisher_.reset();
  std::scoped_lock lock(this->navigator_mutex_);
  this->current_pose_ = geometry_msgs::msg::PoseStamped();
  this->latest_motion_status_ = amr_msgs::msg::MotionStatus();
  this->latest_obstacle_report_ = amr_msgs::msg::ObstacleReport();
  this->has_current_pose_ = false;
  this->has_motion_status_ = false;
  this->has_obstacle_report_ = false;
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse Btnavigator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
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

rclcpp_action::CancelResponse Btnavigator::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Cancel requested for active navigate goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Btnavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void Btnavigator::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  BT::BehaviorTreeFactory factory;
  auto blackboard = BT::Blackboard::create();
  blackboard->set("navigator", this);
  blackboard->set("goal_handle", goal_handle);
  blackboard->set("goal_pose", goal->goal_pose);
  blackboard->set("planned_path", nav_msgs::msg::Path());
  blackboard->set("active_command", amr_msgs::msg::MotionCommand());
  blackboard->set("status_message", std::string("Behavior tree is running."));
  blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kRunning));

  factory.registerSimpleCondition(
    "CheckNavigatorReady",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
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
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
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
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      std::string error_message;
      if (navigator->wait_for_planner_service(error_message)) {
        return BT::NodeStatus::SUCCESS;
      }
      blackboard->set("status_message", error_message);
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleAction(
    "RequestGlobalPlan",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      const auto current_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("current_pose");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      nav_msgs::msg::Path plan;
      std::string error_message;
      if (!navigator->request_global_plan(current_pose, goal_pose, plan, error_message)) {
        blackboard->set("status_message", error_message);
        return BT::NodeStatus::FAILURE;
      }
      blackboard->set("planned_path", plan);
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "PublishMotionCommand",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      const auto goal_pose = blackboard->get<geometry_msgs::msg::PoseStamped>("goal_pose");
      const auto plan = blackboard->get<nav_msgs::msg::Path>("planned_path");
      NavigateToPose::Goal goal_request;
      goal_request.goal_pose = goal_pose;
      auto command = navigator->build_motion_command(goal_request, plan);
      navigator->publish_motion_command(command);
      blackboard->set("active_command", command);
      blackboard->set("status_message", std::string("Motion command dispatched."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleCondition(
    "CheckCancelRequested",
    [blackboard](BT::TreeNode &) {
      const auto goal_handle_local =
        blackboard->get<std::shared_ptr<GoalHandleNavigateToPose>>("goal_handle");
      return goal_handle_local->is_canceling() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleCondition(
    "CheckGoalReached",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      const auto status = navigator->get_motion_status_copy();
      const auto command = blackboard->get<amr_msgs::msg::MotionCommand>("active_command");
      if (status.command_id == command.command_id && status.goal_reached) {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleCondition(
    "CheckObstacleBlocking",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      const auto status = navigator->get_motion_status_copy();
      const auto obstacle = navigator->get_obstacle_report_copy();
      if (status.obstacle_detected || (obstacle.active && obstacle.blocks_path)) {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    });

  factory.registerSimpleAction(
    "PublishStopCommand",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      navigator->publish_stop_command();
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "MarkCanceled",
    [blackboard](BT::TreeNode &) {
      blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kCanceled));
      blackboard->set("status_message", std::string("Route execution canceled."));
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
    "MarkStopped",
    [blackboard](BT::TreeNode &) {
      blackboard->set("bt_outcome", static_cast<int>(BtOutcome::kStopped));
      blackboard->set(
        "status_message",
        std::string("Obstacle blocking path; navigator issued stop command."));
      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "WaitFeedbackPeriod",
    [blackboard](BT::TreeNode &) {
      auto * navigator = blackboard->get<Btnavigator *>("navigator");
      std::this_thread::sleep_for(std::chrono::milliseconds(navigator->feedback_period_ms_));
      return BT::NodeStatus::SUCCESS;
    });

  BT::Tree plan_tree;
  BT::Tree monitor_tree;
  try {
    factory.registerBehaviorTreeFromFile(this->behavior_tree_xml_path_);
    plan_tree = factory.createTree("PlanAndDispatch", blackboard);
    monitor_tree = factory.createTree("MonitorExecution", blackboard);
  } catch (const std::exception & error) {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = std::string("Failed to initialize behavior tree: ") + error.what();
    goal_handle->abort(result);
    return;
  }

  if (plan_tree.tickRoot() != BT::NodeStatus::SUCCESS) {
    auto result = std::make_shared<NavigateToPose::Result>();
    result->success = false;
    result->message = blackboard->get<std::string>("status_message");
    goal_handle->abort(result);
    return;
  }

  while (rclcpp::ok()) {
    const auto status = this->get_motion_status_copy();
    const auto pose = this->get_current_pose_copy();

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = pose;
    feedback->remaining_distance = status.remaining_distance;
    feedback->heading_error = status.heading_error;
    goal_handle->publish_feedback(feedback);

    blackboard->set("current_pose", pose);
    const auto monitor_status = monitor_tree.tickRoot();
    (void)monitor_status;

    const auto outcome = static_cast<BtOutcome>(blackboard->get<int>("bt_outcome"));
    const auto message = blackboard->get<std::string>("status_message");
    if (outcome == BtOutcome::kSucceeded) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = true;
      result->message = message;
      goal_handle->succeed(result);
      return;
    }
    if (outcome == BtOutcome::kCanceled) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = false;
      result->message = message;
      goal_handle->canceled(result);
      return;
    }
    if (outcome == BtOutcome::kStopped) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = false;
      result->message = message;
      goal_handle->abort(result);
      return;
    }
  }

  auto result = std::make_shared<NavigateToPose::Result>();
  result->success = false;
  result->message = "Navigator stopped because ROS is shutting down.";
  goal_handle->abort(result);
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

void Btnavigator::handle_obstacle_report(const amr_msgs::msg::ObstacleReport::SharedPtr message)
{
  std::scoped_lock lock(this->navigator_mutex_);
  this->latest_obstacle_report_ = *message;
  this->has_obstacle_report_ = true;
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

amr_msgs::msg::ObstacleReport Btnavigator::get_obstacle_report_copy() const
{
  std::scoped_lock lock(this->navigator_mutex_);
  return this->latest_obstacle_report_;
}

bool Btnavigator::is_navigator_ready(std::string & error_message) const
{
  if (
    !this->motion_command_publisher_ || !this->motion_command_publisher_->is_activated() ||
    !this->plan_segment_client_)
  {
    error_message = "Navigator is not active.";
    return false;
  }
  error_message.clear();
  return true;
}

bool Btnavigator::wait_for_planner_service(std::string & error_message)
{
  if (!this->plan_segment_client_) {
    error_message = "Global planner client is not configured.";
    return false;
  }
  if (!this->plan_segment_client_->wait_for_service(
      std::chrono::milliseconds(this->planner_wait_timeout_ms_)))
  {
    error_message = "Global planner service is not available.";
    return false;
  }
  error_message.clear();
  return true;
}

bool Btnavigator::request_global_plan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan,
  std::string & error_message)
{
  auto request = std::make_shared<amr_msgs::srv::PlanSegment::Request>();
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
  if (future.wait_for(std::chrono::milliseconds(this->planner_wait_timeout_ms_)) !=
      std::future_status::ready)
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

amr_msgs::msg::MotionCommand Btnavigator::build_motion_command(
  const NavigateToPose::Goal & goal,
  const nav_msgs::msg::Path & plan)
{
  amr_msgs::msg::MotionCommand command;
  command.header.stamp = this->now();
  command.header.frame_id =
    goal.goal_pose.header.frame_id.empty() ? std::string("map") : goal.goal_pose.header.frame_id;
  command.command_id = this->next_command_id_++;
  command.route_id = "navigate_to_pose";
  command.node_id = this->default_node_id_;
  command.plan = plan;
  command.goal_pose = goal.goal_pose;
  command.align_heading_at_goal = true;
  return command;
}

void Btnavigator::publish_motion_command(const amr_msgs::msg::MotionCommand & command)
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
  stop_command.route_id = "navigate_to_pose";
  stop_command.node_id = this->default_node_id_;
  stop_command.goal_pose = current_pose;
  stop_command.align_heading_at_goal = false;
  this->motion_command_publisher_->publish(stop_command);
  RCLCPP_INFO(
    this->get_logger(),
    "Published stop command %u",
    stop_command.command_id);
}

}  // namespace amr_bt_navigator
