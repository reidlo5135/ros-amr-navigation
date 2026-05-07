#include "amr_visualization/ros_worker.hpp"

#include <QJsonDocument>
#include <QJsonObject>

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <utility>

namespace amr::visualization
{

namespace
{

using namespace std::chrono_literals;

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

}  // namespace

Pose2D RosWorker::compose_pose(const Pose2D & parent, const Pose2D & child)
{
  Pose2D pose;
  pose.x = parent.x + (std::cos(parent.yaw) * child.x) - (std::sin(parent.yaw) * child.y);
  pose.y = parent.y + (std::sin(parent.yaw) * child.x) + (std::cos(parent.yaw) * child.y);
  pose.yaw = parent.yaw + child.yaw;
  pose.valid = parent.valid && child.valid;
  return pose;
}

RosWorker::RosWorker(QObject * parent)
: QObject(parent)
{
}

RosWorker::~RosWorker()
{
  stop();
}

void RosWorker::start()
{
  if (running_.exchange(true)) {
    return;
  }

  node_ = std::make_shared<rclcpp::Node>("amr_visualization");
  configure_ros_interfaces();
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { spin(); });
  Q_EMIT connectionStateChanged("ROS connected");
}

void RosWorker::stop()
{
  if (!running_.exchange(false)) {
    return;
  }

  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (node_) {
    executor_.remove_node(node_);
    node_.reset();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  Q_EMIT connectionStateChanged("ROS stopped");
}

void RosWorker::configure_ros_interfaces()
{
  default_frame_id_ = node_->declare_parameter<std::string>("default_frame_id", default_frame_id_);
  map_topic_ = node_->declare_parameter<std::string>("map_topic", map_topic_);
  global_costmap_topic_ =
    node_->declare_parameter<std::string>("global_costmap_topic", global_costmap_topic_);
  local_costmap_topic_ =
    node_->declare_parameter<std::string>("local_costmap_topic", local_costmap_topic_);
  pose_topic_ = node_->declare_parameter<std::string>("pose_topic", pose_topic_);
  initial_pose_topic_ =
    node_->declare_parameter<std::string>("initial_pose_topic", initial_pose_topic_);
  global_path_topic_ = node_->declare_parameter<std::string>("global_path_topic", global_path_topic_);
  local_path_topic_ = node_->declare_parameter<std::string>("local_path_topic", local_path_topic_);
  motion_status_topic_ =
    node_->declare_parameter<std::string>("motion_status_topic", motion_status_topic_);
  runtime_summary_topic_ =
    node_->declare_parameter<std::string>("runtime_summary_topic", runtime_summary_topic_);
  runtime_event_topic_ =
    node_->declare_parameter<std::string>("runtime_event_topic", runtime_event_topic_);
  battery_state_topic_ =
    node_->declare_parameter<std::string>("battery_state_topic", battery_state_topic_);
  scan_topic_ = node_->declare_parameter<std::string>("scan_topic", scan_topic_);
  tf_topic_ = node_->declare_parameter<std::string>("tf_topic", tf_topic_);
  tf_static_topic_ = node_->declare_parameter<std::string>("tf_static_topic", tf_static_topic_);
  navigate_to_pose_action_ =
    node_->declare_parameter<std::string>("navigate_to_pose_action", navigate_to_pose_action_);
  navigate_to_poses_action_ =
    node_->declare_parameter<std::string>("navigate_to_poses_action", navigate_to_poses_action_);
  subscribe_global_costmap_ =
    node_->declare_parameter<bool>("subscribe_global_costmap", subscribe_global_costmap_);
  subscribe_local_costmap_ =
    node_->declare_parameter<bool>("subscribe_local_costmap", subscribe_local_costmap_);
  subscribe_scan_ =
    node_->declare_parameter<bool>("subscribe_scan", subscribe_scan_);
  costmap_emit_period_ms_ =
    node_->declare_parameter<int>("costmap_emit_period_ms", costmap_emit_period_ms_);

  const auto latched_map_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto live_qos = rclcpp::SystemDefaultsQoS();

  map_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, latched_map_qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      Q_EMIT mapChanged(convert_grid(*message));
    });
  update_global_costmap_subscription();
  update_local_costmap_subscription();
  update_scan_subscription();
  pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, live_qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      Q_EMIT robotPoseChanged(convert_pose(*message));
    });
  global_path_subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
    global_path_topic_, live_qos, [this](const nav_msgs::msg::Path::SharedPtr message) {
      Q_EMIT globalPathChanged(convert_path(*message));
    });
  local_path_subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
    local_path_topic_, live_qos, [this](const nav_msgs::msg::Path::SharedPtr message) {
      Q_EMIT localPathChanged(convert_path(*message));
    });
  motion_status_subscription_ = node_->create_subscription<amr_msgs::msg::MotionStatus>(
    motion_status_topic_, live_qos,
    [this](const amr_msgs::msg::MotionStatus::SharedPtr message) {
      MotionStatusData status;
      status.active = message->active;
      status.command_completed = message->command_completed;
      status.goal_reached = message->goal_reached;
      status.blocked = message->blocked;
      status.stalled = message->stalled;
      status.obstacle_detected = message->obstacle_detected;
      status.local_plan_valid = message->local_plan_valid;
      status.costmap_blocked = message->costmap_blocked;
      status.safety_gate_blocked = message->safety_gate_blocked;
      status.remaining_distance = message->remaining_distance;
      status.heading_error = message->heading_error;
      if (message->safety_gate_blocked) {
        status.blocked_source = "Safety Gate";
      } else if (message->costmap_blocked) {
        status.blocked_source = "Costmap";
      } else if (message->obstacle_detected) {
        status.blocked_source = "Obstacle";
      } else if (message->stalled) {
        status.blocked_source = "Stalled";
      } else if (message->blocked) {
        status.blocked_source = "Blocked";
      } else if (!message->local_plan_valid && message->active) {
        status.blocked_source = "Local Plan";
      } else {
        status.blocked_source = "Clear";
      }
      Q_EMIT motionStatusChanged(status);
    });
  runtime_summary_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    runtime_summary_topic_, live_qos, [this](const std_msgs::msg::String::SharedPtr message) {
      Q_EMIT runtimeSummaryChanged(parse_runtime_summary(message->data));
    });
  runtime_event_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    runtime_event_topic_, live_qos, [this](const std_msgs::msg::String::SharedPtr message) {
      Q_EMIT eventReceived(QString::fromStdString(message->data));
    });
  battery_subscription_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_state_topic_, live_qos,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr message) {
      double percentage = message->percentage;
      if (std::isfinite(percentage) && percentage <= 1.0) {
        percentage *= 100.0;
      }
      if (!std::isfinite(percentage)) {
        percentage = -1.0;
      }
      Q_EMIT batteryStateChanged(percentage, message->present);
    });
  tf_subscription_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic_, live_qos, [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
      handle_tf_message(*message, false);
    });
  tf_static_subscription_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    tf_static_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
      handle_tf_message(*message, true);
    });

  initial_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, live_qos);
  navigate_to_pose_client_ =
    rclcpp_action::create_client<NavigateToPose>(node_, navigate_to_pose_action_);
  navigate_to_poses_client_ =
    rclcpp_action::create_client<NavigateToPoses>(node_, navigate_to_poses_action_);
}

void RosWorker::handle_tf_message(const tf2_msgs::msg::TFMessage & message, bool is_static)
{
  auto & storage = is_static ? static_frames_ : dynamic_frames_;
  for (const auto & transform : message.transforms) {
    FrameVisual frame;
    frame.parent_frame = QString::fromStdString(transform.header.frame_id);
    frame.child_frame = QString::fromStdString(transform.child_frame_id);
    frame.pose.x = transform.transform.translation.x;
    frame.pose.y = transform.transform.translation.y;
    frame.pose.yaw = quaternion_to_yaw(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    frame.pose.valid = true;
    frame.is_static = is_static;
    storage[transform.child_frame_id] = frame;
  }
  Q_EMIT tfFramesChanged(build_frame_visuals());
}

QVector<FrameVisual> RosWorker::build_frame_visuals() const
{
  QVector<FrameVisual> frames;

  FrameVisual map_frame;
  map_frame.child_frame = "map";
  map_frame.pose.valid = true;
  map_frame.is_static = true;
  frames.push_back(map_frame);

  auto find_frame = [this](const std::string & child) -> const FrameVisual * {
    const auto dynamic_it = dynamic_frames_.find(child);
    if (dynamic_it != dynamic_frames_.end()) {
      return &dynamic_it->second;
    }
    const auto static_it = static_frames_.find(child);
    if (static_it != static_frames_.end()) {
      return &static_it->second;
    }
    return nullptr;
  };

  Pose2D map_pose;
  map_pose.valid = true;

  Pose2D odom_pose;
  const FrameVisual * odom = find_frame("odom");
  if (odom && odom->parent_frame == "map") {
    odom_pose = odom->pose;
    FrameVisual odom_frame = *odom;
    frames.push_back(odom_frame);
  }

  Pose2D base_footprint_pose;
  const FrameVisual * base_footprint = find_frame("base_footprint");
  if (base_footprint && base_footprint->parent_frame == "odom" && odom_pose.valid) {
    base_footprint_pose = compose_pose(odom_pose, base_footprint->pose);
    FrameVisual frame = *base_footprint;
    frame.pose = base_footprint_pose;
    frames.push_back(frame);
  }

  Pose2D base_link_pose;
  const FrameVisual * base_link = find_frame("base_link");
  if (base_link) {
    if (base_link->parent_frame == "odom" && odom_pose.valid) {
      base_link_pose = compose_pose(odom_pose, base_link->pose);
    } else if (base_link->parent_frame == "base_footprint" && base_footprint_pose.valid) {
      base_link_pose = compose_pose(base_footprint_pose, base_link->pose);
    }
    if (base_link_pose.valid) {
      FrameVisual frame = *base_link;
      frame.pose = base_link_pose;
      frames.push_back(frame);
    }
  }

  const FrameVisual * base_scan = find_frame("base_scan");
  if (base_scan && base_scan->parent_frame == "base_link" && base_link_pose.valid) {
    FrameVisual frame = *base_scan;
    frame.pose = compose_pose(base_link_pose, base_scan->pose);
    frames.push_back(frame);
  }

  (void)map_pose;
  return frames;
}

void RosWorker::update_global_costmap_subscription()
{
  global_costmap_subscription_.reset();
  if (!node_ || !subscribe_global_costmap_) {
    return;
  }

  const auto costmap_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  global_costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_, costmap_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      const rclcpp::Time now = node_->now();
      if (last_global_costmap_emit_time_.nanoseconds() != 0 &&
        ((now - last_global_costmap_emit_time_).seconds() * 1000.0) < costmap_emit_period_ms_)
      {
        return;
      }
      last_global_costmap_emit_time_ = now;
      Q_EMIT globalCostmapChanged(convert_grid(*message));
    });
  Q_EMIT eventReceived("Global costmap subscription enabled");
}

void RosWorker::update_local_costmap_subscription()
{
  local_costmap_subscription_.reset();
  if (!node_ || !subscribe_local_costmap_) {
    return;
  }

  const auto costmap_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  local_costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, costmap_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      const rclcpp::Time now = node_->now();
      if (last_local_costmap_emit_time_.nanoseconds() != 0 &&
        ((now - last_local_costmap_emit_time_).seconds() * 1000.0) < costmap_emit_period_ms_)
      {
        return;
      }
      last_local_costmap_emit_time_ = now;
      Q_EMIT localCostmapChanged(convert_grid(*message));
    });
  Q_EMIT eventReceived("Local costmap subscription enabled");
}

void RosWorker::update_scan_subscription()
{
  scan_subscription_.reset();
  if (!node_ || !subscribe_scan_) {
    return;
  }

  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      Q_EMIT scanChanged(convert_scan(*message));
    });
  Q_EMIT eventReceived("Scan subscription enabled");
}

void RosWorker::spin()
{
  try {
    executor_.spin();
  } catch (const std::exception & error) {
    Q_EMIT eventReceived(QString("ROS executor stopped: %1").arg(error.what()));
  }
}

GridMap RosWorker::convert_grid(const nav_msgs::msg::OccupancyGrid & message) const
{
  GridMap map;
  map.width = static_cast<int>(message.info.width);
  map.height = static_cast<int>(message.info.height);
  map.resolution = message.info.resolution;
  map.origin_x = message.info.origin.position.x;
  map.origin_y = message.info.origin.position.y;
  map.origin_yaw = quaternion_to_yaw(
    message.info.origin.orientation.x,
    message.info.origin.orientation.y,
    message.info.origin.orientation.z,
    message.info.origin.orientation.w);
  map.cells.reserve(static_cast<int>(message.data.size()));
  for (const int8_t cell : message.data) {
    map.cells.push_back(cell);
  }
  map.valid = map.width > 0 && map.height > 0 &&
    map.cells.size() == static_cast<qsizetype>(map.width * map.height);
  return map;
}

PathData RosWorker::convert_path(const nav_msgs::msg::Path & message) const
{
  PathData path;
  path.points.reserve(static_cast<int>(message.poses.size()));
  for (const auto & pose : message.poses) {
    path.points.push_back(QPointF(pose.pose.position.x, pose.pose.position.y));
  }
  return path;
}

Pose2D RosWorker::convert_pose(const geometry_msgs::msg::PoseStamped & message) const
{
  Pose2D pose;
  pose.x = message.pose.position.x;
  pose.y = message.pose.position.y;
  pose.yaw = quaternion_to_yaw(
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z,
    message.pose.orientation.w);
  pose.valid = true;
  return pose;
}

ScanData RosWorker::convert_scan(const sensor_msgs::msg::LaserScan & message) const
{
  ScanData scan;
  const Pose2D frame_pose = resolve_frame_pose(message.header.frame_id);
  if (!frame_pose.valid) {
    return scan;
  }

  scan.points.reserve(static_cast<int>(message.ranges.size()));
  double angle = message.angle_min;
  for (const float range : message.ranges) {
    if (
      std::isfinite(range) &&
      range >= message.range_min &&
      range <= message.range_max)
    {
      const double local_x = std::cos(angle) * range;
      const double local_y = std::sin(angle) * range;
      const double world_x =
        frame_pose.x + (std::cos(frame_pose.yaw) * local_x) - (std::sin(frame_pose.yaw) * local_y);
      const double world_y =
        frame_pose.y + (std::sin(frame_pose.yaw) * local_x) + (std::cos(frame_pose.yaw) * local_y);
      scan.points.push_back(QPointF(world_x, world_y));
    }
    angle += message.angle_increment;
  }
  return scan;
}

geometry_msgs::msg::PoseStamped RosWorker::to_pose_stamped(const Pose2D & pose) const
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = default_frame_id_;
  stamped.header.stamp = node_->now();
  stamped.pose.position.x = pose.x;
  stamped.pose.position.y = pose.y;
  stamped.pose.position.z = 0.0;
  stamped.pose.orientation = quaternion_from_yaw(pose.yaw);
  return stamped;
}

RuntimeSummary RosWorker::parse_runtime_summary(const std::string & payload) const
{
  RuntimeSummary summary;
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(payload));
  if (!document.isObject()) {
    return summary;
  }
  const auto object = document.object();
  summary.runtime_state = object.value("runtime_state").toString(summary.runtime_state);
  summary.blocked_context = object.value("blocked_context").toString(summary.blocked_context);
  summary.recovery_phase = object.value("recovery_phase").toString(summary.recovery_phase);
  summary.recovery_reason = object.value("recovery_reason").toString(summary.recovery_reason);
  summary.action_status = object.value("action_status").toString(summary.action_status);
  return summary;
}

Pose2D RosWorker::resolve_frame_pose(const std::string & child_frame) const
{
  const auto find_frame = [this](const std::string & child) -> const FrameVisual * {
    const auto dynamic_it = dynamic_frames_.find(child);
    if (dynamic_it != dynamic_frames_.end()) {
      return &dynamic_it->second;
    }
    const auto static_it = static_frames_.find(child);
    if (static_it != static_frames_.end()) {
      return &static_it->second;
    }
    return nullptr;
  };

  std::function<Pose2D(const std::string &, int)> resolve =
    [&](const std::string & frame_name, int depth) -> Pose2D {
      if (depth > 16) {
        return {};
      }
      if (frame_name == default_frame_id_ || frame_name == "map") {
        Pose2D identity;
        identity.valid = true;
        return identity;
      }

      const FrameVisual * frame = find_frame(frame_name);
      if (!frame || !frame->pose.valid || frame->parent_frame.isEmpty()) {
        return {};
      }

      const Pose2D parent_pose = resolve(frame->parent_frame.toStdString(), depth + 1);
      if (!parent_pose.valid) {
        return {};
      }
      return compose_pose(parent_pose, frame->pose);
    };

  return resolve(child_frame, 0);
}

void RosWorker::sendSingleGoal(const Pose2D & pose)
{
  if (!node_ || !pose.valid) {
    return;
  }
  if (!navigate_to_pose_client_->action_server_is_ready() &&
    !navigate_to_pose_client_->wait_for_action_server(100ms))
  {
    Q_EMIT eventReceived("NavigateToPose action server is not ready");
    return;
  }

  NavigateToPose::Goal goal;
  goal.goal_pose = to_pose_stamped(pose);
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & handle) {
      const QString state = handle ? "Accepted" : "Rejected";
      Q_EMIT goalStateChanged(state);
      Q_EMIT eventReceived(QString("Single goal %1").arg(state.toLower()));
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
      const QString state = QString("Finished (%1)").arg(static_cast<int>(result.code));
      Q_EMIT goalStateChanged(state);
      Q_EMIT navigationCompleted(result.code == rclcpp_action::ResultCode::SUCCEEDED);
      Q_EMIT eventReceived(QString("Single goal %1").arg(state));
    };
  navigate_to_pose_client_->async_send_goal(goal, options);
  Q_EMIT goalStateChanged("Sending");
  Q_EMIT eventReceived("Single goal sent");
}

void RosWorker::sendRoute(const QVector<Pose2D> & route)
{
  if (!node_ || route.empty()) {
    return;
  }
  if (!navigate_to_poses_client_->action_server_is_ready() &&
    !navigate_to_poses_client_->wait_for_action_server(100ms))
  {
    Q_EMIT eventReceived("NavigateToPoses action server is not ready");
    return;
  }

  NavigateToPoses::Goal goal;
  goal.goal_poses.reserve(static_cast<size_t>(route.size()));
  for (const auto & pose : route) {
    if (pose.valid) {
      goal.goal_poses.push_back(to_pose_stamped(pose));
    }
  }
  if (goal.goal_poses.empty()) {
    return;
  }
  rclcpp_action::Client<NavigateToPoses>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::SharedPtr & handle) {
      const QString state = handle ? "Accepted" : "Rejected";
      Q_EMIT goalStateChanged(state);
      Q_EMIT eventReceived(QString("Route %1").arg(state.toLower()));
    };
  options.feedback_callback =
    [this](
      rclcpp_action::ClientGoalHandle<NavigateToPoses>::SharedPtr,
      const std::shared_ptr<const NavigateToPoses::Feedback> feedback) {
      Q_EMIT goalStateChanged(
        QString("Running %1/%2")
          .arg(feedback->current_goal_index + 1U)
          .arg(feedback->goal_count));
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::WrappedResult & result) {
      const QString state = QString("Finished (%1)").arg(static_cast<int>(result.code));
      Q_EMIT goalStateChanged(state);
      Q_EMIT navigationCompleted(result.code == rclcpp_action::ResultCode::SUCCEEDED);
      Q_EMIT eventReceived(QString("Route %1").arg(state));
    };
  navigate_to_poses_client_->async_send_goal(goal, options);
  Q_EMIT goalStateChanged("Sending");
  Q_EMIT eventReceived(
    QString("Route sent: %1 waypoint(s)").arg(static_cast<int>(goal.goal_poses.size())));
}

void RosWorker::publishInitialPose(const Pose2D & pose)
{
  if (!node_ || !pose.valid) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = default_frame_id_;
  message.header.stamp = node_->now();
  message.pose.pose.position.x = pose.x;
  message.pose.pose.position.y = pose.y;
  message.pose.pose.orientation = quaternion_from_yaw(pose.yaw);
  message.pose.covariance[0] = 0.25;
  message.pose.covariance[7] = 0.25;
  message.pose.covariance[35] = 0.06853891945200942;
  initial_pose_publisher_->publish(message);
  Q_EMIT eventReceived("Initial pose published");
}

void RosWorker::cancelNavigation()
{
  if (!node_) {
    return;
  }
  if (navigate_to_pose_client_) {
    navigate_to_pose_client_->async_cancel_all_goals();
  }
  if (navigate_to_poses_client_) {
    navigate_to_poses_client_->async_cancel_all_goals();
  }
  Q_EMIT goalStateChanged("Canceling");
  Q_EMIT eventReceived("Cancel requested");
}

void RosWorker::setGlobalCostmapSubscriptionEnabled(bool enabled)
{
  subscribe_global_costmap_ = enabled;
  last_global_costmap_emit_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  update_global_costmap_subscription();
  if (!enabled) {
    Q_EMIT eventReceived("Global costmap subscription disabled");
  }
}

void RosWorker::setLocalCostmapSubscriptionEnabled(bool enabled)
{
  subscribe_local_costmap_ = enabled;
  last_local_costmap_emit_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  update_local_costmap_subscription();
  if (!enabled) {
    Q_EMIT eventReceived("Local costmap subscription disabled");
  }
}

void RosWorker::setScanSubscriptionEnabled(bool enabled)
{
  subscribe_scan_ = enabled;
  update_scan_subscription();
  if (!enabled) {
    Q_EMIT scanChanged(ScanData{});
    Q_EMIT eventReceived("Scan subscription disabled");
  }
}

}  // namespace amr::visualization
