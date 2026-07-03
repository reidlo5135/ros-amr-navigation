#ifndef AMR_VISUALIZATION__ROS_WORKER_HPP_
#define AMR_VISUALIZATION__ROS_WORKER_HPP_

/**
 * @file ros_worker.hpp
 * @brief Qt object that owns ROS subscriptions, action clients, and data conversion for the UI.
 */

#include <QObject>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_msgs/srv/ai_chat.hpp"
#include "amr_visualization/operator_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace amr::visualization
{

/// @brief Runs ROS spinning in a worker thread and emits Qt-friendly visualization data.
class RosWorker : public QObject
{
  Q_OBJECT

public:
  /// @brief Construct the worker without starting ROS spinning.
  explicit RosWorker(QObject *parent = nullptr);
  /// @brief Stop spinning and destroy the worker.
  ~RosWorker() override;

  /// @brief Configure ROS interfaces and start the executor thread.
  void start();
  /// @brief Stop the executor thread and release runtime activity.
  void stop();

public Q_SLOTS:
  /// @brief Send a NavigateToPose goal from a UI pose.
  void sendSingleGoal(const amr::visualization::Pose2D &pose);
  /// @brief Send a NavigateToPoses route from UI waypoints.
  void sendRoute(const QVector<amr::visualization::Pose2D> &route);
  /// @brief Publish an initial pose message.
  void publishInitialPose(const amr::visualization::Pose2D &pose);
  /// @brief Cancel active navigation action goals.
  void cancelNavigation();
  /// @brief Publish a velocity command to the configured cmd_vel topic.
  void publishVelocityCommand(double linear_x, double angular_z);
  /// @brief Publish one zero velocity command to stop manual motion.
  void publishStopCommand();
  /// @brief Send an AI chat request to the AMR MCP server.
  void sendAiChatRequest(
    const QString &provider,
    const QString &robot_id,
    const QString &default_frame,
    const QString &message);
  /// @brief Recreate the AI chat service client for a new service name.
  void setAiChatServiceName(const QString &service_name);
  /// @brief Enable or disable the global costmap subscription.
  void setGlobalCostmapSubscriptionEnabled(bool enabled);
  /// @brief Enable or disable the local costmap subscription.
  void setLocalCostmapSubscriptionEnabled(bool enabled);
  /// @brief Enable or disable the laser scan subscription.
  void setScanSubscriptionEnabled(bool enabled);

Q_SIGNALS:
  /// @brief Emitted when ROS connection/spin state changes.
  void connectionStateChanged(const QString &state);
  /// @brief Emitted when a map update is available.
  void mapChanged(const amr::visualization::GridMap &map);
  /// @brief Emitted when TF frame visuals change.
  void tfFramesChanged(const QVector<amr::visualization::FrameVisual> &frames);
  /// @brief Emitted when robot model visuals change.
  void robotModelChanged(const QVector<amr::visualization::RobotVisual> &visuals);
  /// @brief Emitted when global costmap data changes.
  void globalCostmapChanged(const amr::visualization::GridMap &map);
  /// @brief Emitted when local costmap data changes.
  void localCostmapChanged(const amr::visualization::GridMap &map);
  /// @brief Emitted when scan points change.
  void scanChanged(const amr::visualization::ScanData &scan);
  /// @brief Emitted when robot pose changes.
  void robotPoseChanged(const amr::visualization::Pose2D &pose);
  /// @brief Emitted when the global path changes.
  void globalPathChanged(const amr::visualization::PathData &path);
  /// @brief Emitted when the local path changes.
  void localPathChanged(const amr::visualization::PathData &path);
  /// @brief Emitted when motion status changes.
  void motionStatusChanged(const amr::visualization::MotionStatusData &status);
  /// @brief Emitted when runtime observation summary changes.
  void runtimeSummaryChanged(const amr::visualization::RuntimeSummary &summary);
  /// @brief Emitted when battery percentage or presence changes.
  void batteryStateChanged(double percentage, bool present);
  /// @brief Emitted when action goal state changes.
  void goalStateChanged(const QString &state);
  /// @brief Emitted when navigation completes.
  void navigationCompleted(bool succeeded);
  /// @brief Emitted when a runtime event line is received.
  void eventReceived(const QString &event);
  /// @brief Emitted after joystick-related ROS parameters are loaded.
  void joystickConfigurationChanged(
    double max_linear_speed,
    double max_angular_speed,
    double publish_rate_hz);
  /// @brief Emitted when the MCP chat service availability changes.
  void aiChatServiceAvailabilityChanged(bool available);
  /// @brief Emitted when an MCP chat response is received.
  void aiChatResponseReceived(
    bool accepted,
    bool command_executed,
    const QString &command_type,
    const QString &response,
    const QString &request_id,
    const QString &error_message);
  /// @brief Emitted when one asynchronous MCP feedback line is received.
  void mcpFeedbackReceived(const QString &message);

private:
  /// @brief Parsed fixed joint from robot_description.
  struct RobotJoint
  {
    /// @brief Parent link frame.
    QString parent_frame;
    /// @brief Child link frame.
    QString child_frame;
    /// @brief Joint origin transform.
    Pose2D origin;
    /// @brief True when the joint was parsed successfully.
    bool valid{false};
  };

  /// @brief NavigateToPose action alias.
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  /// @brief NavigateToPoses action alias.
  using NavigateToPoses = amr_msgs::action::NavigateToPoses;
  /// @brief AI chat service alias.
  using AiChat = amr_msgs::srv::AiChat;

  /// @brief Create subscriptions, publishers, and action clients.
  void configure_ros_interfaces();
  /// @brief Recreate or drop the global costmap subscription based on UI state.
  void update_global_costmap_subscription();
  /// @brief Recreate or drop the local costmap subscription based on UI state.
  void update_local_costmap_subscription();
  /// @brief Recreate or drop the scan subscription based on UI state.
  void update_scan_subscription();
  /// @brief Cache transforms from a TF or TF static message.
  void handle_tf_message(
    const tf2_msgs::msg::TFMessage &message,
    bool is_static);
  /// @brief Spin the ROS executor on the worker thread.
  void spin();

  /// @brief Convert an occupancy grid into a Qt-friendly grid model.
  GridMap convert_grid(const nav_msgs::msg::OccupancyGrid &message) const;
  /// @brief Convert a nav_msgs path into Qt-friendly path points.
  PathData convert_path(const nav_msgs::msg::Path &message) const;
  /// @brief Convert a stamped pose into a UI pose.
  Pose2D convert_pose(const geometry_msgs::msg::PoseStamped &message) const;
  /// @brief Convert a laser scan into projected UI points.
  ScanData convert_scan(const sensor_msgs::msg::LaserScan &message) const;
  /// @brief Convert a UI pose into a stamped ROS pose.
  geometry_msgs::msg::PoseStamped to_pose_stamped(const Pose2D &pose) const;
  /// @brief Parse runtime observation summary JSON into UI state.
  RuntimeSummary parse_runtime_summary(const std::string &payload) const;
  /// @brief Parse robot_description XML into visual elements and joints.
  QVector<RobotVisual> parse_robot_description(const std::string &payload);
  /// @brief Resolve parsed robot visuals into the fixed frame.
  QVector<RobotVisual> build_robot_visuals() const;
  /// @brief Resolve a robot link pose from TF and fixed joints.
  Pose2D resolve_robot_link_pose(const QString &link_frame) const;
  /// @brief Resolve the primary robot base pose from TF when /pose is unavailable.
  Pose2D resolve_primary_robot_pose() const;
  /// @brief Build frame visuals from cached dynamic and static transforms.
  QVector<FrameVisual> build_frame_visuals() const;
  /// @brief Compose two planar poses.
  static Pose2D compose_pose(const Pose2D &parent, const Pose2D &child);
  /// @brief Resolve an arbitrary child frame pose in the fixed frame.
  Pose2D resolve_frame_pose(const std::string &child_frame) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
  std::atomic_bool running_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<amr_msgs::msg::MotionStatus>::SharedPtr motion_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr runtime_summary_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr runtime_event_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mcp_feedback_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Client<AiChat>::SharedPtr ai_chat_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<NavigateToPoses>::SharedPtr navigate_to_poses_client_;
  rclcpp::Time last_global_costmap_emit_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_costmap_emit_time_{0, 0, RCL_ROS_TIME};

  std::string default_frame_id_{"map"};
  std::string map_topic_{"/map"};
  std::string global_costmap_topic_{"/global_costmap"};
  std::string local_costmap_topic_{"/local_costmap"};
  std::string pose_topic_{"/pose"};
  std::string initial_pose_topic_{"/initialpose"};
  std::string global_path_topic_{"/global_plan"};
  std::string local_path_topic_{"/local_plan"};
  std::string motion_status_topic_{"/motion_status"};
  std::string runtime_summary_topic_{"/observation/runtime/summary"};
  std::string runtime_event_topic_{"/observation/runtime/events"};
  std::string mcp_feedback_topic_{"/amr_mcp/feedback"};
  std::string battery_state_topic_{"/battery_state"};
  std::string robot_description_topic_{"/robot_description"};
  std::string scan_topic_{"/scan"};
  std::string tf_topic_{"/tf"};
  std::string tf_static_topic_{"/tf_static"};
  std::string navigate_to_pose_action_{"/navigate_to_pose"};
  std::string navigate_to_poses_action_{"/navigate_to_poses"};
  std::string ai_chat_service_name_{"/amr_mcp/chat"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  double max_linear_speed_{0.22};
  double max_angular_speed_{1.8};
  double joystick_publish_rate_hz_{20.0};
  bool subscribe_global_costmap_{true};
  bool subscribe_local_costmap_{true};
  bool subscribe_scan_{true};
  int costmap_emit_period_ms_{1000};
  std::map<std::string, FrameVisual> dynamic_frames_;
  std::map<std::string, FrameVisual> static_frames_;
  QVector<RobotVisual> robot_description_visuals_;
  std::map<std::string, RobotJoint> robot_joints_;
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__ROS_WORKER_HPP_
