#ifndef AMR_VISUALIZATION__ROS_WORKER_HPP_
#define AMR_VISUALIZATION__ROS_WORKER_HPP_

#include <QObject>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_poses.hpp"
#include "amr_msgs/msg/motion_status.hpp"
#include "amr_visualization/operator_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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

class RosWorker : public QObject
{
  Q_OBJECT

public:
  explicit RosWorker(QObject * parent = nullptr);
  ~RosWorker() override;

  void start();
  void stop();

public Q_SLOTS:
  void sendSingleGoal(const amr::visualization::Pose2D & pose);
  void sendRoute(const QVector<amr::visualization::Pose2D> & route);
  void publishInitialPose(const amr::visualization::Pose2D & pose);
  void cancelNavigation();
  void setGlobalCostmapSubscriptionEnabled(bool enabled);
  void setLocalCostmapSubscriptionEnabled(bool enabled);
  void setScanSubscriptionEnabled(bool enabled);

Q_SIGNALS:
  void connectionStateChanged(const QString & state);
  void mapChanged(const amr::visualization::GridMap & map);
  void tfFramesChanged(const QVector<amr::visualization::FrameVisual> & frames);
  void globalCostmapChanged(const amr::visualization::GridMap & map);
  void localCostmapChanged(const amr::visualization::GridMap & map);
  void scanChanged(const amr::visualization::ScanData & scan);
  void robotPoseChanged(const amr::visualization::Pose2D & pose);
  void globalPathChanged(const amr::visualization::PathData & path);
  void localPathChanged(const amr::visualization::PathData & path);
  void motionStatusChanged(const amr::visualization::MotionStatusData & status);
  void runtimeSummaryChanged(const amr::visualization::RuntimeSummary & summary);
  void batteryStateChanged(double percentage, bool present);
  void goalStateChanged(const QString & state);
  void navigationCompleted(bool succeeded);
  void eventReceived(const QString & event);

private:
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  using NavigateToPoses = amr_msgs::action::NavigateToPoses;

  void configure_ros_interfaces();
  void update_global_costmap_subscription();
  void update_local_costmap_subscription();
  void update_scan_subscription();
  void handle_tf_message(
    const tf2_msgs::msg::TFMessage & message,
    bool is_static);
  void spin();

  GridMap convert_grid(const nav_msgs::msg::OccupancyGrid & message) const;
  PathData convert_path(const nav_msgs::msg::Path & message) const;
  Pose2D convert_pose(const geometry_msgs::msg::PoseStamped & message) const;
  ScanData convert_scan(const sensor_msgs::msg::LaserScan & message) const;
  geometry_msgs::msg::PoseStamped to_pose_stamped(const Pose2D & pose) const;
  RuntimeSummary parse_runtime_summary(const std::string & payload) const;
  QVector<FrameVisual> build_frame_visuals() const;
  static Pose2D compose_pose(const Pose2D & parent, const Pose2D & child);
  Pose2D resolve_frame_pose(const std::string & child_frame) const;

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
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<NavigateToPoses>::SharedPtr navigate_to_poses_client_;
  rclcpp::Time last_global_costmap_emit_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_costmap_emit_time_{0, 0, RCL_ROS_TIME};

  std::string default_frame_id_{"map"};
  std::string map_topic_{"/amr/map/data"};
  std::string global_costmap_topic_{"/amr/costmap/global"};
  std::string local_costmap_topic_{"/amr/costmap/local"};
  std::string pose_topic_{"/amr/localization/pose"};
  std::string initial_pose_topic_{"/amr/localization/initial_pose"};
  std::string global_path_topic_{"/amr/planner/global"};
  std::string local_path_topic_{"/amr/planner/local"};
  std::string motion_status_topic_{"/amr/motion/status"};
  std::string runtime_summary_topic_{"/amr/observation/runtime/summary"};
  std::string runtime_event_topic_{"/amr/observation/runtime/events"};
  std::string battery_state_topic_{"/battery_state"};
  std::string scan_topic_{"/scan"};
  std::string tf_topic_{"/tf"};
  std::string tf_static_topic_{"/tf_static"};
  std::string navigate_to_pose_action_{"/amr/navigator/navigate_to_pose"};
  std::string navigate_to_poses_action_{"/amr/navigator/navigate_to_poses"};
  bool subscribe_global_costmap_{false};
  bool subscribe_local_costmap_{false};
  bool subscribe_scan_{true};
  int costmap_emit_period_ms_{1000};
  std::map<std::string, FrameVisual> dynamic_frames_;
  std::map<std::string, FrameVisual> static_frames_;
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__ROS_WORKER_HPP_
