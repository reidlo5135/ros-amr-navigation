#ifndef AMR_MCP_SERVER__TOPIC_SNAPSHOT_HPP_
#define AMR_MCP_SERVER__TOPIC_SNAPSHOT_HPP_

#include <mutex>
#include <string>

#include "amr_msgs/msg/motion_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr::mcp
{

struct SnapshotData
{
  bool has_battery{false};
  double battery_percentage{-1.0};
  bool has_pose{false};
  double pose_x{0.0};
  double pose_y{0.0};
  double pose_yaw{0.0};
  bool has_motion{false};
  bool motion_active{false};
  bool motion_blocked{false};
  bool motion_stalled{false};
  double remaining_distance{0.0};
  double heading_error{0.0};
  bool has_runtime_summary{false};
  std::string runtime_state;
  std::string blocked_context;
  std::string recovery_phase;
  std::string last_event;
  std::string last_update_iso;
};

class TopicSnapshot
{
public:
  void updateBattery(const sensor_msgs::msg::BatteryState &message, const rclcpp::Time &now);
  void updatePose(const geometry_msgs::msg::PoseStamped &message, const rclcpp::Time &now);
  void updateMotionStatus(const amr_msgs::msg::MotionStatus &message, const rclcpp::Time &now);
  void updateRuntimeSummary(const std_msgs::msg::String &message, const rclcpp::Time &now);
  void updateRuntimeEvent(const std_msgs::msg::String &message, const rclcpp::Time &now);

  SnapshotData data() const;
  std::string toJsonString() const;
  std::string toKoreanStatusText() const;

private:
  static double yawFromQuaternion(double x, double y, double z, double w);
  static std::string isoTime(const rclcpp::Time &time);
  static std::string extractJsonString(const std::string &payload, const std::string &key);

  mutable std::mutex mutex_;
  SnapshotData data_;
};

}  // namespace amr::mcp

#endif  // AMR_MCP_SERVER__TOPIC_SNAPSHOT_HPP_
