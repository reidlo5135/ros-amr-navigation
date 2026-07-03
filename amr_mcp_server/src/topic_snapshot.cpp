#include "amr_mcp_server/topic_snapshot.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace amr::mcp
{

void TopicSnapshot::updateBattery(
  const sensor_msgs::msg::BatteryState &message,
  const rclcpp::Time &now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.has_battery = message.present;
  data_.battery_percentage = message.percentage;
  if (std::isfinite(data_.battery_percentage) && data_.battery_percentage <= 1.0) {
    data_.battery_percentage *= 100.0;
  }
  data_.last_update_iso = isoTime(now);
}

void TopicSnapshot::updatePose(
  const geometry_msgs::msg::PoseStamped &message,
  const rclcpp::Time &now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.has_pose = true;
  data_.pose_x = message.pose.position.x;
  data_.pose_y = message.pose.position.y;
  data_.pose_yaw = yawFromQuaternion(
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z,
    message.pose.orientation.w);
  data_.last_update_iso = isoTime(now);
}

void TopicSnapshot::updateMotionStatus(
  const amr_msgs::msg::MotionStatus &message,
  const rclcpp::Time &now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.has_motion = true;
  data_.motion_active = message.active;
  data_.motion_blocked = message.blocked || message.costmap_blocked || message.safety_gate_blocked;
  data_.motion_stalled = message.stalled;
  data_.remaining_distance = message.remaining_distance;
  data_.heading_error = message.heading_error;
  data_.last_update_iso = isoTime(now);
}

void TopicSnapshot::updateRuntimeSummary(
  const std_msgs::msg::String &message,
  const rclcpp::Time &now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.has_runtime_summary = true;
  data_.runtime_state = extractJsonString(message.data, "runtime_state");
  data_.blocked_context = extractJsonString(message.data, "blocked_context");
  data_.recovery_phase = extractJsonString(message.data, "recovery_phase");
  data_.last_update_iso = isoTime(now);
}

void TopicSnapshot::updateRuntimeEvent(
  const std_msgs::msg::String &message,
  const rclcpp::Time &now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.last_event = message.data;
  data_.last_update_iso = isoTime(now);
}

SnapshotData TopicSnapshot::data() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return data_;
}

std::string TopicSnapshot::toJsonString() const
{
  const SnapshotData snapshot = data();
  std::ostringstream stream;
  stream << "{";
  stream << "\"has_battery\":" << (snapshot.has_battery ? "true" : "false") << ",";
  stream << "\"battery_percentage\":" << snapshot.battery_percentage << ",";
  stream << "\"has_pose\":" << (snapshot.has_pose ? "true" : "false") << ",";
  stream << "\"pose_x\":" << snapshot.pose_x << ",";
  stream << "\"pose_y\":" << snapshot.pose_y << ",";
  stream << "\"pose_yaw\":" << snapshot.pose_yaw << ",";
  stream << "\"has_motion\":" << (snapshot.has_motion ? "true" : "false") << ",";
  stream << "\"motion_active\":" << (snapshot.motion_active ? "true" : "false") << ",";
  stream << "\"motion_blocked\":" << (snapshot.motion_blocked ? "true" : "false") << ",";
  stream << "\"motion_stalled\":" << (snapshot.motion_stalled ? "true" : "false") << ",";
  stream << "\"remaining_distance\":" << snapshot.remaining_distance << ",";
  stream << "\"heading_error\":" << snapshot.heading_error << ",";
  stream << "\"runtime_state\":\"" << snapshot.runtime_state << "\",";
  stream << "\"blocked_context\":\"" << snapshot.blocked_context << "\",";
  stream << "\"recovery_phase\":\"" << snapshot.recovery_phase << "\",";
  stream << "\"last_update\":\"" << snapshot.last_update_iso << "\"";
  stream << "}";
  return stream.str();
}

std::string TopicSnapshot::toKoreanStatusText() const
{
  const SnapshotData snapshot = data();
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "현재 수신된 AMR 상태입니다.\n";
  if (snapshot.has_battery) {
    stream << "- 배터리: " << std::setprecision(0) << snapshot.battery_percentage << "%\n";
    stream << std::setprecision(2);
  } else {
    stream << "- 배터리: 수신된 데이터 없음\n";
  }
  if (snapshot.has_pose) {
    stream << "- 위치(map): x=" << snapshot.pose_x << ", y=" << snapshot.pose_y <<
      ", yaw=" << snapshot.pose_yaw << "\n";
  } else {
    stream << "- 위치: 수신된 데이터 없음\n";
  }
  if (snapshot.has_motion) {
    stream << "- 주행: " << (snapshot.motion_active ? "활성" : "대기") <<
      ", 남은 거리 " << snapshot.remaining_distance <<
      " m, heading error " << snapshot.heading_error << " rad\n";
    stream << "- 막힘/정체: " << (snapshot.motion_blocked ? "막힘 감지" : "clear") <<
      ", " << (snapshot.motion_stalled ? "stalled" : "not stalled") << "\n";
  } else {
    stream << "- 주행 상태: 수신된 데이터 없음\n";
  }
  if (snapshot.has_runtime_summary) {
    stream << "- 런타임: " <<
      (snapshot.runtime_state.empty() ? "unknown" : snapshot.runtime_state) <<
      ", blocked=" <<
      (snapshot.blocked_context.empty() ? "unknown" : snapshot.blocked_context) <<
      ", recovery=" <<
      (snapshot.recovery_phase.empty() ? "unknown" : snapshot.recovery_phase) << "\n";
  } else {
    stream << "- 런타임 요약: 수신된 데이터 없음\n";
  }
  if (!snapshot.last_event.empty()) {
    stream << "- 최근 이벤트: " << snapshot.last_event << "\n";
  }
  return stream.str();
}

double TopicSnapshot::yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string TopicSnapshot::isoTime(const rclcpp::Time &time)
{
  std::ostringstream stream;
  stream << time.seconds();
  return stream.str();
}

std::string TopicSnapshot::extractJsonString(const std::string &payload, const std::string &key)
{
  const std::string needle = "\"" + key + "\"";
  const auto key_pos = payload.find(needle);
  if (key_pos == std::string::npos) {
    return {};
  }
  const auto colon_pos = payload.find(':', key_pos + needle.size());
  if (colon_pos == std::string::npos) {
    return {};
  }
  const auto first_quote = payload.find('"', colon_pos + 1U);
  if (first_quote == std::string::npos) {
    return {};
  }
  const auto second_quote = payload.find('"', first_quote + 1U);
  if (second_quote == std::string::npos) {
    return {};
  }
  return payload.substr(first_quote + 1U, second_quote - first_quote - 1U);
}

}  // namespace amr::mcp
