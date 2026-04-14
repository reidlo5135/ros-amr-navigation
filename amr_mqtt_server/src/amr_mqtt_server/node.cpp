#include "amr_mqtt_server/node.hpp"

namespace amr::mqtt::server
{

namespace
{

using namespace std::chrono_literals;

constexpr size_t k_max_string_length = 512;
constexpr uint64_t k_max_processed_mqtt_messages = 8U;

using NavigateToPoses = amr_msgs::action::NavigateToPoses;
using GoalHandleNavigateToPoses = rclcpp_action::ClientGoalHandle<NavigateToPoses>;

template<typename... Args>
void appendf(std::string & out, const char * format, Args... args)
{
  int length = std::snprintf(nullptr, 0, format, args...);
  if (length <= 0) {
    return;
  }
  std::string buffer(static_cast<size_t>(length), '\0');
  std::snprintf(buffer.data(), buffer.size() + 1U, format, args...);
  out += buffer;
}

std::string escape_json(std::string_view text)
{
  std::string out;
  out.reserve(text.size() + 8U);
  out.push_back('"');
  for (char current : text) {
    switch (current) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(current);
        break;
    }
  }
  out.push_back('"');
  return out;
}

uint64_t now_ms()
{
  struct timespec timestamp;
  if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
    return 0U;
  }
  return (static_cast<uint64_t>(timestamp.tv_sec) * 1000ULL) +
         (static_cast<uint64_t>(timestamp.tv_nsec) / 1000000ULL);
}

bool is_valid_robot_id(const std::string & robot_id)
{
  if (robot_id.empty()) {
    return false;
  }
  for (unsigned char ch : robot_id) {
    if ((ch >= 'a' && ch <= 'z') ||
        (ch >= 'A' && ch <= 'Z') ||
        (ch >= '0' && ch <= '9') ||
        ch == '_' || ch == '-') {
      continue;
    }
    return false;
  }
  return true;
}

std::string normalize_root(const std::string & root)
{
  if (root.empty()) {
    return "/amr";
  }
  std::string normalized = root.front() == '/' ? root : "/" + root;
  while (normalized.size() > 1U && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

std::string scoped_topic(
  const std::string & root,
  const std::string & robot_id,
  const std::string & suffix)
{
  return normalize_root(root) + "/" + robot_id + "/" + suffix;
}

std::string build_viz_topic(const std::string & raw_topic)
{
  const std::string marker = "/telemetry/";
  const std::string::size_type position = raw_topic.find(marker);
  if (position == std::string::npos) {
    return {};
  }
  return raw_topic.substr(0U, position) + "/viz/" + raw_topic.substr(position + marker.size());
}

double quaternion_to_yaw(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

void append_header(std::string & out, const std_msgs::msg::Header & header)
{
  appendf(
    out,
    "\"header\":{\"stamp\":{\"sec\":%d,\"nanosec\":%u},\"frame_id\":%s}",
    static_cast<int>(header.stamp.sec),
    header.stamp.nanosec,
    escape_json(header.frame_id).c_str());
}

void append_frame(std::string & out, const std::string & frame_id)
{
  if (frame_id.empty()) {
    return;
  }
  out += "\"frame\":";
  out += escape_json(frame_id);
}

void append_pose_fields(std::string & out, const geometry_msgs::msg::Pose & pose)
{
  const double yaw = quaternion_to_yaw(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  appendf(
    out,
    "\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}",
    pose.position.x,
    pose.position.y,
    pose.position.z,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w,
    yaw);
}

void append_pose_stamped(std::string & out, const geometry_msgs::msg::PoseStamped & pose)
{
  out += "{";
  if (!pose.header.frame_id.empty()) {
    append_frame(out, pose.header.frame_id);
    out += ",";
  }
  append_pose_fields(out, pose.pose);
  out += "}";
}

std::string serialize_pose_stamped(const geometry_msgs::msg::PoseStamped & pose)
{
  std::string out;
  out.reserve(256U);
  append_pose_stamped(out, pose);
  return out;
}

std::string serialize_path(const nav_msgs::msg::Path & path)
{
  std::string out;
  out.reserve(1024U);
  out += "{";
  if (!path.header.frame_id.empty()) {
    append_frame(out, path.header.frame_id);
    out += ",";
  }
  appendf(out, "\"pose_count\":%zu,\"poses\":[", path.poses.size());
  for (size_t index = 0; index < path.poses.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    append_pose_stamped(out, path.poses[index]);
  }
  out += "]}";
  return out;
}

std::string serialize_occupancy_grid(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::string out;
  out.reserve(2048U);
  const double origin_yaw = quaternion_to_yaw(
    grid.info.origin.orientation.x,
    grid.info.origin.orientation.y,
    grid.info.origin.orientation.z,
    grid.info.origin.orientation.w);
  out += "{";
  if (!grid.header.frame_id.empty()) {
    append_frame(out, grid.header.frame_id);
    out += ",";
  }
  appendf(
    out,
    "\"info\":{\"width\":%u,\"height\":%u,\"resolution\":%.6f,"
    "\"origin\":{\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}}},\"data\":[",
    grid.info.width,
    grid.info.height,
    grid.info.resolution,
    grid.info.origin.position.x,
    grid.info.origin.position.y,
    grid.info.origin.position.z,
    grid.info.origin.orientation.x,
    grid.info.origin.orientation.y,
    grid.info.origin.orientation.z,
    grid.info.origin.orientation.w,
    origin_yaw);
  for (size_t index = 0; index < grid.data.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(out, "%d", static_cast<int>(grid.data[index]));
  }
  out += "]}";
  return out;
}

std::string serialize_motion_status(const amr_msgs::msg::MotionStatus & status)
{
  std::string out;
  out.reserve(1024U);
  out += "{";
  if (!status.header.frame_id.empty()) {
    append_frame(out, status.header.frame_id);
    out += ",";
  }
  appendf(
    out,
    "\"command_id\":%u,\"active\":%s,\"goal_reached\":%s,\"obstacle_detected\":%s,"
    "\"blocked\":%s,\"stalled\":%s,\"local_plan_valid\":%s,\"costmap_blocked\":%s,"
    "\"safety_gate_blocked\":%s,\"has_blocked_pose\":%s,"
    "\"remaining_distance\":%.6f,\"heading_error\":%.6f,\"current_pose\":",
    status.command_id,
    status.active ? "true" : "false",
    status.goal_reached ? "true" : "false",
    status.obstacle_detected ? "true" : "false",
    status.blocked ? "true" : "false",
    status.stalled ? "true" : "false",
    status.local_plan_valid ? "true" : "false",
    status.costmap_blocked ? "true" : "false",
    status.safety_gate_blocked ? "true" : "false",
    status.has_blocked_pose ? "true" : "false",
    status.remaining_distance,
    status.heading_error);
  append_pose_stamped(out, status.current_pose);
  out += ",\"blocked_pose\":";
  append_pose_stamped(out, status.blocked_pose);
  out += "}";
  return out;
}

std::string serialize_scan(const sensor_msgs::msg::LaserScan & scan)
{
  std::string out;
  out.reserve(1024U);
  out += "{";
  if (!scan.header.frame_id.empty()) {
    append_frame(out, scan.header.frame_id);
    out += ",";
  }
  appendf(
    out,
    "\"angle_min\":%.6f,\"angle_max\":%.6f,\"angle_increment\":%.6f,"
    "\"range_min\":%.6f,\"range_max\":%.6f,\"ranges_count\":%zu,\"ranges\":[",
    scan.angle_min,
    scan.angle_max,
    scan.angle_increment,
    scan.range_min,
    scan.range_max,
    scan.ranges.size());
  for (size_t index = 0; index < scan.ranges.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    if (!std::isfinite(scan.ranges[index])) {
      out += "null";
    } else {
      appendf(out, "%.6f", scan.ranges[index]);
    }
  }
  out += "]}";
  return out;
}

std::string serialize_odom(const nav_msgs::msg::Odometry & odom)
{
  std::string out;
  out.reserve(512U);
  out += "{";
  if (!odom.header.frame_id.empty()) {
    append_frame(out, odom.header.frame_id);
    out += ",";
  }
  out += "\"child_frame\":";
  out += escape_json(odom.child_frame_id);
  appendf(
    out,
    ",\"pose\":{\"position\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f}},"
    "\"twist\":{\"linear\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"angular\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}}}",
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z,
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w,
    odom.twist.twist.linear.x,
    odom.twist.twist.linear.y,
    odom.twist.twist.linear.z,
    odom.twist.twist.angular.x,
    odom.twist.twist.angular.y,
    odom.twist.twist.angular.z);
  return out;
}

std::string serialize_imu(const sensor_msgs::msg::Imu & imu)
{
  std::string out;
  out.reserve(512U);
  out += "{";
  if (!imu.header.frame_id.empty()) {
    append_frame(out, imu.header.frame_id);
    out += ",";
  }
  appendf(
    out,
    "\"orientation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f},"
    "\"angular_velocity\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"linear_acceleration\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}}",
    imu.orientation.x,
    imu.orientation.y,
    imu.orientation.z,
    imu.orientation.w,
    imu.angular_velocity.x,
    imu.angular_velocity.y,
    imu.angular_velocity.z,
    imu.linear_acceleration.x,
    imu.linear_acceleration.y,
    imu.linear_acceleration.z);
  return out;
}

void append_double_array(std::string & out, const std::vector<double> & values)
{
  out += "[";
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(out, "%.6f", values[index]);
  }
  out += "]";
}

void append_string_array(std::string & out, const std::vector<std::string> & values)
{
  out += "[";
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    out += escape_json(values[index]);
  }
  out += "]";
}

std::string serialize_joint_states(const sensor_msgs::msg::JointState & joint_states)
{
  std::string out;
  out.reserve(768U);
  out += "{";
  if (!joint_states.header.frame_id.empty()) {
    append_frame(out, joint_states.header.frame_id);
    out += ",";
  }
  out += "\"name\":";
  append_string_array(out, joint_states.name);
  out += ",\"position\":[";
  for (size_t index = 0; index < joint_states.position.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(out, "%.6f", joint_states.position[index]);
  }
  out += "],\"velocity\":[";
  for (size_t index = 0; index < joint_states.velocity.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(out, "%.6f", joint_states.velocity[index]);
  }
  out += "],\"effort\":[";
  for (size_t index = 0; index < joint_states.effort.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(out, "%.6f", joint_states.effort[index]);
  }
  out += "]}";
  return out;
}

void append_transform_stamped(std::string & out, const geometry_msgs::msg::TransformStamped & transform)
{
  const double yaw = quaternion_to_yaw(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  out += "{";
  append_header(out, transform.header);
  out += ",\"child_frame_id\":";
  out += escape_json(transform.child_frame_id);
  appendf(
    out,
    ",\"translation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
    "\"rotation\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"w\":%.6f,\"yaw\":%.6f}}",
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z,
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w,
    yaw);
}

std::string serialize_tf_message(const tf2_msgs::msg::TFMessage & tf_message)
{
  std::string out;
  out.reserve(1024U);
  out += "{\"transforms\":[";
  for (size_t index = 0; index < tf_message.transforms.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    append_transform_stamped(out, tf_message.transforms[index]);
  }
  out += "]}";
  return out;
}

std::string serialize_string_message(
  const std_msgs::msg::String & message,
  const std::vector<double> & footprint_polygon)
{
  std::string out = "{\"data\":";
  out += escape_json(message.data);
  if (!footprint_polygon.empty()) {
    out += ",\"footprint_polygon\":";
    append_double_array(out, footprint_polygon);
  }
  out += "}";
  return out;
}

std::string serialize_string_json_message(const std_msgs::msg::String & message)
{
  return message.data;
}

std::string serialize_battery_state(const sensor_msgs::msg::BatteryState & battery_state)
{
  std::string out;
  out.reserve(512U);
  double percentage = -1.0;
  if (std::isfinite(battery_state.percentage)) {
    percentage = battery_state.percentage <= 1.0 ? battery_state.percentage * 100.0 : battery_state.percentage;
  }
  out += "{";
  if (!battery_state.header.frame_id.empty()) {
    append_frame(out, battery_state.header.frame_id);
    out += ",";
  }
  appendf(out, "\"voltage\":%.3f", battery_state.voltage);
  appendf(out, ",\"current\":%.3f", battery_state.current);
  appendf(out, ",\"percentage\":%.2f", percentage);
  appendf(out, ",\"power_supply_status\":%u", battery_state.power_supply_status);
  appendf(out, ",\"power_supply_health\":%u", battery_state.power_supply_health);
  appendf(out, ",\"power_supply_technology\":%u", battery_state.power_supply_technology);
  appendf(out, ",\"present\":%s}", battery_state.present ? "true" : "false");
  return out;
}

std::string uuid_to_hex(const std::array<uint8_t, 16> & uuid)
{
  static const char hex_chars[] = "0123456789abcdef";
  std::string out;
  out.reserve(uuid.size() * 2U);
  for (uint8_t value : uuid) {
    out.push_back(hex_chars[(value >> 4U) & 0x0FU]);
    out.push_back(hex_chars[value & 0x0FU]);
  }
  return out;
}

std::string serialize_navigation_feedback(
  const std::string & goal_id_hex,
  const NavigateToPoses::Feedback & feedback)
{
  std::string out;
  out.reserve(512U);
  appendf(
    out,
    "{\"goal_id\":\"%s\",\"current_goal_index\":%u,\"goal_count\":%u,"
    "\"distance_remaining\":%.6f,\"number_of_recoveries\":%d,"
    "\"navigation_time\":{\"sec\":%d,\"nanosec\":%u},"
    "\"estimated_time_remaining\":{\"sec\":%d,\"nanosec\":%u},"
    "\"current_pose\":",
    goal_id_hex.c_str(),
    feedback.current_goal_index,
    feedback.goal_count,
    static_cast<double>(feedback.distance_remaining),
    feedback.number_of_recoveries,
    static_cast<int>(feedback.navigation_time.sec),
    feedback.navigation_time.nanosec,
    static_cast<int>(feedback.estimated_time_remaining.sec),
    feedback.estimated_time_remaining.nanosec);
  append_pose_stamped(out, feedback.current_pose);
  out += "}";
  return out;
}

std::string serialize_navigation_status(const action_msgs::msg::GoalStatusArray & status_array)
{
  std::string out = "{\"status_list\":[";
  for (size_t index = 0; index < status_array.status_list.size(); ++index) {
    if (index > 0U) {
      out += ",";
    }
    appendf(
      out,
      "{\"goal_id\":\"%s\",\"status\":%d}",
      uuid_to_hex(status_array.status_list[index].goal_info.goal_id.uuid).c_str(),
      static_cast<int>(status_array.status_list[index].status));
  }
  out += "]}";
  return out;
}

std::string serialize_simple_response(
  const std::string & request_id,
  bool success,
  const std::string & message)
{
  std::string out = "{\"request_id\":";
  out += escape_json(request_id);
  appendf(out, ",\"success\":%s,\"message\":", success ? "true" : "false");
  out += escape_json(message);
  out += "}";
  return out;
}

std::string serialize_ping_response(
  const std::string & request_id,
  bool success,
  double sent_at_ms,
  uint64_t bridge_time_ms,
  const std::string & message)
{
  std::string out = "{\"request_id\":";
  out += escape_json(request_id);
  appendf(
    out,
    ",\"success\":%s,\"sent_at_ms\":%.3f,\"bridge_time_ms\":%llu,\"message\":",
    success ? "true" : "false",
    sent_at_ms,
    static_cast<unsigned long long>(bridge_time_ms));
  out += escape_json(message);
  out += "}";
  return out;
}

std::string serialize_navigation_result(
  const std::string & request_id,
  bool success,
  int status_code,
  bool accepted,
  bool completed,
  uint32_t completed_goals,
  const std::string & message)
{
  std::string out = "{\"request_id\":";
  out += escape_json(request_id);
  appendf(
    out,
    ",\"success\":%s,\"accepted\":%s,\"completed\":%s,"
    "\"status_code\":%d,\"completed_goals\":%u,\"message\":",
    success ? "true" : "false",
    accepted ? "true" : "false",
    completed ? "true" : "false",
    status_code,
    completed_goals);
  out += escape_json(message);
  out += "}";
  return out;
}

const char * skip_ws(const char * cursor, const char * end)
{
  while (cursor < end && std::isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  return cursor;
}

const char * find_key(const char * begin, const char * end, const char * key)
{
  std::string pattern = "\"";
  pattern += key;
  pattern += "\"";
  for (const char * cursor = begin; cursor + static_cast<ptrdiff_t>(pattern.size()) <= end; ++cursor) {
    if (std::memcmp(cursor, pattern.data(), pattern.size()) == 0) {
      return cursor + static_cast<ptrdiff_t>(pattern.size());
    }
  }
  return nullptr;
}

bool extract_json_string_in_range(
  const char * begin,
  const char * end,
  const char * key,
  std::string & output)
{
  const char * cursor = find_key(begin, end, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor = skip_ws(cursor, end);
  if (cursor >= end || *cursor != ':') {
    return false;
  }
  cursor = skip_ws(cursor + 1, end);
  if (cursor >= end || *cursor != '"') {
    return false;
  }
  ++cursor;
  output.clear();
  while (cursor < end && *cursor != '"') {
    if (*cursor == '\\' && cursor + 1 < end) {
      ++cursor;
    }
    output.push_back(*cursor);
    ++cursor;
  }
  return cursor < end;
}

bool extract_json_double_in_range(
  const char * begin,
  const char * end,
  const char * key,
  double & output)
{
  const char * cursor = find_key(begin, end, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor = skip_ws(cursor, end);
  if (cursor >= end || *cursor != ':') {
    return false;
  }
  cursor = skip_ws(cursor + 1, end);
  char * parsed_end = nullptr;
  output = std::strtod(cursor, &parsed_end);
  return parsed_end != cursor;
}

bool extract_json_object_in_range(
  const char * begin,
  const char * end,
  const char * key,
  const char *& object_begin,
  const char *& object_end)
{
  const char * cursor = find_key(begin, end, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor = skip_ws(cursor, end);
  if (cursor >= end || *cursor != ':') {
    return false;
  }
  cursor = skip_ws(cursor + 1, end);
  if (cursor >= end || *cursor != '{') {
    return false;
  }
  object_begin = cursor;
  int depth = 0;
  while (cursor < end) {
    if (*cursor == '{') {
      ++depth;
    } else if (*cursor == '}') {
      --depth;
      if (depth == 0) {
        object_end = cursor;
        return true;
      }
    }
    ++cursor;
  }
  return false;
}

bool extract_json_array_in_range(
  const char * begin,
  const char * end,
  const char * key,
  const char *& array_begin,
  const char *& array_end)
{
  const char * cursor = find_key(begin, end, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor = skip_ws(cursor, end);
  if (cursor >= end || *cursor != ':') {
    return false;
  }
  cursor = skip_ws(cursor + 1, end);
  if (cursor >= end || *cursor != '[') {
    return false;
  }
  array_begin = cursor;
  int depth = 0;
  while (cursor < end) {
    if (*cursor == '[') {
      ++depth;
    } else if (*cursor == ']') {
      --depth;
      if (depth == 0) {
        array_end = cursor;
        return true;
      }
    }
    ++cursor;
  }
  return false;
}

bool parse_pose_stamped_in_range(
  const char * begin,
  const char * end,
  geometry_msgs::msg::PoseStamped & pose)
{
  const char * header_begin = nullptr;
  const char * header_end = nullptr;
  const char * position_begin = nullptr;
  const char * position_end = nullptr;
  const char * orientation_begin = nullptr;
  const char * orientation_end = nullptr;
  const char * pose_begin = nullptr;
  const char * pose_end = nullptr;
  std::string frame_id;

  pose = geometry_msgs::msg::PoseStamped();

  if (extract_json_object_in_range(begin, end, "header", header_begin, header_end)) {
    if (extract_json_string_in_range(header_begin, header_end + 1, "frame_id", frame_id)) {
      pose.header.frame_id = frame_id;
    }
  }
  if (pose.header.frame_id.empty()) {
    if (extract_json_string_in_range(begin, end, "frame", frame_id) ||
        extract_json_string_in_range(begin, end, "frame_id", frame_id)) {
      pose.header.frame_id = frame_id;
    }
  }

  if (!extract_json_object_in_range(begin, end, "position", position_begin, position_end)) {
    if (!extract_json_object_in_range(begin, end, "pose", pose_begin, pose_end) ||
        !extract_json_object_in_range(pose_begin, pose_end + 1, "position", position_begin, position_end)) {
      return false;
    }
    if (!extract_json_object_in_range(pose_begin, pose_end + 1, "orientation", orientation_begin, orientation_end)) {
      return false;
    }
  } else if (!extract_json_object_in_range(begin, end, "orientation", orientation_begin, orientation_end)) {
    return false;
  }

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double ox = 0.0;
  double oy = 0.0;
  double oz = 0.0;
  double ow = 1.0;

  if (!extract_json_double_in_range(position_begin, position_end + 1, "x", x) ||
      !extract_json_double_in_range(position_begin, position_end + 1, "y", y) ||
      !extract_json_double_in_range(position_begin, position_end + 1, "z", z) ||
      !extract_json_double_in_range(orientation_begin, orientation_end + 1, "x", ox) ||
      !extract_json_double_in_range(orientation_begin, orientation_end + 1, "y", oy) ||
      !extract_json_double_in_range(orientation_begin, orientation_end + 1, "z", oz) ||
      !extract_json_double_in_range(orientation_begin, orientation_end + 1, "w", ow)) {
    return false;
  }

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = ox;
  pose.pose.orientation.y = oy;
  pose.pose.orientation.z = oz;
  pose.pose.orientation.w = ow;
  return true;
}

bool parse_waypoints_array(
  const char * begin,
  const char * end,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  const char * cursor = begin;
  waypoints.clear();
  if (cursor == nullptr || end == nullptr || *cursor != '[') {
    return false;
  }
  ++cursor;
  while (cursor < end) {
    cursor = skip_ws(cursor, end);
    if (cursor >= end || *cursor == ']') {
      break;
    }
    if (*cursor != '{') {
      ++cursor;
      continue;
    }
    int depth = 0;
    const char * object_end = cursor;
    while (object_end <= end) {
      if (*object_end == '{') {
        ++depth;
      } else if (*object_end == '}') {
        --depth;
        if (depth == 0) {
          geometry_msgs::msg::PoseStamped pose;
          if (!parse_pose_stamped_in_range(cursor, object_end + 1, pose)) {
            return false;
          }
          waypoints.push_back(pose);
          cursor = object_end + 1;
          break;
        }
      }
      ++object_end;
    }
    if (depth != 0) {
      return false;
    }
  }
  return true;
}

bool deserialize_twist_raw(
  const void * payload,
  size_t payload_length,
  geometry_msgs::msg::Twist & twist)
{
  rclcpp::SerializedMessage serialized(payload_length);
  rcl_serialized_message_t & rmw = serialized.get_rcl_serialized_message();
  std::memcpy(rmw.buffer, payload, payload_length);
  rmw.buffer_length = payload_length;
  rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
  try {
    serializer.deserialize_message(&serialized, &twist);
    return true;
  } catch (...) {
    return false;
  }
}

bool extract_twist_from_json(const std::string & payload, geometry_msgs::msg::Twist & twist)
{
  const char * begin = payload.c_str();
  const char * end = begin + payload.size();
  const char * linear_begin = nullptr;
  const char * linear_end = nullptr;
  const char * angular_begin = nullptr;
  const char * angular_end = nullptr;
  twist = geometry_msgs::msg::Twist();
  if (extract_json_object_in_range(begin, end, "linear", linear_begin, linear_end)) {
    (void)extract_json_double_in_range(linear_begin, linear_end + 1, "x", twist.linear.x);
    (void)extract_json_double_in_range(linear_begin, linear_end + 1, "y", twist.linear.y);
    (void)extract_json_double_in_range(linear_begin, linear_end + 1, "z", twist.linear.z);
  } else {
    (void)extract_json_double_in_range(begin, end, "linear_x", twist.linear.x);
    (void)extract_json_double_in_range(begin, end, "linear_y", twist.linear.y);
    (void)extract_json_double_in_range(begin, end, "linear_z", twist.linear.z);
  }
  if (extract_json_object_in_range(begin, end, "angular", angular_begin, angular_end)) {
    (void)extract_json_double_in_range(angular_begin, angular_end + 1, "x", twist.angular.x);
    (void)extract_json_double_in_range(angular_begin, angular_end + 1, "y", twist.angular.y);
    (void)extract_json_double_in_range(angular_begin, angular_end + 1, "z", twist.angular.z);
  } else {
    (void)extract_json_double_in_range(begin, end, "angular_x", twist.angular.x);
    (void)extract_json_double_in_range(begin, end, "angular_y", twist.angular.y);
    (void)extract_json_double_in_range(begin, end, "angular_z", twist.angular.z);
  }
  return true;
}

bool ensure_directory_exists(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  std::string buffer = path;
  for (size_t index = 1; index < buffer.size(); ++index) {
    if (buffer[index] != '/') {
      continue;
    }
    char original = buffer[index];
    buffer[index] = '\0';
    if (::mkdir(buffer.c_str(), 0775) != 0 && errno != EEXIST) {
      return false;
    }
    buffer[index] = original;
  }
  return (::mkdir(buffer.c_str(), 0775) == 0 || errno == EEXIST);
}

bool is_valid_map_basename(const std::string & basename)
{
  if (basename.empty()) {
    return false;
  }
  for (char ch : basename) {
    const bool alnum =
      (ch >= 'a' && ch <= 'z') ||
      (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9');
    if (!alnum && ch != '_' && ch != '-' && ch != '.') {
      return false;
    }
  }
  return true;
}

bool write_temp_map_files(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::string & directory,
  const std::string & basename,
  std::string & image_path,
  std::string & yaml_path)
{
  if (map.info.width == 0U || map.info.height == 0U || map.data.empty()) {
    return false;
  }
  if (!ensure_directory_exists(directory)) {
    return false;
  }
  image_path = directory + "/" + basename + ".pgm";
  yaml_path = directory + "/" + basename + ".yaml";
  FILE * image_file = std::fopen(image_path.c_str(), "wb");
  if (image_file == nullptr) {
    return false;
  }
  std::fprintf(image_file, "P5\n%u %u\n255\n", map.info.width, map.info.height);
  for (size_t row = 0U; row < map.info.height; ++row) {
    const size_t map_row = map.info.height - 1U - row;
    for (size_t col = 0U; col < map.info.width; ++col) {
      const size_t index = (map_row * map.info.width) + col;
      const int8_t cell = map.data[index];
      uint8_t pixel = 205U;
      if (cell == 0) {
        pixel = 254U;
      } else if (cell >= 50) {
        pixel = 0U;
      }
      std::fwrite(&pixel, sizeof(pixel), 1U, image_file);
    }
  }
  std::fclose(image_file);

  FILE * yaml_file = std::fopen(yaml_path.c_str(), "wb");
  if (yaml_file == nullptr) {
    return false;
  }
  std::fprintf(yaml_file, "image: %s.pgm\n", basename.c_str());
  std::fprintf(yaml_file, "resolution: %.9f\n", map.info.resolution);
  std::fprintf(
    yaml_file,
    "origin: [%.9f, %.9f, %.9f]\n",
    map.info.origin.position.x,
    map.info.origin.position.y,
    quaternion_to_yaw(
      map.info.origin.orientation.x,
      map.info.origin.orientation.y,
      map.info.origin.orientation.z,
      map.info.origin.orientation.w));
  std::fprintf(yaml_file, "negate: 0\n");
  std::fprintf(yaml_file, "occupied_thresh: 0.65\n");
  std::fprintf(yaml_file, "free_thresh: 0.196\n");
  std::fclose(yaml_file);
  return true;
}

int wrapped_result_status(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return 4;
    case rclcpp_action::ResultCode::CANCELED:
      return 5;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      return 6;
  }
}

template<typename MsgT>
std::vector<uint8_t> serialize_raw_message(const MsgT & message)
{
  rclcpp::Serialization<MsgT> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  const rcl_serialized_message_t & rmw = serialized.get_rcl_serialized_message();
  return std::vector<uint8_t>(rmw.buffer, rmw.buffer + rmw.buffer_length);
}

class MqttServerNode : public rclcpp::Node
{
private:
  struct BrokerConfig
  {
    std::string host{"192.168.61.35"};
    int port{1883};
    std::string client_id_prefix{"amr_mqtt_server"};
    std::string client_id{"amr_mqtt_server_burger1"};
    int keep_alive_sec{300};
    bool clean_session{false};
    std::string username;
    std::string password;
  };

  struct MqttTopics
  {
    std::string root{"/amr"};
    std::string robot_id{"burger1"};
    int telemetry_qos{0};
    int command_qos{0};
    int service_qos{0};
    std::string telemetry_map;
    std::string telemetry_robot_pose;
    std::string telemetry_global_path;
    std::string telemetry_local_path;
    std::string telemetry_global_costmap;
    std::string telemetry_local_costmap;
    std::string telemetry_motion_status;
    std::string telemetry_scan;
    std::string telemetry_odom;
    std::string telemetry_imu;
    std::string telemetry_tf;
    std::string telemetry_tf_static;
    std::string telemetry_joint_states;
    std::string telemetry_robot_description;
    std::string telemetry_battery_state;
    std::string telemetry_temp_map;
    std::string telemetry_temp_map_raw;
    std::string telemetry_mapping_pose;
    std::string telemetry_slam_graph;
    std::string telemetry_observation_runtime_summary;
    std::string telemetry_observation_runtime_events;
    std::string command_cmd_vel;
    std::string command_save_map;
    std::string command_set_initial_pose;
    std::string command_navigate_to_poses;
    std::string command_cancel_navigate_to_poses;
    std::string command_ping;
    std::string command_set_robot_id;
    std::string feedback_navigate_to_poses;
    std::string status_navigate_to_poses;
    std::string response_set_initial_pose;
    std::string response_navigate_to_poses;
    std::string response_save_map;
    std::string response_ping;
    std::string response_set_robot_id;
    std::string request_plan_segment;
    std::string request_plan_route;
    std::string response_plan_segment;
    std::string response_plan_route;
  };

  struct RosInterfaces
  {
    std::string topic_map{"/amr/map/data"};
    std::string topic_robot_pose{"/amr/localization/pose"};
    std::string topic_global_path{"/amr/planner/global"};
    std::string topic_local_path{"/amr/planner/local"};
    std::string topic_global_costmap{"/amr/costmap/global"};
    std::string topic_local_costmap{"/amr/costmap/local"};
    std::string topic_motion_status{"/amr/motion/status"};
    std::string topic_scan{"/scan"};
    std::string topic_odom{"/odom"};
    std::string topic_imu{"/imu"};
    std::string topic_tf{"/tf"};
    std::string topic_tf_static{"/tf_static"};
    std::string topic_joint_states{"/joint_states"};
    std::string topic_robot_description{"/robot_description"};
    std::string topic_battery_state{"/battery_state"};
    std::string topic_temp_map{"/slam/map/temp/refined"};
    std::string topic_temp_map_raw{"/slam/map/temp/raw"};
    std::string topic_mapping_pose{"/slam/mapper/pose"};
    std::string topic_slam_graph{"/slam/mapper/graph_debug"};
    std::string topic_observation_runtime_summary{"/amr/observation/runtime/summary"};
    std::string topic_observation_runtime_events{"/amr/observation/runtime/events"};
    std::string topic_cmd_vel{"/cmd_vel"};
    std::string topic_initial_pose{"/amr/localization/initial_pose"};
    std::string service_plan_segment{"/amr/global_planner/plan_segment"};
    std::string service_plan_route{"/amr/global_planner/plan_route"};
    std::string action_navigate_to_poses{"/amr/navigator/navigate_to_poses"};
    std::string save_directory{"/home/burger1/ws/data/maps"};
  };

  struct EndpointState
  {
    std::string label;
    std::string ros_topic;
    std::string mqtt_topic;
    std::string viz_topic;
    bool retained{false};
    bool raw_passthrough{true};
    bool raw_publish_once{false};
    bool viz_publish_once{false};
    uint64_t raw_min_period_ms{0U};
    uint64_t viz_min_period_ms{0U};
    uint64_t last_raw_publish_ms{0U};
    uint64_t last_viz_publish_ms{0U};
    bool raw_published_once{false};
    bool viz_published_once{false};
  };

  template<typename MsgT>
  using JsonSerializer = std::function<std::string(const MsgT &)>;

  void load_parameters()
  {
    this->get_parameter_or("broker.host", broker_.host, broker_.host);
    this->get_parameter_or("broker.port", broker_.port, broker_.port);
    this->get_parameter_or("broker.client_id", broker_.client_id_prefix, broker_.client_id_prefix);
    this->get_parameter_or("broker.keep_alive_sec", broker_.keep_alive_sec, broker_.keep_alive_sec);
    this->get_parameter_or("broker.clean_session", broker_.clean_session, broker_.clean_session);
    this->get_parameter_or("broker.username", broker_.username, broker_.username);
    this->get_parameter_or("broker.password", broker_.password, broker_.password);

    this->get_parameter_or("mqtt.root", mqtt_.root, mqtt_.root);
    this->get_parameter_or("mqtt.topics.header", mqtt_.root, mqtt_.root);
    this->get_parameter_or("mqtt.robot_id", mqtt_.robot_id, mqtt_.robot_id);
    this->get_parameter_or("mqtt.qos.telemetry", mqtt_.telemetry_qos, mqtt_.telemetry_qos);
    this->get_parameter_or("mqtt.qos.command", mqtt_.command_qos, mqtt_.command_qos);
    this->get_parameter_or("mqtt.qos.service", mqtt_.service_qos, mqtt_.service_qos);

    this->get_parameter_or("ros.topics.map", ros_.topic_map, ros_.topic_map);
    this->get_parameter_or("ros.topics.robot_pose", ros_.topic_robot_pose, ros_.topic_robot_pose);
    this->get_parameter_or("ros.topics.global_path", ros_.topic_global_path, ros_.topic_global_path);
    this->get_parameter_or("ros.topics.local_path", ros_.topic_local_path, ros_.topic_local_path);
    this->get_parameter_or("ros.topics.global_costmap", ros_.topic_global_costmap, ros_.topic_global_costmap);
    this->get_parameter_or("ros.topics.local_costmap", ros_.topic_local_costmap, ros_.topic_local_costmap);
    this->get_parameter_or("ros.topics.motion_status", ros_.topic_motion_status, ros_.topic_motion_status);
    this->get_parameter_or("ros.topics.scan", ros_.topic_scan, ros_.topic_scan);
    this->get_parameter_or("ros.topics.odom", ros_.topic_odom, ros_.topic_odom);
    this->get_parameter_or("ros.topics.imu", ros_.topic_imu, ros_.topic_imu);
    this->get_parameter_or("ros.topics.tf", ros_.topic_tf, ros_.topic_tf);
    this->get_parameter_or("ros.topics.tf_static", ros_.topic_tf_static, ros_.topic_tf_static);
    this->get_parameter_or("ros.topics.joint_states", ros_.topic_joint_states, ros_.topic_joint_states);
    this->get_parameter_or("ros.topics.robot_description", ros_.topic_robot_description, ros_.topic_robot_description);
    this->get_parameter_or("ros.topics.battery_state", ros_.topic_battery_state, ros_.topic_battery_state);
    this->get_parameter_or("ros.topics.temp_map", ros_.topic_temp_map, ros_.topic_temp_map);
    this->get_parameter_or("ros.topics.temp_map_raw", ros_.topic_temp_map_raw, ros_.topic_temp_map_raw);
    this->get_parameter_or("ros.topics.mapping_pose", ros_.topic_mapping_pose, ros_.topic_mapping_pose);
    this->get_parameter_or("ros.topics.slam_graph", ros_.topic_slam_graph, ros_.topic_slam_graph);
    this->get_parameter_or("ros.topics.observation_runtime_summary", ros_.topic_observation_runtime_summary, ros_.topic_observation_runtime_summary);
    this->get_parameter_or("ros.topics.observation_runtime_events", ros_.topic_observation_runtime_events, ros_.topic_observation_runtime_events);
    this->get_parameter_or("ros.topics.cmd_vel", ros_.topic_cmd_vel, ros_.topic_cmd_vel);
    this->get_parameter_or("ros.topics.initial_pose", ros_.topic_initial_pose, ros_.topic_initial_pose);
    this->get_parameter_or("ros.services.plan_segment", ros_.service_plan_segment, ros_.service_plan_segment);
    this->get_parameter_or("ros.services.plan_route", ros_.service_plan_route, ros_.service_plan_route);
    this->get_parameter_or("ros.actions.navigate_to_poses", ros_.action_navigate_to_poses, ros_.action_navigate_to_poses);
    this->get_parameter_or("ros.save.directory", ros_.save_directory, ros_.save_directory);
    this->get_parameter_or("footprint.polygon", footprint_polygon_, footprint_polygon_);
  }

  void rebuild_mqtt_topics()
  {
    mqtt_.root = normalize_root(mqtt_.root);
    if (!is_valid_robot_id(mqtt_.robot_id)) {
      mqtt_.robot_id = "burger1";
    }

    mqtt_.telemetry_map = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/map");
    mqtt_.telemetry_robot_pose = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/robot_pose");
    mqtt_.telemetry_global_path = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/global_path");
    mqtt_.telemetry_local_path = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/local_path");
    mqtt_.telemetry_global_costmap = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/global_costmap");
    mqtt_.telemetry_local_costmap = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/local_costmap");
    mqtt_.telemetry_motion_status = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/motion_status");
    mqtt_.telemetry_scan = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/scan");
    mqtt_.telemetry_odom = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/odom");
    mqtt_.telemetry_imu = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/imu");
    mqtt_.telemetry_tf = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/tf");
    mqtt_.telemetry_tf_static = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/tf_static");
    mqtt_.telemetry_joint_states = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/joint_states");
    mqtt_.telemetry_robot_description = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/robot_description");
    mqtt_.telemetry_battery_state = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/battery_state");
    mqtt_.telemetry_temp_map = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/temp_map/refined");
    mqtt_.telemetry_temp_map_raw = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/temp_map/raw");
    mqtt_.telemetry_mapping_pose = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/mapping_pose");
    mqtt_.telemetry_slam_graph = scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/slam_graph");
    mqtt_.telemetry_observation_runtime_summary =
      scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/observation/runtime/summary");
    mqtt_.telemetry_observation_runtime_events =
      scoped_topic(mqtt_.root, mqtt_.robot_id, "telemetry/observation/runtime/events");

    mqtt_.command_cmd_vel = scoped_topic(mqtt_.root, mqtt_.robot_id, "motion/command");
    mqtt_.command_save_map = scoped_topic(mqtt_.root, mqtt_.robot_id, "map/save");
    mqtt_.command_set_initial_pose = scoped_topic(mqtt_.root, mqtt_.robot_id, "pose/set");
    mqtt_.command_navigate_to_poses = scoped_topic(mqtt_.root, mqtt_.robot_id, "navigation/command");
    mqtt_.command_cancel_navigate_to_poses = scoped_topic(mqtt_.root, mqtt_.robot_id, "navigation/cancel");
    mqtt_.command_ping = scoped_topic(mqtt_.root, mqtt_.robot_id, "system/ping");
    mqtt_.command_set_robot_id = scoped_topic(mqtt_.root, mqtt_.robot_id, "system/robot");

    mqtt_.feedback_navigate_to_poses = scoped_topic(mqtt_.root, mqtt_.robot_id, "navigation/feedback");
    mqtt_.status_navigate_to_poses = scoped_topic(mqtt_.root, mqtt_.robot_id, "navigation/status");
    mqtt_.response_navigate_to_poses = scoped_topic(mqtt_.root, mqtt_.robot_id, "navigation/result");
    mqtt_.response_set_initial_pose = scoped_topic(mqtt_.root, mqtt_.robot_id, "pose/result");
    mqtt_.response_save_map = scoped_topic(mqtt_.root, mqtt_.robot_id, "map/result");
    mqtt_.response_ping = scoped_topic(mqtt_.root, mqtt_.robot_id, "system/result");
    mqtt_.response_set_robot_id = scoped_topic(mqtt_.root, mqtt_.robot_id, "system/result");
    mqtt_.request_plan_segment = scoped_topic(mqtt_.root, mqtt_.robot_id, "segment/request");
    mqtt_.response_plan_segment = scoped_topic(mqtt_.root, mqtt_.robot_id, "segment/response");
    mqtt_.request_plan_route = scoped_topic(mqtt_.root, mqtt_.robot_id, "route/request");
    mqtt_.response_plan_route = scoped_topic(mqtt_.root, mqtt_.robot_id, "route/response");

    broker_.client_id = broker_.client_id_prefix + "_" + mqtt_.robot_id;
  }

  void init_ros_interfaces()
  {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(ros_.topic_cmd_vel, 10);
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(ros_.topic_initial_pose, 10);
    plan_segment_client_ = this->create_client<amr_msgs::srv::PlanSegment>(ros_.service_plan_segment);
    plan_route_client_ = this->create_client<amr_msgs::srv::PlanRoute>(ros_.service_plan_route);
    navigate_client_ = rclcpp_action::create_client<NavigateToPoses>(this, ros_.action_navigate_to_poses);

    status_subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      ros_.action_navigate_to_poses + "/_action/status",
      10,
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr message) {
        if (!active_navigation_) {
          return;
        }
        publish_payload(mqtt_.status_navigate_to_poses, serialize_navigation_status(*message), mqtt_.telemetry_qos, false);
      });

    add_endpoint<nav_msgs::msg::OccupancyGrid>(
      "map",
      ros_.topic_map,
      mqtt_.telemetry_map,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      true,
      true,
      serialize_occupancy_grid,
      true,
      true,
      0U,
      0U);
    add_endpoint<geometry_msgs::msg::PoseStamped>(
      "robot_pose",
      ros_.topic_robot_pose,
      mqtt_.telemetry_robot_pose,
      rclcpp::QoS(10),
      false,
      true,
      serialize_pose_stamped,
      false,
      false,
      0U,
      50U);
    add_endpoint<nav_msgs::msg::Path>(
      "global_path",
      ros_.topic_global_path,
      mqtt_.telemetry_global_path,
      rclcpp::QoS(10),
      false,
      true,
      serialize_path,
      false,
      false,
      0U,
      200U);
    add_endpoint<nav_msgs::msg::Path>(
      "local_path",
      ros_.topic_local_path,
      mqtt_.telemetry_local_path,
      rclcpp::QoS(10),
      false,
      true,
      serialize_path,
      false,
      false,
      0U,
      100U);
    add_endpoint<nav_msgs::msg::OccupancyGrid>(
      "global_costmap",
      ros_.topic_global_costmap,
      mqtt_.telemetry_global_costmap,
      rclcpp::QoS(10),
      false,
      true,
      serialize_occupancy_grid,
      false,
      false,
      0U,
      300U);
    add_endpoint<nav_msgs::msg::OccupancyGrid>(
      "local_costmap",
      ros_.topic_local_costmap,
      mqtt_.telemetry_local_costmap,
      rclcpp::QoS(10),
      false,
      true,
      serialize_occupancy_grid,
      false,
      false,
      0U,
      150U);
    add_endpoint<amr_msgs::msg::MotionStatus>(
      "motion_status",
      ros_.topic_motion_status,
      mqtt_.telemetry_motion_status,
      rclcpp::QoS(10),
      false,
      true,
      serialize_motion_status,
      false,
      false,
      0U,
      100U);
    add_endpoint<sensor_msgs::msg::LaserScan>(
      "scan",
      ros_.topic_scan,
      mqtt_.telemetry_scan,
      rclcpp::SensorDataQoS(),
      false,
      true,
      serialize_scan,
      false,
      false,
      0U,
      100U);
    add_endpoint<nav_msgs::msg::Odometry>(
      "odom",
      ros_.topic_odom,
      mqtt_.telemetry_odom,
      rclcpp::SensorDataQoS(),
      false,
      true,
      serialize_odom,
      false,
      false,
      0U,
      0U);
    add_endpoint<sensor_msgs::msg::Imu>(
      "imu",
      ros_.topic_imu,
      mqtt_.telemetry_imu,
      rclcpp::SensorDataQoS(),
      false,
      true,
      serialize_imu,
      false,
      false,
      0U,
      0U);
    add_endpoint<tf2_msgs::msg::TFMessage>(
      "tf",
      ros_.topic_tf,
      mqtt_.telemetry_tf,
      rclcpp::QoS(10),
      false,
      true,
      serialize_tf_message,
      false,
      false,
      0U,
      50U);
    add_endpoint<tf2_msgs::msg::TFMessage>(
      "tf_static",
      ros_.topic_tf_static,
      mqtt_.telemetry_tf_static,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      true,
      true,
      serialize_tf_message,
      true,
      true,
      0U,
      0U);
    add_endpoint<sensor_msgs::msg::JointState>(
      "joint_state",
      ros_.topic_joint_states,
      mqtt_.telemetry_joint_states,
      rclcpp::QoS(10),
      false,
      true,
      serialize_joint_states,
      false,
      false,
      0U,
      0U);
    add_endpoint<std_msgs::msg::String>(
      "robot_description",
      ros_.topic_robot_description,
      mqtt_.telemetry_robot_description,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      true,
      true,
      [this](const std_msgs::msg::String & message) {
        return serialize_string_message(message, footprint_polygon_);
      },
      true,
      true,
      0U,
      0U);
    add_endpoint<sensor_msgs::msg::BatteryState>(
      "battery_state",
      ros_.topic_battery_state,
      mqtt_.telemetry_battery_state,
      rclcpp::QoS(10),
      false,
      true,
      serialize_battery_state,
      false,
      false,
      0U,
      1000U);
    add_endpoint<nav_msgs::msg::OccupancyGrid>(
      "temp_map_raw",
      ros_.topic_temp_map_raw,
      mqtt_.telemetry_temp_map_raw,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      false,
      true,
      serialize_occupancy_grid,
      false,
      false,
      0U,
      1000U);
    add_endpoint<nav_msgs::msg::OccupancyGrid>(
      "temp_map",
      ros_.topic_temp_map,
      mqtt_.telemetry_temp_map,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      false,
      true,
      serialize_occupancy_grid,
      false,
      false,
      0U,
      700U,
      [this](const nav_msgs::msg::OccupancyGrid & message) {
        latest_temp_map_ = message;
        has_temp_map_ = true;
      });
    add_endpoint<geometry_msgs::msg::PoseStamped>(
      "mapping_pose",
      ros_.topic_mapping_pose,
      mqtt_.telemetry_mapping_pose,
      rclcpp::QoS(10),
      false,
      true,
      serialize_pose_stamped,
      false,
      false,
      0U,
      100U);
    add_endpoint<std_msgs::msg::String>(
      "slam_graph",
      ros_.topic_slam_graph,
      mqtt_.telemetry_slam_graph,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      false,
      true,
      serialize_string_json_message,
      false,
      false,
      0U,
      700U);
    add_endpoint<std_msgs::msg::String>(
      "runtime_summary",
      ros_.topic_observation_runtime_summary,
      mqtt_.telemetry_observation_runtime_summary,
      rclcpp::QoS(10),
      false,
      true,
      serialize_string_json_message,
      false,
      false,
      0U,
      200U);
    add_endpoint<std_msgs::msg::String>(
      "runtime_events",
      ros_.topic_observation_runtime_events,
      mqtt_.telemetry_observation_runtime_events,
      rclcpp::QoS(10),
      false,
      true,
      serialize_string_json_message,
      false,
      false,
      0U,
      50U);

    RCLCPP_INFO(
      this->get_logger(),
      "amr_mqtt_server ready for broker %s:%d with robot_id=%s",
      broker_.host.c_str(),
      broker_.port,
      mqtt_.robot_id.c_str());
  }

  template<typename MsgT>
  void add_endpoint(
    const std::string & label,
    const std::string & ros_topic,
    const std::string & mqtt_topic,
    const rclcpp::QoS & qos,
    bool retained,
    bool raw_passthrough,
    JsonSerializer<MsgT> serializer,
    bool raw_publish_once,
    bool viz_publish_once,
    uint64_t raw_min_period_ms,
    uint64_t viz_min_period_ms,
    std::function<void(const MsgT &)> on_receive = {})
  {
    std::shared_ptr<EndpointState> endpoint = std::make_shared<EndpointState>();
    endpoint->label = label;
    endpoint->ros_topic = ros_topic;
    endpoint->mqtt_topic = mqtt_topic;
    endpoint->viz_topic = build_viz_topic(mqtt_topic);
    endpoint->retained = retained;
    endpoint->raw_passthrough = raw_passthrough;
    endpoint->raw_publish_once = raw_publish_once;
    endpoint->viz_publish_once = viz_publish_once;
    endpoint->raw_min_period_ms = raw_min_period_ms;
    endpoint->viz_min_period_ms = viz_min_period_ms;
    endpoints_.push_back(endpoint);

    typename rclcpp::Subscription<MsgT>::SharedPtr subscription = this->create_subscription<MsgT>(
      ros_topic,
      qos,
      [this, endpoint, serializer, on_receive](const typename MsgT::SharedPtr message) {
        if (on_receive) {
          on_receive(*message);
        }
        const uint64_t now = now_ms();
        if (endpoint->raw_passthrough) {
          const bool should_publish_raw =
            (!endpoint->raw_publish_once || !endpoint->raw_published_once) &&
            (endpoint->raw_min_period_ms == 0U ||
            now >= endpoint->last_raw_publish_ms + endpoint->raw_min_period_ms);
          if (should_publish_raw) {
            publish_binary_payload(
              endpoint->mqtt_topic,
              serialize_raw_message<MsgT>(*message),
              mqtt_.telemetry_qos,
              endpoint->retained);
            endpoint->last_raw_publish_ms = now;
            endpoint->raw_published_once = true;
          }

          if (serializer && !endpoint->viz_topic.empty()) {
            const bool should_publish_viz =
              (!endpoint->viz_publish_once || !endpoint->viz_published_once) &&
              (endpoint->viz_min_period_ms == 0U ||
              now >= endpoint->last_viz_publish_ms + endpoint->viz_min_period_ms);
            if (should_publish_viz) {
              publish_payload(
                endpoint->viz_topic,
                serializer(*message),
                mqtt_.telemetry_qos,
                endpoint->retained);
              endpoint->last_viz_publish_ms = now;
              endpoint->viz_published_once = true;
            }
          }
        } else if (serializer) {
          publish_payload(endpoint->mqtt_topic, serializer(*message), mqtt_.telemetry_qos, endpoint->retained);
        }
      });
    subscriptions_.push_back(subscription);
  }

  void connect_mqtt()
  {
    disconnect_mqtt();

    broker_uri_ = "tcp://" + broker_.host + ":" + std::to_string(broker_.port);
    const int create_rc = MQTTClient_create(
      &mqtt_client_,
      broker_uri_.c_str(),
      broker_.client_id.c_str(),
      MQTTCLIENT_PERSISTENCE_NONE,
      nullptr);
    if (create_rc != MQTTCLIENT_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create MQTT client: rc=%d", create_rc);
      mqtt_client_ = nullptr;
      connected_ = false;
      return;
    }

    MQTTClient_connectOptions options = MQTTClient_connectOptions_initializer;
    options.keepAliveInterval = broker_.keep_alive_sec;
    options.cleansession = broker_.clean_session ? 1 : 0;
    if (!broker_.username.empty()) {
      options.username = broker_.username.c_str();
    }
    if (!broker_.password.empty()) {
      options.password = broker_.password.c_str();
    }

    const int connect_rc = MQTTClient_connect(mqtt_client_, &options);
    if (connect_rc != MQTTCLIENT_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect MQTT broker %s: rc=%d", broker_uri_.c_str(), connect_rc);
      MQTTClient_destroy(&mqtt_client_);
      mqtt_client_ = nullptr;
      connected_ = false;
      return;
    }

    connected_ = true;
    subscribe_command_topics();
  }

  void disconnect_mqtt()
  {
    if (mqtt_client_ != nullptr) {
      if (connected_) {
        (void)MQTTClient_disconnect(mqtt_client_, 1000);
      }
      MQTTClient_destroy(&mqtt_client_);
      mqtt_client_ = nullptr;
    }
    connected_ = false;
  }

  bool ensure_connected()
  {
    if (mqtt_client_ == nullptr) {
      connect_mqtt();
      return connected_;
    }
    if (MQTTClient_isConnected(mqtt_client_)) {
      connected_ = true;
      return true;
    }
    connected_ = false;
    const std::time_t now = std::time(nullptr);
    if (now == last_reconnect_attempt_sec_) {
      return false;
    }
    last_reconnect_attempt_sec_ = now;
    MQTTClient_connectOptions options = MQTTClient_connectOptions_initializer;
    options.keepAliveInterval = broker_.keep_alive_sec;
    options.cleansession = broker_.clean_session ? 1 : 0;
    if (!broker_.username.empty()) {
      options.username = broker_.username.c_str();
    }
    if (!broker_.password.empty()) {
      options.password = broker_.password.c_str();
    }
    const int rc = MQTTClient_connect(mqtt_client_, &options);
    if (rc != MQTTCLIENT_SUCCESS) {
      return false;
    }
    connected_ = true;
    subscribe_command_topics();
    return true;
  }

  void subscribe_command_topics()
  {
    if (mqtt_client_ == nullptr) {
      return;
    }
    const std::vector<std::pair<std::string, int>> topics = {
      {mqtt_.command_cmd_vel, mqtt_.command_qos},
      {mqtt_.command_save_map, mqtt_.command_qos},
      {mqtt_.command_set_initial_pose, mqtt_.command_qos},
      {mqtt_.command_navigate_to_poses, mqtt_.command_qos},
      {mqtt_.command_cancel_navigate_to_poses, mqtt_.command_qos},
      {mqtt_.command_ping, mqtt_.command_qos},
      {mqtt_.command_set_robot_id, mqtt_.command_qos},
      {mqtt_.request_plan_segment, mqtt_.service_qos},
      {mqtt_.request_plan_route, mqtt_.service_qos},
    };
    for (const std::pair<std::string, int> & entry : topics) {
      (void)MQTTClient_subscribe(mqtt_client_, entry.first.c_str(), entry.second);
    }
  }

  void publish_payload(const std::string & topic, const std::string & payload, int qos, bool retained)
  {
    if (!ensure_connected()) {
      return;
    }
    MQTTClient_message message = MQTTClient_message_initializer;
    message.payload = const_cast<char *>(payload.data());
    message.payloadlen = static_cast<int>(payload.size());
    message.qos = qos;
    message.retained = retained ? 1 : 0;
    MQTTClient_deliveryToken token;
    const int rc = MQTTClient_publishMessage(mqtt_client_, topic.c_str(), &message, &token);
    if (rc != MQTTCLIENT_SUCCESS) {
      connected_ = false;
      return;
    }
    (void)MQTTClient_waitForCompletion(mqtt_client_, token, 1000L);
  }

  void publish_binary_payload(
    const std::string & topic,
    const std::vector<uint8_t> & payload,
    int qos,
    bool retained)
  {
    if (payload.empty() || !ensure_connected()) {
      return;
    }
    MQTTClient_message message = MQTTClient_message_initializer;
    message.payload = const_cast<uint8_t *>(payload.data());
    message.payloadlen = static_cast<int>(payload.size());
    message.qos = qos;
    message.retained = retained ? 1 : 0;
    MQTTClient_deliveryToken token;
    const int rc = MQTTClient_publishMessage(mqtt_client_, topic.c_str(), &message, &token);
    if (rc != MQTTCLIENT_SUCCESS) {
      connected_ = false;
      return;
    }
    (void)MQTTClient_waitForCompletion(mqtt_client_, token, 1000L);
  }

  void publish_simple_response(
    const std::string & topic,
    const std::string & request_id,
    bool success,
    const std::string & message)
  {
    publish_payload(topic, serialize_simple_response(request_id, success, message), mqtt_.service_qos, false);
  }

  void publish_navigation_result(
    const std::string & request_id,
    bool success,
    int status_code,
    bool accepted,
    bool completed,
    uint32_t completed_goals,
    const std::string & message)
  {
    publish_payload(
      mqtt_.response_navigate_to_poses,
      serialize_navigation_result(
        request_id,
        success,
        status_code,
        accepted,
        completed,
        completed_goals,
        message),
      mqtt_.service_qos,
      false);
  }

  void handle_cmd_vel_message(const void * payload, size_t payload_length)
  {
    geometry_msgs::msg::Twist twist;
    if (payload != nullptr && payload_length > 0U && static_cast<const unsigned char *>(payload)[0] == '{') {
      std::string text(static_cast<const char *>(payload), payload_length);
      if (!extract_twist_from_json(text, twist)) {
        return;
      }
    } else if (!deserialize_twist_raw(payload, payload_length, twist)) {
      return;
    }
    cmd_vel_publisher_->publish(twist);
  }

  void handle_set_initial_pose_command(const std::string & payload)
  {
    std::string request_id;
    std::string frame_id{"map"};
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double covariance_x = 0.25;
    double covariance_y = 0.25;
    double covariance_yaw = 0.06853891945200942;
    bool has_flat_pose = false;
    bool has_structured_pose = false;

    geometry_msgs::msg::PoseStamped pose;
    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id)) {
      publish_simple_response(mqtt_.response_set_initial_pose, "", false, "invalid set_initial_pose payload");
      return;
    }

    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "frame", frame_id)) {
      (void)extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "frame_id", frame_id);
    }
    (void)extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "covariance_x", covariance_x);
    (void)extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "covariance_y", covariance_y);
    (void)extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "covariance_yaw", covariance_yaw);

    has_flat_pose =
      extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "x", x) &&
      extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "y", y) &&
      extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "yaw", yaw);

    if (!has_flat_pose) {
      has_structured_pose = parse_pose_stamped_in_range(payload.c_str(), payload.c_str() + payload.size(), pose);
      if (has_structured_pose) {
        x = pose.pose.position.x;
        y = pose.pose.position.y;
        yaw = quaternion_to_yaw(
          pose.pose.orientation.x,
          pose.pose.orientation.y,
          pose.pose.orientation.z,
          pose.pose.orientation.w);
        if (!pose.header.frame_id.empty()) {
          frame_id = pose.header.frame_id;
        }
      }
    }

    if (!has_flat_pose && !has_structured_pose) {
      publish_simple_response(mqtt_.response_set_initial_pose, request_id, false, "invalid set_initial_pose payload");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = frame_id;
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation = quaternion_from_yaw(yaw);
    std::fill(initial_pose.pose.covariance.begin(), initial_pose.pose.covariance.end(), 0.0);
    initial_pose.pose.covariance[0] = covariance_x;
    initial_pose.pose.covariance[7] = covariance_y;
    initial_pose.pose.covariance[35] = covariance_yaw;
    initial_pose_publisher_->publish(initial_pose);
    publish_simple_response(mqtt_.response_set_initial_pose, request_id, true, "initial pose published");
  }

  void handle_save_map_command(const std::string & payload)
  {
    std::string request_id;
    std::string basename;
    (void)extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id);
    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "basename", basename) ||
        !is_valid_map_basename(basename)) {
      publish_simple_response(mqtt_.response_save_map, request_id, false, "invalid map basename");
      return;
    }
    if (!has_temp_map_) {
      publish_simple_response(mqtt_.response_save_map, request_id, false, "temporary map is not available");
      return;
    }
    std::string image_path;
    std::string yaml_path;
    if (!write_temp_map_files(latest_temp_map_, ros_.save_directory, basename, image_path, yaml_path)) {
      publish_simple_response(mqtt_.response_save_map, request_id, false, "failed to save map files");
      return;
    }
    publish_simple_response(mqtt_.response_save_map, request_id, true, "saved map files");
  }

  void handle_plan_segment_request(const std::string & payload)
  {
    std::string request_id;
    const char * start_begin = nullptr;
    const char * start_end = nullptr;
    const char * goal_begin = nullptr;
    const char * goal_end = nullptr;
    amr_msgs::srv::PlanSegment::Request request;

    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id) ||
        !extract_json_object_in_range(payload.c_str(), payload.c_str() + payload.size(), "start", start_begin, start_end) ||
        !extract_json_object_in_range(payload.c_str(), payload.c_str() + payload.size(), "goal", goal_begin, goal_end) ||
        !parse_pose_stamped_in_range(start_begin, start_end + 1, request.start) ||
        !parse_pose_stamped_in_range(goal_begin, goal_end + 1, request.goal)) {
      publish_simple_response(mqtt_.response_plan_segment, request_id, false, "invalid plan_segment payload");
      return;
    }

    if (!plan_segment_client_->wait_for_service(1s)) {
      publish_simple_response(mqtt_.response_plan_segment, request_id, false, "plan_segment service unavailable");
      return;
    }

    (void)plan_segment_client_->async_send_request(std::make_shared<amr_msgs::srv::PlanSegment::Request>(request));
    publish_simple_response(mqtt_.response_plan_segment, request_id, true, "plan_segment request dispatched");
  }

  void handle_plan_route_request(const std::string & payload)
  {
    std::string request_id;
    const char * start_begin = nullptr;
    const char * start_end = nullptr;
    const char * waypoints_begin = nullptr;
    const char * waypoints_end = nullptr;
    amr_msgs::srv::PlanRoute::Request request;

    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id) ||
        !extract_json_object_in_range(payload.c_str(), payload.c_str() + payload.size(), "start", start_begin, start_end) ||
        !extract_json_array_in_range(payload.c_str(), payload.c_str() + payload.size(), "waypoints", waypoints_begin, waypoints_end) ||
        !parse_pose_stamped_in_range(start_begin, start_end + 1, request.start) ||
        !parse_waypoints_array(waypoints_begin, waypoints_end + 1, request.waypoints)) {
      publish_simple_response(mqtt_.response_plan_route, request_id, false, "invalid plan_route payload");
      return;
    }

    if (!plan_route_client_->wait_for_service(1s)) {
      publish_simple_response(mqtt_.response_plan_route, request_id, false, "plan_route service unavailable");
      return;
    }

    (void)plan_route_client_->async_send_request(std::make_shared<amr_msgs::srv::PlanRoute::Request>(request));
    publish_simple_response(mqtt_.response_plan_route, request_id, true, "plan_route request dispatched");
  }

  void handle_navigation_command(const std::string & payload)
  {
    std::string request_id;
    const char * goals_begin = nullptr;
    const char * goals_end = nullptr;
    NavigateToPoses::Goal goal;

    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id) ||
        !extract_json_array_in_range(payload.c_str(), payload.c_str() + payload.size(), "goal_poses", goals_begin, goals_end) ||
        !parse_waypoints_array(goals_begin, goals_end + 1, goal.goal_poses)) {
      publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "invalid navigate_to_poses payload");
      return;
    }
    if (goal.goal_poses.empty()) {
      publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "goal_poses must not be empty");
      return;
    }
    if (active_navigation_) {
      publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "another navigation goal is already active");
      return;
    }
    if (!navigate_client_->wait_for_action_server(1s)) {
      publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "navigate_to_poses action unavailable");
      return;
    }

    active_navigation_ = true;
    current_request_id_ = request_id;
    current_goal_uuid_hex_.clear();
    active_goal_handle_.reset();

    rclcpp_action::Client<NavigateToPoses>::SendGoalOptions options;
    options.goal_response_callback =
      [this, request_id](GoalHandleNavigateToPoses::SharedPtr goal_handle) {
        if (!goal_handle) {
          active_navigation_ = false;
          current_request_id_.clear();
          publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "goal rejected");
          return;
        }
        active_goal_handle_ = goal_handle;
        current_goal_uuid_hex_ = uuid_to_hex(goal_handle->get_goal_id());
        publish_simple_response(mqtt_.response_navigate_to_poses, request_id, true, "goal accepted; route execution pending");
      };
    options.feedback_callback =
      [this](GoalHandleNavigateToPoses::SharedPtr goal_handle, const std::shared_ptr<const NavigateToPoses::Feedback> feedback) {
        if (!active_navigation_ || !goal_handle || !feedback) {
          return;
        }
        publish_payload(
          mqtt_.feedback_navigate_to_poses,
          serialize_navigation_feedback(uuid_to_hex(goal_handle->get_goal_id()), *feedback),
          mqtt_.telemetry_qos,
          false);
      };
    options.result_callback =
      [this](const GoalHandleNavigateToPoses::WrappedResult & result) {
        const bool success =
          result.result != nullptr &&
          result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result->error_code == NavigateToPoses::Result::NONE;
        const uint32_t completed_goals = result.result ? result.result->completed_goals : 0U;
        const std::string message =
          (result.result == nullptr || result.result->error_msg.empty()) ?
          (result.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded" :
          (result.code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "aborted")) :
          result.result->error_msg;
        publish_navigation_result(
          current_request_id_,
          success,
          wrapped_result_status(result.code),
          true,
          true,
          completed_goals,
          message);
        active_navigation_ = false;
        current_request_id_.clear();
        current_goal_uuid_hex_.clear();
        active_goal_handle_.reset();
      };

    (void)navigate_client_->async_send_goal(goal, options);
  }

  void handle_navigation_cancel(const std::string & payload)
  {
    std::string request_id;
    (void)extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id);
    if (!active_navigation_ || !active_goal_handle_) {
      publish_simple_response(mqtt_.response_navigate_to_poses, request_id, false, "no active navigate_to_poses goal");
      return;
    }
    (void)navigate_client_->async_cancel_goal(active_goal_handle_);
    publish_simple_response(mqtt_.response_navigate_to_poses, request_id, true, "goal cancel dispatched");
  }

  void handle_ping(const std::string & payload)
  {
    std::string request_id;
    double sent_at_ms = 0.0;
    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id)) {
      return;
    }
    (void)extract_json_double_in_range(payload.c_str(), payload.c_str() + payload.size(), "sent_at_ms", sent_at_ms);
    publish_payload(
      mqtt_.response_ping,
      serialize_ping_response(request_id, true, sent_at_ms, now_ms(), "pong"),
      mqtt_.service_qos,
      false);
  }

  void handle_set_robot_id(const std::string & payload)
  {
    std::string request_id;
    std::string robot_id;
    if (!extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "request_id", request_id) ||
        !extract_json_string_in_range(payload.c_str(), payload.c_str() + payload.size(), "robot_id", robot_id)) {
      publish_simple_response(mqtt_.response_set_robot_id, request_id, false, "invalid set_robot_id payload");
      return;
    }
    if (!is_valid_robot_id(robot_id)) {
      publish_simple_response(mqtt_.response_set_robot_id, request_id, false, "robot_id must contain only letters, digits, '_' or '-'");
      return;
    }
    if (robot_id == mqtt_.robot_id) {
      publish_simple_response(mqtt_.response_set_robot_id, request_id, true, "robot_id unchanged");
      return;
    }
    pending_robot_id_ = robot_id;
    robot_id_change_pending_ = true;
    publish_simple_response(mqtt_.response_set_robot_id, request_id, true, "robot_id change scheduled");
  }

  void apply_pending_robot_id_change()
  {
    if (!robot_id_change_pending_) {
      return;
    }
    if (!is_valid_robot_id(pending_robot_id_)) {
      robot_id_change_pending_ = false;
      pending_robot_id_.clear();
      return;
    }
    const std::string previous_robot_id = mqtt_.robot_id;
    mqtt_.robot_id = pending_robot_id_;
    rebuild_mqtt_topics();
    connect_mqtt();
    if (!connected_) {
      mqtt_.robot_id = previous_robot_id;
      rebuild_mqtt_topics();
      connect_mqtt();
      RCLCPP_ERROR(this->get_logger(), "Failed to switch MQTT robot_id to '%s'", pending_robot_id_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Switched MQTT robot_id to '%s'", mqtt_.robot_id.c_str());
    }
    robot_id_change_pending_ = false;
    pending_robot_id_.clear();
  }

  void poll_mqtt()
  {
    if (!ensure_connected()) {
      return;
    }
    int processed = 0;
    while (processed < static_cast<int>(k_max_processed_mqtt_messages)) {
      char * topic_name = nullptr;
      int topic_length = 0;
      MQTTClient_message * message = nullptr;
      const int rc = MQTTClient_receive(mqtt_client_, &topic_name, &topic_length, &message, 0UL);
      if (rc != MQTTCLIENT_SUCCESS) {
        if (rc == MQTTCLIENT_DISCONNECTED) {
          connected_ = false;
        }
        break;
      }
      if (message == nullptr || topic_name == nullptr) {
        break;
      }

      const std::string topic(topic_name, topic_length > 0 ? static_cast<size_t>(topic_length) : std::strlen(topic_name));
      if (topic == mqtt_.command_cmd_vel) {
        handle_cmd_vel_message(message->payload, static_cast<size_t>(message->payloadlen));
      } else {
        std::string text(static_cast<const char *>(message->payload), static_cast<size_t>(message->payloadlen));
        if (topic == mqtt_.command_save_map) {
          handle_save_map_command(text);
        } else if (topic == mqtt_.command_set_initial_pose) {
          handle_set_initial_pose_command(text);
        } else if (topic == mqtt_.command_navigate_to_poses) {
          handle_navigation_command(text);
        } else if (topic == mqtt_.command_cancel_navigate_to_poses) {
          handle_navigation_cancel(text);
        } else if (topic == mqtt_.command_ping) {
          handle_ping(text);
        } else if (topic == mqtt_.command_set_robot_id) {
          handle_set_robot_id(text);
        } else if (topic == mqtt_.request_plan_segment) {
          handle_plan_segment_request(text);
        } else if (topic == mqtt_.request_plan_route) {
          handle_plan_route_request(text);
        }
      }

      MQTTClient_freeMessage(&message);
      MQTTClient_free(topic_name);
      ++processed;
    }
  }

  BrokerConfig broker_;
  MqttTopics mqtt_;
  RosInterfaces ros_;
  std::vector<double> footprint_polygon_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<std::shared_ptr<EndpointState>> endpoints_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Client<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_client_;
  rclcpp::Client<amr_msgs::srv::PlanRoute>::SharedPtr plan_route_client_;
  rclcpp_action::Client<NavigateToPoses>::SharedPtr navigate_client_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr robot_id_timer_;

  nav_msgs::msg::OccupancyGrid latest_temp_map_;
  bool has_temp_map_{false};

  MQTTClient mqtt_client_{nullptr};
  std::string broker_uri_;
  bool connected_{false};
  long last_reconnect_attempt_sec_{0};
  bool robot_id_change_pending_{false};
  std::string pending_robot_id_;

  bool active_navigation_{false};
  GoalHandleNavigateToPoses::SharedPtr active_goal_handle_;
  std::string current_request_id_;
  std::string current_goal_uuid_hex_;

public:
  explicit MqttServerNode();
  virtual ~MqttServerNode();
};

MqttServerNode::MqttServerNode()
: rclcpp::Node(
    "mqtt_server",
    "/amr",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  load_parameters();
  rebuild_mqtt_topics();
  init_ros_interfaces();
  connect_mqtt();
  poll_timer_ = this->create_wall_timer(20ms, std::bind(&MqttServerNode::poll_mqtt, this));
  robot_id_timer_ = this->create_wall_timer(200ms, std::bind(&MqttServerNode::apply_pending_robot_id_change, this));
}

MqttServerNode::~MqttServerNode()
{
  disconnect_mqtt();
}

}  // namespace

std::shared_ptr<rclcpp::Node> make_node()
{
  return std::make_shared<MqttServerNode>();
}

}  // namespace amr::mqtt::server
