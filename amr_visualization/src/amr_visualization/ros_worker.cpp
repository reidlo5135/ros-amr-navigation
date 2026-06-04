#include "amr_visualization/ros_worker.hpp"

#include "amr_visualization/file_probe.hpp"
#include "amr_visualization/robot_model_renderer.hpp"

#include <QElapsedTimer>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QStringList>
#include <QUrl>
#include <QXmlStreamReader>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <utility>

namespace amr::visualization
{

namespace
{

using namespace std::chrono_literals;

QString utf8_hex_dump(const QString &value)
{
  const QByteArray bytes = value.toUtf8();
  QStringList chunks;
  chunks.reserve(bytes.size());
  for (const char byte : bytes) {
    chunks.push_back(QString("%1").arg(static_cast<unsigned char>(byte), 2, 16, QChar('0')));
  }
  return chunks.join(' ');
}

QString escaped_text(const QString &value)
{
  QString escaped;
  escaped.reserve(value.size());
  for (const QChar ch : value) {
    if (ch == '\\') {
      escaped += "\\\\";
    } else if (ch == '\n') {
      escaped += "\\n";
    } else if (ch == '\r') {
      escaped += "\\r";
    } else if (ch == '\t') {
      escaped += "\\t";
    } else if (ch.unicode() < 0x20U || ch.unicode() == 0x7FU) {
      escaped += QString("\\u%1").arg(ch.unicode(), 4, 16, QChar('0'));
    } else {
      escaped += ch;
    }
  }
  return escaped;
}

QString trim_outer_quotes(QString value)
{
  value = value.trimmed();
  if (value.size() >= 2) {
    const QChar first = value.front();
    const QChar last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      value = value.mid(1, value.size() - 2).trimmed();
    }
  }
  return value;
}

QString trim_accidental_mesh_suffix(QString value)
{
  value = trim_outer_quotes(value);
  while (value.endsWith(',') || value.endsWith(';')) {
    if (QFileInfo(QDir::cleanPath(value)).exists()) {
      break;
    }
    const QString stripped = value.left(value.size() - 1).trimmed();
    if (stripped.isEmpty()) {
      break;
    }
    const bool stripped_exists = QFileInfo(QDir::cleanPath(stripped)).exists();
    const bool uri_punctuation =
      (value.startsWith("package://") || value.startsWith("file://")) &&
      stripped.endsWith(".stl", Qt::CaseInsensitive);
    if (!stripped_exists && !uri_punctuation) {
      break;
    }
    value = stripped;
  }
  return value;
}

QString normalized_filesystem_path(const QString &path)
{
  const QString trimmed = trim_accidental_mesh_suffix(path);
  if (trimmed.isEmpty()) {
    return {};
  }
  return QFileInfo(QDir::cleanPath(trimmed)).absoluteFilePath();
}

QString canonical_or_absolute_path(const QFileInfo &file_info)
{
  const QString canonical = file_info.canonicalFilePath();
  if (!canonical.isEmpty()) {
    return canonical;
  }
  return file_info.absoluteFilePath();
}

QString file_access_diagnostics_text(const QString &uri, const QString &resolved_path)
{
  const FileProbe probe = probeFilePath(uri, resolved_path);
  return fileProbeToDiagnosticText(probe);
}

struct Rotation3D
{
  std::array<std::array<double, 3>, 3> m{};
};

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

Rotation3D rotation_from_rpy(const double roll, const double pitch, const double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Rotation3D rotation;
  rotation.m = {{
    {{cy * cp, (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)}},
    {{sy * cp, (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)}},
    {{-sp, cp * sr, cp * cr}},
  }};
  return rotation;
}

Rotation3D multiply_rotation(const Rotation3D &lhs, const Rotation3D &rhs)
{
  Rotation3D result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.m[row][column] =
        (lhs.m[row][0] * rhs.m[0][column]) +
        (lhs.m[row][1] * rhs.m[1][column]) +
        (lhs.m[row][2] * rhs.m[2][column]);
    }
  }
  return result;
}

std::array<double, 3> rotate_point(
  const Rotation3D &rotation,
  const double x,
  const double y,
  const double z)
{
  return {
    (rotation.m[0][0] * x) + (rotation.m[0][1] * y) + (rotation.m[0][2] * z),
    (rotation.m[1][0] * x) + (rotation.m[1][1] * y) + (rotation.m[1][2] * z),
    (rotation.m[2][0] * x) + (rotation.m[2][1] * y) + (rotation.m[2][2] * z),
  };
}

Pose2D pose_from_rotation(const Rotation3D &rotation)
{
  Pose2D pose;
  pose.pitch = std::asin(std::clamp(-rotation.m[2][0], -1.0, 1.0));
  if (std::abs(std::cos(pose.pitch)) > 1e-6) {
    pose.roll = std::atan2(rotation.m[2][1], rotation.m[2][2]);
    pose.yaw = std::atan2(rotation.m[1][0], rotation.m[0][0]);
  } else {
    pose.roll = 0.0;
    pose.yaw = std::atan2(-rotation.m[0][1], rotation.m[1][1]);
  }
  return pose;
}

void apply_quaternion_orientation(
  Pose2D &pose,
  const double x,
  const double y,
  const double z,
  const double w)
{
  const double sinr_cosp = 2.0 * ((w * x) + (y * z));
  const double cosr_cosp = 1.0 - 2.0 * ((x * x) + (y * y));
  pose.roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * ((w * y) - (z * x));
  pose.pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
  pose.yaw = quaternion_to_yaw(x, y, z, w);
}

QVector<double> parse_scalar_list(const QString &text, int expected_count)
{
  QVector<double> values;
  const QStringList tokens = text.split(' ', Qt::SkipEmptyParts);
  values.reserve(tokens.size());
  for (const auto &token : tokens) {
    bool ok = false;
    const double value = token.toDouble(&ok);
    if (!ok) {
      return {};
    }
    values.push_back(value);
  }
  if (expected_count > 0 && values.size() != expected_count) {
    return {};
  }
  return values;
}

Pose2D parse_origin_attributes(const QXmlStreamAttributes &attributes)
{
  Pose2D pose;
  pose.valid = true;
  const QVector<double> xyz = parse_scalar_list(attributes.value("xyz").toString(), 3);
  if (xyz.size() == 3) {
    pose.x = xyz[0];
    pose.y = xyz[1];
    pose.z = xyz[2];
  }
  const QVector<double> rpy = parse_scalar_list(attributes.value("rpy").toString(), 3);
  if (rpy.size() == 3) {
    pose.roll = rpy[0];
    pose.pitch = rpy[1];
    pose.yaw = rpy[2];
  }
  return pose;
}

bool parse_geometry(
  QXmlStreamReader &reader,
  const QString &frame_id,
  const Pose2D &origin,
  RobotVisual &visual)
{
  while (reader.readNextStartElement()) {
    const auto name = reader.name();
    visual = RobotVisual();
    visual.frame_id = frame_id;
    visual.pose = origin;
    visual.pose.valid = true;
    visual.valid = true;

    if (name == QLatin1String("box")) {
      const QVector<double> size =
        parse_scalar_list(reader.attributes().value("size").toString(), 3);
      reader.skipCurrentElement();
      if (size.size() != 3) {
        return false;
      }
      visual.type = RobotGeometryType::Box;
      visual.size_x = size[0];
      visual.size_y = size[1];
      visual.size_z = size[2];
      return visual.size_x > 0.0 && visual.size_y > 0.0;
    }
    if (name == QLatin1String("cylinder")) {
      bool radius_ok = false;
      bool length_ok = false;
      visual.type = RobotGeometryType::Cylinder;
      visual.radius = reader.attributes().value("radius").toDouble(&radius_ok);
      visual.length = reader.attributes().value("length").toDouble(&length_ok);
      reader.skipCurrentElement();
      return radius_ok && length_ok && visual.radius > 0.0;
    }
    if (name == QLatin1String("sphere")) {
      bool radius_ok = false;
      visual.type = RobotGeometryType::Sphere;
      visual.radius = reader.attributes().value("radius").toDouble(&radius_ok);
      reader.skipCurrentElement();
      return radius_ok && visual.radius > 0.0;
    }
    if (name == QLatin1String("mesh")) {
      visual.type = RobotGeometryType::Mesh;
      visual.mesh_filename = reader.attributes().value("filename").toString();
      const QVector<double> scale =
        parse_scalar_list(reader.attributes().value("scale").toString(), 3);
      if (scale.size() == 3) {
        visual.mesh_scale_x = scale[0];
        visual.mesh_scale_y = scale[1];
        visual.mesh_scale_z = scale[2];
      }
      reader.skipCurrentElement();
      return !visual.mesh_filename.isEmpty();
    }

    reader.skipCurrentElement();
  }
  return false;
}

QString opengl_module_name(const QOpenGLContext::OpenGLModuleType module_type)
{
  switch (module_type) {
    case QOpenGLContext::LibGL:
      return "desktop";
    case QOpenGLContext::LibGLES:
      return "gles";
  }
  return "unknown";
}

QString probe_opengl_availability(bool &available, QString &failure_reason)
{
  QSurfaceFormat requested_format;
  requested_format.setDepthBufferSize(24);
  requested_format.setStencilBufferSize(0);
  requested_format.setSamples(4);

  QOpenGLContext probe_context;
  probe_context.setFormat(requested_format);
  available = probe_context.create();
  if (!available) {
    failure_reason = "QOpenGLContext::create() returned false";
    return QString("OpenGL available: no (%1); fallback backend=proxy").arg(failure_reason);
  }

  const QSurfaceFormat actual_format = probe_context.format();
  failure_reason.clear();
  return QString("OpenGL available: yes, module=%1, version=%2.%3")
    .arg(opengl_module_name(QOpenGLContext::openGLModuleType()))
    .arg(actual_format.majorVersion())
    .arg(actual_format.minorVersion());
}

}  // namespace

Pose2D RosWorker::compose_pose(const Pose2D &parent, const Pose2D &child)
{
  Pose2D pose;
  const Rotation3D parent_rotation = rotation_from_rpy(parent.roll, parent.pitch, parent.yaw);
  const auto child_translation = rotate_point(parent_rotation, child.x, child.y, child.z);
  pose.x = parent.x + child_translation[0];
  pose.y = parent.y + child_translation[1];
  pose.z = parent.z + child_translation[2];
  const Rotation3D child_rotation = rotation_from_rpy(child.roll, child.pitch, child.yaw);
  const Pose2D composed_orientation =
    pose_from_rotation(multiply_rotation(parent_rotation, child_rotation));
  pose.roll = composed_orientation.roll;
  pose.pitch = composed_orientation.pitch;
  pose.yaw = composed_orientation.yaw;
  pose.valid = parent.valid && child.valid;
  return pose;
}

RosWorker::RosWorker(QObject *parent)
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
  robot_description_topic_ =
    node_->declare_parameter<std::string>("robot_description_topic", robot_description_topic_);
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
  enable_robot_model_ =
    node_->declare_parameter<bool>("enable_robot_model", enable_robot_model_);
  enable_robot_meshes_ =
    node_->declare_parameter<bool>("enable_robot_meshes", enable_robot_meshes_);
  enable_tf_visualization_ =
    node_->declare_parameter<bool>("enable_tf_visualization", enable_tf_visualization_);
  enable_scan_visualization_ =
    node_->declare_parameter<bool>("enable_scan_visualization", enable_scan_visualization_);
  enable_map_visualization_ =
    node_->declare_parameter<bool>("enable_map_visualization", enable_map_visualization_);
  enable_costmap_visualization_ =
    node_->declare_parameter<bool>("enable_costmap_visualization", enable_costmap_visualization_);
  mesh_load_async_ =
    node_->declare_parameter<bool>("mesh_load_async", mesh_load_async_);
  robot_mesh_render_mode_ =
    node_->declare_parameter<std::string>("robot_mesh_render_mode", robot_mesh_render_mode_);
  robot_model_renderer_backend_ =
    node_->declare_parameter<std::string>(
      "robot_model_renderer_backend",
      robot_model_renderer_backend_);
  const QString requested_renderer_backend =
    QString::fromStdString(robot_model_renderer_backend_).trimmed().toLower();
  auto renderer = RobotModelRenderer::create(requested_renderer_backend);
  if (renderer->backendName() != requested_renderer_backend) {
    Q_EMIT eventReceived(
      QString("Invalid robot_model_renderer_backend '%1'; falling back to %2")
        .arg(QString::fromStdString(robot_model_renderer_backend_), renderer->backendName()));
  }
  bool opengl_available = false;
  QString opengl_failure_reason;
  const QString opengl_status = probe_opengl_availability(opengl_available, opengl_failure_reason);
  RCLCPP_INFO(node_->get_logger(), "%s", opengl_status.toStdString().c_str());
  if (renderer->backendName() == "opengl" && !opengl_available) {
    Q_EMIT eventReceived(
      QString("OpenGL robot renderer failed before startup: %1; falling back to proxy backend")
        .arg(opengl_failure_reason));
    renderer = RobotModelRenderer::create("proxy");
  }
  robot_model_renderer_backend_ = renderer->backendName().toStdString();
  robot_mesh_render_mode_ = renderer->meshRenderMode().toStdString();
  robot_renderer_loads_mesh_files_ = renderer->loadsMeshFiles();
  node_->set_parameter(rclcpp::Parameter("robot_model_renderer_backend", robot_model_renderer_backend_));
  node_->set_parameter(rclcpp::Parameter("robot_mesh_render_mode", robot_mesh_render_mode_));
  robot_mesh_auto_unit_scale_ =
    node_->declare_parameter<bool>("robot_mesh_auto_unit_scale", robot_mesh_auto_unit_scale_);
  robot_opengl_debug_camera_ =
    node_->declare_parameter<bool>("robot_opengl_debug_camera", robot_opengl_debug_camera_);
  robot_opengl_debug_axes_ =
    node_->declare_parameter<bool>("robot_opengl_debug_axes", robot_opengl_debug_axes_);
  robot_opengl_debug_cube_ =
    node_->declare_parameter<bool>("robot_opengl_debug_cube", robot_opengl_debug_cube_);
  robot_opengl_force_visible_ =
    node_->declare_parameter<bool>("robot_opengl_force_visible", robot_opengl_force_visible_);
  robot_opengl_stl_only_debug_ =
    node_->declare_parameter<bool>("robot_opengl_stl_only_debug", robot_opengl_stl_only_debug_);
  robot_opengl_debug_mesh_bbox_ =
    node_->declare_parameter<bool>("robot_opengl_debug_mesh_bbox", robot_opengl_debug_mesh_bbox_);
  robot_opengl_mesh_path_self_test_ =
    node_->declare_parameter<std::string>(
      "robot_opengl_mesh_path_self_test",
      robot_opengl_mesh_path_self_test_);
  robot_opengl_debug_size_m_ =
    node_->declare_parameter<double>("robot_opengl_debug_size_m", robot_opengl_debug_size_m_);
  costmap_emit_period_ms_ =
    node_->declare_parameter<int>("costmap_emit_period_ms", costmap_emit_period_ms_);
  tf_emit_period_ms_ =
    node_->declare_parameter<int>("tf_emit_period_ms", tf_emit_period_ms_);
  robot_model_emit_period_ms_ =
    node_->declare_parameter<int>("robot_model_emit_period_ms", robot_model_emit_period_ms_);
  scan_emit_period_ms_ =
    node_->declare_parameter<int>("scan_emit_period_ms", scan_emit_period_ms_);
  mesh_max_loaded_triangles_ =
    node_->declare_parameter<int>("mesh_max_loaded_triangles", mesh_max_loaded_triangles_);
  mesh_max_rendered_faces_ =
    node_->declare_parameter<int>("mesh_max_rendered_faces", mesh_max_rendered_faces_);
  mesh_max_file_size_mb_ =
    node_->declare_parameter<int>("mesh_max_file_size_mb", mesh_max_file_size_mb_);
  mesh_max_extent_m_ =
    node_->declare_parameter<double>("mesh_max_extent_m", mesh_max_extent_m_);
  mesh_max_abs_coordinate_m_ =
    node_->declare_parameter<double>("mesh_max_abs_coordinate_m", mesh_max_abs_coordinate_m_);
  mesh_max_projected_extent_px_ =
    node_->declare_parameter<double>("mesh_max_projected_extent_px", mesh_max_projected_extent_px_);
  robot_mesh_unit_scale_ =
    node_->declare_parameter<double>("robot_mesh_unit_scale", robot_mesh_unit_scale_);
  max_grid_cells_ =
    node_->declare_parameter<int>("max_grid_cells", max_grid_cells_);
  max_scan_points_ =
    node_->declare_parameter<int>("max_scan_points", max_scan_points_);
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("UI update throttling enabled: TF %1 ms, robot model %2 ms, scan %3 ms")
      .arg(tf_emit_period_ms_)
      .arg(robot_model_emit_period_ms_)
      .arg(scan_emit_period_ms_).toStdString().c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("Robot mesh limits: %1 loaded triangle(s), %2 rendered face(s), %3 MiB file size")
      .arg(mesh_max_loaded_triangles_)
      .arg(mesh_max_rendered_faces_)
      .arg(mesh_max_file_size_mb_).toStdString().c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("Robot mesh validation thresholds: mesh_max_extent_m=%1, mesh_max_abs_coordinate_m=%2, mesh_max_file_size_mb=%3, mesh_max_loaded_triangles=%4")
      .arg(mesh_max_extent_m_)
      .arg(mesh_max_abs_coordinate_m_)
      .arg(mesh_max_file_size_mb_)
      .arg(mesh_max_loaded_triangles_).toStdString().c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("OpenGL robot debug: camera=%1, axes=%2, cube=%3, force_visible=%4, stl_only=%5, mesh_bbox=%6, debug_size_m=%7")
      .arg(robot_opengl_debug_camera_ ? "true" : "false")
      .arg(robot_opengl_debug_axes_ ? "true" : "false")
      .arg(robot_opengl_debug_cube_ ? "true" : "false")
      .arg(robot_opengl_force_visible_ ? "true" : "false")
      .arg(robot_opengl_stl_only_debug_ ? "true" : "false")
      .arg(robot_opengl_debug_mesh_bbox_ ? "true" : "false")
      .arg(robot_opengl_debug_size_m_).toStdString().c_str());
  const QString mesh_self_test_raw_path = QString::fromStdString(robot_opengl_mesh_path_self_test_);
  if (!mesh_self_test_raw_path.trimmed().isEmpty()) {
    emit_mesh_path_access_diagnostics(
      "robot_opengl_mesh_path_self_test",
      mesh_self_test_raw_path,
      mesh_self_test_raw_path);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("Safe mode: robot model %1, robot meshes %2, mesh render mode %3, mesh async %4")
      .arg(enable_robot_model_ ? "enabled" : "disabled")
      .arg(enable_robot_meshes_ ? "enabled" : "disabled")
      .arg(QString::fromStdString(robot_mesh_render_mode_))
      .arg(mesh_load_async_ ? "requested" : "disabled").toStdString().c_str());
  RCLCPP_INFO(node_->get_logger(), "%s", renderer->statusMessage().toStdString().c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString(
      "Build/runtime feature summary: enable_robot_model=%1, enable_robot_meshes=%2, "
      "robot_mesh_render_mode=%3, mesh_max_loaded_triangles=%4, "
      "mesh_max_rendered_faces=%5, renderer_backend=%6, "
      "robot_opengl_debug_camera=%7, robot_opengl_debug_axes=%8, "
      "robot_opengl_debug_cube=%9, robot_opengl_force_visible=%10, "
      "robot_opengl_stl_only_debug=%11, robot_opengl_debug_mesh_bbox=%12, "
      "robot_opengl_debug_size_m=%13")
      .arg(enable_robot_model_ ? "true" : "false")
      .arg(enable_robot_meshes_ ? "true" : "false")
      .arg(QString::fromStdString(robot_mesh_render_mode_))
      .arg(mesh_max_loaded_triangles_)
      .arg(mesh_max_rendered_faces_)
      .arg(QString::fromStdString(robot_model_renderer_backend_))
      .arg(robot_opengl_debug_camera_ ? "true" : "false")
      .arg(robot_opengl_debug_axes_ ? "true" : "false")
      .arg(robot_opengl_debug_cube_ ? "true" : "false")
      .arg(robot_opengl_force_visible_ ? "true" : "false")
      .arg(robot_opengl_stl_only_debug_ ? "true" : "false")
      .arg(robot_opengl_debug_mesh_bbox_ ? "true" : "false")
      .arg(robot_opengl_debug_size_m_).toStdString().c_str());
  if (mesh_load_async_) {
    Q_EMIT eventReceived("mesh_load_async is not implemented yet; disabling mesh file loading for safety");
  }

  const auto latched_map_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto live_qos = rclcpp::SystemDefaultsQoS();

  if (enable_map_visualization_) {
    map_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, latched_map_qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        emit_diagnostic_once("first_map", "First map received");
        const uint64_t cell_count =
          static_cast<uint64_t>(message->info.width) * static_cast<uint64_t>(message->info.height);
        if (cell_count > static_cast<uint64_t>(std::max(max_grid_cells_, 1))) {
          emit_diagnostic_once(
            "huge_map",
            QString("Map rejected: %1 cell(s) exceeds max_grid_cells=%2")
              .arg(static_cast<qulonglong>(cell_count))
              .arg(max_grid_cells_));
          return;
        }
        Q_EMIT mapChanged(convert_grid(*message));
      });
  } else {
    Q_EMIT eventReceived("Map visualization disabled by enable_map_visualization");
  }
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
  robot_description_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    robot_description_topic_, latched_map_qos,
    [this](const std_msgs::msg::String::SharedPtr message) {
      emit_diagnostic_once("first_robot_description", "First robot_description received");
      if (!enable_robot_model_) {
        emit_diagnostic_once(
          "robot_model_disabled",
          "Robot model disabled: skipping robot_description parsing and model emit");
        return;
      }
      QElapsedTimer parse_timer;
      parse_timer.start();
      robot_description_visuals_ = parse_robot_description(message->data);
      Q_EMIT eventReceived(
        QString("parse_robot_description elapsed: %1 ms").arg(parse_timer.elapsed()));
      if (enable_tf_visualization_) {
        Q_EMIT tfFramesChanged(build_frame_visuals());
      }
      emit_robot_model_update("robot_description");
      const int mesh_visuals = std::count_if(
        robot_description_visuals_.begin(), robot_description_visuals_.end(),
        [](const RobotVisual &visual) {
          return visual.type == RobotGeometryType::Mesh;
        });
      Q_EMIT eventReceived(
        QString("Robot description loaded: %1 link(s), %2 joint(s), %3 visual(s), %4 mesh visual(s)")
          .arg(robot_description_link_count_)
          .arg(static_cast<int>(robot_joints_.size()))
          .arg(robot_description_visuals_.size())
          .arg(mesh_visuals));
      if (mesh_visuals == 0) {
        emit_diagnostic_once(
          "robot_description_no_mesh_visuals",
          "No URDF mesh visuals found; using proxy renderer");
      } else {
        for (const auto &visual : robot_description_visuals_) {
          if (visual.type != RobotGeometryType::Mesh) {
            continue;
          }
          const QString key = "mesh_visual:" + visual.frame_id + ":" + visual.mesh_filename;
          emit_diagnostic_once(
            key,
            QString("Mesh visual: frame=%1, uri=%2, resolved=%3, scale=%4 %5 %6, triangle count=pending, backend=%7")
              .arg(visual.frame_id)
              .arg(visual.mesh_filename)
              .arg(visual.mesh_resolved_path.isEmpty() ? "<unresolved>" : visual.mesh_resolved_path)
              .arg(visual.mesh_scale_x)
              .arg(visual.mesh_scale_y)
              .arg(visual.mesh_scale_z)
              .arg(QString::fromStdString(robot_model_renderer_backend_)));
        }
      }
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

void RosWorker::handle_tf_message(const tf2_msgs::msg::TFMessage &message, bool is_static)
{
  emit_diagnostic_once(
    is_static ? "first_tf_static" : "first_tf",
    is_static ? "First tf_static received" : "First tf received");
  auto &storage = is_static ? static_frames_ : dynamic_frames_;
  for (const auto &transform : message.transforms) {
    if (
      !std::isfinite(transform.transform.translation.x) ||
      !std::isfinite(transform.transform.translation.y) ||
      !std::isfinite(transform.transform.translation.z) ||
      !std::isfinite(transform.transform.rotation.x) ||
      !std::isfinite(transform.transform.rotation.y) ||
      !std::isfinite(transform.transform.rotation.z) ||
      !std::isfinite(transform.transform.rotation.w))
    {
      emit_diagnostic_once("invalid_tf_transform", "Invalid TF transform ignored: non-finite value");
      continue;
    }
    FrameVisual frame;
    frame.parent_frame = QString::fromStdString(transform.header.frame_id);
    frame.child_frame = QString::fromStdString(transform.child_frame_id);
    frame.pose.x = transform.transform.translation.x;
    frame.pose.y = transform.transform.translation.y;
    frame.pose.z = transform.transform.translation.z;
    apply_quaternion_orientation(
      frame.pose,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    frame.pose.valid = true;
    frame.is_static = is_static;
    storage[transform.child_frame_id] = frame;
  }

  const bool emit_tf = is_static || should_emit_now(last_tf_emit_time_, tf_emit_period_ms_);
  if (enable_tf_visualization_ && emit_tf) {
    Q_EMIT tfFramesChanged(build_frame_visuals());
  }

  if (
    enable_robot_model_ &&
    !robot_description_visuals_.isEmpty() &&
    (is_static || should_emit_now(last_robot_model_emit_time_, robot_model_emit_period_ms_)))
  {
    emit_robot_model_update(is_static ? "tf_static" : "tf");
  }
}

void RosWorker::emit_diagnostic_once(const QString &key, const QString &event)
{
  if (diagnostic_event_cache_.contains(key)) {
    return;
  }
  diagnostic_event_cache_.insert(key);
  Q_EMIT eventReceived(event);
}

void RosWorker::emit_mesh_path_access_diagnostics(
  const QString &label,
  const QString &uri,
  const QString &resolved_path)
{
  const QString key = QString("mesh_path_access:%1:%2:%3").arg(label, uri, resolved_path);
  if (diagnostic_event_cache_.contains(key)) {
    return;
  }
  diagnostic_event_cache_.insert(key);
  RCLCPP_INFO(
    node_->get_logger(),
    "%s",
    QString("%1 mesh path access diagnostics: %2")
      .arg(label, file_access_diagnostics_text(uri, resolved_path)).toStdString().c_str());
}

void RosWorker::emit_robot_model_update(const QString &reason)
{
  if (!enable_robot_model_) {
    return;
  }

  QElapsedTimer build_timer;
  build_timer.start();
  QVector<RobotVisual> visuals = build_robot_visuals();
  const qint64 elapsed_ms = build_timer.elapsed();
  emit_robot_visual_diagnostics_once(visuals, reason);
  ++robot_model_emit_count_;
  Q_EMIT robotModelChanged(visuals);
  last_robot_model_emit_time_ = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (robot_model_emit_count_ <= 3 || (robot_model_emit_count_ % 20) == 0 || elapsed_ms > 16) {
    RCLCPP_INFO(
      node_->get_logger(),
      "%s",
      QString("Robot model emit #%1 (%2): %3 visual(s), build_robot_visuals elapsed %4 ms")
        .arg(robot_model_emit_count_)
        .arg(reason)
        .arg(visuals.size())
        .arg(elapsed_ms).toStdString().c_str());
  }
}

void RosWorker::emit_robot_visual_diagnostics_once(
  const QVector<RobotVisual> &visuals,
  const QString &reason)
{
  int mesh_visual_count = 0;
  int opengl_mesh_candidate_count = 0;
  for (const auto &visual : visuals) {
    if (visual.type != RobotGeometryType::Mesh) {
      continue;
    }
    ++mesh_visual_count;
    if (
      visual.mesh_enabled &&
      visual.mesh_render_mode == "opengl" &&
      !visual.mesh_resolved_path.isEmpty())
    {
      ++opengl_mesh_candidate_count;
    }
  }

  const QString summary_key = QString("robot_visual_summary:%1:%2:%3:%4")
    .arg(reason)
    .arg(visuals.size())
    .arg(mesh_visual_count)
    .arg(opengl_mesh_candidate_count);
  if (!diagnostic_event_cache_.contains(summary_key)) {
    diagnostic_event_cache_.insert(summary_key);
    RCLCPP_INFO(
      node_->get_logger(),
      "%s",
      QString("RobotVisual build diagnostics (%1): total=%2, mesh visual count=%3, opengl mesh candidate count=%4")
        .arg(reason)
        .arg(visuals.size())
        .arg(mesh_visual_count)
        .arg(opengl_mesh_candidate_count).toStdString().c_str());
  }

  for (const auto &visual : visuals) {
    if (visual.type != RobotGeometryType::Mesh) {
      continue;
    }
    const bool opengl_mesh_visual = visual.mesh_enabled && visual.mesh_render_mode == "opengl";
    if (!opengl_mesh_visual) {
      continue;
    }
    if (visual.mesh_filename.isEmpty()) {
      continue;
    }
    if (visual.mesh_resolved_path.isEmpty()) {
      const QString key = QString("mesh_unresolved:%1:%2").arg(visual.frame_id, visual.mesh_filename);
      if (!diagnostic_event_cache_.contains(key)) {
        diagnostic_event_cache_.insert(key);
        RCLCPP_WARN(
          node_->get_logger(),
          "%s",
          QString("Mesh skipped: unresolved URI, frame=%1, uri=%2")
            .arg(visual.frame_id, visual.mesh_filename).toStdString().c_str());
      }
      continue;
    }
    const FileProbe probe = probeFilePath(
      QString("RobotVisual %1").arg(visual.frame_id),
      visual.mesh_resolved_path);
    emit_mesh_path_access_diagnostics(
      QString("RobotVisual %1").arg(visual.frame_id),
      visual.mesh_filename,
      visual.mesh_resolved_path);
    const bool accepted_for_opengl = !visual.mesh_resolved_path.isEmpty();
    const QString key = QString("robot_visual_mesh:%1:%2:%3:%4:%5")
      .arg(visual.frame_id)
      .arg(visual.mesh_filename)
      .arg(visual.mesh_resolved_path)
      .arg(visual.mesh_enabled ? "1" : "0")
      .arg(visual.mesh_render_mode);
    if (!diagnostic_event_cache_.contains(key)) {
      diagnostic_event_cache_.insert(key);
      RCLCPP_INFO(
        node_->get_logger(),
        "%s",
        QString("RobotVisual mesh diagnostic: frame_id=%1, uri=%2, resolved_path=%3, path_probe_status={%4}, visual_origin_included_in_pose=true, mesh_scale=%5 %6 %7, mesh_enabled=%8, mesh_render_mode=%9, accepted_for_opengl=%10, pose=(%11,%12,%13,%14,%15,%16), debug_axes=%17, debug_cube=%18, force_visible=%19, stl_only=%20, mesh_bbox=%21, debug_size_m=%22")
          .arg(visual.frame_id)
          .arg(visual.mesh_filename)
          .arg(visual.mesh_resolved_path)
          .arg(fileProbeToDiagnosticText(probe))
          .arg(visual.mesh_scale_x)
          .arg(visual.mesh_scale_y)
          .arg(visual.mesh_scale_z)
          .arg(visual.mesh_enabled ? "true" : "false")
          .arg(visual.mesh_render_mode)
          .arg(accepted_for_opengl ? "true" : "false")
          .arg(visual.pose.x)
          .arg(visual.pose.y)
          .arg(visual.pose.z)
          .arg(visual.pose.roll)
          .arg(visual.pose.pitch)
          .arg(visual.pose.yaw)
          .arg(visual.robot_opengl_debug_axes ? "true" : "false")
          .arg(visual.robot_opengl_debug_cube ? "true" : "false")
          .arg(visual.robot_opengl_force_visible ? "true" : "false")
          .arg(visual.robot_opengl_stl_only_debug ? "true" : "false")
          .arg(visual.robot_opengl_debug_mesh_bbox ? "true" : "false")
          .arg(visual.robot_opengl_debug_size_m).toStdString().c_str());
    }
  }
}

bool RosWorker::should_emit_now(rclcpp::Time &last_emit_time, const int period_ms) const
{
  if (!node_ || period_ms <= 0) {
    return true;
  }

  const rclcpp::Time now = node_->now();
  if (last_emit_time.nanoseconds() == 0 || now.nanoseconds() < last_emit_time.nanoseconds()) {
    last_emit_time = now;
    return true;
  }

  const double elapsed_ms = (now - last_emit_time).seconds() * 1000.0;
  if (elapsed_ms < period_ms) {
    return false;
  }

  last_emit_time = now;
  return true;
}

QVector<FrameVisual> RosWorker::build_frame_visuals() const
{
  QVector<FrameVisual> frames;
  std::set<std::string> emitted_frames;

  FrameVisual map_frame;
  map_frame.child_frame = "map";
  map_frame.pose.valid = true;
  map_frame.is_static = true;
  frames.push_back(map_frame);
  emitted_frames.insert("map");

  auto add_frame =
    [this, &frames, &emitted_frames](const std::string &child_frame) {
      if (child_frame.empty() || emitted_frames.count(child_frame) > 0) {
        return;
      }

      Pose2D pose = resolve_frame_pose(child_frame);
      if (!pose.valid) {
        pose = resolve_robot_link_pose(QString::fromStdString(child_frame));
      }
      if (!pose.valid) {
        return;
      }

      FrameVisual frame;
      frame.child_frame = QString::fromStdString(child_frame);
      if (const auto dynamic_it = dynamic_frames_.find(child_frame); dynamic_it != dynamic_frames_.end()) {
        frame.parent_frame = dynamic_it->second.parent_frame;
        frame.is_static = dynamic_it->second.is_static;
      } else if (const auto static_it = static_frames_.find(child_frame); static_it != static_frames_.end()) {
        frame.parent_frame = static_it->second.parent_frame;
        frame.is_static = true;
      } else if (const auto joint_it = robot_joints_.find(child_frame); joint_it != robot_joints_.end()) {
        frame.parent_frame = joint_it->second.parent_frame;
        frame.is_static = true;
      }
      frame.pose = pose;
      frame.pose.valid = true;
      frames.push_back(frame);
      emitted_frames.insert(child_frame);
    };

  for (const auto &[child_frame, frame] : static_frames_) {
    (void)frame;
    add_frame(child_frame);
  }
  for (const auto &[child_frame, frame] : dynamic_frames_) {
    (void)frame;
    add_frame(child_frame);
  }
  for (const auto &[child_frame, joint] : robot_joints_) {
    (void)joint;
    add_frame(child_frame);
  }

  return frames;
}

void RosWorker::update_global_costmap_subscription()
{
  global_costmap_subscription_.reset();
  if (!node_ || !subscribe_global_costmap_ || !enable_costmap_visualization_) {
    return;
  }

  const auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
  global_costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_, costmap_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      emit_diagnostic_once("first_global_costmap", "First global costmap received");
      const uint64_t cell_count =
        static_cast<uint64_t>(message->info.width) * static_cast<uint64_t>(message->info.height);
      if (cell_count > static_cast<uint64_t>(std::max(max_grid_cells_, 1))) {
        emit_diagnostic_once(
          "huge_global_costmap",
          QString("Global costmap rejected: %1 cell(s) exceeds max_grid_cells=%2")
            .arg(static_cast<qulonglong>(cell_count))
            .arg(max_grid_cells_));
        return;
      }
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
  if (!node_ || !subscribe_local_costmap_ || !enable_costmap_visualization_) {
    return;
  }

  const auto costmap_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  local_costmap_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, costmap_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      emit_diagnostic_once("first_local_costmap", "First local costmap received");
      const uint64_t cell_count =
        static_cast<uint64_t>(message->info.width) * static_cast<uint64_t>(message->info.height);
      if (cell_count > static_cast<uint64_t>(std::max(max_grid_cells_, 1))) {
        emit_diagnostic_once(
          "huge_local_costmap",
          QString("Local costmap rejected: %1 cell(s) exceeds max_grid_cells=%2")
            .arg(static_cast<qulonglong>(cell_count))
            .arg(max_grid_cells_));
        return;
      }
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
  if (!node_ || !subscribe_scan_ || !enable_scan_visualization_) {
    return;
  }

  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      emit_diagnostic_once("first_scan", "First scan received");
      if (!should_emit_now(last_scan_emit_time_, scan_emit_period_ms_)) {
        return;
      }
      Q_EMIT scanChanged(convert_scan(*message));
    });
  Q_EMIT eventReceived("Scan subscription enabled");
}

void RosWorker::spin()
{
  try {
    executor_.spin();
  } catch (const std::exception &error) {
    Q_EMIT eventReceived(QString("ROS executor stopped: %1").arg(error.what()));
  }
}

GridMap RosWorker::convert_grid(const nav_msgs::msg::OccupancyGrid &message) const
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

PathData RosWorker::convert_path(const nav_msgs::msg::Path &message) const
{
  PathData path;
  path.points.reserve(static_cast<int>(message.poses.size()));
  for (const auto &pose : message.poses) {
    path.points.push_back(QPointF(pose.pose.position.x, pose.pose.position.y));
  }
  return path;
}

Pose2D RosWorker::convert_pose(const geometry_msgs::msg::PoseStamped &message) const
{
  Pose2D pose;
  pose.x = message.pose.position.x;
  pose.y = message.pose.position.y;
  pose.z = message.pose.position.z;
  apply_quaternion_orientation(
    pose,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z,
    message.pose.orientation.w);
  pose.valid = true;
  return pose;
}

ScanData RosWorker::convert_scan(const sensor_msgs::msg::LaserScan &message) const
{
  ScanData scan;
  const Pose2D frame_pose = resolve_frame_pose(message.header.frame_id);
  if (!frame_pose.valid) {
    return scan;
  }

  const int max_scan_points = std::max(max_scan_points_, 1);
  const int stride = std::max(
    1,
    static_cast<int>(
      std::ceil(static_cast<double>(message.ranges.size()) / static_cast<double>(max_scan_points))));
  scan.points.reserve(std::min(static_cast<int>(message.ranges.size()), max_scan_points));
  double angle = message.angle_min;
  for (size_t i = 0; i < message.ranges.size(); ++i) {
    const float range = message.ranges[i];
    if (
      (static_cast<int>(i) % stride) == 0 &&
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

geometry_msgs::msg::PoseStamped RosWorker::to_pose_stamped(const Pose2D &pose) const
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = default_frame_id_;
  stamped.header.stamp = node_->now();
  stamped.pose.position.x = pose.x;
  stamped.pose.position.y = pose.y;
  stamped.pose.position.z = pose.z;
  stamped.pose.orientation = quaternion_from_yaw(pose.yaw);
  return stamped;
}

RuntimeSummary RosWorker::parse_runtime_summary(const std::string &payload) const
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
  summary.local_escape_active = object.value("local_escape_active").toBool(summary.local_escape_active);
  return summary;
}

QVector<RobotVisual> RosWorker::parse_robot_description(const std::string &payload)
{
  robot_joints_.clear();
  robot_description_link_count_ = 0;
  QVector<RobotVisual> visuals;
  QXmlStreamReader reader(QString::fromStdString(payload));

  while (reader.readNextStartElement()) {
    if (reader.name() != QLatin1String("robot")) {
      reader.skipCurrentElement();
      continue;
    }

    while (reader.readNextStartElement()) {
      if (reader.name() == QLatin1String("link")) {
        ++robot_description_link_count_;
        const QString frame_id = reader.attributes().value("name").toString();
        QVector<RobotVisual> link_visuals;
        QVector<RobotVisual> link_collisions;

        while (reader.readNextStartElement()) {
          const bool is_visual = reader.name() == QLatin1String("visual");
          const bool is_collision = reader.name() == QLatin1String("collision");
          if (!is_visual && !is_collision) {
            reader.skipCurrentElement();
            continue;
          }

          Pose2D origin;
          origin.valid = true;
          RobotVisual visual;
          bool parsed = false;

          while (reader.readNextStartElement()) {
            if (reader.name() == QLatin1String("origin")) {
              origin = parse_origin_attributes(reader.attributes());
              if (parsed) {
                visual.pose = origin;
              }
              reader.skipCurrentElement();
              continue;
            }
            if (reader.name() == QLatin1String("geometry")) {
              parsed = parse_geometry(reader, frame_id, origin, visual);
              continue;
            }
            reader.skipCurrentElement();
          }

          if (parsed) {
            if (visual.type == RobotGeometryType::Mesh) {
              visual.mesh_resolved_path = resolve_mesh_uri(visual.mesh_filename);
            }
            if (is_visual) {
              link_visuals.push_back(visual);
            } else {
              link_collisions.push_back(visual);
            }
          }
        }

        if (!link_visuals.isEmpty()) {
          visuals += link_visuals;
        } else {
          visuals += link_collisions;
        }
        continue;
      }

      if (reader.name() == QLatin1String("joint")) {
        RobotJoint joint;
        joint.child_frame.clear();
        joint.parent_frame.clear();
        joint.origin.valid = true;

        while (reader.readNextStartElement()) {
          if (reader.name() == QLatin1String("parent")) {
            joint.parent_frame = reader.attributes().value("link").toString();
            reader.skipCurrentElement();
            continue;
          }
          if (reader.name() == QLatin1String("child")) {
            joint.child_frame = reader.attributes().value("link").toString();
            reader.skipCurrentElement();
            continue;
          }
          if (reader.name() == QLatin1String("origin")) {
            joint.origin = parse_origin_attributes(reader.attributes());
            reader.skipCurrentElement();
            continue;
          }
          reader.skipCurrentElement();
        }

        joint.valid = !joint.parent_frame.isEmpty() && !joint.child_frame.isEmpty();
        if (joint.valid) {
          robot_joints_[joint.child_frame.toStdString()] = joint;
        }
        continue;
      }

      reader.skipCurrentElement();
    }
  }

  if (reader.hasError()) {
    Q_EMIT eventReceived(QString("Invalid robot_description URDF: %1").arg(reader.errorString()));
  }

  return visuals;
}

QString RosWorker::resolve_mesh_uri(const QString &uri)
{
  const QString normalized_uri = trim_accidental_mesh_suffix(uri);
  if (normalized_uri.isEmpty()) {
    return {};
  }

  auto emit_mesh_event_once = [this](const QString &event) {
      if (mesh_resolution_event_cache_.contains(event)) {
        return;
      }
      mesh_resolution_event_cache_.insert(event);
      const bool warning =
        event.contains("failed", Qt::CaseInsensitive) ||
        event.contains("not ", Qt::CaseInsensitive) ||
        event.contains("invalid", Qt::CaseInsensitive) ||
        event.contains("unsupported", Qt::CaseInsensitive) ||
        event.contains("empty", Qt::CaseInsensitive);
      if (warning) {
        RCLCPP_WARN(node_->get_logger(), "%s", event.toStdString().c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), "%s", event.toStdString().c_str());
      }
    };

  auto validate_stl_path =
    [this, &normalized_uri, &emit_mesh_event_once](const QString &path, const QString &label) -> QString {
      const QString normalized_path = normalized_filesystem_path(path);
      const FileProbe probe = probeFilePath(label, normalized_path);
      emit_mesh_path_access_diagnostics(label, normalized_uri, normalized_path);

      if (normalized_path.isEmpty() || probe.reason == "empty path") {
        emit_mesh_event_once(
          QString("Robot mesh candidate path is empty: uri=%1, label=%2")
            .arg(normalized_uri, label));
        return {};
      }
      if (!probe.exists) {
        emit_mesh_event_once(
          QString("Robot mesh not found: uri=%1, candidate='%2'")
            .arg(normalized_uri, normalized_path));
        return {};
      }
      if (!probe.is_file) {
        emit_mesh_event_once(
          QString("Robot mesh candidate is not a file: uri=%1, candidate='%2'")
            .arg(normalized_uri, normalized_path));
        return {};
      }
      if (!probe.readable || !probe.open_ok) {
        emit_mesh_event_once(
          QString("Robot mesh candidate is not readable: uri=%1, candidate='%2', readable=%3, open_ok=%4%5")
            .arg(normalized_uri)
            .arg(normalized_path)
            .arg(probe.readable ? "true" : "false")
            .arg(probe.open_ok ? "true" : "false")
            .arg(probe.open_ok ? QString() : QString(", QFile.error=%1").arg(probe.error_string)));
        return {};
      }

      const QFileInfo file_info(probe.cleaned_path);
      if (file_info.suffix().compare("stl", Qt::CaseInsensitive) != 0) {
        emit_mesh_event_once(
          QString("Unsupported robot mesh extension for %1: .%2")
            .arg(normalized_uri, file_info.suffix()));
        return {};
      }
      const QString resolved_path = probe.canonical_path.isEmpty() ? probe.absolute_path : probe.canonical_path;
      emit_mesh_event_once(
        QString("Robot mesh resolved: %1 -> %2").arg(normalized_uri, resolved_path));
      return resolved_path;
    };

  if (normalized_uri.startsWith("package://")) {
    const QString package_path = normalized_uri.mid(QString("package://").size());
    const int slash_index = package_path.indexOf('/');
    if (slash_index <= 0 || slash_index == package_path.size() - 1) {
      emit_mesh_event_once(QString("Invalid package mesh URI: %1").arg(normalized_uri));
      return {};
    }

    const QString package = package_path.left(slash_index);
    const QString relative_path = QDir::cleanPath(package_path.mid(slash_index + 1));
    try {
      const QString share_dir = QString::fromStdString(
        ament_index_cpp::get_package_share_directory(package.toStdString()));
      const QString candidate_path = normalized_filesystem_path(share_dir + "/" + relative_path);
      const QFileInfo candidate_info(candidate_path);
      emit_mesh_event_once(
        QString("Robot mesh package resolution: uri=%1, package=%2, relative_path=%3, ament_share_dir=%4, candidate='%5', candidate_exists=%6, candidate_readable=%7, candidate_size=%8")
          .arg(normalized_uri)
          .arg(package)
          .arg(relative_path)
          .arg(share_dir)
          .arg(candidate_path)
          .arg(candidate_info.exists() ? "true" : "false")
          .arg(candidate_info.isReadable() ? "true" : "false")
          .arg(candidate_info.exists() ? candidate_info.size() : 0));
      if (!candidate_info.exists() || !candidate_info.isFile() || !candidate_info.isReadable()) {
        emit_mesh_event_once(
          QString("Robot mesh package candidate QFileInfo failed: uri=%1, candidate='%2', absolute=%3, canonical=%4, exists=%5, isFile=%6, readable=%7, size=%8")
            .arg(normalized_uri)
            .arg(candidate_path)
            .arg(candidate_info.absoluteFilePath())
            .arg(candidate_info.canonicalFilePath().isEmpty() ? QString("<empty>") : candidate_info.canonicalFilePath())
            .arg(candidate_info.exists() ? "true" : "false")
            .arg(candidate_info.isFile() ? "true" : "false")
            .arg(candidate_info.isReadable() ? "true" : "false")
            .arg(candidate_info.exists() ? candidate_info.size() : 0));
      }
      return validate_stl_path(candidate_path, QString("package:// %1").arg(package));
    } catch (const std::exception &error) {
      emit_mesh_event_once(
        QString("Robot mesh package resolution failed for %1: %2")
          .arg(normalized_uri, QString::fromLocal8Bit(error.what())));
      return {};
    }
  }

  if (normalized_uri.startsWith("file://")) {
    return validate_stl_path(QUrl(normalized_uri).toLocalFile(), "file://");
  }

  const QFileInfo file_info(normalized_uri);
  if (file_info.isAbsolute()) {
    return validate_stl_path(file_info.absoluteFilePath(), "absolute mesh path");
  }

  emit_mesh_event_once(QString("Relative robot mesh URI is not supported: %1").arg(normalized_uri));
  return {};
}

QVector<RobotVisual> RosWorker::build_robot_visuals() const
{
  QVector<RobotVisual> visuals;
  visuals.reserve(robot_description_visuals_.size());
  std::set<QString> visual_frames;

  for (const auto &source_visual : robot_description_visuals_) {
    const Pose2D frame_pose = resolve_robot_link_pose(source_visual.frame_id);
    if (!frame_pose.valid) {
      continue;
    }

    RobotVisual visual = source_visual;
    visual.mesh_enabled = enable_robot_meshes_ && robot_renderer_loads_mesh_files_ && !mesh_load_async_;
    visual.mesh_render_mode = QString::fromStdString(robot_mesh_render_mode_);
    visual.mesh_max_loaded_triangles = std::max(mesh_max_loaded_triangles_, 1);
    visual.mesh_max_rendered_faces = std::max(mesh_max_rendered_faces_, 1);
    visual.mesh_max_file_size_mb = std::max(mesh_max_file_size_mb_, 1);
    visual.mesh_max_extent_m = std::max(mesh_max_extent_m_, 0.001);
    visual.mesh_max_abs_coordinate_m = std::max(mesh_max_abs_coordinate_m_, 0.001);
    visual.mesh_max_projected_extent_px = std::max(mesh_max_projected_extent_px_, 1.0);
    visual.mesh_auto_unit_scale = robot_mesh_auto_unit_scale_;
    visual.mesh_unit_scale = std::max(robot_mesh_unit_scale_, 0.000001);
    visual.robot_opengl_debug_camera = robot_opengl_debug_camera_;
    visual.robot_opengl_debug_axes = robot_opengl_debug_axes_;
    visual.robot_opengl_debug_cube = robot_opengl_debug_cube_;
    visual.robot_opengl_force_visible = robot_opengl_force_visible_;
    visual.robot_opengl_stl_only_debug = robot_opengl_stl_only_debug_;
    visual.robot_opengl_debug_mesh_bbox = robot_opengl_debug_mesh_bbox_;
    visual.robot_opengl_debug_size_m = std::clamp(robot_opengl_debug_size_m_, 0.01, 5.0);
    // Compose world/link pose with the URDF visual origin exactly once. The OpenGL
    // model matrix later applies only the URDF mesh scale to raw STL vertices.
    visual.pose = compose_pose(frame_pose, source_visual.pose);
    visual.valid = visual.pose.valid;
    visuals.push_back(visual);
    visual_frames.insert(source_visual.frame_id);
  }

  auto add_proxy_visual = [this, &visuals, &visual_frames](const QString &frame_id) {
      if (visual_frames.count(frame_id) > 0) {
        return;
      }

      const QString key = frame_id.toLower();
      const bool is_base =
        key.contains("base_link") || key.contains("base_footprint") || key.contains("base_plate");
      const bool is_wheel = key.contains("wheel") || key.contains("tire");
      const bool is_scan = key.contains("scan") || key.contains("lidar") || key.contains("lds");
      const bool is_caster = key.contains("caster");
      if (!is_base && !is_wheel && !is_scan && !is_caster) {
        return;
      }

      const Pose2D pose = resolve_robot_link_pose(frame_id);
      if (!pose.valid) {
        return;
      }

      RobotVisual visual;
      visual.frame_id = frame_id;
      visual.pose = pose;
      visual.valid = true;
      visual.type = RobotGeometryType::Mesh;
      visual.mesh_filename = frame_id;
      visual.mesh_enabled = enable_robot_meshes_ && robot_renderer_loads_mesh_files_ && !mesh_load_async_;
      visual.mesh_render_mode = QString::fromStdString(robot_mesh_render_mode_);
      visual.robot_opengl_debug_camera = robot_opengl_debug_camera_;
      visual.robot_opengl_debug_axes = robot_opengl_debug_axes_;
      visual.robot_opengl_debug_cube = robot_opengl_debug_cube_;
      visual.robot_opengl_force_visible = robot_opengl_force_visible_;
      visual.robot_opengl_stl_only_debug = robot_opengl_stl_only_debug_;
      visual.robot_opengl_debug_mesh_bbox = robot_opengl_debug_mesh_bbox_;
      visual.robot_opengl_debug_size_m = std::clamp(robot_opengl_debug_size_m_, 0.01, 5.0);
      visual_frames.insert(frame_id);
      visuals.push_back(visual);
    };

  for (const auto &[child_frame, joint] : robot_joints_) {
    (void)joint;
    add_proxy_visual(QString::fromStdString(child_frame));
  }
  for (const auto &[child_frame, frame] : static_frames_) {
    (void)frame;
    add_proxy_visual(QString::fromStdString(child_frame));
  }
  for (const auto &[child_frame, frame] : dynamic_frames_) {
    (void)frame;
    add_proxy_visual(QString::fromStdString(child_frame));
  }

  return visuals;
}

Pose2D RosWorker::resolve_robot_link_pose(const QString &link_frame) const
{
  const Pose2D tf_pose = resolve_frame_pose(link_frame.toStdString());
  if (tf_pose.valid) {
    return tf_pose;
  }

  std::function<Pose2D(const QString &, int)> resolve =
    [this, &resolve](const QString &frame_name, int depth) -> Pose2D {
      if (depth > 32) {
        return {};
      }

      const Pose2D direct_tf_pose = resolve_frame_pose(frame_name.toStdString());
      if (direct_tf_pose.valid) {
        return direct_tf_pose;
      }

      const auto joint_it = robot_joints_.find(frame_name.toStdString());
      if (joint_it == robot_joints_.end() || !joint_it->second.valid) {
        return {};
      }

      const Pose2D parent_pose = resolve(joint_it->second.parent_frame, depth + 1);
      if (!parent_pose.valid) {
        return {};
      }
      return compose_pose(parent_pose, joint_it->second.origin);
    };

  return resolve(link_frame, 0);
}

Pose2D RosWorker::resolve_frame_pose(const std::string &child_frame) const
{
  const auto find_frame = [this](const std::string &child) -> const FrameVisual *{
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
    [&](const std::string &frame_name, int depth) -> Pose2D {
      if (depth > 16) {
        return {};
      }
      if (frame_name == default_frame_id_ || frame_name == "map") {
        Pose2D identity;
        identity.valid = true;
        return identity;
      }

      const FrameVisual *frame = find_frame(frame_name);
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

void RosWorker::sendSingleGoal(const Pose2D &pose)
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
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle) {
      const QString state = handle ? "Accepted" : "Rejected";
      Q_EMIT goalStateChanged(state);
      Q_EMIT eventReceived(QString("Single goal %1").arg(state.toLower()));
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result) {
      const QString state = QString("Finished (%1)").arg(static_cast<int>(result.code));
      Q_EMIT goalStateChanged(state);
      Q_EMIT navigationCompleted(result.code == rclcpp_action::ResultCode::SUCCEEDED);
      Q_EMIT eventReceived(QString("Single goal %1").arg(state));
    };
  navigate_to_pose_client_->async_send_goal(goal, options);
  Q_EMIT goalStateChanged("Sending");
  Q_EMIT eventReceived("Single goal sent");
}

void RosWorker::sendRoute(const QVector<Pose2D> &route)
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
  for (const auto &pose : route) {
    if (pose.valid) {
      goal.goal_poses.push_back(to_pose_stamped(pose));
    }
  }
  if (goal.goal_poses.empty()) {
    return;
  }
  rclcpp_action::Client<NavigateToPoses>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::SharedPtr &handle) {
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
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPoses>::WrappedResult &result) {
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

void RosWorker::publishInitialPose(const Pose2D &pose)
{
  if (!node_ || !pose.valid) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = default_frame_id_;
  message.header.stamp = node_->now();
  message.pose.pose.position.x = pose.x;
  message.pose.pose.position.y = pose.y;
  message.pose.pose.position.z = pose.z;
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
