#include "amr_visualization/scene_widget.hpp"

#include "amr_visualization/robot_open_gl_widget.hpp"

#include <QByteArray>
#include <QDataStream>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QRectF>
#include <QResizeEvent>
#include <QStyle>
#include <QTransform>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr::visualization
{

namespace
{

QColor with_alpha(QColor color, int alpha)
{
  color.setAlpha(alpha);
  return color;
}

bool finite_vector(const QVector3D &point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

bool finite_point(const QPointF &point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

double triangle_area(const QPointF &a, const QPointF &b, const QPointF &c)
{
  return std::abs(
    ((b.x() - a.x()) * (c.y() - a.y())) -
    ((b.y() - a.y()) * (c.x() - a.x()))) * 0.5;
}

constexpr int k_rviz_background = 48;
constexpr int k_rviz_grid_alpha = 64;
constexpr double k_half_pi = 1.5707963267948966;
constexpr double k_pi = 3.14159265358979323846;
constexpr double k_min_camera_pitch = 0.15;
constexpr double k_max_camera_pitch = k_half_pi;

QVector3D rotate_rpy(
  const QVector3D &point,
  const double roll,
  const double pitch,
  const double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  return QVector3D(
    static_cast<float>(
      ((cy * cp) * point.x()) +
      (((cy * sp * sr) - (sy * cr)) * point.y()) +
      (((cy * sp * cr) + (sy * sr)) * point.z())),
    static_cast<float>(
      ((sy * cp) * point.x()) +
      (((sy * sp * sr) + (cy * cr)) * point.y()) +
      (((sy * sp * cr) - (cy * sr)) * point.z())),
    static_cast<float>(
      ((-sp) * point.x()) +
      ((cp * sr) * point.y()) +
      ((cp * cr) * point.z())));
}

}  // namespace

SceneWidget::SceneWidget(QWidget *parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumSize(640, 480);

  robot_open_gl_widget_ = new RobotOpenGLWidget(this);
  connect(
    robot_open_gl_widget_,
    &RobotOpenGLWidget::visualizationEvent,
    this,
    &SceneWidget::visualizationEvent,
    Qt::QueuedConnection);

  camera_controls_ = new QWidget(this);
  camera_controls_->setObjectName("cameraControls");
  auto *controls_layout = new QHBoxLayout(camera_controls_);
  controls_layout->setContentsMargins(4, 4, 4, 4);
  controls_layout->setSpacing(4);

  follow_robot_button_ =
    makeCameraButton(style()->standardIcon(QStyle::SP_ArrowForward), "F", "Follow robot", true);
  reset_view_button_ =
    makeCameraButton(style()->standardIcon(QStyle::SP_BrowserReload), "R", "Reset camera", false);
  controls_layout->addWidget(follow_robot_button_);
  controls_layout->addWidget(reset_view_button_);

  connect(follow_robot_button_, &QToolButton::toggled, this, &SceneWidget::setFollowRobotEnabled);
  connect(reset_view_button_, &QToolButton::clicked, this, &SceneWidget::resetView);
  updateCameraControlState();
  syncOpenGLRobotViewport();
  updateCameraControlsGeometry();
}

Pose2D SceneWidget::aimPose() const
{
  return aim_pose_;
}

QVector<Pose2D> SceneWidget::waypoints() const
{
  return waypoints_;
}

int SceneWidget::selectedWaypointIndex() const
{
  return selected_waypoint_index_;
}

bool SceneWidget::followRobotEnabled() const
{
  return follow_robot_;
}

void SceneWidget::resetView()
{
  scale_ = 90.0;
  focal_point_ = QVector3D(0.0F, 0.0F, 0.0F);
  camera_yaw_ = 0.0;
  camera_pitch_ = k_half_pi;
  camera_distance_ = 8.0;
  follow_robot_ = false;
  updateCameraControlState();
  syncOpenGLRobotViewport();
  update();
}

void SceneWidget::setMap(const GridMap &map)
{
  map_ = map;
  map_image_ = makeGridImage(map_, QColor("#000000"), QColor("#ffffff"));
  update();
}

void SceneWidget::setTfFrames(const QVector<amr::visualization::FrameVisual> &frames)
{
  tf_frames_ = frames;
  update();
}

void SceneWidget::setRobotModel(const QVector<amr::visualization::RobotVisual> &visuals)
{
  QElapsedTimer set_timer;
  set_timer.start();
  robot_visuals_ = visuals;
  QSet<QString> active_mesh_paths;
  for (const auto &visual : robot_visuals_) {
    if (
      visual.type == RobotGeometryType::Mesh &&
      visual.mesh_enabled &&
      visual.mesh_render_mode == "wireframe" &&
      !visual.mesh_resolved_path.isEmpty())
    {
      active_mesh_paths.insert(visual.mesh_resolved_path);
    }
  }
  for (auto it = mesh_cache_.begin(); it != mesh_cache_.end();) {
    if (active_mesh_paths.contains(it.key())) {
      ++it;
    } else {
      mesh_warning_cache_.remove(it.key());
      mesh_success_cache_.remove(it.key());
      it = mesh_cache_.erase(it);
    }
  }
  for (const auto &visual : robot_visuals_) {
    if (visual.type != RobotGeometryType::Mesh) {
      continue;
    }
    if (!visual.mesh_enabled || visual.mesh_render_mode == "proxy") {
      if (!diagnostic_event_cache_.contains("mesh_loading_disabled")) {
        diagnostic_event_cache_.insert("mesh_loading_disabled");
        Q_EMIT visualizationEvent("Robot mesh triangle rendering disabled; rendering proxy visuals");
      }
      continue;
    }
    if (visual.mesh_render_mode != "wireframe") {
      continue;
    }
    if (visual.mesh_resolved_path.isEmpty()) {
      continue;
    }
    if (mesh_cache_.contains(visual.mesh_resolved_path)) {
      continue;
    }

    MeshCacheEntry entry;
    entry.attempted = true;
    loadStlMesh(visual, entry);
    mesh_cache_.insert(visual.mesh_resolved_path, entry);

    if (entry.triangles.isEmpty()) {
      if (!mesh_warning_cache_.contains(visual.mesh_resolved_path)) {
        mesh_warning_cache_.insert(visual.mesh_resolved_path);
        Q_EMIT visualizationEvent(
          QString("Robot mesh load failed for %1: %2").arg(visual.mesh_filename, entry.error));
      }
      continue;
    }

    if (!mesh_success_cache_.contains(visual.mesh_resolved_path)) {
      mesh_success_cache_.insert(visual.mesh_resolved_path);
      Q_EMIT visualizationEvent(
        QString("Robot mesh loaded: %1 (%2 triangle(s), bbox=%3, scale=%4 %5 %6, frame=%7, mode=%8)")
          .arg(visual.mesh_filename)
          .arg(entry.triangles.size())
          .arg(meshBoundsText(entry))
          .arg(visual.mesh_scale_x)
          .arg(visual.mesh_scale_y)
          .arg(visual.mesh_scale_z)
          .arg(visual.frame_id)
          .arg(visual.mesh_render_mode));
    }
  }

  int mesh_visuals = 0;
  int opengl_mesh_candidates = 0;
  QString mesh_render_mode = "proxy";
  for (const auto &visual : robot_visuals_) {
    if (visual.type != RobotGeometryType::Mesh) {
      continue;
    }
    ++mesh_visuals;
    mesh_render_mode = visual.mesh_render_mode;
    if (
      visual.mesh_enabled &&
      visual.mesh_render_mode == "opengl" &&
      !visual.mesh_resolved_path.isEmpty())
    {
      ++opengl_mesh_candidates;
    }
  }
  if (mesh_visuals > 0 && (mesh_render_mode != "opengl" || opengl_mesh_candidates == 0)) {
    int loaded_meshes = 0;
    int rejected_meshes = 0;
    for (auto it = mesh_cache_.cbegin(); it != mesh_cache_.cend(); ++it) {
      if (it->rejected) {
        ++rejected_meshes;
      } else if (!it->triangles.isEmpty()) {
        ++loaded_meshes;
      }
    }
    const QString summary_key = QString("mesh_summary:%1:%2:%3:%4")
      .arg(mesh_render_mode)
      .arg(mesh_visuals)
      .arg(loaded_meshes)
      .arg(rejected_meshes);
    if (!diagnostic_event_cache_.contains(summary_key)) {
      diagnostic_event_cache_.insert(summary_key);
      Q_EMIT visualizationEvent(
        QString("Robot description mesh render summary: backend=%1, visual elements=%2, loaded meshes=%3, rejected meshes=%4")
          .arg(mesh_render_mode)
          .arg(mesh_visuals)
          .arg(loaded_meshes)
          .arg(rejected_meshes));
    }
  }
  if (robot_open_gl_widget_) {
    robot_open_gl_widget_->setRobotVisuals(robot_visuals_);
    syncOpenGLRobotViewport();
  }
  const qint64 elapsed_ms = set_timer.elapsed();
  if (!diagnostic_event_cache_.contains("set_robot_model_first")) {
    diagnostic_event_cache_.insert("set_robot_model_first");
    Q_EMIT visualizationEvent(
      QString("setRobotModel elapsed: %1 ms (%2 visual(s))").arg(elapsed_ms).arg(robot_visuals_.size()));
  } else if (elapsed_ms > 33 && !diagnostic_event_cache_.contains("set_robot_model_slow_33")) {
    diagnostic_event_cache_.insert("set_robot_model_slow_33");
    Q_EMIT visualizationEvent(QString("setRobotModel slow: %1 ms (>33 ms)").arg(elapsed_ms));
  } else if (elapsed_ms > 16 && !diagnostic_event_cache_.contains("set_robot_model_slow_16")) {
    diagnostic_event_cache_.insert("set_robot_model_slow_16");
    Q_EMIT visualizationEvent(QString("setRobotModel slow: %1 ms (>16 ms)").arg(elapsed_ms));
  }
  update();
}

void SceneWidget::setGlobalCostmap(const GridMap &map)
{
  global_costmap_ = map;
  global_costmap_image_ = makeCostmapImage(global_costmap_);
  update();
}

void SceneWidget::setLocalCostmap(const GridMap &map)
{
  local_costmap_ = map;
  local_costmap_image_ = makeCostmapImage(local_costmap_);
  update();
}

void SceneWidget::setScan(const amr::visualization::ScanData &scan)
{
  scan_ = scan;
  update();
}

void SceneWidget::setRobotPose(const Pose2D &pose)
{
  robot_pose_ = pose;
  if (follow_robot_) {
    centerViewOnRobot();
  }
  syncOpenGLRobotViewport();
  update();
}

void SceneWidget::setGlobalPath(const PathData &path)
{
  global_path_ = path;
  update();
}

void SceneWidget::setLocalPath(const PathData &path)
{
  local_path_ = path;
  update();
}

void SceneWidget::setGridVisible(bool visible) { show_grid_ = visible; update(); }
void SceneWidget::setMapVisible(bool visible) { show_map_ = visible; update(); }
void SceneWidget::setGlobalCostmapVisible(bool visible) { show_global_costmap_ = visible; update(); }
void SceneWidget::setLocalCostmapVisible(bool visible) { show_local_costmap_ = visible; update(); }
void SceneWidget::setFootprintVisible(bool visible) { show_footprint_ = visible; update(); }
void SceneWidget::setRobotVisible(bool visible) { show_robot_ = visible; syncOpenGLRobotViewport(); update(); }
void SceneWidget::setTfVisible(bool visible) { show_tf_ = visible; update(); }
void SceneWidget::setScanVisible(bool visible) { show_scan_ = visible; update(); }
void SceneWidget::setGlobalPathVisible(bool visible) { show_global_path_ = visible; update(); }
void SceneWidget::setLocalPathVisible(bool visible) { show_local_path_ = visible; update(); }
void SceneWidget::setAddWaypointMode(bool enabled) { add_waypoint_mode_ = enabled; }

void SceneWidget::addAimAsWaypoint()
{
  if (!aim_pose_.valid) {
    return;
  }
  waypoints_.push_back(aim_pose_);
  selected_waypoint_index_ = waypoints_.size() - 1;
  aim_pose_ = {};
  Q_EMIT aimPoseChanged(aim_pose_);
  emitWaypointState();
  update();
}

void SceneWidget::clearWaypoints()
{
  waypoints_.clear();
  selected_waypoint_index_ = -1;
  emitWaypointState();
  update();
}

void SceneWidget::clearSchedule()
{
  aim_pose_ = {};
  waypoints_.clear();
  selected_waypoint_index_ = -1;
  Q_EMIT aimPoseChanged(aim_pose_);
  emitWaypointState();
  update();
}

void SceneWidget::clearNavigationOverlays()
{
  global_path_ = {};
  local_path_ = {};
  clearSchedule();
}

void SceneWidget::clearAimPose()
{
  if (!aim_pose_.valid) {
    return;
  }
  aim_pose_ = {};
  Q_EMIT aimPoseChanged(aim_pose_);
  update();
}

void SceneWidget::removeSelectedWaypoint()
{
  if (selected_waypoint_index_ < 0 || selected_waypoint_index_ >= waypoints_.size()) {
    return;
  }
  waypoints_.removeAt(selected_waypoint_index_);
  if (waypoints_.empty()) {
    selected_waypoint_index_ = -1;
  } else {
    selected_waypoint_index_ =
      std::min(selected_waypoint_index_, static_cast<int>(waypoints_.size()) - 1);
  }
  clearAimPose();
  emitWaypointState();
  update();
}

void SceneWidget::moveSelectedWaypointUp()
{
  if (selected_waypoint_index_ <= 0 || selected_waypoint_index_ >= waypoints_.size()) {
    return;
  }
  std::swap(waypoints_[selected_waypoint_index_], waypoints_[selected_waypoint_index_ - 1]);
  --selected_waypoint_index_;
  emitWaypointState();
  update();
}

void SceneWidget::moveSelectedWaypointDown()
{
  if (selected_waypoint_index_ < 0 || selected_waypoint_index_ >= waypoints_.size() - 1) {
    return;
  }
  std::swap(waypoints_[selected_waypoint_index_], waypoints_[selected_waypoint_index_ + 1]);
  ++selected_waypoint_index_;
  emitWaypointState();
  update();
}

void SceneWidget::selectWaypoint(int index)
{
  selected_waypoint_index_ = index >= 0 && index < waypoints_.size() ? index : -1;
  Q_EMIT selectedWaypointChanged(selected_waypoint_index_);
  update();
}

void SceneWidget::setFollowRobotEnabled(bool enabled)
{
  follow_robot_ = enabled;
  if (follow_robot_) {
    centerViewOnRobot();
  }
  updateCameraControlState();
  update();
}

void SceneWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(k_rviz_background, k_rviz_background, k_rviz_background));

  if (show_map_) {
    drawGridLayer(painter, map_, map_image_, 1.0);
  }
  if (show_global_costmap_) {
    drawGridLayer(painter, global_costmap_, global_costmap_image_, 0.74);
  }
  if (show_local_costmap_) {
    drawGridLayer(painter, local_costmap_, local_costmap_image_, 0.82);
  }
  if (show_grid_) {
    drawGrid(painter);
  }
  if (show_scan_) {
    drawScan(painter);
  }
  if (show_global_path_) {
    drawPath(painter, global_path_, QColor(255, 96, 64), 3.5);
  }
  if (show_local_path_) {
    drawPath(painter, local_path_, QColor(0, 200, 255), 3.5);
  }
  if (show_footprint_) {
    drawExactFootprint(painter);
  }
  if (show_robot_) {
    drawRobotModel(painter);
  }

  drawWaypointRoute(painter);
  drawWaypoints(painter);
  if (aim_pose_.valid) {
    drawPose(painter, aim_pose_, QColor(255, 180, 0));
  }
  if (show_tf_) {
    drawTfFrames(painter);
  }
}

void SceneWidget::mousePressEvent(QMouseEvent *event)
{
  if (event->modifiers() &Qt::ShiftModifier) {
    orbiting_camera_ = true;
    last_mouse_position_ = event->pos();
    return;
  }

  if (
    event->button() == Qt::RightButton || event->button() == Qt::MiddleButton ||
    (event->button() == Qt::LeftButton && !add_waypoint_mode_))
  {
    panning_ = true;
    follow_robot_ = false;
    updateCameraControlState();
    last_mouse_position_ = event->pos();
    return;
  }
  if (event->button() == Qt::LeftButton) {
    const int waypoint_index = waypointAt(event->position());
    if (waypoint_index >= 0) {
      selectWaypoint(waypoint_index);
      return;
    }
    if (!add_waypoint_mode_) {
      return;
    }
    const QPointF world = screenToWorld(event->position());
    aim_pose_.x = world.x();
    aim_pose_.y = world.y();
    aim_pose_.valid = true;
    setting_aim_heading_ = true;
    Q_EMIT aimPoseChanged(aim_pose_);
    update();
  }
}

void SceneWidget::mouseMoveEvent(QMouseEvent *event)
{
  if (orbiting_camera_) {
    const QPoint delta = event->pos() - last_mouse_position_;
    camera_yaw_ = std::fmod(
      camera_yaw_ - (static_cast<double>(delta.x()) * 0.006) + (2.0 * k_pi),
      2.0 * k_pi);
    camera_pitch_ = std::clamp(
      camera_pitch_ + (static_cast<double>(delta.y()) * 0.006),
      k_min_camera_pitch,
      k_max_camera_pitch);
    last_mouse_position_ = event->pos();
    syncOpenGLRobotViewport();
    update();
    return;
  }

  if (setting_aim_heading_ && aim_pose_.valid) {
    const QPointF world = screenToWorld(event->position());
    aim_pose_.yaw = std::atan2(world.y() - aim_pose_.y, world.x() - aim_pose_.x);
    Q_EMIT aimPoseChanged(aim_pose_);
    update();
    return;
  }

  if (!panning_) {
    return;
  }
  const QPoint delta = event->pos() - last_mouse_position_;
  panCameraByPixels(delta);
  last_mouse_position_ = event->pos();
  syncOpenGLRobotViewport();
  update();
}

void SceneWidget::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    panning_ = false;
  }
  if (event->button() == Qt::LeftButton) {
    panning_ = false;
  }
  if (orbiting_camera_) {
    orbiting_camera_ = false;
  }
  if (event->button() == Qt::LeftButton) {
    if (setting_aim_heading_ && add_waypoint_mode_ && aim_pose_.valid) {
      waypoints_.push_back(aim_pose_);
      selected_waypoint_index_ = waypoints_.size() - 1;
      aim_pose_ = {};
      Q_EMIT aimPoseChanged(aim_pose_);
      emitWaypointState();
    }
    setting_aim_heading_ = false;
    update();
  }
}

void SceneWidget::wheelEvent(QWheelEvent *event)
{
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 0.89;
  scale_ = std::clamp(scale_ * factor, 18.0, 540.0);
  camera_distance_ = std::clamp(camera_distance_ / factor, 1.0, 80.0);
  if (follow_robot_) {
    centerViewOnRobot();
  }
  syncOpenGLRobotViewport();
  update();
}

void SceneWidget::resizeEvent(QResizeEvent *event)
{
  QWidget::resizeEvent(event);
  syncOpenGLRobotViewport();
  updateCameraControlsGeometry();
  if (follow_robot_) {
    centerViewOnRobot();
    syncOpenGLRobotViewport();
  }
}

QPointF SceneWidget::worldToScreen(const QPointF &point) const
{
  return worldToScreen3D(point.x(), point.y(), 0.0);
}

QPointF SceneWidget::worldToScreen3D(const double x, const double y, const double z) const
{
  const QVector3D point(
    static_cast<float>(x),
    static_cast<float>(y),
    static_cast<float>(z));
  const QVector3D relative = point - focal_point_;
  return QPointF(
    (width() * 0.5) + (QVector3D::dotProduct(relative, cameraRight()) * scale_),
    (height() * 0.5) - (QVector3D::dotProduct(relative, cameraUp()) * scale_));
}

QVector3D SceneWidget::cameraRight() const
{
  return QVector3D(
    static_cast<float>(std::cos(camera_yaw_)),
    static_cast<float>(std::sin(camera_yaw_)),
    0.0F).normalized();
}

QVector3D SceneWidget::cameraForward() const
{
  const QVector3D ground_up(
    static_cast<float>(-std::sin(camera_yaw_)),
    static_cast<float>(std::cos(camera_yaw_)),
    0.0F);
  return QVector3D(
    (ground_up.x() * static_cast<float>(std::cos(camera_pitch_))),
    (ground_up.y() * static_cast<float>(std::cos(camera_pitch_))),
    static_cast<float>(-std::sin(camera_pitch_))).normalized();
}

QVector3D SceneWidget::cameraUp() const
{
  return QVector3D::crossProduct(cameraRight(), cameraForward()).normalized();
}

QPointF SceneWidget::screenToWorld(const QPointF &point) const
{
  const QVector3D origin =
    focal_point_ +
    (cameraRight() * static_cast<float>((point.x() - (width() * 0.5)) / scale_)) -
    (cameraUp() * static_cast<float>((point.y() - (height() * 0.5)) / scale_)) -
    (cameraForward() * static_cast<float>(camera_distance_));
  const QVector3D direction = cameraForward();
  if (std::abs(direction.z()) < 1e-5F) {
    return QPointF(focal_point_.x(), focal_point_.y());
  }
  const float t = -origin.z() / direction.z();
  const QVector3D world = origin + (direction * t);
  return QPointF(world.x(), world.y());
}

void SceneWidget::panCameraByPixels(const QPoint &delta)
{
  QVector3D ground_up(cameraUp().x(), cameraUp().y(), 0.0F);
  if (ground_up.lengthSquared() < 1e-6F) {
    ground_up = QVector3D(
      static_cast<float>(-std::cos(camera_yaw_)),
      static_cast<float>(-std::sin(camera_yaw_)),
      0.0F);
  }
  ground_up.normalize();
  focal_point_ -= cameraRight() * static_cast<float>(delta.x() / scale_);
  focal_point_ += ground_up * static_cast<float>(delta.y() / scale_);
  focal_point_.setZ(0.0F);
}

void SceneWidget::centerViewOnRobot()
{
  if (!robot_pose_.valid) {
    return;
  }
  focal_point_ = QVector3D(
    static_cast<float>(robot_pose_.x),
    static_cast<float>(robot_pose_.y),
    static_cast<float>(robot_pose_.z));
}

void SceneWidget::updateCameraControlsGeometry()
{
  if (!camera_controls_) {
    return;
  }
  camera_controls_->adjustSize();
  const QSize size = camera_controls_->sizeHint();
  camera_controls_->setGeometry(width() - size.width() - 12, 12, size.width(), size.height());
}

void SceneWidget::updateCameraControlState()
{
  if (follow_robot_button_ && follow_robot_button_->isChecked() != follow_robot_) {
    follow_robot_button_->blockSignals(true);
    follow_robot_button_->setChecked(follow_robot_);
    follow_robot_button_->blockSignals(false);
  }
}

void SceneWidget::syncOpenGLRobotViewport()
{
  if (!robot_open_gl_widget_) {
    return;
  }
  robot_open_gl_widget_->setGeometry(rect());
  robot_open_gl_widget_->setCamera(
    focal_point_,
    camera_yaw_,
    camera_pitch_,
    camera_distance_,
    scale_);
  robot_open_gl_widget_->setRenderVisible(show_robot_ && robot_open_gl_widget_->hasRenderableVisuals());
  robot_open_gl_widget_->raise();
  if (camera_controls_) {
    camera_controls_->raise();
  }
}

QToolButton *SceneWidget::makeCameraButton(
  const QIcon &icon,
  const QString &text,
  const QString &tooltip,
  const bool checkable)
{
  auto *button = new QToolButton(camera_controls_);
  button->setObjectName("cameraToolButton");
  button->setIcon(icon);
  button->setText(text);
  button->setToolTip(tooltip);
  button->setCheckable(checkable);
  button->setFixedSize(32, 32);
  button->setAutoRaise(false);
  return button;
}

QImage SceneWidget::makeGridImage(
  const GridMap &map,
  const QColor &occupied,
  const QColor &free) const
{
  if (!map.valid) {
    return {};
  }

  QImage image(map.width, map.height, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  for (int y = 0; y < map.height; ++y) {
    QRgb *row = reinterpret_cast<QRgb *>(image.scanLine(map.height - 1 - y));
    for (int x = 0; x < map.width; ++x) {
      const int index = (y * map.width) + x;
      const int value = map.cells[index];
      if (value < 0) {
        row[x] = free.alpha() == 0 ? QColor(Qt::transparent).rgba() :
          QColor(128, 128, 128, 92).rgba();
      } else if (value > 50) {
        QColor color = occupied;
        color.setAlpha(free.alpha() == 0 ? std::clamp(50 + value, 120, 210) : 255);
        row[x] = color.rgba();
      } else {
        row[x] = free.rgba();
      }
    }
  }
  return image;
}

QImage SceneWidget::makeCostmapImage(const GridMap &map) const
{
  if (!map.valid) {
    return {};
  }

  QImage image(map.width, map.height, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);

  for (int y = 0; y < map.height; ++y) {
    QRgb *row = reinterpret_cast<QRgb *>(image.scanLine(map.height - 1 - y));
    for (int x = 0; x < map.width; ++x) {
      const int index = (y * map.width) + x;
      const int value = map.cells[index];
      if (value < 0) {
        row[x] = QColor(Qt::transparent).rgba();
      } else if (value >= 99) {
        row[x] = QColor(0, 252, 252, 245).rgba();
      } else if (value >= 85) {
        row[x] = QColor(0, 235, 255, 220).rgba();
      } else if (value >= 70) {
        row[x] = QColor(255, 126, 150, 205).rgba();
      } else if (value >= 50) {
        row[x] = QColor(244, 151, 178, 178).rgba();
      } else if (value > 0) {
        row[x] = QColor(184, 160, 232, 145).rgba();
      } else {
        row[x] = QColor(Qt::transparent).rgba();
      }
    }
  }
  return image;
}

void SceneWidget::drawGrid(QPainter &painter) const
{
  const double step = scale_ > 150.0 ? 0.5 : 1.0;
  const double view_span =
    std::max(width(), height()) / std::max(scale_, 1.0) + camera_distance_ + 2.0;
  const int x_start = static_cast<int>(std::floor((focal_point_.x() - view_span) / step)) - 1;
  const int x_end = static_cast<int>(std::ceil((focal_point_.x() + view_span) / step)) + 1;
  const int y_start = static_cast<int>(std::floor((focal_point_.y() - view_span) / step)) - 1;
  const int y_end = static_cast<int>(std::ceil((focal_point_.y() + view_span) / step)) + 1;

  painter.setPen(QPen(QColor(160, 160, 164, k_rviz_grid_alpha), 1));
  for (int x = x_start; x <= x_end; ++x) {
    const QPointF a = worldToScreen(QPointF(x * step, y_start * step));
    const QPointF b = worldToScreen(QPointF(x * step, y_end * step));
    painter.drawLine(a, b);
  }
  for (int y = y_start; y <= y_end; ++y) {
    const QPointF a = worldToScreen(QPointF(x_start * step, y * step));
    const QPointF b = worldToScreen(QPointF(x_end * step, y * step));
    painter.drawLine(a, b);
  }
}

void SceneWidget::drawGridLayer(
  QPainter &painter,
  const GridMap &map,
  const QImage &image,
  qreal opacity)
{
  if (!map.valid || image.isNull()) {
    return;
  }

  const QPointF top_left =
    worldToScreen(QPointF(map.origin_x, map.origin_y + (map.height * map.resolution)));
  const QPointF top_right =
    worldToScreen(QPointF(
      map.origin_x + (map.width * map.resolution),
      map.origin_y + (map.height * map.resolution)));
  const QPointF bottom_right =
    worldToScreen(QPointF(map.origin_x + (map.width * map.resolution), map.origin_y));
  const QPointF bottom_left = worldToScreen(QPointF(map.origin_x, map.origin_y));

  QPolygonF source;
  source << QPointF(0.0, 0.0)
         << QPointF(image.width(), 0.0)
         << QPointF(image.width(), image.height())
         << QPointF(0.0, image.height());
  QPolygonF target;
  target << top_left << top_right << bottom_right << bottom_left;
  QTransform transform;
  if (!QTransform::quadToQuad(source, target, transform)) {
    return;
  }

  painter.save();
  painter.setOpacity(opacity);
  painter.setTransform(transform, true);
  painter.drawImage(QPointF(0.0, 0.0), image);
  painter.restore();
}

void SceneWidget::drawPath(
  QPainter &painter,
  const PathData &path,
  const QColor &color,
  qreal width) const
{
  if (path.points.size() < 2) {
    return;
  }

  QPainterPath painter_path(worldToScreen(path.points.front()));
  for (int i = 1; i < path.points.size(); ++i) {
    painter_path.lineTo(worldToScreen(path.points[i]));
  }
  painter.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPath(painter_path);
}

void SceneWidget::drawPose(QPainter &painter, const Pose2D &pose, const QColor &color) const
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  auto point = [&](const double lx, const double ly) {
      return worldToScreen3D(
        pose.x + (c * lx) - (s * ly),
        pose.y + (s * lx) + (c * ly),
        pose.z + 0.03);
    };

  const QPointF tip = point(0.22, 0.0);
  const QPointF rear_left = point(-0.08, 0.035);
  const QPointF rear_right = point(-0.08, -0.035);
  const QPointF neck_left = point(0.12, 0.035);
  const QPointF neck_right = point(0.12, -0.035);
  const QPointF head_left = point(0.12, 0.08);
  const QPointF head_right = point(0.12, -0.08);

  QPolygonF arrow;
  arrow << tip
        << head_left
        << neck_left
        << rear_left
        << rear_right
        << neck_right
        << head_right;

  painter.setPen(QPen(color, 2));
  painter.setBrush(with_alpha(color, 120));
  painter.drawPolygon(arrow);
}

void SceneWidget::drawExactFootprint(QPainter &painter) const
{
  Pose2D pose = robot_pose_;
  for (const auto &frame : tf_frames_) {
    if (frame.child_frame == "base_footprint" && frame.pose.valid) {
      pose = frame.pose;
      break;
    }
  }
  if (!pose.valid) {
    return;
  }

  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  auto corner = [&](const double lx, const double ly) {
      return worldToScreen3D(
        pose.x + (c * lx) - (s * ly),
        pose.y + (s * lx) + (c * ly),
        pose.z + 0.01);
    };

  QPolygonF footprint;
  footprint << corner(-0.10, -0.09)
            << corner(0.10, -0.09)
            << corner(0.10, 0.09)
            << corner(-0.10, 0.09);
  painter.setPen(QPen(QColor(88, 150, 255, 210), 1.5, Qt::DashLine));
  painter.setBrush(Qt::NoBrush);
  painter.drawPolygon(footprint);
}

void SceneWidget::drawRobotModel(QPainter &painter)
{
  if (robot_visuals_.isEmpty()) {
    return;
  }

  QElapsedTimer draw_timer;
  draw_timer.start();
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing, true);

  QVector<RobotVisual> ordered_visuals = robot_visuals_;
  std::sort(
    ordered_visuals.begin(), ordered_visuals.end(),
    [](const RobotVisual &lhs, const RobotVisual &rhs) {
      return lhs.pose.z < rhs.pose.z;
    });

  for (const auto &visual : ordered_visuals) {
    if (!visual.valid) {
      continue;
    }
    if (isOpenGLMeshRendered(visual)) {
      continue;
    }

    const QPointF ground_center = worldToScreen3D(visual.pose.x, visual.pose.y, 0.0);
    const QPointF center = worldToScreen3D(visual.pose.x, visual.pose.y, visual.pose.z);
    painter.save();
    if (std::abs(visual.pose.z) > 0.02) {
      painter.setPen(QPen(QColor(62, 110, 255, 120), 1.2, Qt::DashLine));
      painter.drawLine(ground_center, center);
    }

    switch (visual.type) {
      case RobotGeometryType::Box: {
        drawBox3D(
          painter,
          visual.pose,
          std::max(visual.size_x, 0.02),
          std::max(visual.size_y, 0.02),
          std::max(visual.size_z, 0.02),
          QColor(205, 209, 214, 130));
        break;
      }
      case RobotGeometryType::Cylinder:
      case RobotGeometryType::Sphere: {
        drawCylinderProxy3D(
          painter,
          visual.pose,
          std::max(visual.radius, 0.02),
          visual.type == RobotGeometryType::Cylinder ? std::max(visual.length, 0.02) :
          std::max(visual.radius * 2.0, 0.02),
          visual.type == RobotGeometryType::Cylinder ? QColor(182, 188, 196, 120) :
          QColor(168, 196, 224, 130));
        break;
      }
      case RobotGeometryType::Mesh: {
        const QString key = (visual.frame_id + " " + visual.mesh_filename).toLower();
        if (drawMesh3D(painter, visual, QColor(74, 78, 84, 230))) {
          break;
        }
        if (key.contains("wheel")) {
          drawWheelProxy3D(painter, visual.pose, QColor(24, 28, 32, 235));
        } else if (key.contains("scan") || key.contains("lidar") || key.contains("lds")) {
          Pose2D scan_pose = visual.pose;
          scan_pose.x += std::cos(scan_pose.yaw) * 0.015;
          scan_pose.y += std::sin(scan_pose.yaw) * 0.015;
          scan_pose.z -= 0.0065;
          drawCylinderProxy3D(painter, scan_pose, 0.055, 0.0315, QColor(24, 28, 32, 235));
        } else if (key.contains("caster")) {
          drawCylinderProxy3D(painter, visual.pose, 0.025, 0.025, QColor(42, 46, 50, 220));
        } else if (key.contains("base") || key.contains("body") || key.contains("plate")) {
          drawBurgerBaseProxy3D(painter, visual.pose);
        } else {
          drawBox3D(painter, visual.pose, 0.10, 0.08, 0.05, QColor(84, 90, 96, 190));
        }
        break;
      }
    }

    painter.restore();
  }

  painter.restore();
  const qint64 elapsed_ms = draw_timer.elapsed();
  if (elapsed_ms > 100 && !diagnostic_event_cache_.contains("draw_robot_model_slow_100")) {
    diagnostic_event_cache_.insert("draw_robot_model_slow_100");
    Q_EMIT visualizationEvent(QString("drawRobotModel slow: %1 ms (>100 ms)").arg(elapsed_ms));
  } else if (elapsed_ms > 33 && !diagnostic_event_cache_.contains("draw_robot_model_slow_33")) {
    diagnostic_event_cache_.insert("draw_robot_model_slow_33");
    Q_EMIT visualizationEvent(QString("drawRobotModel slow: %1 ms (>33 ms)").arg(elapsed_ms));
  } else if (elapsed_ms > 16 && !diagnostic_event_cache_.contains("draw_robot_model_slow_16")) {
    diagnostic_event_cache_.insert("draw_robot_model_slow_16");
    Q_EMIT visualizationEvent(QString("drawRobotModel slow: %1 ms (>16 ms)").arg(elapsed_ms));
  }
}

void SceneWidget::drawBox3D(
  QPainter &painter,
  const Pose2D &pose,
  const double size_x,
  const double size_y,
  const double size_z,
  const QColor &color) const
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  auto corner = [&](const double lx, const double ly, const double lz) {
      const double wx = pose.x + (c * lx) - (s * ly);
      const double wy = pose.y + (s * lx) + (c * ly);
      return worldToScreen3D(wx, wy, pose.z + lz);
    };

  const double hx = size_x * 0.5;
  const double hy = size_y * 0.5;
  const double hz = size_z * 0.5;
  const QPointF b0 = corner(-hx, -hy, -hz);
  const QPointF b1 = corner(hx, -hy, -hz);
  const QPointF b2 = corner(hx, hy, -hz);
  const QPointF b3 = corner(-hx, hy, -hz);
  const QPointF t0 = corner(-hx, -hy, hz);
  const QPointF t1 = corner(hx, -hy, hz);
  const QPointF t2 = corner(hx, hy, hz);
  const QPointF t3 = corner(-hx, hy, hz);

  QColor side_color = color;
  side_color.setAlpha(std::max(70, color.alpha() - 45));
  QColor top_color = color;
  top_color.setAlpha(std::min(255, color.alpha() + 20));

  painter.setPen(QPen(QColor(12, 15, 18, 220), 1.2));
  painter.setBrush(side_color);
  QPolygonF face;
  face << b0 << b1 << t1 << t0;
  painter.drawPolygon(face);
  face.clear();
  face << b1 << b2 << t2 << t1;
  painter.drawPolygon(face);
  face.clear();
  face << b2 << b3 << t3 << t2;
  painter.drawPolygon(face);
  face.clear();
  face << b3 << b0 << t0 << t3;
  painter.drawPolygon(face);
  painter.setBrush(top_color);
  face.clear();
  face << t0 << t1 << t2 << t3;
  painter.drawPolygon(face);
}

void SceneWidget::drawCylinderProxy3D(
  QPainter &painter,
  const Pose2D &pose,
  const double radius,
  const double height,
  const QColor &color) const
{
  constexpr int k_segments = 18;
  QPolygonF bottom;
  QPolygonF top;
  bottom.reserve(k_segments);
  top.reserve(k_segments);
  for (int i = 0; i < k_segments; ++i) {
    const double angle = (2.0 * k_pi * i) / k_segments;
    const double wx = pose.x + (std::cos(angle) * radius);
    const double wy = pose.y + (std::sin(angle) * radius);
    bottom << worldToScreen3D(wx, wy, pose.z - (height * 0.5));
    top << worldToScreen3D(wx, wy, pose.z + (height * 0.5));
  }

  QColor side_color = color;
  side_color.setAlpha(std::max(70, color.alpha() - 45));
  QColor top_color = color;
  top_color.setAlpha(std::min(255, color.alpha() + 25));

  painter.setPen(QPen(QColor(12, 15, 18, 220), 1.2));
  painter.setBrush(side_color);
  for (int i = 0; i < k_segments; ++i) {
    const int next = (i + 1) % k_segments;
    QPolygonF face;
    face << bottom[i] << bottom[next] << top[next] << top[i];
    painter.drawPolygon(face);
  }
  painter.setBrush(top_color);
  painter.drawPolygon(top);
}

void SceneWidget::drawBurgerBaseProxy3D(QPainter &painter, const Pose2D &pose) const
{
  Pose2D lower_plate = pose;
  lower_plate.z += 0.012;
  drawBox3D(painter, lower_plate, 0.140, 0.140, 0.012, QColor(48, 52, 56, 215));

  Pose2D upper_plate = pose;
  upper_plate.z += 0.135;
  drawBox3D(painter, upper_plate, 0.125, 0.125, 0.012, QColor(64, 68, 72, 210));

  const double pillar_offsets[4][2] = {
    {0.052, 0.052},
    {0.052, -0.052},
    {-0.052, 0.052},
    {-0.052, -0.052},
  };
  for (const auto &offset : pillar_offsets) {
    Pose2D pillar = pose;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    pillar.x += (c * offset[0]) - (s * offset[1]);
    pillar.y += (s * offset[0]) + (c * offset[1]);
    pillar.z += 0.074;
    drawCylinderProxy3D(painter, pillar, 0.006, 0.120, QColor(34, 38, 42, 210));
  }
}

void SceneWidget::drawWheelProxy3D(
  QPainter &painter,
  const Pose2D &pose,
  const QColor &color) const
{
  constexpr int k_segments = 18;
  constexpr double k_radius = 0.036;
  constexpr double k_half_width = 0.018;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  auto point = [&](const double lx, const double ly, const double lz) {
      return worldToScreen3D(
        pose.x + (c * lx) - (s * ly),
        pose.y + (s * lx) + (c * ly),
        pose.z + lz);
    };

  QPolygonF left_face;
  QPolygonF right_face;
  left_face.reserve(k_segments);
  right_face.reserve(k_segments);
  for (int i = 0; i < k_segments; ++i) {
    const double angle = (2.0 * k_pi * i) / k_segments;
    const double lx = std::cos(angle) * k_radius;
    const double lz = std::sin(angle) * k_radius;
    left_face << point(lx, -k_half_width, lz);
    right_face << point(lx, k_half_width, lz);
  }

  QColor side_color = color;
  side_color.setAlpha(std::max(90, color.alpha() - 35));
  QColor face_color = color;
  face_color.setAlpha(std::min(255, color.alpha() + 10));

  painter.setPen(QPen(QColor(8, 10, 12, 230), 1.1));
  painter.setBrush(side_color);
  for (int i = 0; i < k_segments; ++i) {
    const int next = (i + 1) % k_segments;
    QPolygonF side;
    side << left_face[i] << left_face[next] << right_face[next] << right_face[i];
    painter.drawPolygon(side);
  }
  painter.setBrush(face_color);
  painter.drawPolygon(left_face);
  painter.drawPolygon(right_face);
}

bool SceneWidget::drawMesh3D(
  QPainter &painter,
  const RobotVisual &visual,
  const QColor &color)
{
  if (!visual.mesh_enabled || visual.mesh_render_mode == "proxy") {
    return false;
  }
  const MeshCacheEntry *mesh = meshForVisual(visual);
  if (!mesh || mesh->rejected || mesh->triangles.isEmpty()) {
    return false;
  }

  struct ProjectedFace
  {
    QPolygonF polygon;
    double depth{0.0};
  };

  const int triangle_count = static_cast<int>(mesh->triangles.size());
  const int max_rendered_faces = std::max(visual.mesh_max_rendered_faces, 1);
  const int stride = std::max(1, triangle_count / max_rendered_faces);
  QVector<ProjectedFace> faces;
  faces.reserve(std::min(triangle_count, max_rendered_faces));
  const double max_projected_extent = std::max(visual.mesh_max_projected_extent_px, 1.0);
  const double max_area = max_projected_extent * max_projected_extent;
  const QRectF viewport(
    -max_projected_extent,
    -max_projected_extent,
    width() + (2.0 * max_projected_extent),
    height() + (2.0 * max_projected_extent));

  for (int i = 0; i < triangle_count; i += stride) {
    const auto &triangle = mesh->triangles[i];
    const QVector3D a = meshPointToWorld(visual, triangle.a);
    const QVector3D b = meshPointToWorld(visual, triangle.b);
    const QVector3D c = meshPointToWorld(visual, triangle.c);
    if (!finite_vector(a) || !finite_vector(b) || !finite_vector(c)) {
      continue;
    }
    QVector3D normal = QVector3D::crossProduct(b - a, c - a);
    if (normal.lengthSquared() < 1e-9F) {
      continue;
    }
    normal.normalize();

    ProjectedFace face;
    const QPointF pa = worldToScreen3D(a.x(), a.y(), a.z());
    const QPointF pb = worldToScreen3D(b.x(), b.y(), b.z());
    const QPointF pc = worldToScreen3D(c.x(), c.y(), c.z());
    if (!finite_point(pa) || !finite_point(pb) || !finite_point(pc)) {
      continue;
    }

    const QRectF projected_bbox = QPolygonF{pa, pb, pc}.boundingRect();
    if (!projected_bbox.intersects(viewport)) {
      continue;
    }
    if (
      projected_bbox.width() > max_projected_extent ||
      projected_bbox.height() > max_projected_extent)
    {
      continue;
    }
    const double area = triangle_area(pa, pb, pc);
    if (!std::isfinite(area) || area < 0.5 || area > max_area) {
      continue;
    }

    face.polygon << pa << pb << pc;
    face.depth =
      (QVector3D::dotProduct(a - focal_point_, cameraForward()) +
      QVector3D::dotProduct(b - focal_point_, cameraForward()) +
      QVector3D::dotProduct(c - focal_point_, cameraForward())) / 3.0;

    faces.push_back(face);
  }

  std::sort(
    faces.begin(), faces.end(),
    [](const ProjectedFace &lhs, const ProjectedFace &rhs) {
      return lhs.depth > rhs.depth;
    });

  QColor edge_color = color;
  edge_color.setAlpha(std::max(120, color.alpha()));
  painter.setPen(QPen(edge_color, 0.8));
  if (faces.isEmpty()) {
    return false;
  }

  if (!diagnostic_event_cache_.contains("mesh_faces_drawn:" + visual.mesh_resolved_path)) {
    diagnostic_event_cache_.insert("mesh_faces_drawn:" + visual.mesh_resolved_path);
    Q_EMIT visualizationEvent(
      QString("Robot mesh rendered: %1, mode=%2, drawn face(s)=%3")
        .arg(visual.mesh_filename)
        .arg(visual.mesh_render_mode)
        .arg(faces.size()));
  }

  for (const auto &face : faces) {
    if (visual.mesh_render_mode == "wireframe") {
      painter.setBrush(Qt::NoBrush);
      painter.drawPolygon(face.polygon);
    } else {
      return false;
    }
  }
  return true;
}

bool SceneWidget::isOpenGLMeshRendered(const RobotVisual &visual) const
{
  return robot_open_gl_widget_ && robot_open_gl_widget_->isVisible() &&
    robot_open_gl_widget_->shouldSuppressProxyForVisual(visual);
}

const SceneWidget::MeshCacheEntry *SceneWidget::meshForVisual(const RobotVisual &visual)
{
  const QString path = visual.mesh_resolved_path;
  if (path.isEmpty()) {
    return nullptr;
  }

  auto it = mesh_cache_.find(path);
  if (it == mesh_cache_.end()) {
    return nullptr;
  }
  if (!it->attempted || it->rejected || it->triangles.isEmpty()) {
    return nullptr;
  }
  return &(*it);
}

bool SceneWidget::loadStlMesh(const RobotVisual &visual, MeshCacheEntry &entry)
{
  QElapsedTimer load_timer;
  load_timer.start();
  const QString path = visual.mesh_resolved_path.trimmed();
  Q_EMIT visualizationEvent(QString("Robot mesh load start: %1").arg(visual.mesh_filename));
  if (path.isEmpty()) {
    entry.error = "empty resolved mesh path";
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }
  const QFileInfo file_info(path);
  if (file_info.suffix().compare("stl", Qt::CaseInsensitive) != 0) {
    entry.error = QString("unsupported extension .%1").arg(file_info.suffix());
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    entry.error = file.errorString();
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }

  const qint64 file_size = file.size();
  const qint64 max_file_size_bytes =
    static_cast<qint64>(std::max(visual.mesh_max_file_size_mb, 1)) * 1024LL * 1024LL;
  if (file_size <= 0) {
    entry.error = "empty STL file";
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }
  if (file_size > max_file_size_bytes) {
    entry.error = QString("STL file exceeds %1 MiB limit").arg(visual.mesh_max_file_size_mb);
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }

  if (file_size >= 84) {
    file.seek(80);
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    quint32 triangle_count = 0;
    stream >> triangle_count;
    const qint64 expected_size = 84 + (static_cast<qint64>(triangle_count) * 50);
    if (triangle_count > 0 && expected_size == file_size) {
      const quint32 max_loaded_triangles =
        static_cast<quint32>(std::max(visual.mesh_max_loaded_triangles, 1));
      const quint32 loaded_count = std::min(triangle_count, max_loaded_triangles);
      entry.triangles.reserve(static_cast<int>(loaded_count));
      for (quint32 i = 0; i < loaded_count && !stream.atEnd(); ++i) {
        float nx = 0.0F;
        float ny = 0.0F;
        float nz = 0.0F;
        float ax = 0.0F;
        float ay = 0.0F;
        float az = 0.0F;
        float bx = 0.0F;
        float by = 0.0F;
        float bz = 0.0F;
        float cx = 0.0F;
        float cy = 0.0F;
        float cz = 0.0F;
        quint16 attribute = 0;
        stream >> nx >> ny >> nz >> ax >> ay >> az >> bx >> by >> bz >> cx >> cy >> cz >> attribute;
        (void)nx;
        (void)ny;
        (void)nz;
        entry.triangles.push_back(
          MeshTriangle{QVector3D(ax, ay, az), QVector3D(bx, by, bz), QVector3D(cx, cy, cz)});
      }
      if (triangle_count > loaded_count) {
        entry.error =
          QString("loaded %1 of %2 binary STL triangle(s)").arg(loaded_count).arg(triangle_count);
        Q_EMIT visualizationEvent(QString("Robot mesh triangle load capped: %1").arg(entry.error));
      } else {
        entry.error = entry.triangles.isEmpty() ? "binary STL contained no triangles" : QString();
      }
      Q_EMIT visualizationEvent(
        QString("Robot mesh load end: %1 ms, %2 triangle(s)")
          .arg(load_timer.elapsed())
          .arg(entry.triangles.size()));
      return validateMeshEntry(visual, entry);
    }
  }

  file.seek(0);
  const QByteArray header = file.peek(512).trimmed();
  if (!header.startsWith("solid")) {
    entry.error = "not a recognized binary or ASCII STL";
    Q_EMIT visualizationEvent(
      QString("Robot mesh load failed after %1 ms: %2").arg(load_timer.elapsed()).arg(entry.error));
    return false;
  }

  const QString content = QString::fromLatin1(file.readAll());
  QVector<QVector3D> vertices;
  vertices.reserve(3);
  quint32 parsed_triangles = 0;
  const quint32 max_loaded_triangles =
    static_cast<quint32>(std::max(visual.mesh_max_loaded_triangles, 1));
  const QStringList lines = content.split('\n');
  for (const QString &line : lines) {
    if (parsed_triangles >= max_loaded_triangles) {
      break;
    }
    const QString normalized = line.simplified();
    if (!normalized.startsWith("vertex ")) {
      continue;
    }
    const QStringList tokens = normalized.split(' ', Qt::SkipEmptyParts);
    if (tokens.size() != 4) {
      continue;
    }
    bool ok_x = false;
    bool ok_y = false;
    bool ok_z = false;
    const float x = tokens[1].toFloat(&ok_x);
    const float y = tokens[2].toFloat(&ok_y);
    const float z = tokens[3].toFloat(&ok_z);
    if (!ok_x || !ok_y || !ok_z) {
      continue;
    }
    vertices.push_back(QVector3D(x, y, z));
    if (vertices.size() == 3) {
      entry.triangles.push_back(MeshTriangle{vertices[0], vertices[1], vertices[2]});
      vertices.clear();
      ++parsed_triangles;
    }
  }
  if (!entry.triangles.isEmpty() && parsed_triangles >= max_loaded_triangles) {
    entry.error = QString("loaded first %1 ASCII STL triangle(s)").arg(max_loaded_triangles);
    Q_EMIT visualizationEvent(QString("Robot mesh triangle load capped: %1").arg(entry.error));
    Q_EMIT visualizationEvent(
      QString("Robot mesh load end: %1 ms, %2 triangle(s)")
        .arg(load_timer.elapsed())
        .arg(entry.triangles.size()));
    return validateMeshEntry(visual, entry);
  }
  entry.error = entry.triangles.isEmpty() ? "not a supported ASCII or binary STL" : QString();
  Q_EMIT visualizationEvent(
    QString("Robot mesh load %1 after %2 ms: %3 triangle(s)%4")
      .arg(entry.triangles.isEmpty() ? "failed" : "end")
      .arg(load_timer.elapsed())
      .arg(entry.triangles.size())
      .arg(entry.error.isEmpty() ? QString() : QString(" (%1)").arg(entry.error)));
  if (entry.triangles.isEmpty()) {
    return false;
  }
  return validateMeshEntry(visual, entry);
}

bool SceneWidget::validateMeshEntry(const RobotVisual &visual, MeshCacheEntry &entry)
{
  if (entry.triangles.isEmpty()) {
    entry.rejected = true;
    entry.error = "triangle count is zero";
  } else {
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    bool finite = true;
    bool coordinate_in_range = true;
    const float max_abs_coordinate =
      static_cast<float>(std::max(visual.mesh_max_abs_coordinate_m, 0.001));
    const double unit_scale = visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0;

    auto update_bounds = [&](const QVector3D &point) {
        const QVector3D scaled_point(
          static_cast<float>(point.x() * visual.mesh_scale_x * unit_scale),
          static_cast<float>(point.y() * visual.mesh_scale_y * unit_scale),
          static_cast<float>(point.z() * visual.mesh_scale_z * unit_scale));
        if (!finite_vector(scaled_point)) {
          finite = false;
          return;
        }
        if (
          std::abs(scaled_point.x()) > max_abs_coordinate ||
          std::abs(scaled_point.y()) > max_abs_coordinate ||
          std::abs(scaled_point.z()) > max_abs_coordinate)
        {
          coordinate_in_range = false;
        }
        min_x = std::min(min_x, scaled_point.x());
        min_y = std::min(min_y, scaled_point.y());
        min_z = std::min(min_z, scaled_point.z());
        max_x = std::max(max_x, scaled_point.x());
        max_y = std::max(max_y, scaled_point.y());
        max_z = std::max(max_z, scaled_point.z());
      };

    for (const auto &triangle : entry.triangles) {
      update_bounds(triangle.a);
      update_bounds(triangle.b);
      update_bounds(triangle.c);
    }

    entry.min_bounds = QVector3D(min_x, min_y, min_z);
    entry.max_bounds = QVector3D(max_x, max_y, max_z);
    entry.extent = entry.max_bounds - entry.min_bounds;
    entry.diagonal = static_cast<double>(entry.extent.length());

    if (!finite) {
      entry.rejected = true;
      entry.error = "non-finite coordinate";
    } else if (!coordinate_in_range) {
      entry.rejected = true;
      entry.error =
        QString("coordinate exceeds %1 m absolute limit").arg(visual.mesh_max_abs_coordinate_m);
    } else if (!std::isfinite(entry.diagonal) || entry.diagonal <= 0.0) {
      entry.rejected = true;
      entry.error = "invalid bbox diagonal";
    } else if (entry.diagonal > visual.mesh_max_extent_m) {
      entry.rejected = true;
      entry.error =
        QString("bbox diagonal %1 m exceeds %2 m").arg(entry.diagonal).arg(visual.mesh_max_extent_m);
    }
  }

  if (entry.rejected) {
    Q_EMIT visualizationEvent(
      QString("Robot mesh rejected: %1, bbox=%2, reason=%3")
        .arg(visual.mesh_filename)
        .arg(meshBoundsText(entry))
        .arg(entry.error));
    entry.triangles.clear();
    return false;
  }

  Q_EMIT visualizationEvent(
    QString("Robot mesh diagnostics: path=%1, triangles=%2, vertices=%3, bbox=%4, extent=%5 %6 %7, scale=%8 %9 %10, frame=%11, mode=%12")
      .arg(visual.mesh_resolved_path)
      .arg(entry.triangles.size())
      .arg(entry.triangles.size() * 3)
      .arg(meshBoundsText(entry))
      .arg(entry.extent.x())
      .arg(entry.extent.y())
      .arg(entry.extent.z())
      .arg(visual.mesh_scale_x)
      .arg(visual.mesh_scale_y)
      .arg(visual.mesh_scale_z)
      .arg(visual.frame_id)
      .arg(visual.mesh_render_mode));
  return true;
}

QString SceneWidget::meshBoundsText(const MeshCacheEntry &entry) const
{
  return QString("min=(%1,%2,%3), max=(%4,%5,%6), diag=%7")
    .arg(entry.min_bounds.x())
    .arg(entry.min_bounds.y())
    .arg(entry.min_bounds.z())
    .arg(entry.max_bounds.x())
    .arg(entry.max_bounds.y())
    .arg(entry.max_bounds.z())
    .arg(entry.diagonal);
}

QVector3D SceneWidget::meshPointToWorld(const RobotVisual &visual, const QVector3D &point) const
{
  const double unit_scale = visual.mesh_auto_unit_scale ? visual.mesh_unit_scale : 1.0;
  const QVector3D local_point(
    static_cast<float>(point.x() * visual.mesh_scale_x * unit_scale),
    static_cast<float>(point.y() * visual.mesh_scale_y * unit_scale),
    static_cast<float>(point.z() * visual.mesh_scale_z * unit_scale));
  const QVector3D rotated = rotate_rpy(
    local_point,
    visual.pose.roll,
    visual.pose.pitch,
    visual.pose.yaw);
  return QVector3D(
    static_cast<float>(visual.pose.x + rotated.x()),
    static_cast<float>(visual.pose.y + rotated.y()),
    static_cast<float>(visual.pose.z + rotated.z()));
}

void SceneWidget::drawTfFrames(QPainter &painter) const
{
  painter.save();
  auto find_frame_pose = [this](const QString &child_frame) -> Pose2D {
      for (const auto &frame : tf_frames_) {
        if (frame.child_frame == child_frame && frame.pose.valid) {
          return frame.pose;
        }
      }
      return {};
    };

  const Pose2D map_pose = find_frame_pose("map");
  const Pose2D odom_pose = find_frame_pose("odom");
  const Pose2D base_footprint_pose = find_frame_pose("base_footprint");
  if (map_pose.valid && odom_pose.valid) {
    drawTfChainLine(painter, map_pose, odom_pose);
  }
  if (odom_pose.valid && base_footprint_pose.valid) {
    drawTfChainLine(painter, odom_pose, base_footprint_pose);
  }

  for (const auto &frame : tf_frames_) {
    if (!frame.pose.valid) {
      continue;
    }
    const double axis_length = 0.18;
    const QPointF center = worldToScreen3D(frame.pose.x, frame.pose.y, frame.pose.z);
    const QPointF x_axis = worldToScreen3D(
      frame.pose.x + (std::cos(frame.pose.yaw) * axis_length),
      frame.pose.y + (std::sin(frame.pose.yaw) * axis_length),
      frame.pose.z);
    const QPointF y_axis = worldToScreen3D(
      frame.pose.x + (std::cos(frame.pose.yaw + k_half_pi) * axis_length),
      frame.pose.y + (std::sin(frame.pose.yaw + k_half_pi) * axis_length),
      frame.pose.z);
    const QPointF z_axis = worldToScreen3D(frame.pose.x, frame.pose.y, frame.pose.z + axis_length);

    painter.setPen(QPen(QColor(230, 60, 48), 4, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, x_axis);
    painter.setPen(QPen(QColor(80, 190, 96), 4, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, y_axis);
    painter.setPen(QPen(QColor(62, 110, 255), 4, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, z_axis);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(70, 125, 255, 220));
    painter.drawEllipse(center, 4.5, 4.5);

    painter.setPen(QPen(QColor(255, 214, 102, 230), 1));
    painter.drawText(center + QPointF(8.0, -8.0), frame.child_frame);
  }
  painter.restore();
}

void SceneWidget::drawTfChainLine(QPainter &painter, const Pose2D &from, const Pose2D &to) const
{
  const QPointF start = worldToScreen3D(from.x, from.y, from.z);
  const QPointF end = worldToScreen3D(to.x, to.y, to.z);
  const QPointF delta = end - start;
  const double length = std::hypot(delta.x(), delta.y());
  QPointF normal(0.0, 0.0);
  if (length > 1e-3) {
    normal = QPointF(-delta.y() / length, delta.x() / length);
  }

  const struct
  {
    QColor color;
    double offset;
  } lines[] = {
    {QColor(230, 60, 48, 150), -1.2},
    {QColor(80, 190, 96, 150), 0.0},
    {QColor(62, 110, 255, 150), 1.2},
  };

  for (const auto &line : lines) {
    const QPointF offset = normal * line.offset;
    painter.setPen(QPen(line.color, 1.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(start + offset, end + offset);
  }
}

void SceneWidget::drawScan(QPainter &painter) const
{
  if (scan_.points.isEmpty()) {
    return;
  }

  painter.save();
  painter.setPen(QPen(QColor(156, 255, 92, 220), 2, Qt::SolidLine, Qt::RoundCap));
  for (const auto &point : scan_.points) {
    const QPointF center = worldToScreen(point);
    painter.drawPoint(center);
  }
  painter.restore();
}

void SceneWidget::drawWaypointRoute(QPainter &painter) const
{
  QVector<QPointF> route_points;
  route_points.reserve(waypoints_.size() + 1);
  for (const auto &waypoint : waypoints_) {
    if (waypoint.valid) {
      route_points.push_back(QPointF(waypoint.x, waypoint.y));
    }
  }
  if (route_points.size() < 2) {
    return;
  }

  QPainterPath route_path(worldToScreen(route_points.front()));
  for (int i = 1; i < route_points.size(); ++i) {
    route_path.lineTo(worldToScreen(route_points[i]));
  }

  painter.save();
  painter.setPen(QPen(QColor(255, 210, 64, 230), 2.5, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPath(route_path);
  painter.restore();
}

void SceneWidget::drawWaypoints(QPainter &painter) const
{
  for (int i = 0; i < waypoints_.size(); ++i) {
    const QPointF center = worldToScreen3D(waypoints_[i].x, waypoints_[i].y, waypoints_[i].z);
    const bool selected = i == selected_waypoint_index_;
    painter.setPen(QPen(selected ? QColor(255, 180, 0) : QColor(255, 96, 64), selected ? 3 : 2));
    painter.setBrush(selected ? QColor(255, 180, 0, 80) : QColor(255, 96, 64, 60));
    painter.drawEllipse(center, 7.0, 7.0);
    const QPointF heading = worldToScreen3D(
      waypoints_[i].x + (std::cos(waypoints_[i].yaw) * 0.22),
      waypoints_[i].y + (std::sin(waypoints_[i].yaw) * 0.22),
      waypoints_[i].z);
    painter.drawLine(center, heading);
    painter.drawText(center + QPointF(10.0, -8.0), QString::number(i + 1));
  }
}

int SceneWidget::waypointAt(const QPointF &screen_position) const
{
  for (int i = waypoints_.size() - 1; i >= 0; --i) {
    const QPointF center = worldToScreen(QPointF(waypoints_[i].x, waypoints_[i].y));
    const QPointF delta = screen_position - center;
    if ((delta.x() * delta.x()) + (delta.y() * delta.y()) <= 144.0) {
      return i;
    }
  }
  return -1;
}

void SceneWidget::emitWaypointState()
{
  Q_EMIT waypointCountChanged(waypoints_.size());
  Q_EMIT waypointsChanged(waypoints_);
  Q_EMIT selectedWaypointChanged(selected_waypoint_index_);
}

}  // namespace amr::visualization
