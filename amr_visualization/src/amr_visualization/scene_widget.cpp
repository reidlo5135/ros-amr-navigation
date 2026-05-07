#include "amr_visualization/scene_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace amr::visualization
{

namespace
{

QColor with_alpha(QColor color, int alpha)
{
  color.setAlpha(alpha);
  return color;
}

constexpr int k_rviz_background = 48;
constexpr int k_rviz_grid_alpha = 64;
constexpr double k_half_pi = 1.5707963267948966;

}  // namespace

SceneWidget::SceneWidget(QWidget * parent)
: QWidget(parent)
{
  setMouseTracking(true);
  setMinimumSize(640, 480);
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

void SceneWidget::resetView()
{
  scale_ = 90.0;
  pan_ = QPointF(0.0, 0.0);
  update();
}

void SceneWidget::setMap(const GridMap & map)
{
  map_ = map;
  map_image_ = makeGridImage(map_, QColor("#000000"), QColor("#ffffff"));
  update();
}

void SceneWidget::setTfFrames(const QVector<amr::visualization::FrameVisual> & frames)
{
  tf_frames_ = frames;
  update();
}

void SceneWidget::setGlobalCostmap(const GridMap & map)
{
  global_costmap_ = map;
  global_costmap_image_ =
    makeGridImage(global_costmap_, QColor("#ff6040"), QColor(0, 0, 0, 0));
  update();
}

void SceneWidget::setLocalCostmap(const GridMap & map)
{
  local_costmap_ = map;
  local_costmap_image_ =
    makeGridImage(local_costmap_, QColor("#00c8ff"), QColor(0, 0, 0, 0));
  update();
}

void SceneWidget::setScan(const amr::visualization::ScanData & scan)
{
  scan_ = scan;
  update();
}

void SceneWidget::setRobotPose(const Pose2D & pose)
{
  robot_pose_ = pose;
  update();
}

void SceneWidget::setGlobalPath(const PathData & path)
{
  global_path_ = path;
  update();
}

void SceneWidget::setLocalPath(const PathData & path)
{
  local_path_ = path;
  update();
}

void SceneWidget::setGridVisible(bool visible) { show_grid_ = visible; update(); }
void SceneWidget::setMapVisible(bool visible) { show_map_ = visible; update(); }
void SceneWidget::setGlobalCostmapVisible(bool visible) { show_global_costmap_ = visible; update(); }
void SceneWidget::setLocalCostmapVisible(bool visible) { show_local_costmap_ = visible; update(); }
void SceneWidget::setRobotVisible(bool visible) { show_robot_ = visible; update(); }
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

void SceneWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(k_rviz_background, k_rviz_background, k_rviz_background));

  if (show_map_) {
    drawGridLayer(painter, map_, map_image_, 1.0);
  }
  if (show_grid_) {
    drawGrid(painter);
  }
  if (show_global_costmap_) {
    drawGridLayer(painter, global_costmap_, global_costmap_image_, 0.45);
  }
  if (show_local_costmap_) {
    drawGridLayer(painter, local_costmap_, local_costmap_image_, 0.45);
  }
  if (show_global_path_) {
    drawPath(painter, global_path_, QColor(255, 96, 64), 3.5);
  }
  if (show_local_path_) {
    drawPath(painter, local_path_, QColor(0, 200, 255), 3.5);
  }
  if (show_tf_) {
    drawTfFrames(painter);
  }
  if (show_scan_) {
    drawScan(painter);
  }

  drawWaypointRoute(painter);
  drawWaypoints(painter);
  if (aim_pose_.valid) {
    drawPose(painter, aim_pose_, QColor(255, 180, 0));
  }
}

void SceneWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    panning_ = true;
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

void SceneWidget::mouseMoveEvent(QMouseEvent * event)
{
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
  pan_ += QPointF(delta.x(), delta.y());
  last_mouse_position_ = event->pos();
  update();
}

void SceneWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    panning_ = false;
  }
  if (event->button() == Qt::LeftButton) {
    if (setting_aim_heading_ && add_waypoint_mode_ && aim_pose_.valid) {
      waypoints_.push_back(aim_pose_);
      selected_waypoint_index_ = waypoints_.size() - 1;
      emitWaypointState();
    }
    setting_aim_heading_ = false;
    update();
  }
}

void SceneWidget::wheelEvent(QWheelEvent * event)
{
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 0.89;
  scale_ = std::clamp(scale_ * factor, 18.0, 540.0);
  update();
}

QPointF SceneWidget::worldToScreen(const QPointF & point) const
{
  return QPointF(
    (width() * 0.5) + pan_.x() + (point.x() * scale_),
    (height() * 0.5) + pan_.y() - (point.y() * scale_));
}

QPointF SceneWidget::screenToWorld(const QPointF & point) const
{
  return QPointF(
    (point.x() - (width() * 0.5) - pan_.x()) / scale_,
    -((point.y() - (height() * 0.5) - pan_.y()) / scale_));
}

QImage SceneWidget::makeGridImage(
  const GridMap & map,
  const QColor & occupied,
  const QColor & free) const
{
  if (!map.valid) {
    return {};
  }

  QImage image(map.width, map.height, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  for (int y = 0; y < map.height; ++y) {
    QRgb * row = reinterpret_cast<QRgb *>(image.scanLine(map.height - 1 - y));
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

void SceneWidget::drawGrid(QPainter & painter) const
{
  const QPointF top_left = screenToWorld(QPointF(0.0, 0.0));
  const QPointF bottom_right = screenToWorld(QPointF(width(), height()));
  const double step = scale_ > 150.0 ? 0.5 : 1.0;
  const int x_start = static_cast<int>(std::floor(top_left.x() / step)) - 1;
  const int x_end = static_cast<int>(std::ceil(bottom_right.x() / step)) + 1;
  const int y_start = static_cast<int>(std::floor(bottom_right.y() / step)) - 1;
  const int y_end = static_cast<int>(std::ceil(top_left.y() / step)) + 1;

  painter.setPen(QPen(QColor(160, 160, 164, k_rviz_grid_alpha), 1));
  for (int x = x_start; x <= x_end; ++x) {
    const QPointF a = worldToScreen(QPointF(x * step, bottom_right.y()));
    const QPointF b = worldToScreen(QPointF(x * step, top_left.y()));
    painter.drawLine(a, b);
  }
  for (int y = y_start; y <= y_end; ++y) {
    const QPointF a = worldToScreen(QPointF(top_left.x(), y * step));
    const QPointF b = worldToScreen(QPointF(bottom_right.x(), y * step));
    painter.drawLine(a, b);
  }
}

void SceneWidget::drawGridLayer(
  QPainter & painter,
  const GridMap & map,
  const QImage & image,
  qreal opacity)
{
  if (!map.valid || image.isNull()) {
    return;
  }

  const QPointF top_left =
    worldToScreen(QPointF(map.origin_x, map.origin_y + (map.height * map.resolution)));
  const QPointF bottom_right =
    worldToScreen(QPointF(map.origin_x + (map.width * map.resolution), map.origin_y));
  const QRectF target(top_left, bottom_right);
  painter.save();
  painter.setOpacity(opacity);
  painter.drawImage(target.normalized(), image);
  painter.restore();
}

void SceneWidget::drawPath(
  QPainter & painter,
  const PathData & path,
  const QColor & color,
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

void SceneWidget::drawPose(QPainter & painter, const Pose2D & pose, const QColor & color) const
{
  const QPointF center = worldToScreen(QPointF(pose.x, pose.y));
  const double length = pose.valid && &pose == &robot_pose_ ? 34.0 : 28.0;
  const double shaft = length * 0.58;
  const double half_width = 5.0;
  const double head_width = 11.0;
  const QPointF forward(std::cos(pose.yaw), -std::sin(pose.yaw));
  const QPointF lateral(-forward.y(), forward.x());
  const QPointF tail = center - (forward * (length * 0.36));
  const QPointF neck = tail + (forward * shaft);
  const QPointF tip = tail + (forward * length);

  QPolygonF arrow;
  arrow << tip
        << neck + (lateral * head_width)
        << neck + (lateral * half_width)
        << tail + (lateral * half_width)
        << tail - (lateral * half_width)
        << neck - (lateral * half_width)
        << neck - (lateral * head_width);

  painter.setPen(QPen(color, 2));
  painter.setBrush(with_alpha(color, 120));
  painter.drawPolygon(arrow);
}

void SceneWidget::drawTfFrames(QPainter & painter) const
{
  painter.save();
  for (const auto & frame : tf_frames_) {
    if (!frame.pose.valid) {
      continue;
    }
    const QPointF center = worldToScreen(QPointF(frame.pose.x, frame.pose.y));
    const QPointF x_axis(
      center.x() + std::cos(frame.pose.yaw) * 18.0,
      center.y() - std::sin(frame.pose.yaw) * 18.0);
    const QPointF y_axis(
      center.x() + std::cos(frame.pose.yaw + k_half_pi) * 18.0,
      center.y() - std::sin(frame.pose.yaw + k_half_pi) * 18.0);

    painter.setPen(QPen(QColor(230, 60, 48), 4, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, x_axis);
    painter.setPen(QPen(QColor(80, 190, 96), 4, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, y_axis);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(70, 125, 255, 220));
    painter.drawEllipse(center, 4.5, 4.5);

    painter.setPen(QPen(QColor(255, 214, 102, 230), 1));
    painter.drawText(center + QPointF(8.0, -8.0), frame.child_frame);
  }
  painter.restore();
}

void SceneWidget::drawScan(QPainter & painter) const
{
  if (scan_.points.isEmpty()) {
    return;
  }

  painter.save();
  painter.setPen(QPen(QColor(156, 255, 92, 220), 2, Qt::SolidLine, Qt::RoundCap));
  for (const auto & point : scan_.points) {
    const QPointF center = worldToScreen(point);
    painter.drawPoint(center);
  }
  painter.restore();
}

void SceneWidget::drawRobotProxy(QPainter & painter) const
{
  const FrameVisual * base_frame = nullptr;
  const FrameVisual * body_frame = nullptr;
  for (const auto & frame : tf_frames_) {
    if (frame.child_frame == "base_footprint") {
      base_frame = &frame;
    } else if (frame.child_frame == "base_link") {
      body_frame = &frame;
    }
  }

  const Pose2D pose = body_frame ? body_frame->pose :
    (base_frame ? base_frame->pose : robot_pose_);
  if (!pose.valid) {
    return;
  }

  const QPointF center = worldToScreen(QPointF(pose.x, pose.y));
  const QPointF forward(std::cos(pose.yaw), -std::sin(pose.yaw));
  const QPointF lateral(-forward.y(), forward.x());
  const double half_length = 14.0;
  const double half_width = 10.0;

  QPolygonF body;
  body << center + (forward * half_length) + (lateral * half_width)
       << center + (forward * half_length) - (lateral * half_width)
       << center - (forward * half_length) - (lateral * half_width)
       << center - (forward * half_length) + (lateral * half_width);

  painter.save();
  painter.setPen(QPen(QColor(220, 220, 220, 180), 1.5));
  painter.setBrush(QColor(110, 110, 112, 70));
  painter.drawPolygon(body);
  painter.restore();
}

void SceneWidget::drawWaypointRoute(QPainter & painter) const
{
  QVector<QPointF> route_points;
  route_points.reserve(waypoints_.size() + 1);
  for (const auto & waypoint : waypoints_) {
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

void SceneWidget::drawWaypoints(QPainter & painter) const
{
  for (int i = 0; i < waypoints_.size(); ++i) {
    const QPointF center = worldToScreen(QPointF(waypoints_[i].x, waypoints_[i].y));
    const bool selected = i == selected_waypoint_index_;
    painter.setPen(QPen(selected ? QColor(255, 180, 0) : QColor(255, 96, 64), selected ? 3 : 2));
    painter.setBrush(selected ? QColor(255, 180, 0, 80) : QColor(255, 96, 64, 60));
    painter.drawEllipse(center, 7.0, 7.0);
    const QPointF heading(
      center.x() + std::cos(waypoints_[i].yaw) * 18.0,
      center.y() - std::sin(waypoints_[i].yaw) * 18.0);
    painter.drawLine(center, heading);
    painter.drawText(center + QPointF(10.0, -8.0), QString::number(i + 1));
  }
}

int SceneWidget::waypointAt(const QPointF & screen_position) const
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
