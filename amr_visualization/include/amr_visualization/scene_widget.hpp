#ifndef AMR_VISUALIZATION__SCENE_WIDGET_HPP_
#define AMR_VISUALIZATION__SCENE_WIDGET_HPP_

#include <QElapsedTimer>
#include <QImage>
#include <QPoint>
#include <QWidget>

#include "amr_visualization/operator_state.hpp"

namespace amr::visualization
{

class SceneWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SceneWidget(QWidget * parent = nullptr);

  Pose2D aimPose() const;
  QVector<Pose2D> waypoints() const;
  int selectedWaypointIndex() const;
  void resetView();

public Q_SLOTS:
  void setMap(const amr::visualization::GridMap & map);
  void setTfFrames(const QVector<amr::visualization::FrameVisual> & frames);
  void setGlobalCostmap(const amr::visualization::GridMap & map);
  void setLocalCostmap(const amr::visualization::GridMap & map);
  void setScan(const amr::visualization::ScanData & scan);
  void setRobotPose(const amr::visualization::Pose2D & pose);
  void setGlobalPath(const amr::visualization::PathData & path);
  void setLocalPath(const amr::visualization::PathData & path);
  void setGridVisible(bool visible);
  void setMapVisible(bool visible);
  void setGlobalCostmapVisible(bool visible);
  void setLocalCostmapVisible(bool visible);
  void setRobotVisible(bool visible);
  void setTfVisible(bool visible);
  void setScanVisible(bool visible);
  void setGlobalPathVisible(bool visible);
  void setLocalPathVisible(bool visible);
  void setAddWaypointMode(bool enabled);
  void addAimAsWaypoint();
  void clearWaypoints();
  void clearSchedule();
  void clearNavigationOverlays();
  void removeSelectedWaypoint();
  void moveSelectedWaypointUp();
  void moveSelectedWaypointDown();
  void selectWaypoint(int index);

Q_SIGNALS:
  void aimPoseChanged(const amr::visualization::Pose2D & pose);
  void waypointCountChanged(int count);
  void waypointsChanged(const QVector<amr::visualization::Pose2D> & waypoints);
  void selectedWaypointChanged(int index);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  QPointF worldToScreen(const QPointF & point) const;
  QPointF screenToWorld(const QPointF & point) const;
  QImage makeGridImage(const GridMap & map, const QColor & occupied, const QColor & free) const;
  void drawGrid(QPainter & painter) const;
  void drawGridLayer(QPainter & painter, const GridMap & map, const QImage & image, qreal opacity);
  void drawPath(QPainter & painter, const PathData & path, const QColor & color, qreal width) const;
  void drawPose(QPainter & painter, const Pose2D & pose, const QColor & color) const;
  void drawTfFrames(QPainter & painter) const;
  void drawScan(QPainter & painter) const;
  void drawRobotProxy(QPainter & painter) const;
  void drawWaypointRoute(QPainter & painter) const;
  void drawWaypoints(QPainter & painter) const;
  int waypointAt(const QPointF & screen_position) const;
  void emitWaypointState();

  GridMap map_;
  GridMap global_costmap_;
  GridMap local_costmap_;
  QImage map_image_;
  QImage global_costmap_image_;
  QImage local_costmap_image_;
  QVector<FrameVisual> tf_frames_;
  ScanData scan_;
  Pose2D robot_pose_;
  Pose2D aim_pose_;
  PathData global_path_;
  PathData local_path_;
  QVector<Pose2D> waypoints_;
  int selected_waypoint_index_{-1};

  bool show_grid_{true};
  bool show_map_{true};
  bool show_global_costmap_{false};
  bool show_local_costmap_{false};
  bool show_robot_{true};
  bool show_tf_{true};
  bool show_scan_{true};
  bool show_global_path_{true};
  bool show_local_path_{true};
  bool add_waypoint_mode_{false};
  double scale_{90.0};
  QPointF pan_{0.0, 0.0};
  bool panning_{false};
  bool setting_aim_heading_{false};
  QPoint last_mouse_position_;
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__SCENE_WIDGET_HPP_
