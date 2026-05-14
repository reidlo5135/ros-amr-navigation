#ifndef AMR_VISUALIZATION__SCENE_WIDGET_HPP_
#define AMR_VISUALIZATION__SCENE_WIDGET_HPP_

#include <QElapsedTimer>
#include <QHash>
#include <QIcon>
#include <QImage>
#include <QPoint>
#include <QResizeEvent>
#include <QToolButton>
#include <QVector3D>
#include <QWidget>

#include "amr_visualization/operator_state.hpp"

namespace amr::visualization
{

class SceneWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SceneWidget(QWidget *parent = nullptr);

  Pose2D aimPose() const;
  QVector<Pose2D> waypoints() const;
  int selectedWaypointIndex() const;
  void resetView();
  bool followRobotEnabled() const;

public Q_SLOTS:
  void setMap(const amr::visualization::GridMap &map);
  void setTfFrames(const QVector<amr::visualization::FrameVisual> &frames);
  void setRobotModel(const QVector<amr::visualization::RobotVisual> &visuals);
  void setGlobalCostmap(const amr::visualization::GridMap &map);
  void setLocalCostmap(const amr::visualization::GridMap &map);
  void setScan(const amr::visualization::ScanData &scan);
  void setRobotPose(const amr::visualization::Pose2D &pose);
  void setGlobalPath(const amr::visualization::PathData &path);
  void setLocalPath(const amr::visualization::PathData &path);
  void setGridVisible(bool visible);
  void setMapVisible(bool visible);
  void setGlobalCostmapVisible(bool visible);
  void setLocalCostmapVisible(bool visible);
  void setFootprintVisible(bool visible);
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
  void clearAimPose();
  void removeSelectedWaypoint();
  void moveSelectedWaypointUp();
  void moveSelectedWaypointDown();
  void selectWaypoint(int index);
  void setFollowRobotEnabled(bool enabled);

Q_SIGNALS:
  void aimPoseChanged(const amr::visualization::Pose2D &pose);
  void waypointCountChanged(int count);
  void waypointsChanged(const QVector<amr::visualization::Pose2D> &waypoints);
  void selectedWaypointChanged(int index);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private:
  struct MeshTriangle
  {
    QVector3D a;
    QVector3D b;
    QVector3D c;
  };

  struct MeshCacheEntry
  {
    bool attempted{false};
    QVector<MeshTriangle> triangles;
  };

  QPointF worldToScreen(const QPointF &point) const;
  QPointF worldToScreen3D(double x, double y, double z) const;
  QVector3D cameraRight() const;
  QVector3D cameraForward() const;
  QVector3D cameraUp() const;
  QPointF screenToWorld(const QPointF &point) const;
  void panCameraByPixels(const QPoint &delta);
  void centerViewOnRobot();
  void updateCameraControlsGeometry();
  void updateCameraControlState();
  QToolButton *makeCameraButton(
    const QIcon &icon,
    const QString &text,
    const QString &tooltip,
    bool checkable);
  QImage makeGridImage(const GridMap &map, const QColor &occupied, const QColor &free) const;
  QImage makeCostmapImage(const GridMap &map) const;
  void drawGrid(QPainter &painter) const;
  void drawGridLayer(QPainter &painter, const GridMap &map, const QImage &image, qreal opacity);
  void drawPath(QPainter &painter, const PathData &path, const QColor &color, qreal width) const;
  void drawPose(QPainter &painter, const Pose2D &pose, const QColor &color) const;
  void drawExactFootprint(QPainter &painter) const;
  void drawRobotModel(QPainter &painter) const;
  void drawBox3D(
    QPainter &painter,
    const Pose2D &pose,
    double size_x,
    double size_y,
    double size_z,
    const QColor &color) const;
  void drawCylinderProxy3D(
    QPainter &painter,
    const Pose2D &pose,
    double radius,
    double height,
    const QColor &color) const;
  void drawBurgerBaseProxy3D(QPainter &painter, const Pose2D &pose) const;
  void drawWheelProxy3D(QPainter &painter, const Pose2D &pose, const QColor &color) const;
  bool drawMesh3D(QPainter &painter, const RobotVisual &visual, const QColor &color) const;
  QString resolveMeshPath(const QString &uri) const;
  const MeshCacheEntry *meshForVisual(const RobotVisual &visual) const;
  bool loadStlMesh(const QString &path, MeshCacheEntry &entry) const;
  QVector3D meshPointToWorld(const RobotVisual &visual, const QVector3D &point) const;
  void drawTfFrames(QPainter &painter) const;
  void drawTfChainLine(QPainter &painter, const Pose2D &from, const Pose2D &to) const;
  void drawScan(QPainter &painter) const;
  void drawWaypointRoute(QPainter &painter) const;
  void drawWaypoints(QPainter &painter) const;
  int waypointAt(const QPointF &screen_position) const;
  void emitWaypointState();

  GridMap map_;
  GridMap global_costmap_;
  GridMap local_costmap_;
  QImage map_image_;
  QImage global_costmap_image_;
  QImage local_costmap_image_;
  QVector<FrameVisual> tf_frames_;
  QVector<RobotVisual> robot_visuals_;
  ScanData scan_;
  Pose2D robot_pose_;
  Pose2D aim_pose_;
  PathData global_path_;
  PathData local_path_;
  QVector<Pose2D> waypoints_;
  int selected_waypoint_index_{-1};

  bool show_grid_{true};
  bool show_map_{true};
  bool show_global_costmap_{true};
  bool show_local_costmap_{true};
  bool show_footprint_{true};
  bool show_robot_{true};
  bool show_tf_{true};
  bool show_scan_{true};
  bool show_global_path_{true};
  bool show_local_path_{true};
  bool add_waypoint_mode_{false};
  double scale_{90.0};
  QVector3D focal_point_{0.0F, 0.0F, 0.0F};
  double camera_yaw_{0.0};
  double camera_pitch_{1.5707963267948966};
  double camera_distance_{8.0};
  bool follow_robot_{false};
  bool panning_{false};
  bool orbiting_camera_{false};
  bool setting_aim_heading_{false};
  QPoint last_mouse_position_;
  QWidget *camera_controls_{nullptr};
  QToolButton *follow_robot_button_{nullptr};
  QToolButton *reset_view_button_{nullptr};
  mutable QHash<QString, MeshCacheEntry> mesh_cache_;
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__SCENE_WIDGET_HPP_
