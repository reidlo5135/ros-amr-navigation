#ifndef AMR_VISUALIZATION__SCENE_WIDGET_HPP_
#define AMR_VISUALIZATION__SCENE_WIDGET_HPP_

/**
 * @file scene_widget.hpp
 * @brief Interactive Qt scene widget for map, robot, path, scan, TF, and waypoint rendering.
 */

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

/// @brief Renders the AMR world view and manages waypoint/aim-pose interaction.
class SceneWidget : public QWidget
{
  Q_OBJECT

public:
  /// @brief Construct the scene widget and camera controls.
  explicit SceneWidget(QWidget *parent = nullptr);

  /// @brief Return the current aim pose selected in the scene.
  Pose2D aimPose() const;
  /// @brief Return the current ordered waypoint route.
  QVector<Pose2D> waypoints() const;
  /// @brief Return the selected waypoint index, or -1 when none is selected.
  int selectedWaypointIndex() const;
  /// @brief Reset the camera view to its default framing.
  void resetView();
  /// @brief Return true when the camera follows the robot pose.
  bool followRobotEnabled() const;

public Q_SLOTS:
  /// @brief Replace the displayed static map.
  void setMap(const amr::visualization::GridMap &map);
  /// @brief Replace displayed TF frame visuals.
  void setTfFrames(const QVector<amr::visualization::FrameVisual> &frames);
  /// @brief Replace displayed robot model visuals.
  void setRobotModel(const QVector<amr::visualization::RobotVisual> &visuals);
  /// @brief Replace the displayed global costmap.
  void setGlobalCostmap(const amr::visualization::GridMap &map);
  /// @brief Replace the displayed local costmap.
  void setLocalCostmap(const amr::visualization::GridMap &map);
  /// @brief Replace the displayed laser scan.
  void setScan(const amr::visualization::ScanData &scan);
  /// @brief Update the displayed robot pose.
  void setRobotPose(const amr::visualization::Pose2D &pose);
  /// @brief Replace the displayed global path.
  void setGlobalPath(const amr::visualization::PathData &path);
  /// @brief Replace the displayed local path.
  void setLocalPath(const amr::visualization::PathData &path);
  /// @brief Toggle grid overlay visibility.
  void setGridVisible(bool visible);
  /// @brief Toggle map visibility.
  void setMapVisible(bool visible);
  /// @brief Toggle global costmap visibility.
  void setGlobalCostmapVisible(bool visible);
  /// @brief Toggle local costmap visibility.
  void setLocalCostmapVisible(bool visible);
  /// @brief Toggle exact footprint visibility.
  void setFootprintVisible(bool visible);
  /// @brief Toggle robot model visibility.
  void setRobotVisible(bool visible);
  /// @brief Toggle TF visualization visibility.
  void setTfVisible(bool visible);
  /// @brief Toggle scan visibility.
  void setScanVisible(bool visible);
  /// @brief Toggle global path visibility.
  void setGlobalPathVisible(bool visible);
  /// @brief Toggle local path visibility.
  void setLocalPathVisible(bool visible);
  /// @brief Enable or disable add-waypoint click mode.
  void setAddWaypointMode(bool enabled);
  /// @brief Append the current aim pose to the waypoint list.
  void addAimAsWaypoint();
  /// @brief Remove all waypoints.
  void clearWaypoints();
  /// @brief Clear waypoints and selected route state.
  void clearSchedule();
  /// @brief Clear displayed navigation paths.
  void clearNavigationOverlays();
  /// @brief Clear the current aim pose.
  void clearAimPose();
  /// @brief Remove the currently selected waypoint.
  void removeSelectedWaypoint();
  /// @brief Move the selected waypoint one slot earlier.
  void moveSelectedWaypointUp();
  /// @brief Move the selected waypoint one slot later.
  void moveSelectedWaypointDown();
  /// @brief Select a waypoint by index.
  void selectWaypoint(int index);
  /// @brief Enable or disable follow-robot camera mode.
  void setFollowRobotEnabled(bool enabled);

Q_SIGNALS:
  /// @brief Emitted when the aim pose changes.
  void aimPoseChanged(const amr::visualization::Pose2D &pose);
  /// @brief Emitted when the waypoint count changes.
  void waypointCountChanged(int count);
  /// @brief Emitted when the waypoint route changes.
  void waypointsChanged(const QVector<amr::visualization::Pose2D> &waypoints);
  /// @brief Emitted when waypoint selection changes.
  void selectedWaypointChanged(int index);

protected:
  /// @brief Render all visible scene layers.
  void paintEvent(QPaintEvent *event) override;
  /// @brief Handle aim/waypoint selection and camera input press events.
  void mousePressEvent(QMouseEvent *event) override;
  /// @brief Handle panning, orbiting, and aim-heading drag updates.
  void mouseMoveEvent(QMouseEvent *event) override;
  /// @brief Complete mouse-driven scene interactions.
  void mouseReleaseEvent(QMouseEvent *event) override;
  /// @brief Zoom the camera view.
  void wheelEvent(QWheelEvent *event) override;
  /// @brief Reposition camera controls after resize.
  void resizeEvent(QResizeEvent *event) override;

private:
  /// @brief Triangle primitive loaded from an STL mesh.
  struct MeshTriangle
  {
    /// @brief First vertex.
    QVector3D a;
    /// @brief Second vertex.
    QVector3D b;
    /// @brief Third vertex.
    QVector3D c;
  };

  /// @brief Cached mesh load result for a visual mesh path.
  struct MeshCacheEntry
  {
    /// @brief True after the mesh path has been attempted.
    bool attempted{false};
    /// @brief Loaded mesh triangles.
    QVector<MeshTriangle> triangles;
  };

  /// @brief Project a world point into screen coordinates.
  QPointF worldToScreen(const QPointF &point) const;
  /// @brief Project a 3D world point into screen coordinates.
  QPointF worldToScreen3D(double x, double y, double z) const;
  /// @brief Return camera right vector.
  QVector3D cameraRight() const;
  /// @brief Return camera forward vector.
  QVector3D cameraForward() const;
  /// @brief Return camera up vector.
  QVector3D cameraUp() const;
  /// @brief Convert screen coordinates into world plane coordinates.
  QPointF screenToWorld(const QPointF &point) const;
  /// @brief Pan the camera by a pixel delta.
  void panCameraByPixels(const QPoint &delta);
  /// @brief Center the camera focal point on the robot.
  void centerViewOnRobot();
  /// @brief Update camera control widget placement.
  void updateCameraControlsGeometry();
  /// @brief Update camera button checked/enabled state.
  void updateCameraControlState();
  /// @brief Create a camera control button.
  QToolButton *makeCameraButton(
    const QIcon &icon,
    const QString &text,
    const QString &tooltip,
    bool checkable);
  /// @brief Rasterize occupancy map cells into an image.
  QImage makeGridImage(const GridMap &map, const QColor &occupied, const QColor &free) const;
  /// @brief Rasterize costmap cells into an image.
  QImage makeCostmapImage(const GridMap &map) const;
  /// @brief Draw the background metric grid.
  void drawGrid(QPainter &painter) const;
  /// @brief Draw one occupancy grid layer.
  void drawGridLayer(QPainter &painter, const GridMap &map, const QImage &image, qreal opacity);
  /// @brief Draw a world-frame path polyline.
  void drawPath(QPainter &painter, const PathData &path, const QColor &color, qreal width) const;
  /// @brief Draw a pose marker with heading.
  void drawPose(QPainter &painter, const Pose2D &pose, const QColor &color) const;
  /// @brief Draw the configured exact footprint overlay.
  void drawExactFootprint(QPainter &painter) const;
  /// @brief Draw all robot visual elements.
  void drawRobotModel(QPainter &painter) const;
  /// @brief Draw a projected 3D box primitive.
  void drawBox3D(
    QPainter &painter,
    const Pose2D &pose,
    double size_x,
    double size_y,
    double size_z,
    const QColor &color) const;
  /// @brief Draw a projected 3D cylinder proxy.
  void drawCylinderProxy3D(
    QPainter &painter,
    const Pose2D &pose,
    double radius,
    double height,
    const QColor &color) const;
  /// @brief Draw a TurtleBot Burger-style base proxy when mesh data is unavailable.
  void drawBurgerBaseProxy3D(QPainter &painter, const Pose2D &pose) const;
  /// @brief Draw a wheel proxy for simple robot visuals.
  void drawWheelProxy3D(QPainter &painter, const Pose2D &pose, const QColor &color) const;
  /// @brief Draw a loaded mesh visual if available.
  bool drawMesh3D(QPainter &painter, const RobotVisual &visual, const QColor &color) const;
  /// @brief Resolve package/file mesh URIs into local paths.
  QString resolveMeshPath(const QString &uri) const;
  /// @brief Return a cached mesh entry for a robot visual.
  const MeshCacheEntry *meshForVisual(const RobotVisual &visual) const;
  /// @brief Load an STL mesh into a cache entry.
  bool loadStlMesh(const QString &path, MeshCacheEntry &entry) const;
  /// @brief Transform a mesh-local point into world coordinates.
  QVector3D meshPointToWorld(const RobotVisual &visual, const QVector3D &point) const;
  /// @brief Draw TF frame markers and labels.
  void drawTfFrames(QPainter &painter) const;
  /// @brief Draw one TF parent-child chain line.
  void drawTfChainLine(QPainter &painter, const Pose2D &from, const Pose2D &to) const;
  /// @brief Draw laser scan hit points.
  void drawScan(QPainter &painter) const;
  /// @brief Draw the waypoint route polyline.
  void drawWaypointRoute(QPainter &painter) const;
  /// @brief Draw waypoint markers and labels.
  void drawWaypoints(QPainter &painter) const;
  /// @brief Return the waypoint index at a screen position, or -1.
  int waypointAt(const QPointF &screen_position) const;
  /// @brief Emit all waypoint-related state signals.
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
