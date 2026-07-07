#ifndef AMR_VISUALIZATION__OPERATOR_STATE_HPP_
#define AMR_VISUALIZATION__OPERATOR_STATE_HPP_

/**
 * @file operator_state.hpp
 * @brief Qt-friendly value types exchanged between ROS worker and AMR UI widgets.
 */

#include <QMetaType>
#include <QPointF>
#include <QString>
#include <QVector>

#include <cstdint>

namespace amr::visualization
{

/// @brief Lightweight 2D/planar pose with optional z value for visualization.
struct Pose2D
{
  /// @brief X coordinate in meters.
  double x{0.0};
  /// @brief Y coordinate in meters.
  double y{0.0};
  /// @brief Z coordinate in meters.
  double z{0.0};
  /// @brief Planar yaw in radians.
  double yaw{0.0};
  /// @brief True when the pose contains usable data.
  bool valid{false};
};

/// @brief Visualized TF edge and child-frame pose.
struct FrameVisual
{
  /// @brief Parent TF frame id.
  QString parent_frame;
  /// @brief Child TF frame id.
  QString child_frame;
  /// @brief Child pose resolved in the fixed frame.
  Pose2D pose;
  /// @brief True when the transform came from /tf_static.
  bool is_static{false};
};

/// @brief Supported robot geometry primitives for visualization.
enum class RobotGeometryType
{
  /// @brief Box/cuboid geometry.
  Box,
  /// @brief Cylinder geometry.
  Cylinder,
  /// @brief Sphere geometry.
  Sphere,
  /// @brief Mesh geometry.
  Mesh,
};

/// @brief One robot visual element resolved from robot_description and TF.
struct RobotVisual
{
  /// @brief Frame id that owns this visual.
  QString frame_id;
  /// @brief Visual pose in the fixed frame.
  Pose2D pose;
  /// @brief Geometry primitive or mesh type.
  RobotGeometryType type{RobotGeometryType::Box};
  /// @brief Box size along X in meters.
  double size_x{0.0};
  /// @brief Box size along Y in meters.
  double size_y{0.0};
  /// @brief Box size along Z in meters.
  double size_z{0.0};
  /// @brief Radius for sphere/cylinder primitives.
  double radius{0.0};
  /// @brief Length/height for cylinder primitives.
  double length{0.0};
  /// @brief Mesh URI or path for mesh visuals.
  QString mesh_filename;
  /// @brief Mesh scale along X.
  double mesh_scale_x{1.0};
  /// @brief Mesh scale along Y.
  double mesh_scale_y{1.0};
  /// @brief Mesh scale along Z.
  double mesh_scale_z{1.0};
  /// @brief True when this visual has enough data to render.
  bool valid{false};
};

/// @brief Occupancy grid data converted into Qt containers.
struct GridMap
{
  /// @brief Grid width in cells.
  int width{0};
  /// @brief Grid height in cells.
  int height{0};
  /// @brief Cell resolution in meters.
  double resolution{0.0};
  /// @brief World X coordinate of grid origin.
  double origin_x{0.0};
  /// @brief World Y coordinate of grid origin.
  double origin_y{0.0};
  /// @brief Yaw of grid origin.
  double origin_yaw{0.0};
  /// @brief Row-major occupancy/cost values.
  QVector<int8_t> cells;
  /// @brief True when the grid has usable dimensions and data.
  bool valid{false};
};

/// @brief Polyline path data in world coordinates.
struct PathData
{
  /// @brief Ordered 2D path points.
  QVector<QPointF> points;
};

/// @brief Laser scan points projected into the fixed/world frame.
struct ScanData
{
  /// @brief Ordered scan hit points.
  QVector<QPointF> points;
};

/// @brief Motion status data displayed by the operator UI.
struct MotionStatusData
{
  /// @brief True when a motion command is active.
  bool active{false};
  /// @brief True when the active command has completed.
  bool command_completed{false};
  /// @brief True when the navigation goal has been reached.
  bool goal_reached{false};
  /// @brief True when controller blocked semantics are active.
  bool blocked{false};
  /// @brief True when controller stalled semantics are active.
  bool stalled{false};
  /// @brief True when obstacle context is active.
  bool obstacle_detected{false};
  /// @brief True when the latest local plan is valid.
  bool local_plan_valid{false};
  /// @brief True when costmap blockage is reported.
  bool costmap_blocked{false};
  /// @brief True when the motion safety gate is blocking.
  bool safety_gate_blocked{false};
  /// @brief Remaining distance reported by the controller.
  double remaining_distance{0.0};
  /// @brief Heading error reported by the controller.
  double heading_error{0.0};
  /// @brief UI-friendly source label for blocked state.
  QString blocked_source{"Clear"};
};

/// @brief Runtime observation summary data displayed by the operator UI.
struct RuntimeSummary
{
  /// @brief Top-level runtime state label.
  QString runtime_state{"idle"};
  /// @brief Blocked context label.
  QString blocked_context{"clear"};
  /// @brief Recovery phase label.
  QString recovery_phase{"idle"};
  /// @brief Recovery reason label.
  QString recovery_reason{"none"};
  /// @brief Action status label.
  QString action_status{"unknown"};
  /// @brief True when local escape is active.
  bool local_escape_active{false};
};

/// @brief Frontier navigation status data displayed by the operator UI.
struct FrontierStatusData
{
  /// @brief High-level frontier navigation phase.
  QString phase{"IDLE"};
  /// @brief Human-readable detail from the frontier navigator.
  QString message;
  /// @brief Current staging iteration.
  int iteration{0};
  /// @brief True while the unknown-goal action is active.
  bool active{false};
  /// @brief True when the original goal is known/free in the latest map.
  bool original_goal_known{false};
  /// @brief Distance from the current pose to the original goal.
  double distance_to_original_goal{-1.0};
};

/// @brief Convert a quaternion into planar yaw.
double quaternion_to_yaw(double x, double y, double z, double w);

}  // namespace amr::visualization

Q_DECLARE_METATYPE(amr::visualization::Pose2D)
Q_DECLARE_METATYPE(amr::visualization::FrameVisual)
Q_DECLARE_METATYPE(amr::visualization::RobotVisual)
Q_DECLARE_METATYPE(amr::visualization::GridMap)
Q_DECLARE_METATYPE(amr::visualization::PathData)
Q_DECLARE_METATYPE(amr::visualization::ScanData)
Q_DECLARE_METATYPE(amr::visualization::MotionStatusData)
Q_DECLARE_METATYPE(amr::visualization::RuntimeSummary)
Q_DECLARE_METATYPE(amr::visualization::FrontierStatusData)
Q_DECLARE_METATYPE(QVector<amr::visualization::FrameVisual>)
Q_DECLARE_METATYPE(QVector<amr::visualization::RobotVisual>)

#endif  // AMR_VISUALIZATION__OPERATOR_STATE_HPP_
