#ifndef AMR_VISUALIZATION__OPERATOR_STATE_HPP_
#define AMR_VISUALIZATION__OPERATOR_STATE_HPP_

#include <QMetaType>
#include <QPointF>
#include <QString>
#include <QVector>

#include <cstdint>

namespace amr::visualization
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
};

struct FrameVisual
{
  QString parent_frame;
  QString child_frame;
  Pose2D pose;
  bool is_static{false};
};

struct GridMap
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  QVector<int8_t> cells;
  bool valid{false};
};

struct PathData
{
  QVector<QPointF> points;
};

struct ScanData
{
  QVector<QPointF> points;
};

struct MotionStatusData
{
  bool active{false};
  bool command_completed{false};
  bool goal_reached{false};
  bool blocked{false};
  bool stalled{false};
  bool obstacle_detected{false};
  bool local_plan_valid{false};
  bool costmap_blocked{false};
  bool safety_gate_blocked{false};
  double remaining_distance{0.0};
  double heading_error{0.0};
  QString blocked_source{"Clear"};
};

struct RuntimeSummary
{
  QString runtime_state{"idle"};
  QString blocked_context{"clear"};
  QString recovery_phase{"idle"};
  QString recovery_reason{"none"};
  QString action_status{"unknown"};
};

double quaternion_to_yaw(double x, double y, double z, double w);

}  // namespace amr::visualization

Q_DECLARE_METATYPE(amr::visualization::Pose2D)
Q_DECLARE_METATYPE(amr::visualization::FrameVisual)
Q_DECLARE_METATYPE(amr::visualization::GridMap)
Q_DECLARE_METATYPE(amr::visualization::PathData)
Q_DECLARE_METATYPE(amr::visualization::ScanData)
Q_DECLARE_METATYPE(amr::visualization::MotionStatusData)
Q_DECLARE_METATYPE(amr::visualization::RuntimeSummary)
Q_DECLARE_METATYPE(QVector<amr::visualization::FrameVisual>)

#endif  // AMR_VISUALIZATION__OPERATOR_STATE_HPP_
