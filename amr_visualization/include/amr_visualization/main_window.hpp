#ifndef AMR_VISUALIZATION__MAIN_WINDOW_HPP_
#define AMR_VISUALIZATION__MAIN_WINDOW_HPP_

#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>

#include <memory>

#include "amr_visualization/operator_state.hpp"
#include "amr_visualization/ros_worker.hpp"
#include "amr_visualization/scene_widget.hpp"

class QCheckBox;
class QVBoxLayout;

namespace amr::visualization
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

private Q_SLOTS:
  void updateAimPose(const amr::visualization::Pose2D & pose);
  void updateMotionStatus(const amr::visualization::MotionStatusData & status);
  void updateRuntimeSummary(const amr::visualization::RuntimeSummary & summary);
  void updateWaypointList(const QVector<amr::visualization::Pose2D> & waypoints);
  void appendEvent(const QString & event);
  void sendGoal();

private:
  QWidget * makeLeftPanel();
  QWidget * makeRightPanel();
  QWidget * makeTopBar();
  QCheckBox * makeLayerCheckBox(const QString & label, const QColor & color, bool checked);
  void appendStatusRow(QVBoxLayout * layout, const QString & label, QLabel * value);
  void applyStyle();

  std::unique_ptr<RosWorker> ros_worker_;
  SceneWidget * scene_{nullptr};
  QLabel * frame_label_{nullptr};
  QLabel * aim_label_{nullptr};
  QLabel * mode_label_{nullptr};
  QLabel * ai_label_{nullptr};
  QLabel * battery_label_{nullptr};
  QProgressBar * battery_bar_{nullptr};
  QLabel * motion_label_{nullptr};
  QLabel * remaining_label_{nullptr};
  QLabel * heading_label_{nullptr};
  QLabel * goal_label_{nullptr};
  QLabel * blocked_label_{nullptr};
  QLabel * recovery_label_{nullptr};
  QListWidget * waypoint_list_{nullptr};
  QListWidget * event_list_{nullptr};
  QPushButton * send_button_{nullptr};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__MAIN_WINDOW_HPP_
