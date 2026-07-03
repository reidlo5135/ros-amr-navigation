#ifndef AMR_VISUALIZATION__MAIN_WINDOW_HPP_
#define AMR_VISUALIZATION__MAIN_WINDOW_HPP_

/**
 * @file main_window.hpp
 * @brief Main Qt window for the AMR operator visualization.
 */

#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>

#include <memory>

#include "amr_visualization/operator_state.hpp"
#include "amr_visualization/ai_chat_panel.hpp"
#include "amr_visualization/joystick_widget.hpp"
#include "amr_visualization/ros_worker.hpp"
#include "amr_visualization/scene_widget.hpp"

class QCheckBox;
class QParallelAnimationGroup;
class QVBoxLayout;

namespace amr::visualization
{

/// @brief Top-level operator UI that wires controls, scene rendering, and ROS worker signals.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  /// @brief Construct the main window and initialize UI panels.
  explicit MainWindow(QWidget *parent = nullptr);
  /// @brief Stop worker resources and destroy the window.
  ~MainWindow() override;

private Q_SLOTS:
  /// @brief Switch the operator shell between manual and AI modes.
  void setControlMode(bool ai_mode);
  /// @brief Update the UI label for the current aim pose.
  void updateAimPose(const amr::visualization::Pose2D &pose);
  /// @brief Update motion status labels and waypoint edit lock state.
  void updateMotionStatus(const amr::visualization::MotionStatusData &status);
  /// @brief Update runtime summary labels.
  void updateRuntimeSummary(const amr::visualization::RuntimeSummary &summary);
  /// @brief Refresh the waypoint list widget.
  void updateWaypointList(const QVector<amr::visualization::Pose2D> &waypoints);
  /// @brief Append one event line to the feedback list.
  void appendEvent(const QString &event);
  /// @brief Send either a single goal or route based on current waypoints.
  void sendGoal();

private:
  /// @brief Row widgets associated with one visualization layer checkbox.
  struct LayerCheckRow
  {
    /// @brief Container row widget.
    QWidget *row{nullptr};
    /// @brief Checkbox controlling layer visibility.
    QCheckBox *check{nullptr};
  };

  /// @brief Build the command and visualization layer panel.
  QWidget *makeLeftPanel();
  /// @brief Build the navigation status, events, and joystick panel.
  QWidget *makeRightPanel();
  /// @brief Build the top status bar.
  QWidget *makeTopBar();
  /// @brief Build one visualization layer checkbox row.
  LayerCheckRow makeLayerCheckBox(const QString &label, const QString &icon_name, bool checked);
  /// @brief Append a key/value status row to a panel layout.
  void appendStatusRow(QVBoxLayout *layout, const QString &label, QLabel *value);
  /// @brief Apply the application stylesheet.
  void applyStyle();
  /// @brief Enable or disable waypoint editing while navigation is running.
  void setWaypointEditingLocked(bool locked);
  /// @brief Update the topbar segmented control active state.
  void refreshModeButtons();
  /// @brief Return the AI panel target width for the current window size.
  int aiPanelWidth() const;
  /// @brief Apply a fixed panel width immediately and hide zero-width panels.
  void applyPanelWidth(QWidget *panel, int width);
  /// @brief Add a width slide animation for one panel.
  void animatePanelWidth(QParallelAnimationGroup *group, QWidget *panel, int target_width);

  std::unique_ptr<RosWorker> ros_worker_;
  QWidget *manual_left_panel_{nullptr};
  QWidget *manual_right_panel_{nullptr};
  AiChatPanel *ai_chat_panel_{nullptr};
  SceneWidget *scene_{nullptr};
  QLabel *frame_label_{nullptr};
  QLabel *aim_label_{nullptr};
  QPushButton *manual_mode_button_{nullptr};
  QPushButton *ai_mode_button_{nullptr};
  QLabel *battery_label_{nullptr};
  QProgressBar *battery_bar_{nullptr};
  QLabel *motion_label_{nullptr};
  QLabel *remaining_label_{nullptr};
  QLabel *heading_label_{nullptr};
  QLabel *goal_label_{nullptr};
  QLabel *blocked_label_{nullptr};
  QLabel *recovery_label_{nullptr};
  QLabel *linear_label_{nullptr};
  QLabel *angular_label_{nullptr};
  JoystickWidget *joystick_{nullptr};
  QListWidget *waypoint_list_{nullptr};
  QListWidget *event_list_{nullptr};
  QPushButton *add_waypoint_button_{nullptr};
  QPushButton *send_button_{nullptr};
  QParallelAnimationGroup *mode_animation_{nullptr};
  bool waypoint_editing_locked_{false};
  bool ai_mode_{false};
  bool pose_label_has_value_{false};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__MAIN_WINDOW_HPP_
