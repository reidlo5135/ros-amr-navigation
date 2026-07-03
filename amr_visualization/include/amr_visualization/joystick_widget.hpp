#ifndef AMR_VISUALIZATION__JOYSTICK_WIDGET_HPP_
#define AMR_VISUALIZATION__JOYSTICK_WIDGET_HPP_

/**
 * @file joystick_widget.hpp
 * @brief Circular Qt joystick control that emits cmd_vel-friendly velocity commands.
 */

#include <QPointF>
#include <QTimer>
#include <QWidget>

namespace amr::visualization
{

/// @brief Mouse-driven circular joystick for manual velocity control.
class JoystickWidget : public QWidget
{
  Q_OBJECT

public:
  /// @brief Construct a joystick with TurtleBot3-friendly speed defaults.
  explicit JoystickWidget(QWidget *parent = nullptr);

  /// @brief Return the preferred square size for the joystick pad.
  QSize sizeHint() const override;
  /// @brief Return the minimum square size for usable pointer control.
  QSize minimumSizeHint() const override;

public Q_SLOTS:
  /// @brief Apply ROS parameter-backed speed and publish-rate limits.
  void configure(double max_linear_speed, double max_angular_speed, double publish_rate_hz);
  /// @brief Recenter the stick and emit one stop command.
  void resetControl();

Q_SIGNALS:
  /// @brief Emitted whenever the displayed velocity values change.
  void velocityChanged(double linear_x, double angular_z);
  /// @brief Emitted by the publish timer while the stick is held.
  void commandRequested(double linear_x, double angular_z);
  /// @brief Emitted once whenever the stick returns to center.
  void stopRequested();

protected:
  /// @brief Paint the circular pad and current handle position.
  void paintEvent(QPaintEvent *event) override;
  /// @brief Start manual control from a mouse press.
  void mousePressEvent(QMouseEvent *event) override;
  /// @brief Update the normalized command vector while dragging.
  void mouseMoveEvent(QMouseEvent *event) override;
  /// @brief Stop manual control on release.
  void mouseReleaseEvent(QMouseEvent *event) override;
  /// @brief Stop manual control if the pointer leaves the widget.
  void leaveEvent(QEvent *event) override;
  /// @brief Stop manual control when the widget becomes disabled.
  void changeEvent(QEvent *event) override;

private Q_SLOTS:
  /// @brief Emit the latest nonzero command while dragging.
  void publishCurrentCommand();

private:
  /// @brief Convert a widget position into clamped normalized joystick input.
  void updateCommandFromPosition(const QPointF &position);
  /// @brief Apply deadzone, compute velocities, and repaint.
  void setNormalizedVector(const QPointF &normalized);

  QTimer publish_timer_;
  QPointF normalized_{0.0, 0.0};
  double linear_x_{0.0};
  double angular_z_{0.0};
  double max_linear_speed_{0.22};
  double max_angular_speed_{1.8};
  double deadzone_{0.05};
  double publish_rate_hz_{20.0};
  bool dragging_{false};
};

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__JOYSTICK_WIDGET_HPP_
