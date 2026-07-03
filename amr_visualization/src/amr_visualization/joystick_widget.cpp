/**
 * @file joystick_widget.cpp
 * @brief Implementation of the manual velocity joystick widget.
 */

#include "amr_visualization/joystick_widget.hpp"

#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QSizePolicy>

#include <algorithm>
#include <cmath>

namespace amr::visualization
{

/// @copydoc JoystickWidget::JoystickWidget
JoystickWidget::JoystickWidget(QWidget *parent)
: QWidget(parent)
{
  setObjectName("joystickPad");
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  publish_timer_.setTimerType(Qt::PreciseTimer);
  publish_timer_.setInterval(
    std::max(1, static_cast<int>(std::round(1000.0 / publish_rate_hz_))));
  connect(&publish_timer_, &QTimer::timeout, this, &JoystickWidget::publishCurrentCommand);
}

/// @copydoc JoystickWidget::sizeHint
QSize JoystickWidget::sizeHint() const
{
  return QSize(168, 168);
}

/// @copydoc JoystickWidget::minimumSizeHint
QSize JoystickWidget::minimumSizeHint() const
{
  return QSize(136, 136);
}

/// @copydoc JoystickWidget::configure
void JoystickWidget::configure(
  double max_linear_speed,
  double max_angular_speed,
  double publish_rate_hz)
{
  max_linear_speed_ =
    std::isfinite(max_linear_speed) && max_linear_speed > 0.0 ? max_linear_speed : 0.22;
  max_angular_speed_ =
    std::isfinite(max_angular_speed) && max_angular_speed > 0.0 ? max_angular_speed : 1.8;
  publish_rate_hz_ =
    std::isfinite(publish_rate_hz) && publish_rate_hz > 0.0 ? publish_rate_hz : 20.0;
  publish_timer_.setInterval(
    std::max(1, static_cast<int>(std::round(1000.0 / publish_rate_hz_))));
  setNormalizedVector(normalized_);
}

/// @copydoc JoystickWidget::resetControl
void JoystickWidget::resetControl()
{
  dragging_ = false;
  publish_timer_.stop();
  setNormalizedVector(QPointF(0.0, 0.0));
  Q_EMIT stopRequested();
}

/// @copydoc JoystickWidget::paintEvent
void JoystickWidget::paintEvent(QPaintEvent *event)
{
  (void)event;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const qreal side = std::min(width(), height()) - 16.0;
  const QRectF pad_rect(
    (width() - side) * 0.5,
    (height() - side) * 0.5,
    side,
    side);
  const QPointF center = pad_rect.center();
  const qreal radius = pad_rect.width() * 0.5;
  const QPointF handle_position(
    center.x() + (normalized_.x() * radius),
    center.y() + (normalized_.y() * radius));

  painter.setPen(QPen(QColor("#2e4c80"), 1.4));
  painter.setBrush(QColor("#08101a"));
  painter.drawEllipse(pad_rect);

  painter.setPen(QPen(QColor("#223754"), 1.0, Qt::DashLine));
  painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()));
  painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius));

  painter.setPen(QPen(QColor("#3f6eb0"), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center, radius * deadzone_, radius * deadzone_);

  painter.setPen(QPen(QColor("#ffbc55"), 2.0));
  painter.drawLine(center, handle_position);

  painter.setPen(QPen(QColor("#ff9917"), 2.0));
  painter.setBrush(QColor("#ff9917"));
  painter.drawEllipse(handle_position, 13.0, 13.0);

  painter.setPen(QPen(QColor("#ffe2a6"), 1.0));
  painter.setBrush(QColor("#ffe2a6"));
  painter.drawEllipse(handle_position, 4.2, 4.2);
}

/// @copydoc JoystickWidget::mousePressEvent
void JoystickWidget::mousePressEvent(QMouseEvent *event)
{
  if (event->button() != Qt::LeftButton || !isEnabled()) {
    QWidget::mousePressEvent(event);
    return;
  }

  dragging_ = true;
  updateCommandFromPosition(event->position());
  publish_timer_.start();
  publishCurrentCommand();
  event->accept();
}

/// @copydoc JoystickWidget::mouseMoveEvent
void JoystickWidget::mouseMoveEvent(QMouseEvent *event)
{
  if (!dragging_ || !(event->buttons() & Qt::LeftButton)) {
    QWidget::mouseMoveEvent(event);
    return;
  }

  updateCommandFromPosition(event->position());
  event->accept();
}

/// @copydoc JoystickWidget::mouseReleaseEvent
void JoystickWidget::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton) {
    resetControl();
    event->accept();
    return;
  }

  QWidget::mouseReleaseEvent(event);
}

/// @copydoc JoystickWidget::leaveEvent
void JoystickWidget::leaveEvent(QEvent *event)
{
  if (dragging_) {
    resetControl();
  }
  QWidget::leaveEvent(event);
}

/// @copydoc JoystickWidget::changeEvent
void JoystickWidget::changeEvent(QEvent *event)
{
  if (event->type() == QEvent::EnabledChange && !isEnabled()) {
    resetControl();
  }
  QWidget::changeEvent(event);
}

/// @copydoc JoystickWidget::publishCurrentCommand
void JoystickWidget::publishCurrentCommand()
{
  if (!dragging_) {
    return;
  }
  Q_EMIT commandRequested(linear_x_, angular_z_);
}

/// @copydoc JoystickWidget::updateCommandFromPosition
void JoystickWidget::updateCommandFromPosition(const QPointF &position)
{
  const qreal side = std::min(width(), height()) - 16.0;
  if (side <= 0.0) {
    return;
  }

  const QRectF pad_rect(
    (width() - side) * 0.5,
    (height() - side) * 0.5,
    side,
    side);
  const QPointF center = pad_rect.center();
  const double radius = pad_rect.width() * 0.5;
  QPointF normalized(
    (position.x() - center.x()) / radius,
    (position.y() - center.y()) / radius);

  const double length = std::hypot(normalized.x(), normalized.y());
  if (length > 1.0) {
    normalized /= length;
  }
  setNormalizedVector(normalized);
}

/// @copydoc JoystickWidget::setNormalizedVector
void JoystickWidget::setNormalizedVector(const QPointF &normalized)
{
  QPointF next = normalized;
  if (std::hypot(next.x(), next.y()) < deadzone_) {
    next = QPointF(0.0, 0.0);
  }

  const double next_linear_x = -next.y() * max_linear_speed_;
  const double next_angular_z = -next.x() * max_angular_speed_;
  const bool changed =
    std::abs(next.x() - normalized_.x()) > 1e-6 ||
    std::abs(next.y() - normalized_.y()) > 1e-6 ||
    std::abs(next_linear_x - linear_x_) > 1e-6 ||
    std::abs(next_angular_z - angular_z_) > 1e-6;

  normalized_ = next;
  linear_x_ = next_linear_x;
  angular_z_ = next_angular_z;
  if (changed) {
    Q_EMIT velocityChanged(linear_x_, angular_z_);
    update();
  }
}

}  // namespace amr::visualization
