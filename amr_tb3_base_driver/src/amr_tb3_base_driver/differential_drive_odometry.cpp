#include "amr_tb3_base_driver/differential_drive_odometry.hpp"

#include <algorithm>
#include <cmath>

namespace amr::tb3::base_driver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

DifferentialDriveOdometry::DifferentialDriveOdometry(
  const double wheel_separation_m,
  const double wheel_radius_m)
: wheel_separation_m_(wheel_separation_m),
  wheel_radius_m_(wheel_radius_m)
{
}

void DifferentialDriveOdometry::set_wheel_geometry(
  const double wheel_separation_m,
  const double wheel_radius_m)
{
  this->wheel_separation_m_ = wheel_separation_m;
  this->wheel_radius_m_ = wheel_radius_m;
}

void DifferentialDriveOdometry::reset(const std::chrono::nanoseconds stamp)
{
  this->previous_stamp_ = stamp;
  this->has_previous_wheel_positions_ = false;
  this->previous_left_wheel_position_rad_ = 0.0;
  this->previous_right_wheel_position_rad_ = 0.0;
  this->state_ = OdometryState{};
  this->state_.initialized = true;
}

const OdometryState & DifferentialDriveOdometry::state() const
{
  return this->state_;
}

bool DifferentialDriveOdometry::update_from_wheel_positions(
  const double left_wheel_position_rad,
  const double right_wheel_position_rad,
  const std::chrono::nanoseconds stamp)
{
  if (!this->state_.initialized) {
    this->reset(stamp);
  }

  if (!this->has_previous_wheel_positions_) {
    this->previous_left_wheel_position_rad_ = left_wheel_position_rad;
    this->previous_right_wheel_position_rad_ = right_wheel_position_rad;
    this->previous_stamp_ = stamp;
    this->has_previous_wheel_positions_ = true;
    return false;
  }

  const auto dt_ns = stamp - this->previous_stamp_;
  if (dt_ns.count() <= 0) {
    return false;
  }

  const double left_delta_rad = left_wheel_position_rad - this->previous_left_wheel_position_rad_;
  const double right_delta_rad = right_wheel_position_rad - this->previous_right_wheel_position_rad_;
  const double left_distance_m = left_delta_rad * this->wheel_radius_m_;
  const double right_distance_m = right_delta_rad * this->wheel_radius_m_;
  const double linear_distance_m = 0.5 * (left_distance_m + right_distance_m);
  const double angular_distance_rad =
    (right_distance_m - left_distance_m) / std::max(this->wheel_separation_m_, 1e-9);

  const double dt_sec = static_cast<double>(dt_ns.count()) * 1e-9;
  this->integrate_step(linear_distance_m, angular_distance_rad);
  this->state_.linear_velocity_mps = linear_distance_m / dt_sec;
  this->state_.angular_velocity_radps = angular_distance_rad / dt_sec;

  this->previous_left_wheel_position_rad_ = left_wheel_position_rad;
  this->previous_right_wheel_position_rad_ = right_wheel_position_rad;
  this->previous_stamp_ = stamp;
  return true;
}

bool DifferentialDriveOdometry::update_from_wheel_velocities(
  const double left_wheel_velocity_radps,
  const double right_wheel_velocity_radps,
  const std::chrono::nanoseconds stamp)
{
  if (!this->state_.initialized) {
    this->reset(stamp);
    return false;
  }

  const auto dt_ns = stamp - this->previous_stamp_;
  if (dt_ns.count() <= 0) {
    return false;
  }

  const double dt_sec = static_cast<double>(dt_ns.count()) * 1e-9;
  const double left_distance_m = left_wheel_velocity_radps * this->wheel_radius_m_ * dt_sec;
  const double right_distance_m = right_wheel_velocity_radps * this->wheel_radius_m_ * dt_sec;
  const double linear_distance_m = 0.5 * (left_distance_m + right_distance_m);
  const double angular_distance_rad =
    (right_distance_m - left_distance_m) / std::max(this->wheel_separation_m_, 1e-9);

  this->integrate_step(linear_distance_m, angular_distance_rad);
  this->state_.linear_velocity_mps = linear_distance_m / dt_sec;
  this->state_.angular_velocity_radps = angular_distance_rad / dt_sec;
  this->previous_stamp_ = stamp;
  return true;
}

double DifferentialDriveOdometry::normalize_yaw(const double yaw_rad) const
{
  double normalized = yaw_rad;
  while (normalized > kPi) {
    normalized -= 2.0 * kPi;
  }
  while (normalized < -kPi) {
    normalized += 2.0 * kPi;
  }
  return normalized;
}

void DifferentialDriveOdometry::integrate_step(
  const double linear_distance_m,
  const double angular_distance_rad)
{
  const double mid_yaw = this->state_.yaw_rad + (0.5 * angular_distance_rad);
  this->state_.x_m += linear_distance_m * std::cos(mid_yaw);
  this->state_.y_m += linear_distance_m * std::sin(mid_yaw);
  this->state_.yaw_rad = this->normalize_yaw(this->state_.yaw_rad + angular_distance_rad);
}

}  // namespace amr::tb3::base_driver
