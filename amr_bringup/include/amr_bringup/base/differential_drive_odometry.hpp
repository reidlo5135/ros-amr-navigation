#ifndef AMR_TB3_BASE_DRIVER__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
#define AMR_TB3_BASE_DRIVER__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_

#include <chrono>

namespace amr::tb3::base_driver
{

struct OdometryState
{
  bool initialized{false};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_velocity_mps{0.0};
  double angular_velocity_radps{0.0};
};

class DifferentialDriveOdometry
{
public:
  DifferentialDriveOdometry(double wheel_separation_m, double wheel_radius_m);

  void set_wheel_geometry(double wheel_separation_m, double wheel_radius_m);
  void reset(std::chrono::nanoseconds stamp);

  const OdometryState & state() const;

  bool update_from_wheel_positions(
    double left_wheel_position_rad,
    double right_wheel_position_rad,
    std::chrono::nanoseconds stamp);

  bool update_from_wheel_velocities(
    double left_wheel_velocity_radps,
    double right_wheel_velocity_radps,
    std::chrono::nanoseconds stamp);

private:
  double normalize_yaw(double yaw_rad) const;
  void integrate_step(double linear_distance_m, double angular_distance_rad);

  double wheel_separation_m_;
  double wheel_radius_m_;

  bool has_previous_wheel_positions_{false};
  double previous_left_wheel_position_rad_{0.0};
  double previous_right_wheel_position_rad_{0.0};
  std::chrono::nanoseconds previous_stamp_{0};

  OdometryState state_;
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_TB3_BASE_DRIVER__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
