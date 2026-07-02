/**
 * @file operator_state.cpp
 * @brief Utility implementations for Qt-friendly operator state values.
 */

#include "amr_visualization/operator_state.hpp"

#include <cmath>

namespace amr::visualization
{

double quaternion_to_yaw(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * ((w * z) + (x * y));
  const double cosy_cosp = 1.0 - 2.0 * ((y * y) + (z * z));
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace amr::visualization
