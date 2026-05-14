// Copyright 2026 reidlo
//
// Portions of the control table layout are derived from the ROBOTIS TurtleBot3
// project and remain subject to the Apache License, Version 2.0.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AMR_BRINGUP__BASE__OPENCR_CONTROL_TABLE_HPP_
#define AMR_BRINGUP__BASE__OPENCR_CONTROL_TABLE_HPP_

#include <cstdint>

namespace amr::tb3::base_driver
{

enum class OpenCRMemoryRegion : std::uint8_t
{
  kEeprom = 1,
  kRam = 2,
};

enum class OpenCRAccess : std::uint8_t
{
  kRead = 1,
  kReadWrite = 3,
};

struct ControlItem
{
  std::uint16_t addr;
  OpenCRMemoryRegion memory;
  std::uint16_t length;
  OpenCRAccess access;
};

struct OpenCRControlTable
{
  ControlItem model_number{0, OpenCRMemoryRegion::kEeprom, 2, OpenCRAccess::kRead};
  ControlItem firmware_version{6, OpenCRMemoryRegion::kEeprom, 1, OpenCRAccess::kRead};
  ControlItem id{7, OpenCRMemoryRegion::kEeprom, 1, OpenCRAccess::kRead};
  ControlItem baud_rate{8, OpenCRMemoryRegion::kEeprom, 1, OpenCRAccess::kRead};

  ControlItem millis{10, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem micros{14, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem device_status{18, OpenCRMemoryRegion::kRam, 1, OpenCRAccess::kRead};
  ControlItem heartbeat{19, OpenCRMemoryRegion::kRam, 1, OpenCRAccess::kReadWrite};

  ControlItem battery_voltage{42, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem battery_percentage{46, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem sound{50, OpenCRMemoryRegion::kRam, 1, OpenCRAccess::kReadWrite};

  ControlItem imu_re_calibration{59, OpenCRMemoryRegion::kRam, 1, OpenCRAccess::kReadWrite};
  ControlItem imu_angular_velocity_x{60, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_angular_velocity_y{64, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_angular_velocity_z{68, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_linear_acceleration_x{72, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_linear_acceleration_y{76, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_linear_acceleration_z{80, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_magnetic_x{84, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_magnetic_y{88, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_magnetic_z{92, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_orientation_w{96, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_orientation_x{100, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_orientation_y{104, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem imu_orientation_z{108, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};

  ControlItem present_current_left{120, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem present_current_right{124, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem present_velocity_left{128, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem present_velocity_right{132, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem present_position_left{136, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};
  ControlItem present_position_right{140, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kRead};

  ControlItem motor_torque_enable{149, OpenCRMemoryRegion::kRam, 1, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_linear_x{150, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_linear_y{154, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_linear_z{158, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_angular_x{162, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_angular_y{166, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem cmd_velocity_angular_z{170, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem profile_acceleration_left{174, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
  ControlItem profile_acceleration_right{178, OpenCRMemoryRegion::kRam, 4, OpenCRAccess::kReadWrite};
};

inline constexpr OpenCRControlTable kOpenCRControlTable{};

}  // namespace amr::tb3::base_driver

#endif  // AMR_BRINGUP__BASE__OPENCR_CONTROL_TABLE_HPP_
