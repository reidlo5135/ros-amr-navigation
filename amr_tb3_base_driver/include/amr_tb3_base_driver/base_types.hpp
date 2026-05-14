#ifndef AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_
#define AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace amr::tb3::base_driver
{

enum class CommandType : std::uint8_t
{
  kNone = 0,
  kVelocity = 1,
  kStop = 2,
};

struct BaseCommand
{
  CommandType type{CommandType::kNone};
  double linear_x_mps{0.0};
  double angular_z_radps{0.0};
};

struct BaseState
{
  bool connected{false};
  bool protocol_ready{false};
  std::uint64_t transport_sequence{0U};
  std::string last_error;
};

struct WheelState
{
  bool position_valid{false};
  bool velocity_valid{false};
  double position_rad{0.0};
  double velocity_radps{0.0};
};

struct ImuState
{
  bool orientation_valid{false};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> angular_velocity_xyz{0.0, 0.0, 0.0};
  std::array<double, 3> linear_acceleration_xyz{0.0, 0.0, 0.0};
};

struct BatteryState
{
  bool valid{false};
  double voltage_v{0.0};
  double percentage{0.0};
};

struct BaseFeedback
{
  std::chrono::nanoseconds stamp{0};
  WheelState left_wheel;
  WheelState right_wheel;
  ImuState imu;
  BatteryState battery;
};

struct OpenCRPacket
{
  std::uint8_t device_id{1U};
  std::uint8_t instruction{0U};
  std::vector<std::uint8_t> payload;
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_
