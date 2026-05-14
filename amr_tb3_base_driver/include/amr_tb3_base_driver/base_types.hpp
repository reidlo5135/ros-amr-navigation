#ifndef AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_
#define AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_

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

struct OpenCRPacket
{
  std::uint8_t device_id{1U};
  std::uint8_t instruction{0U};
  std::vector<std::uint8_t> payload;
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_TB3_BASE_DRIVER__BASE_TYPES_HPP_
