#ifndef AMR_TB3_BASE_DRIVER__OPENCR_PROTOCOL_HPP_
#define AMR_TB3_BASE_DRIVER__OPENCR_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "amr_tb3_base_driver/base_types.hpp"

namespace amr::tb3::base_driver
{

class OpenCRProtocol
{
public:
  OpenCRProtocol() = default;

  std::vector<std::uint8_t> build_packet(const OpenCRPacket & packet) const;
  std::optional<OpenCRPacket> try_parse_packet(std::vector<std::uint8_t> & rx_buffer) const;

  std::optional<std::vector<std::uint8_t>> encode_command(const BaseCommand & command) const;
  std::optional<BaseState> decode_state(const OpenCRPacket & packet) const;

  const std::string & last_error() const;

private:
  std::uint16_t compute_checksum(const std::vector<std::uint8_t> & bytes) const;
  void set_error(std::string error_message) const;

  // TODO(reidlo): Replace this provisional frame envelope with hardware-verified OpenCR framing.
  static constexpr std::uint8_t kHeader0 = 0xAAU;
  static constexpr std::uint8_t kHeader1 = 0x55U;
  static constexpr std::size_t kMinimumPacketSize = 6U;

  mutable std::string last_error_;
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_TB3_BASE_DRIVER__OPENCR_PROTOCOL_HPP_
