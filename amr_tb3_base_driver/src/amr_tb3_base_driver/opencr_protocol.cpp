#include "amr_tb3_base_driver/opencr_protocol.hpp"

#include <cstring>
#include <cstddef>
#include <cstdint>

#include <utility>
#include <vector>

namespace amr::tb3::base_driver
{

namespace
{

constexpr std::uint8_t kVelocityInstruction = 0x10U;

void append_float32_le(std::vector<std::uint8_t> & out, const float value)
{
  std::uint32_t encoded = 0U;
  static_assert(sizeof(float) == sizeof(std::uint32_t), "Unexpected float size");
  std::memcpy(&encoded, &value, sizeof(float));
  out.push_back(static_cast<std::uint8_t>(encoded & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((encoded >> 8U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((encoded >> 16U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((encoded >> 24U) & 0xFFU));
}

}  // namespace

std::vector<std::uint8_t> OpenCRProtocol::build_packet(const OpenCRPacket & packet) const
{
  std::vector<std::uint8_t> bytes;
  bytes.reserve(kMinimumPacketSize + packet.payload.size());
  bytes.push_back(kHeader0);
  bytes.push_back(kHeader1);
  bytes.push_back(packet.device_id);
  bytes.push_back(packet.instruction);
  bytes.push_back(static_cast<std::uint8_t>(packet.payload.size()));
  bytes.insert(bytes.end(), packet.payload.begin(), packet.payload.end());

  const std::uint16_t checksum = this->compute_checksum(bytes);
  bytes.push_back(static_cast<std::uint8_t>(checksum & 0xFFU));
  bytes.push_back(static_cast<std::uint8_t>((checksum >> 8U) & 0xFFU));
  return bytes;
}

std::optional<OpenCRPacket> OpenCRProtocol::try_parse_packet(
  std::vector<std::uint8_t> & rx_buffer) const
{
  while (rx_buffer.size() >= 2U) {
    if (rx_buffer[0] == kHeader0 && rx_buffer[1] == kHeader1) {
      break;
    }
    rx_buffer.erase(rx_buffer.begin());
  }

  if (rx_buffer.size() < kMinimumPacketSize + 1U) {
    return std::nullopt;
  }

  const std::size_t payload_size = rx_buffer[4];
  const std::size_t packet_size = 5U + payload_size + 2U;
  if (rx_buffer.size() < packet_size) {
    return std::nullopt;
  }

  std::vector<std::uint8_t> packet_bytes(rx_buffer.begin(), rx_buffer.begin() + packet_size);
  const std::uint16_t expected_checksum =
    static_cast<std::uint16_t>(packet_bytes[packet_size - 2U]) |
    (static_cast<std::uint16_t>(packet_bytes[packet_size - 1U]) << 8U);

  packet_bytes.resize(packet_size - 2U);
  const std::uint16_t actual_checksum = this->compute_checksum(packet_bytes);
  if (actual_checksum != expected_checksum) {
    this->set_error("OpenCR packet checksum mismatch");
    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + packet_size);
    return std::nullopt;
  }

  OpenCRPacket packet;
  packet.device_id = packet_bytes[2];
  packet.instruction = packet_bytes[3];
  packet.payload.assign(packet_bytes.begin() + 5, packet_bytes.end());

  rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + packet_size);
  return packet;
}

std::optional<std::vector<std::uint8_t>> OpenCRProtocol::encode_command(
  const BaseCommand & command) const
{
  if (command.type != CommandType::kVelocity && command.type != CommandType::kStop) {
    this->set_error("Unsupported base command type for OpenCR encoding");
    return std::nullopt;
  }

  OpenCRPacket packet;
  packet.device_id = 1U;
  packet.instruction = kVelocityInstruction;
  packet.payload.reserve(8U);

  const float linear_x_mps =
    command.type == CommandType::kStop ? 0.0F : static_cast<float>(command.linear_x_mps);
  const float angular_z_radps =
    command.type == CommandType::kStop ? 0.0F : static_cast<float>(command.angular_z_radps);

  // TODO(reidlo): Verify the real TurtleBot3 OpenCR velocity instruction, units, and framing
  // against hardware before enabling this path for production use.
  append_float32_le(packet.payload, linear_x_mps);
  append_float32_le(packet.payload, angular_z_radps);

  this->last_error_.clear();
  return this->build_packet(packet);
}

std::optional<BaseState> OpenCRProtocol::decode_state(const OpenCRPacket & packet) const
{
  (void)packet;
  this->set_error(
    "OpenCR state decoding requires hardware-verified feedback packet definitions and is not finalized yet");
  return std::nullopt;
}

const std::string & OpenCRProtocol::last_error() const
{
  return this->last_error_;
}

std::uint16_t OpenCRProtocol::compute_checksum(const std::vector<std::uint8_t> & bytes) const
{
  std::uint16_t checksum = 0U;
  for (const std::uint8_t byte : bytes) {
    checksum = static_cast<std::uint16_t>((checksum + byte) & 0xFFFFU);
  }
  return checksum;
}

void OpenCRProtocol::set_error(std::string error_message) const
{
  this->last_error_ = std::move(error_message);
}

}  // namespace amr::tb3::base_driver
