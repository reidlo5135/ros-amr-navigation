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
constexpr std::uint8_t kFeedbackInstruction = 0x20U;

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

float read_float32_le(const std::vector<std::uint8_t> & bytes, const std::size_t offset)
{
  std::uint32_t encoded = 0U;
  encoded |= static_cast<std::uint32_t>(bytes[offset]);
  encoded |= static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U;
  encoded |= static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U;
  encoded |= static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U;

  float value = 0.0F;
  std::memcpy(&value, &encoded, sizeof(float));
  return value;
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
  BaseState state;
  state.connected = true;
  state.protocol_ready = packet.instruction == kFeedbackInstruction;
  state.transport_sequence = 1U;
  state.last_error.clear();
  this->last_error_.clear();
  return state;
}

std::optional<BaseFeedback> OpenCRProtocol::decode_feedback(const OpenCRPacket & packet) const
{
  if (packet.instruction != kFeedbackInstruction) {
    return std::nullopt;
  }

  // TODO(reidlo): Replace this provisional feedback layout with hardware-verified OpenCR wheel,
  // IMU, and battery feedback parsing once the real packet contract is confirmed on TurtleBot3.
  if (packet.payload.size() < 16U) {
    this->set_error("OpenCR feedback packet is too short for provisional wheel decoding");
    return std::nullopt;
  }

  BaseFeedback feedback;
  feedback.left_wheel.position_valid = true;
  feedback.right_wheel.position_valid = true;
  feedback.left_wheel.velocity_valid = true;
  feedback.right_wheel.velocity_valid = true;
  feedback.left_wheel.position_rad = static_cast<double>(read_float32_le(packet.payload, 0U));
  feedback.right_wheel.position_rad = static_cast<double>(read_float32_le(packet.payload, 4U));
  feedback.left_wheel.velocity_radps = static_cast<double>(read_float32_le(packet.payload, 8U));
  feedback.right_wheel.velocity_radps = static_cast<double>(read_float32_le(packet.payload, 12U));
  this->last_error_.clear();
  return feedback;
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
