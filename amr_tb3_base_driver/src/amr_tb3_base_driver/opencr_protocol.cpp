#include "amr_tb3_base_driver/opencr_protocol.hpp"

#include <cstddef>
#include <cstdint>

#include <utility>
#include <vector>

namespace amr::tb3::base_driver
{

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
  (void)command;
  this->set_error(
    "OpenCR command encoding requires hardware-verified packet definitions and is not finalized yet");
  return std::nullopt;
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
