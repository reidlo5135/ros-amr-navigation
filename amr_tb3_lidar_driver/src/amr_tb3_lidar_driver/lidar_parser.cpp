#include "amr_tb3_lidar_driver/lidar_parser.hpp"

namespace amr::tb3::lidar_driver
{

namespace
{

void trim_oversized_buffer(std::vector<std::uint8_t> & rx_buffer)
{
  constexpr std::size_t kMaxBufferedBytes = 4096U;
  if (rx_buffer.size() > kMaxBufferedBytes) {
    rx_buffer.erase(
      rx_buffer.begin(),
      rx_buffer.begin() + static_cast<std::ptrdiff_t>(rx_buffer.size() - kMaxBufferedBytes));
  }
}

}  // namespace

std::optional<LidarScanFrame> Lds01Parser::parse(std::vector<std::uint8_t> & rx_buffer)
{
  trim_oversized_buffer(rx_buffer);
  this->last_error_ =
    "LDS-01 parser skeleton only: packet header/checksum verification still requires hardware capture";
  return std::nullopt;
}

const std::string & Lds01Parser::name() const
{
  static const std::string kName = "lds_01";
  return kName;
}

const std::string & Lds01Parser::last_error() const
{
  return this->last_error_;
}

std::optional<LidarScanFrame> Ld08Parser::parse(std::vector<std::uint8_t> & rx_buffer)
{
  trim_oversized_buffer(rx_buffer);
  this->last_error_ =
    "LD08 parser skeleton only: packet framing differences must be confirmed on the deployed sensor";
  return std::nullopt;
}

const std::string & Ld08Parser::name() const
{
  static const std::string kName = "lds_02_ld08";
  return kName;
}

const std::string & Ld08Parser::last_error() const
{
  return this->last_error_;
}

std::optional<LidarScanFrame> Lds03Parser::parse(std::vector<std::uint8_t> & rx_buffer)
{
  trim_oversized_buffer(rx_buffer);
  this->last_error_ =
    "LDS-03 parser placeholder only: exact packet contract is not wired in this pass";
  return std::nullopt;
}

const std::string & Lds03Parser::name() const
{
  static const std::string kName = "lds_03";
  return kName;
}

const std::string & Lds03Parser::last_error() const
{
  return this->last_error_;
}

std::unique_ptr<LidarParser> make_lidar_parser(const std::string & sensor_model)
{
  if (sensor_model == "lds_01") {
    return std::make_unique<Lds01Parser>();
  }
  if (sensor_model == "lds_02_ld08" || sensor_model == "ld08") {
    return std::make_unique<Ld08Parser>();
  }
  if (sensor_model == "lds_03") {
    return std::make_unique<Lds03Parser>();
  }

  return std::make_unique<Lds01Parser>();
}

}  // namespace amr::tb3::lidar_driver
