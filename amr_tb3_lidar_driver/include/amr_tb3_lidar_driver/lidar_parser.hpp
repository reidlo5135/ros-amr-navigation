#ifndef AMR_TB3_LIDAR_DRIVER__LIDAR_PARSER_HPP_
#define AMR_TB3_LIDAR_DRIVER__LIDAR_PARSER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace amr::tb3::lidar_driver
{

struct LidarSample
{
  float angle_rad{0.0F};
  float range_m{0.0F};
  float intensity{0.0F};
  bool valid{false};
};

struct LidarScanFrame
{
  std::chrono::nanoseconds stamp{0};
  std::vector<LidarSample> samples;
};

class LidarParser
{
public:
  virtual ~LidarParser() = default;

  virtual std::optional<LidarScanFrame> parse(std::vector<std::uint8_t> & rx_buffer) = 0;
  virtual const std::string & name() const = 0;
  virtual const std::string & last_error() const = 0;
};

class Lds01Parser : public LidarParser
{
public:
  std::optional<LidarScanFrame> parse(std::vector<std::uint8_t> & rx_buffer) override;
  const std::string & name() const override;
  const std::string & last_error() const override;

private:
  std::string last_error_;
};

class Ld08Parser : public LidarParser
{
public:
  std::optional<LidarScanFrame> parse(std::vector<std::uint8_t> & rx_buffer) override;
  const std::string & name() const override;
  const std::string & last_error() const override;

private:
  std::string last_error_;
};

class Lds03Parser : public LidarParser
{
public:
  std::optional<LidarScanFrame> parse(std::vector<std::uint8_t> & rx_buffer) override;
  const std::string & name() const override;
  const std::string & last_error() const override;

private:
  std::string last_error_;
};

std::unique_ptr<LidarParser> make_lidar_parser(const std::string & sensor_model);

}  // namespace amr::tb3::lidar_driver

#endif  // AMR_TB3_LIDAR_DRIVER__LIDAR_PARSER_HPP_
