// Copyright 2026 reidlo
//
// This file is an AMR-side OpenCR access wrapper adapted from the public
// ROBOTIS TurtleBot3/OpenCR register layout and communication pattern.
// It remains subject to the Apache License, Version 2.0.
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

#ifndef AMR_BRINGUP__BASE__OPENCR_SDK_WRAPPER_HPP_
#define AMR_BRINGUP__BASE__OPENCR_SDK_WRAPPER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace amr::tb3::base_driver
{

class OpenCRSdkWrapper
{
public:
  struct DeviceConfig
  {
    std::string port{"/dev/ttyACM0"};
    std::uint8_t id{200U};
    int baudrate{1000000};
    float protocol_version{2.0F};
  };

  OpenCRSdkWrapper();
  ~OpenCRSdkWrapper();

  OpenCRSdkWrapper(const OpenCRSdkWrapper &) = delete;
  OpenCRSdkWrapper & operator=(const OpenCRSdkWrapper &) = delete;
  OpenCRSdkWrapper(OpenCRSdkWrapper &&) = delete;
  OpenCRSdkWrapper & operator=(OpenCRSdkWrapper &&) = delete;

  void configure(DeviceConfig config);
  bool connect();
  void disconnect();
  bool reconnect();

  bool is_open() const;
  bool is_connected_to_device();

  void init_read_memory(std::uint16_t start_addr, std::uint16_t length);
  bool refresh_read_memory();

  template<typename T>
  std::optional<T> get_cached_data(std::uint16_t addr, std::uint16_t length) const
  {
    if (!this->has_read_memory_) {
      return std::nullopt;
    }

    if (addr < this->read_start_addr_) {
      return std::nullopt;
    }

    const std::size_t index = static_cast<std::size_t>(addr - this->read_start_addr_);
    if (index + length > this->read_buffer_.size() || length > sizeof(T)) {
      return std::nullopt;
    }

    T value{};
    auto * raw = reinterpret_cast<std::uint8_t *>(&value);

    std::lock_guard<std::mutex> lock(this->read_mutex_);
    for (std::size_t offset = 0; offset < length; ++offset) {
      raw[offset] = this->read_buffer_[index + offset];
    }
    return value;
  }

  bool write_register(std::uint16_t addr, std::uint16_t length, std::uint8_t * data);
  bool write_byte(std::uint16_t addr, std::uint8_t value);

  const DeviceConfig & device() const;
  const std::string & last_error() const;

private:
  bool read_register(
    std::uint16_t addr,
    std::uint16_t length,
    std::uint8_t * out_data);
  void set_error(std::string message);

  DeviceConfig device_;
  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};

  mutable std::mutex sdk_mutex_;
  mutable std::mutex read_mutex_;

  std::uint16_t read_start_addr_{0U};
  std::uint16_t read_length_{0U};
  std::vector<std::uint8_t> read_buffer_;
  std::vector<std::uint8_t> read_staging_buffer_;
  bool has_read_memory_{false};

  bool is_open_{false};
  std::string last_error_;
};

}  // namespace amr::tb3::base_driver

#endif  // AMR_BRINGUP__BASE__OPENCR_SDK_WRAPPER_HPP_
