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

#include "amr_bringup/base/opencr_sdk_wrapper.hpp"

#include <algorithm>
#include <utility>

namespace amr::tb3::base_driver
{

OpenCRSdkWrapper::OpenCRSdkWrapper() = default;

OpenCRSdkWrapper::~OpenCRSdkWrapper()
{
  this->disconnect();
}

void OpenCRSdkWrapper::configure(DeviceConfig config)
{
  this->device_ = std::move(config);
}

bool OpenCRSdkWrapper::connect()
{
  this->disconnect();

  this->port_handler_ = dynamixel::PortHandler::getPortHandler(this->device_.port.c_str());
  this->packet_handler_ = dynamixel::PacketHandler::getPacketHandler(
    static_cast<int>(this->device_.protocol_version));

  if (this->port_handler_ == nullptr || this->packet_handler_ == nullptr) {
    this->set_error("Failed to create DynamixelSDK handlers for OpenCR");
    return false;
  }

  if (!this->port_handler_->openPort()) {
    this->set_error("Failed to open OpenCR port " + this->device_.port);
    return false;
  }

  if (!this->port_handler_->setBaudRate(this->device_.baudrate)) {
    this->set_error("Failed to set OpenCR baudrate to " + std::to_string(this->device_.baudrate));
    this->port_handler_->closePort();
    return false;
  }

  this->is_open_ = true;
  this->last_error_.clear();
  return true;
}

void OpenCRSdkWrapper::disconnect()
{
  if (this->port_handler_ != nullptr) {
    this->port_handler_->closePort();
  }

  this->port_handler_ = nullptr;
  this->packet_handler_ = nullptr;
  this->is_open_ = false;
}

bool OpenCRSdkWrapper::reconnect()
{
  return this->connect();
}

bool OpenCRSdkWrapper::is_open() const
{
  return this->is_open_;
}

bool OpenCRSdkWrapper::is_connected_to_device()
{
  std::uint8_t probe[2] = {};
  return this->read_register(0U, sizeof(probe), probe);
}

void OpenCRSdkWrapper::init_read_memory(std::uint16_t start_addr, std::uint16_t length)
{
  this->read_start_addr_ = start_addr;
  this->read_length_ = length;
  this->read_buffer_.assign(length, 0U);
  this->read_staging_buffer_.assign(length, 0U);
  this->has_read_memory_ = true;
}

bool OpenCRSdkWrapper::refresh_read_memory()
{
  if (!this->has_read_memory_) {
    this->set_error("OpenCR read memory range has not been initialized");
    return false;
  }

  if (!this->read_register(
      this->read_start_addr_,
      this->read_length_,
      this->read_staging_buffer_.data()))
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->read_mutex_);
    std::copy(
      this->read_staging_buffer_.begin(),
      this->read_staging_buffer_.end(),
      this->read_buffer_.begin());
  }

  this->last_error_.clear();
  return true;
}

bool OpenCRSdkWrapper::write_register(
  std::uint16_t addr,
  std::uint16_t length,
  std::uint8_t * data)
{
  if (!this->is_open_ || this->packet_handler_ == nullptr || this->port_handler_ == nullptr) {
    this->set_error("OpenCR SDK wrapper is not connected");
    return false;
  }

  std::lock_guard<std::mutex> lock(this->sdk_mutex_);

  int dxl_comm_result = COMM_TX_FAIL;
  std::uint8_t dxl_error = 0U;
  dxl_comm_result = this->packet_handler_->writeTxRx(
    this->port_handler_,
    this->device_.id,
    addr,
    length,
    data,
    &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    this->set_error(this->packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  if (dxl_error != 0U) {
    this->set_error(this->packet_handler_->getRxPacketError(dxl_error));
    return false;
  }

  this->last_error_.clear();
  return true;
}

bool OpenCRSdkWrapper::write_byte(std::uint16_t addr, std::uint8_t value)
{
  return this->write_register(addr, 1U, &value);
}

const OpenCRSdkWrapper::DeviceConfig & OpenCRSdkWrapper::device() const
{
  return this->device_;
}

const std::string & OpenCRSdkWrapper::last_error() const
{
  return this->last_error_;
}

bool OpenCRSdkWrapper::read_register(
  std::uint16_t addr,
  std::uint16_t length,
  std::uint8_t * out_data)
{
  if (!this->is_open_ || this->packet_handler_ == nullptr || this->port_handler_ == nullptr) {
    this->set_error("OpenCR SDK wrapper is not connected");
    return false;
  }

  std::lock_guard<std::mutex> lock(this->sdk_mutex_);

  int dxl_comm_result = COMM_RX_FAIL;
  std::uint8_t dxl_error = 0U;
  dxl_comm_result = this->packet_handler_->readTxRx(
    this->port_handler_,
    this->device_.id,
    addr,
    length,
    out_data,
    &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    this->set_error(this->packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  if (dxl_error != 0U) {
    this->set_error(this->packet_handler_->getRxPacketError(dxl_error));
    return false;
  }

  this->last_error_.clear();
  return true;
}

void OpenCRSdkWrapper::set_error(std::string message)
{
  this->last_error_ = std::move(message);
}

}  // namespace amr::tb3::base_driver
