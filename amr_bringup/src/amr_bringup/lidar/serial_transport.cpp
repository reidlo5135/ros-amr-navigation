#include "amr_bringup/lidar/serial_transport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <utility>

namespace amr::tb3::lidar_driver
{

namespace
{

speed_t to_termios_baudrate(int baudrate)
{
  switch (baudrate) {
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    default:
      return 0;
  }
}

}  // namespace

SerialTransport::SerialTransport(std::string port, const int baudrate)
: port_(std::move(port)),
  baudrate_(baudrate),
  file_descriptor_(-1),
  last_error_("")
{
}

SerialTransport::~SerialTransport()
{
  this->close();
}

bool SerialTransport::open()
{
  if (this->is_open()) {
    return true;
  }

  const int fd = ::open(this->port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    this->set_error("Failed to open LiDAR serial port: " + std::string(std::strerror(errno)));
    return false;
  }

  if (!this->configure_port(fd)) {
    ::close(fd);
    return false;
  }

  this->file_descriptor_ = fd;
  this->last_error_.clear();
  return true;
}

void SerialTransport::close()
{
  if (this->file_descriptor_ >= 0) {
    ::close(this->file_descriptor_);
    this->file_descriptor_ = -1;
  }
}

bool SerialTransport::reconnect()
{
  this->close();
  return this->open();
}

bool SerialTransport::is_open() const
{
  return this->file_descriptor_ >= 0;
}

void SerialTransport::set_port(std::string port)
{
  this->port_ = std::move(port);
}

void SerialTransport::set_baudrate(const int baudrate)
{
  this->baudrate_ = baudrate;
}

const std::string & SerialTransport::last_error() const
{
  return this->last_error_;
}

std::ptrdiff_t SerialTransport::read(
  std::uint8_t * buffer,
  const std::size_t buffer_size,
  const std::chrono::milliseconds timeout)
{
  if (!this->is_open()) {
    this->set_error("LiDAR serial port is not open");
    return -1;
  }
  if (!this->wait_until_ready(timeout)) {
    return 0;
  }

  const ssize_t read_size = ::read(this->file_descriptor_, buffer, buffer_size);
  if (read_size < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    this->set_error("Failed to read LiDAR serial port: " + std::string(std::strerror(errno)));
    return -1;
  }
  return static_cast<std::ptrdiff_t>(read_size);
}

bool SerialTransport::configure_port(const int fd)
{
  struct termios options {};
  if (tcgetattr(fd, &options) != 0) {
    this->set_error("Failed to read LiDAR serial attributes: " + std::string(std::strerror(errno)));
    return false;
  }

  const speed_t speed = to_termios_baudrate(this->baudrate_);
  if (speed == 0) {
    this->set_error("Unsupported LiDAR baudrate: " + std::to_string(this->baudrate_));
    return false;
  }

  cfmakeraw(&options);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  options.c_cflag &= static_cast<tcflag_t>(~PARENB);
  options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
    this->set_error("Failed to set LiDAR baudrate: " + std::string(std::strerror(errno)));
    return false;
  }

  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    this->set_error("Failed to apply LiDAR serial attributes: " + std::string(std::strerror(errno)));
    return false;
  }

  return true;
}

bool SerialTransport::wait_until_ready(const std::chrono::milliseconds timeout)
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(this->file_descriptor_, &read_fds);

  struct timeval tv {};
  tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);

  const int ready = ::select(this->file_descriptor_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ready < 0) {
    this->set_error("LiDAR serial select failed: " + std::string(std::strerror(errno)));
    return false;
  }
  return ready > 0;
}

void SerialTransport::set_error(std::string error_message)
{
  this->last_error_ = std::move(error_message);
}

}  // namespace amr::tb3::lidar_driver
