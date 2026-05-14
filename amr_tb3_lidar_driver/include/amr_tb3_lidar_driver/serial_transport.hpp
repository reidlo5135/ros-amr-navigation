#ifndef AMR_TB3_LIDAR_DRIVER__SERIAL_TRANSPORT_HPP_
#define AMR_TB3_LIDAR_DRIVER__SERIAL_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace amr::tb3::lidar_driver
{

class SerialTransport
{
public:
  SerialTransport(std::string port, int baudrate);
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  bool open();
  void close();
  bool reconnect();
  bool is_open() const;

  void set_port(std::string port);
  void set_baudrate(int baudrate);
  const std::string & last_error() const;

  std::ptrdiff_t read(
    std::uint8_t * buffer,
    std::size_t buffer_size,
    std::chrono::milliseconds timeout);

private:
  bool configure_port(int fd);
  bool wait_until_ready(std::chrono::milliseconds timeout);
  void set_error(std::string error_message);

  std::string port_;
  int baudrate_;
  int file_descriptor_;
  std::string last_error_;
};

}  // namespace amr::tb3::lidar_driver

#endif  // AMR_TB3_LIDAR_DRIVER__SERIAL_TRANSPORT_HPP_
