#ifndef AMR_TB3_LIDAR_DRIVER__LIDAR_DRIVER_NODE_HPP_
#define AMR_TB3_LIDAR_DRIVER__LIDAR_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "amr_bringup/lidar/laser_scan_assembler.hpp"
#include "amr_bringup/lidar/lidar_parser.hpp"
#include "amr_bringup/lidar/serial_transport.hpp"

namespace amr::tb3::lidar_driver
{

class LidarDriverNode : public rclcpp::Node
{
public:
  explicit LidarDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();
  void handle_connection_check();
  void poll_lidar();

  SerialTransport transport_;
  std::unique_ptr<LidarParser> parser_;
  LaserScanAssembler assembler_;
  LaserScanConfig scan_config_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::string port_;
  int baudrate_;
  std::string frame_id_;
  std::string scan_topic_;
  std::string sensor_model_;
  bool inverted_;
  double angle_min_rad_;
  double angle_max_rad_;
  double range_min_m_;
  double range_max_m_;
  double scan_time_sec_;
  bool fake_scan_mode_{false};
  std::vector<std::uint8_t> rx_buffer_;
};

}  // namespace amr::tb3::lidar_driver

#endif  // AMR_TB3_LIDAR_DRIVER__LIDAR_DRIVER_NODE_HPP_
