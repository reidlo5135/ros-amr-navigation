#ifndef AMR_TB3_LIDAR_DRIVER__LASER_SCAN_ASSEMBLER_HPP_
#define AMR_TB3_LIDAR_DRIVER__LASER_SCAN_ASSEMBLER_HPP_

#include <string>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "amr_tb3_lidar_driver/lidar_parser.hpp"

namespace amr::tb3::lidar_driver
{

struct LaserScanConfig
{
  std::string frame_id{"base_scan"};
  bool inverted{false};
  float angle_min_rad{0.0F};
  float angle_max_rad{6.28318530718F};
  float range_min_m{0.12F};
  float range_max_m{3.5F};
  float scan_time_sec{0.1F};
};

class LaserScanAssembler
{
public:
  sensor_msgs::msg::LaserScan build_message(
    const LidarScanFrame & frame,
    const LaserScanConfig & config) const;
};

}  // namespace amr::tb3::lidar_driver

#endif  // AMR_TB3_LIDAR_DRIVER__LASER_SCAN_ASSEMBLER_HPP_
