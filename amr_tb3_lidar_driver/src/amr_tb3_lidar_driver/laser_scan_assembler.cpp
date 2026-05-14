#include "amr_tb3_lidar_driver/laser_scan_assembler.hpp"

#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace amr::tb3::lidar_driver
{

sensor_msgs::msg::LaserScan LaserScanAssembler::build_message(
  const LidarScanFrame & frame,
  const LaserScanConfig & config) const
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = rclcpp::Time(frame.stamp.count(), RCL_SYSTEM_TIME);
  scan.header.frame_id = config.frame_id;
  scan.angle_min = config.angle_min_rad;
  scan.angle_max = config.angle_max_rad;
  scan.range_min = config.range_min_m;
  scan.range_max = config.range_max_m;
  scan.scan_time = config.scan_time_sec;

  const float span = std::max(0.001F, config.angle_max_rad - config.angle_min_rad);
  const std::size_t sample_count = std::max<std::size_t>(frame.samples.size(), 1U);
  scan.angle_increment = span / static_cast<float>(sample_count);
  scan.time_increment = config.scan_time_sec / static_cast<float>(sample_count);
  scan.ranges.assign(sample_count, std::numeric_limits<float>::infinity());
  scan.intensities.assign(sample_count, 0.0F);

  for (std::size_t index = 0; index < frame.samples.size(); ++index) {
    const LidarSample & sample = config.inverted ?
      frame.samples[frame.samples.size() - 1U - index] :
      frame.samples[index];
    if (!sample.valid) {
      continue;
    }
    if (sample.range_m < config.range_min_m || sample.range_m > config.range_max_m) {
      continue;
    }
    if (index >= scan.ranges.size()) {
      break;
    }
    scan.ranges[index] = sample.range_m;
    scan.intensities[index] = sample.intensity;
  }

  return scan;
}

}  // namespace amr::tb3::lidar_driver
