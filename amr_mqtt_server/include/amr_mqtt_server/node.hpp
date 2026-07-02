#ifndef AMR_MQTT_SERVER__NODE_HPP_
#define AMR_MQTT_SERVER__NODE_HPP_

/**
 * @file node.hpp
 * @brief Factory for the AMR MQTT bridge node.
 */

#include <MQTTClient.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <amr_msgs/action/navigate_to_poses.hpp>
#include <amr_msgs/msg/motion_status.hpp>
#include <amr_msgs/srv/plan_route.hpp>
#include <amr_msgs/srv/plan_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>
#include <vector>

namespace amr::mqtt::server
{
    /// @brief Create the MQTT bridge ROS node with all configured endpoints.
    std::shared_ptr<rclcpp::Node> make_node();
}

#endif  // AMR_MQTT_SERVER__NODE_HPP_
