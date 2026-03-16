#ifndef AMR_MAP_SERVER__MAP_SERVER_HPP_
#define AMR_MAP_SERVER__MAP_SERVER_HPP_

#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_map_server
{

class MapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  nav_msgs::msg::OccupancyGrid build_placeholder_map() const;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  std::string frame_id_;
  std::string publish_topic_;
  int width_;
  int height_;
  double resolution_;
};

}  // namespace amr_map_server

#endif  // AMR_MAP_SERVER__MAP_SERVER_HPP_
