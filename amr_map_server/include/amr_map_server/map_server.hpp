#ifndef AMR_MAP_SERVER__MAP_SERVER_HPP_
#define AMR_MAP_SERVER__MAP_SERVER_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
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

  bool load_map_from_files();
  void publish_map();
  std::string resolve_path(const std::string & configured_path) const;
  void set_quaternion_from_yaw(geometry_msgs::msg::Quaternion & orientation, double yaw) const;
  void handle_get_map(
    const nav_msgs::srv::GetMap::Request::SharedPtr request,
    nav_msgs::srv::GetMap::Response::SharedPtr response);

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  nav_msgs::msg::OccupancyGrid map_;
  std::mutex map_mutex_;
  std::string yaml_path_;
  std::string frame_id_;
  std::string map_topic_;
  std::string get_map_service_name_;
  bool map_loaded_;
};

}  // namespace amr_map_server

#endif  // AMR_MAP_SERVER__MAP_SERVER_HPP_
