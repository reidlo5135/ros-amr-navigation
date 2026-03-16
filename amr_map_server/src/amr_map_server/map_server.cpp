#include "amr_map_server/map_server.hpp"

#include <vector>

namespace amr_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("map_server", options),
  frame_id_("map"),
  publish_topic_("map"),
  width_(20),
  height_(20),
  resolution_(0.25)
{
  this->declare_parameter("frame_id", this->frame_id_);
  this->declare_parameter("publish_topic", this->publish_topic_);
  this->declare_parameter("width", this->width_);
  this->declare_parameter("height", this->height_);
  this->declare_parameter("resolution", this->resolution_);
}

MapServer::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("frame_id", this->frame_id_);
  this->get_parameter("publish_topic", this->publish_topic_);
  this->get_parameter("width", this->width_);
  this->get_parameter("height", this->height_);
  this->get_parameter("resolution", this->resolution_);

  this->map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->publish_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(this->get_logger(), "Configured map server on topic '%s'", this->publish_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->map_publisher_->on_activate();
  this->map_publisher_->publish(this->build_placeholder_map());
  RCLCPP_INFO(this->get_logger(), "Activated map server");
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->map_publisher_) {
    this->map_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

nav_msgs::msg::OccupancyGrid MapServer::build_placeholder_map() const
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = this->now();
  map.header.frame_id = this->frame_id_;
  map.info.map_load_time = this->now();
  map.info.resolution = static_cast<float>(this->resolution_);
  map.info.width = static_cast<uint32_t>(this->width_);
  map.info.height = static_cast<uint32_t>(this->height_);
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.orientation.w = 1.0;

  const auto cell_count = static_cast<std::size_t>(this->width_ * this->height_);
  map.data.assign(cell_count, 0);
  return map;
}

}  // namespace amr_map_server
