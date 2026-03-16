#include "amr_map_server/map_server.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "yaml-cpp/yaml.h"

namespace amr_map_server
{

namespace
{

std::string resolve_package_uri(const std::string & uri)
{
  constexpr auto prefix = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    return uri;
  }

  const auto relative = uri.substr(std::char_traits<char>::length(prefix));
  const auto separator = relative.find('/');
  if (separator == std::string::npos) {
    throw std::runtime_error("Invalid package URI: " + uri);
  }

  const auto package_name = relative.substr(0, separator);
  const auto package_relative_path = relative.substr(separator + 1);
  const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
  return (std::filesystem::path(package_share) / package_relative_path).lexically_normal().string();
}

}  // namespace

MapServer::MapServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("map_server", options),
  yaml_path_(""),
  frame_id_("map"),
  map_topic_("/amr/map_server/map"),
  get_map_service_name_("/amr/map_server/get_map"),
  map_loaded_(false)
{
  this->declare_parameter("files.yaml", this->yaml_path_);
  this->declare_parameter("frames.map", this->frame_id_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("services.get", this->get_map_service_name_);
}

MapServer::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("files.yaml", this->yaml_path_);
  this->get_parameter("frames.map", this->frame_id_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("services.get", this->get_map_service_name_);

  if (!this->load_map_from_files()) {
    return CallbackReturn::FAILURE;
  }

  this->map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->get_map_service_ = this->create_service<nav_msgs::srv::GetMap>(
    this->get_map_service_name_,
    [this](
      const nav_msgs::srv::GetMap::Request::SharedPtr request,
      nav_msgs::srv::GetMap::Response::SharedPtr response) {
      this->handle_get_map(request, response);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured map server: topic='%s', service='%s', yaml='%s'",
    this->map_topic_.c_str(),
    this->get_map_service_name_.c_str(),
    this->yaml_path_.c_str());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->map_publisher_->on_activate();
  this->publish_map();
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
  {
    std::scoped_lock lock(this->map_mutex_);
    this->map_ = nav_msgs::msg::OccupancyGrid();
    this->map_loaded_ = false;
  }
  this->get_map_service_.reset();
  this->map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  {
    std::scoped_lock lock(this->map_mutex_);
    this->map_ = nav_msgs::msg::OccupancyGrid();
    this->map_loaded_ = false;
  }
  this->get_map_service_.reset();
  this->map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

bool MapServer::load_map_from_files()
{
  if (this->yaml_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'files.yaml' is empty");
    return false;
  }

  YAML::Node map_yaml;
  const auto resolved_yaml_path = this->resolve_path(this->yaml_path_);
  try {
    map_yaml = YAML::LoadFile(resolved_yaml_path);
  } catch (const YAML::Exception & exception) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse map yaml '%s': %s", resolved_yaml_path.c_str(), exception.what());
    return false;
  }

  if (!map_yaml["image"] || !map_yaml["resolution"] || !map_yaml["origin"]) {
    RCLCPP_ERROR(this->get_logger(), "Map yaml must contain image, resolution, and origin");
    return false;
  }

  const auto origin = map_yaml["origin"];
  if (!origin.IsSequence() || origin.size() < 3U) {
    RCLCPP_ERROR(this->get_logger(), "Map origin must be [x, y, yaw]");
    return false;
  }

  const auto resolution = map_yaml["resolution"].as<double>();
  if (resolution <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Map resolution must be positive");
    return false;
  }

  const auto image_path = this->resolve_path(map_yaml["image"].as<std::string>());
  cv::Mat image = cv::imread(image_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map image '%s'", image_path.c_str());
    return false;
  }

  cv::Mat grayscale_image;
  if (image.channels() == 1) {
    grayscale_image = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGRA2GRAY);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported image channel count: %d", image.channels());
    return false;
  }

  if (grayscale_image.depth() != CV_8U) {
    cv::Mat converted_image;
    grayscale_image.convertTo(converted_image, CV_8U);
    grayscale_image = converted_image;
  }

  const auto occupied_thresh = map_yaml["occupied_thresh"] ? map_yaml["occupied_thresh"].as<double>() : 0.65;
  const auto free_thresh = map_yaml["free_thresh"] ? map_yaml["free_thresh"].as<double>() : 0.196;
  const auto negate = map_yaml["negate"] ? map_yaml["negate"].as<int>() != 0 : false;

  nav_msgs::msg::OccupancyGrid loaded_map;
  loaded_map.header.stamp = this->now();
  loaded_map.header.frame_id = this->frame_id_;
  loaded_map.info.map_load_time = loaded_map.header.stamp;
  loaded_map.info.resolution = static_cast<float>(resolution);
  loaded_map.info.width = static_cast<uint32_t>(grayscale_image.cols);
  loaded_map.info.height = static_cast<uint32_t>(grayscale_image.rows);
  loaded_map.info.origin.position.x = origin[0].as<double>();
  loaded_map.info.origin.position.y = origin[1].as<double>();
  loaded_map.info.origin.position.z = 0.0;
  this->set_quaternion_from_yaw(loaded_map.info.origin.orientation, origin[2].as<double>());
  loaded_map.data.resize(static_cast<std::size_t>(grayscale_image.cols * grayscale_image.rows));

  for (int row = 0; row < grayscale_image.rows; ++row) {
    const auto image_row = grayscale_image.rows - 1 - row;
    for (int col = 0; col < grayscale_image.cols; ++col) {
      const auto pixel = grayscale_image.at<std::uint8_t>(image_row, col);
      const auto occupied_probability = negate ?
        static_cast<double>(pixel) / 255.0 :
        static_cast<double>(255 - pixel) / 255.0;

      std::int8_t cell_value = -1;
      if (occupied_probability > occupied_thresh) {
        cell_value = 100;
      } else if (occupied_probability < free_thresh) {
        cell_value = 0;
      }

      loaded_map.data[static_cast<std::size_t>(row * grayscale_image.cols + col)] = cell_value;
    }
  }

  {
    std::scoped_lock lock(this->map_mutex_);
    this->map_ = std::move(loaded_map);
    this->map_loaded_ = true;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded map %ux%u at %.3f m/cell from '%s'",
    this->map_.info.width,
    this->map_.info.height,
    static_cast<double>(this->map_.info.resolution),
    resolved_yaml_path.c_str());
  return true;
}

void MapServer::publish_map()
{
  if (!this->map_publisher_ || !this->map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->map_loaded_) {
      return;
    }

    map_to_publish = this->map_;
    map_to_publish.header.stamp = this->now();
    map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  }

  this->map_publisher_->publish(map_to_publish);
}

std::string MapServer::resolve_path(const std::string & configured_path) const
{
  const auto resolved_uri_path = resolve_package_uri(configured_path);
  std::filesystem::path path(resolved_uri_path);
  if (path.is_absolute()) {
    return path.lexically_normal().string();
  }

  const std::filesystem::path yaml_directory =
    std::filesystem::path(resolve_package_uri(this->yaml_path_)).parent_path();
  return (yaml_directory / path).lexically_normal().string();
}

void MapServer::set_quaternion_from_yaw(
  geometry_msgs::msg::Quaternion & orientation,
  const double yaw) const
{
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
}

void MapServer::handle_get_map(
  const nav_msgs::srv::GetMap::Request::SharedPtr request,
  nav_msgs::srv::GetMap::Response::SharedPtr response)
{
  (void)request;
  std::scoped_lock lock(this->map_mutex_);
  if (!this->map_loaded_) {
    RCLCPP_WARN(this->get_logger(), "GetMap requested before a map was loaded");
    return;
  }

  response->map = this->map_;
  response->map.header.stamp = this->now();
}

}  // namespace amr_map_server
