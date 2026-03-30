#include "amr_map_server/map_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

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
  map_topic_("/amr/map/data"),
  temporary_map_topic_("/amr/map/temp"),
  corrected_odometry_topic_("/amr/slam_mapper/odometry"),
  get_map_service_name_("/amr/map_server/get_map"),
  freeze_temporary_map_service_name_("/amr/map_server/freeze_temporary_map"),
  evaluate_temporary_map_service_name_("/amr/map_server/evaluate_temporary_map"),
  save_temporary_map_service_name_("/amr/map_server/save_temporary_map"),
  mapping_mode_(false),
  save_directory_(""),
  save_basename_("amr_temporary_map"),
  quality_min_known_ratio_(0.35),
  quality_min_free_ratio_(0.10),
  quality_min_occupied_ratio_(0.01),
  quality_inflation_radius_cells_(4),
  quality_min_inflated_free_ratio_(0.05),
  quality_require_corrected_pose_(true),
  auto_save_enabled_(false),
  auto_save_required_consecutive_passes_(3),
  auto_save_consecutive_pass_count_(0),
  auto_save_completed_(false),
  official_map_loaded_(false),
  temporary_map_loaded_(false),
  has_latest_corrected_pose_(false)
{
  this->declare_parameter("files.yaml", this->yaml_path_);
  this->declare_parameter("frames.map", this->frame_id_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.temporary_map", this->temporary_map_topic_);
  this->declare_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->declare_parameter("services.get", this->get_map_service_name_);
  this->declare_parameter("services.freeze", this->freeze_temporary_map_service_name_);
  this->declare_parameter("services.evaluate", this->evaluate_temporary_map_service_name_);
  this->declare_parameter("services.save", this->save_temporary_map_service_name_);
  this->declare_parameter("mode.mapping", this->mapping_mode_);
  this->declare_parameter("save.directory", this->save_directory_);
  this->declare_parameter("save.basename", this->save_basename_);
  this->declare_parameter("quality.min_known_ratio", this->quality_min_known_ratio_);
  this->declare_parameter("quality.min_free_ratio", this->quality_min_free_ratio_);
  this->declare_parameter("quality.min_occupied_ratio", this->quality_min_occupied_ratio_);
  this->declare_parameter("quality.inflation_radius_cells", this->quality_inflation_radius_cells_);
  this->declare_parameter(
    "quality.min_inflated_free_ratio", this->quality_min_inflated_free_ratio_);
  this->declare_parameter(
    "quality.require_corrected_pose", this->quality_require_corrected_pose_);
  this->declare_parameter("auto_save.enabled", this->auto_save_enabled_);
  this->declare_parameter(
    "auto_save.required_consecutive_passes", this->auto_save_required_consecutive_passes_);
}

MapServer::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("files.yaml", this->yaml_path_);
  this->get_parameter("frames.map", this->frame_id_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.temporary_map", this->temporary_map_topic_);
  this->get_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->get_parameter("services.get", this->get_map_service_name_);
  this->get_parameter("services.freeze", this->freeze_temporary_map_service_name_);
  this->get_parameter("services.evaluate", this->evaluate_temporary_map_service_name_);
  this->get_parameter("services.save", this->save_temporary_map_service_name_);
  this->get_parameter("mode.mapping", this->mapping_mode_);
  this->get_parameter("save.directory", this->save_directory_);
  this->get_parameter("save.basename", this->save_basename_);
  this->get_parameter("quality.min_known_ratio", this->quality_min_known_ratio_);
  this->get_parameter("quality.min_free_ratio", this->quality_min_free_ratio_);
  this->get_parameter("quality.min_occupied_ratio", this->quality_min_occupied_ratio_);
  this->get_parameter("quality.inflation_radius_cells", this->quality_inflation_radius_cells_);
  this->get_parameter(
    "quality.min_inflated_free_ratio", this->quality_min_inflated_free_ratio_);
  this->get_parameter(
    "quality.require_corrected_pose", this->quality_require_corrected_pose_);
  this->get_parameter("auto_save.enabled", this->auto_save_enabled_);
  this->get_parameter(
    "auto_save.required_consecutive_passes", this->auto_save_required_consecutive_passes_);

  if (this->map_topic_.empty() || this->temporary_map_topic_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Map topics must not be empty: map='%s' temp='%s'",
      this->map_topic_.c_str(),
      this->temporary_map_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  {
    std::scoped_lock lock(this->map_mutex_);
    this->official_map_ = nav_msgs::msg::OccupancyGrid();
    this->temporary_map_ = nav_msgs::msg::OccupancyGrid();
    this->official_map_loaded_ = false;
    this->temporary_map_loaded_ = false;
    this->has_latest_corrected_pose_ = false;
    this->auto_save_consecutive_pass_count_ = 0;
    this->auto_save_completed_ = false;
  }

  if (!this->mapping_mode_) {
    if (!this->load_static_map_from_files()) {
      return CallbackReturn::FAILURE;
    }
  }

  this->official_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  this->temporary_map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    this->temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      this->handle_temporary_map(message);
    });

  this->corrected_odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->corrected_odometry_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      this->handle_corrected_odometry(message);
    });

  this->get_map_service_ = this->create_service<nav_msgs::srv::GetMap>(
    this->get_map_service_name_,
    [this](
      const nav_msgs::srv::GetMap::Request::SharedPtr request,
      nav_msgs::srv::GetMap::Response::SharedPtr response) {
      this->handle_get_map(request, response);
    });
  this->freeze_temporary_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    this->freeze_temporary_map_service_name_,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_freeze_temporary_map(request, response);
    });
  this->evaluate_temporary_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    this->evaluate_temporary_map_service_name_,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_evaluate_temporary_map(request, response);
    });
  this->save_temporary_map_service_ = this->create_service<std_srvs::srv::Trigger>(
    this->save_temporary_map_service_name_,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_save_temporary_map(request, response);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured map server: mode='%s', map='%s', temp='%s', corrected_odom='%s', yaml='%s'",
    this->mapping_mode_ ? "mapping" : "static",
    this->map_topic_.c_str(),
    this->temporary_map_topic_.c_str(),
    this->corrected_odometry_topic_.c_str(),
    this->yaml_path_.c_str());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->official_map_publisher_) {
    this->official_map_publisher_->on_activate();
  }
  this->publish_official_map();
  RCLCPP_INFO(
    this->get_logger(),
    "Activated map server in %s mode",
    this->mapping_mode_ ? "mapping" : "static");
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->official_map_publisher_) {
    this->official_map_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  {
    std::scoped_lock lock(this->map_mutex_);
    this->official_map_ = nav_msgs::msg::OccupancyGrid();
    this->temporary_map_ = nav_msgs::msg::OccupancyGrid();
    this->official_map_loaded_ = false;
    this->temporary_map_loaded_ = false;
    this->has_latest_corrected_pose_ = false;
    this->auto_save_consecutive_pass_count_ = 0;
    this->auto_save_completed_ = false;
  }
  this->save_temporary_map_service_.reset();
  this->evaluate_temporary_map_service_.reset();
  this->freeze_temporary_map_service_.reset();
  this->get_map_service_.reset();
  this->corrected_odometry_subscription_.reset();
  this->temporary_map_subscription_.reset();
  this->official_map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return this->on_cleanup(state);
}

bool MapServer::load_static_map_from_files()
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
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to parse map yaml '%s': %s",
      resolved_yaml_path.c_str(),
      exception.what());
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

  const auto occupied_thresh =
    map_yaml["occupied_thresh"] ? map_yaml["occupied_thresh"].as<double>() : 0.65;
  const auto free_thresh =
    map_yaml["free_thresh"] ? map_yaml["free_thresh"].as<double>() : 0.196;
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
    this->official_map_ = std::move(loaded_map);
    this->official_map_loaded_ = true;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded official map %ux%u at %.3f m/cell from '%s'",
    this->official_map_.info.width,
    this->official_map_.info.height,
    static_cast<double>(this->official_map_.info.resolution),
    resolved_yaml_path.c_str());
  return true;
}

void MapServer::publish_official_map()
{
  if (!this->official_map_publisher_ || !this->official_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->official_map_loaded_) {
      return;
    }
    map_to_publish = this->official_map_;
    map_to_publish.header.stamp = this->now();
    map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  }

  this->official_map_publisher_->publish(map_to_publish);
}

void MapServer::handle_temporary_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  {
    std::scoped_lock lock(this->map_mutex_);
    this->temporary_map_ = *message;
    this->temporary_map_loaded_ = !this->temporary_map_.data.empty();
    this->temporary_map_.header.frame_id = this->frame_id_;
    this->temporary_map_.header.stamp = this->now();
    this->temporary_map_.info.map_load_time = this->temporary_map_.header.stamp;
  }

  if (this->mapping_mode_) {
    this->maybe_auto_save_temporary_map();
  }
}

void MapServer::handle_corrected_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  (void)message;
  this->has_latest_corrected_pose_ = true;
}

MapServer::MapQualityMetrics MapServer::evaluate_temporary_map_quality() const
{
  MapQualityMetrics metrics{};
  nav_msgs::msg::OccupancyGrid map_snapshot;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->temporary_map_loaded_) {
      metrics.summary = "temporary map is not available";
      return metrics;
    }
    map_snapshot = this->temporary_map_;
    metrics.corrected_pose_ready = this->has_latest_corrected_pose_;
  }

  const std::size_t total_cells = map_snapshot.data.size();
  if (total_cells == 0U) {
    metrics.summary = "temporary map is empty";
    return metrics;
  }

  std::size_t known_cells = 0U;
  std::size_t free_cells = 0U;
  std::size_t occupied_cells = 0U;
  for (const auto cell : map_snapshot.data) {
    if (cell == -1) {
      continue;
    }
    ++known_cells;
    if (cell == 0) {
      ++free_cells;
    } else if (cell >= 50) {
      ++occupied_cells;
    }
  }

  metrics.known_ratio = static_cast<double>(known_cells) / static_cast<double>(total_cells);
  metrics.free_ratio = static_cast<double>(free_cells) / static_cast<double>(total_cells);
  metrics.occupied_ratio = static_cast<double>(occupied_cells) / static_cast<double>(total_cells);

  std::vector<bool> inflated_occupied(total_cells, false);
  const int inflation_radius = std::max(0, this->quality_inflation_radius_cells_);
  const int map_width = static_cast<int>(map_snapshot.info.width);
  const int map_height = static_cast<int>(map_snapshot.info.height);
  for (int grid_y = 0; grid_y < map_height; ++grid_y) {
    for (int grid_x = 0; grid_x < map_width; ++grid_x) {
      std::size_t index = 0U;
      if (!this->grid_index(map_snapshot, grid_x, grid_y, index)) {
        continue;
      }
      if (map_snapshot.data[index] < 50) {
        continue;
      }

      for (int offset_y = -inflation_radius; offset_y <= inflation_radius; ++offset_y) {
        for (int offset_x = -inflation_radius; offset_x <= inflation_radius; ++offset_x) {
          std::size_t inflated_index = 0U;
          if (!this->grid_index(map_snapshot, grid_x + offset_x, grid_y + offset_y, inflated_index)) {
            continue;
          }
          inflated_occupied[inflated_index] = true;
        }
      }
    }
  }

  std::size_t inflated_free_cells = 0U;
  for (std::size_t index = 0; index < total_cells; ++index) {
    if (map_snapshot.data[index] == 0 && !inflated_occupied[index]) {
      ++inflated_free_cells;
    }
  }
  metrics.inflated_free_ratio =
    static_cast<double>(inflated_free_cells) / static_cast<double>(total_cells);

  metrics.passed =
    metrics.known_ratio >= this->quality_min_known_ratio_ &&
    metrics.free_ratio >= this->quality_min_free_ratio_ &&
    metrics.occupied_ratio >= this->quality_min_occupied_ratio_ &&
    metrics.inflated_free_ratio >= this->quality_min_inflated_free_ratio_ &&
    (!this->quality_require_corrected_pose_ || metrics.corrected_pose_ready);

  std::ostringstream summary_stream;
  summary_stream
    << "known=" << metrics.known_ratio
    << ", free=" << metrics.free_ratio
    << ", occupied=" << metrics.occupied_ratio
    << ", inflated_free=" << metrics.inflated_free_ratio
    << ", corrected_pose=" << (metrics.corrected_pose_ready ? "true" : "false");
  metrics.summary = summary_stream.str();
  return metrics;
}

bool MapServer::save_map_to_files(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::string & image_path,
  const std::string & yaml_path) const
{
  if (map.info.width == 0U || map.info.height == 0U || map.data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot save an empty map");
    return false;
  }

  const std::filesystem::path image_file(image_path);
  const std::filesystem::path yaml_file(yaml_path);
  std::filesystem::create_directories(image_file.parent_path());
  std::filesystem::create_directories(yaml_file.parent_path());

  cv::Mat image(
    static_cast<int>(map.info.height),
    static_cast<int>(map.info.width),
    CV_8UC1);
  for (int row = 0; row < static_cast<int>(map.info.height); ++row) {
    const int image_row = static_cast<int>(map.info.height) - 1 - row;
    for (int col = 0; col < static_cast<int>(map.info.width); ++col) {
      const auto cell = map.data[static_cast<std::size_t>(row * static_cast<int>(map.info.width) + col)];
      std::uint8_t pixel = 205U;
      if (cell == 0) {
        pixel = 254U;
      } else if (cell >= 50) {
        pixel = 0U;
      }
      image.at<std::uint8_t>(image_row, col) = pixel;
    }
  }

  if (!cv::imwrite(image_path, image)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write map image '%s'", image_path.c_str());
    return false;
  }

  std::ofstream yaml_stream(yaml_path, std::ios::out | std::ios::trunc);
  if (!yaml_stream.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open map yaml '%s' for writing", yaml_path.c_str());
    return false;
  }

  const auto image_filename = image_file.filename().string();
  yaml_stream << "image: " << image_filename << "\n";
  yaml_stream << "resolution: " << static_cast<double>(map.info.resolution) << "\n";
  yaml_stream << "origin: ["
              << map.info.origin.position.x << ", "
              << map.info.origin.position.y << ", "
              << this->quaternion_to_yaw(map.info.origin.orientation) << "]\n";
  yaml_stream << "negate: 0\n";
  yaml_stream << "occupied_thresh: 0.65\n";
  yaml_stream << "free_thresh: 0.196\n";
  yaml_stream.close();
  return true;
}

bool MapServer::save_temporary_map_to_official_and_files(std::string & message)
{
  const MapQualityMetrics metrics = this->evaluate_temporary_map_quality();
  if (!metrics.passed) {
    message = "map quality check failed: " + metrics.summary;
    return false;
  }

  nav_msgs::msg::OccupancyGrid map_to_save;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->temporary_map_loaded_) {
      message = "temporary map is not available";
      return false;
    }
    map_to_save = this->temporary_map_;
  }

  std::filesystem::path save_directory = this->save_directory_;
  if (save_directory.empty()) {
    if (!this->yaml_path_.empty()) {
      save_directory = std::filesystem::path(this->resolve_path(this->yaml_path_)).parent_path();
    } else {
      save_directory = std::filesystem::current_path();
    }
  }
  const std::string basename =
    this->save_basename_.empty() ? "amr_temporary_map" : this->save_basename_;
  const auto image_path = (save_directory / (basename + ".pgm")).lexically_normal().string();
  const auto yaml_path = (save_directory / (basename + ".yaml")).lexically_normal().string();

  if (!this->save_map_to_files(map_to_save, image_path, yaml_path)) {
    message = "failed to save map files";
    return false;
  }

  {
    std::scoped_lock lock(this->map_mutex_);
    this->official_map_ = map_to_save;
    this->official_map_loaded_ = true;
  }
  this->publish_official_map();

  std::ostringstream message_stream;
  message_stream
    << "map saved after quality pass: " << metrics.summary
    << ", image=" << image_path
    << ", yaml=" << yaml_path;
  message = message_stream.str();
  return true;
}

void MapServer::maybe_auto_save_temporary_map()
{
  if (!this->auto_save_enabled_ || this->auto_save_completed_) {
    return;
  }

  const MapQualityMetrics metrics = this->evaluate_temporary_map_quality();
  if (!metrics.passed) {
    this->auto_save_consecutive_pass_count_ = 0;
    return;
  }

  ++this->auto_save_consecutive_pass_count_;
  if (this->auto_save_consecutive_pass_count_ < std::max(1, this->auto_save_required_consecutive_passes_)) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Temporary map quality pass %d/%d: %s",
      this->auto_save_consecutive_pass_count_,
      std::max(1, this->auto_save_required_consecutive_passes_),
      metrics.summary.c_str());
    return;
  }

  std::string message;
  if (this->save_temporary_map_to_official_and_files(message)) {
    this->auto_save_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "Auto-saved temporary map: %s", message.c_str());
  } else {
    this->auto_save_consecutive_pass_count_ = 0;
    RCLCPP_WARN(this->get_logger(), "Auto-save failed: %s", message.c_str());
  }
}

void MapServer::handle_get_map(
  const nav_msgs::srv::GetMap::Request::SharedPtr request,
  nav_msgs::srv::GetMap::Response::SharedPtr response)
{
  (void)request;
  std::scoped_lock lock(this->map_mutex_);
  if (!this->official_map_loaded_ && !this->temporary_map_loaded_) {
    RCLCPP_WARN(this->get_logger(), "GetMap requested before a map was available");
    return;
  }

  response->map = this->official_map_loaded_ ? this->official_map_ : this->temporary_map_;
  response->map.header.stamp = this->now();
}

void MapServer::handle_freeze_temporary_map(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->temporary_map_loaded_) {
      response->success = false;
      response->message = "temporary map is not available";
      return;
    }

    this->official_map_ = this->temporary_map_;
    this->official_map_loaded_ = true;
  }

  this->publish_official_map();
  response->success = true;
  response->message = "temporary map frozen into /amr/map/data";
  RCLCPP_INFO(this->get_logger(), "Froze temporary map into official map topic");
}

void MapServer::handle_evaluate_temporary_map(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  const MapQualityMetrics metrics = this->evaluate_temporary_map_quality();
  response->success = metrics.passed;
  response->message = metrics.summary;
  if (metrics.passed) {
    RCLCPP_INFO(this->get_logger(), "Temporary map quality passed: %s", metrics.summary.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Temporary map quality failed: %s", metrics.summary.c_str());
  }
}

void MapServer::handle_save_temporary_map(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = this->save_temporary_map_to_official_and_files(response->message);
  if (response->success) {
    this->auto_save_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
}

bool MapServer::grid_index(
  const nav_msgs::msg::OccupancyGrid & map,
  int grid_x,
  int grid_y,
  std::size_t & index) const
{
  if (
    grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(map.info.width) ||
    grid_y >= static_cast<int>(map.info.height))
  {
    return false;
  }

  index = static_cast<std::size_t>((grid_y * static_cast<int>(map.info.width)) + grid_x);
  return true;
}

std::string MapServer::resolve_path(const std::string & configured_path) const
{
  const auto resolved_uri_path = resolve_package_uri(configured_path);
  return std::filesystem::path(resolved_uri_path).lexically_normal().string();
}

void MapServer::set_quaternion_from_yaw(
  geometry_msgs::msg::Quaternion & orientation,
  double yaw) const
{
  const double half_yaw = yaw * 0.5;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(half_yaw);
  orientation.w = std::cos(half_yaw);
}

double MapServer::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp = 2.0 * (
    (orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp = 1.0 - 2.0 * (
    (orientation.y * orientation.y) + (orientation.z * orientation.z));
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace amr_map_server
