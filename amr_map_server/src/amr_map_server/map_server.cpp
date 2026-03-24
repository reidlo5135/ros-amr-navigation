#include "amr_map_server/map_server.hpp"

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
  odom_frame_("odom"),
  map_topic_(""),
  temporary_map_topic_("/amr/map/temporary"),
  get_map_service_name_("/amr/map_server/get_map"),
  freeze_temporary_map_service_name_("/amr/map_server/freeze_temporary_map"),
  evaluate_temporary_map_service_name_("/amr/map_server/evaluate_temporary_map"),
  save_temporary_map_service_name_("/amr/map_server/save_temporary_map"),
  mapping_mode_(false),
  mapping_pose_topic_("/odom"),
  mapping_imu_topic_("/imu"),
  mapping_scan_topic_("/scan"),
  mapping_publish_period_ms_(500),
  mapping_resolution_(0.05),
  mapping_width_(400),
  mapping_height_(400),
  mapping_origin_x_(-10.0),
  mapping_origin_y_(-10.0),
  mapping_origin_yaw_(0.0),
  mapping_min_range_(0.05),
  mapping_max_range_(8.0),
  mapping_publish_identity_tf_(false),
  save_directory_(""),
  save_basename_("temporary_map"),
  scan_matching_enabled_(true),
  scan_matching_linear_window_(0.15),
  scan_matching_linear_step_(0.05),
  scan_matching_angular_window_deg_(12.0),
  scan_matching_angular_step_deg_(3.0),
  scan_matching_max_beams_(32),
  scan_matching_min_valid_beams_(8),
  scan_matching_occupied_search_radius_cells_(1),
  scan_matching_minimum_occupied_cells_(50),
  scan_matching_occupied_match_score_(3.0),
  scan_matching_free_space_penalty_(1.0),
  mapping_hit_score_(20),
  mapping_free_score_(3),
  mapping_decay_score_(1),
  mapping_occupied_score_threshold_(20),
  mapping_free_score_threshold_(-5),
  mapping_score_min_(-20),
  mapping_score_max_(100),
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
  latest_corrected_pose_{},
  start_odom_yaw_(0.0),
  start_imu_yaw_(0.0),
  latest_imu_yaw_(0.0),
  has_latest_odometry_(false),
  has_latest_corrected_pose_(false),
  has_start_odom_yaw_(false),
  has_latest_imu_(false),
  has_start_imu_yaw_(false)
{
  this->declare_parameter("files.yaml", this->yaml_path_);
  this->declare_parameter("frames.map", this->frame_id_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("topics.map", this->map_topic_);
  this->declare_parameter("topics.temporary_map", this->temporary_map_topic_);
  this->declare_parameter("services.get", this->get_map_service_name_);
  this->declare_parameter("services.freeze", this->freeze_temporary_map_service_name_);
  this->declare_parameter("services.evaluate", this->evaluate_temporary_map_service_name_);
  this->declare_parameter("services.save", this->save_temporary_map_service_name_);
  this->declare_parameter("mode.mapping", this->mapping_mode_);
  this->declare_parameter("mapping.topics.pose", this->mapping_pose_topic_);
  this->declare_parameter("mapping.topics.imu", this->mapping_imu_topic_);
  this->declare_parameter("mapping.topics.scan", this->mapping_scan_topic_);
  this->declare_parameter("mapping.publish_period_ms", this->mapping_publish_period_ms_);
  this->declare_parameter("mapping.resolution", this->mapping_resolution_);
  this->declare_parameter("mapping.width", this->mapping_width_);
  this->declare_parameter("mapping.height", this->mapping_height_);
  this->declare_parameter("mapping.origin.x", this->mapping_origin_x_);
  this->declare_parameter("mapping.origin.y", this->mapping_origin_y_);
  this->declare_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  this->declare_parameter("mapping.range.min", this->mapping_min_range_);
  this->declare_parameter("mapping.range.max", this->mapping_max_range_);
  this->declare_parameter("mapping.publish_identity_tf", this->mapping_publish_identity_tf_);
  this->declare_parameter("mapping.save.directory", this->save_directory_);
  this->declare_parameter("mapping.save.basename", this->save_basename_);
  this->declare_parameter("mapping.scan_matching.enabled", this->scan_matching_enabled_);
  this->declare_parameter(
    "mapping.scan_matching.linear_window", this->scan_matching_linear_window_);
  this->declare_parameter(
    "mapping.scan_matching.linear_step", this->scan_matching_linear_step_);
  this->declare_parameter(
    "mapping.scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  this->declare_parameter(
    "mapping.scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  this->declare_parameter(
    "mapping.scan_matching.max_beams", this->scan_matching_max_beams_);
  this->declare_parameter(
    "mapping.scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  this->declare_parameter(
    "mapping.scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  this->declare_parameter(
    "mapping.scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  this->declare_parameter(
    "mapping.scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  this->declare_parameter(
    "mapping.scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  this->declare_parameter("mapping.occupancy.hit_score", this->mapping_hit_score_);
  this->declare_parameter("mapping.occupancy.free_score", this->mapping_free_score_);
  this->declare_parameter("mapping.occupancy.decay_score", this->mapping_decay_score_);
  this->declare_parameter(
    "mapping.occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  this->declare_parameter(
    "mapping.occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  this->declare_parameter("mapping.occupancy.score_min", this->mapping_score_min_);
  this->declare_parameter("mapping.occupancy.score_max", this->mapping_score_max_);
  this->declare_parameter("mapping.quality.min_known_ratio", this->quality_min_known_ratio_);
  this->declare_parameter("mapping.quality.min_free_ratio", this->quality_min_free_ratio_);
  this->declare_parameter("mapping.quality.min_occupied_ratio", this->quality_min_occupied_ratio_);
  this->declare_parameter(
    "mapping.quality.inflation_radius_cells", this->quality_inflation_radius_cells_);
  this->declare_parameter(
    "mapping.quality.min_inflated_free_ratio", this->quality_min_inflated_free_ratio_);
  this->declare_parameter(
    "mapping.quality.require_corrected_pose", this->quality_require_corrected_pose_);
  this->declare_parameter("mapping.auto_save.enabled", this->auto_save_enabled_);
  this->declare_parameter(
    "mapping.auto_save.required_consecutive_passes",
    this->auto_save_required_consecutive_passes_);
}

MapServer::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("files.yaml", this->yaml_path_);
  this->get_parameter("frames.map", this->frame_id_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("topics.map", this->map_topic_);
  this->get_parameter("topics.temporary_map", this->temporary_map_topic_);
  this->get_parameter("services.get", this->get_map_service_name_);
  this->get_parameter("services.freeze", this->freeze_temporary_map_service_name_);
  this->get_parameter("services.evaluate", this->evaluate_temporary_map_service_name_);
  this->get_parameter("services.save", this->save_temporary_map_service_name_);
  this->get_parameter("mode.mapping", this->mapping_mode_);
  this->get_parameter("mapping.topics.pose", this->mapping_pose_topic_);
  this->get_parameter("mapping.topics.imu", this->mapping_imu_topic_);
  this->get_parameter("mapping.topics.scan", this->mapping_scan_topic_);
  this->get_parameter("mapping.publish_period_ms", this->mapping_publish_period_ms_);
  this->get_parameter("mapping.resolution", this->mapping_resolution_);
  this->get_parameter("mapping.width", this->mapping_width_);
  this->get_parameter("mapping.height", this->mapping_height_);
  this->get_parameter("mapping.origin.x", this->mapping_origin_x_);
  this->get_parameter("mapping.origin.y", this->mapping_origin_y_);
  this->get_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  this->get_parameter("mapping.range.min", this->mapping_min_range_);
  this->get_parameter("mapping.range.max", this->mapping_max_range_);
  this->get_parameter("mapping.publish_identity_tf", this->mapping_publish_identity_tf_);
  this->get_parameter("mapping.save.directory", this->save_directory_);
  this->get_parameter("mapping.save.basename", this->save_basename_);
  this->get_parameter("mapping.scan_matching.enabled", this->scan_matching_enabled_);
  this->get_parameter(
    "mapping.scan_matching.linear_window", this->scan_matching_linear_window_);
  this->get_parameter(
    "mapping.scan_matching.linear_step", this->scan_matching_linear_step_);
  this->get_parameter(
    "mapping.scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  this->get_parameter(
    "mapping.scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  this->get_parameter(
    "mapping.scan_matching.max_beams", this->scan_matching_max_beams_);
  this->get_parameter(
    "mapping.scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  this->get_parameter(
    "mapping.scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  this->get_parameter(
    "mapping.scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  this->get_parameter(
    "mapping.scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  this->get_parameter(
    "mapping.scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  this->get_parameter("mapping.occupancy.hit_score", this->mapping_hit_score_);
  this->get_parameter("mapping.occupancy.free_score", this->mapping_free_score_);
  this->get_parameter("mapping.occupancy.decay_score", this->mapping_decay_score_);
  this->get_parameter(
    "mapping.occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  this->get_parameter(
    "mapping.occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  this->get_parameter("mapping.occupancy.score_min", this->mapping_score_min_);
  this->get_parameter("mapping.occupancy.score_max", this->mapping_score_max_);
  this->get_parameter("mapping.quality.min_known_ratio", this->quality_min_known_ratio_);
  this->get_parameter("mapping.quality.min_free_ratio", this->quality_min_free_ratio_);
  this->get_parameter("mapping.quality.min_occupied_ratio", this->quality_min_occupied_ratio_);
  this->get_parameter(
    "mapping.quality.inflation_radius_cells", this->quality_inflation_radius_cells_);
  this->get_parameter(
    "mapping.quality.min_inflated_free_ratio", this->quality_min_inflated_free_ratio_);
  this->get_parameter(
    "mapping.quality.require_corrected_pose", this->quality_require_corrected_pose_);
  this->get_parameter("mapping.auto_save.enabled", this->auto_save_enabled_);
  this->get_parameter(
    "mapping.auto_save.required_consecutive_passes",
    this->auto_save_required_consecutive_passes_);

  if (this->map_topic_.empty() || this->temporary_map_topic_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Map topics must not be empty: map='%s' temporary_map='%s'",
      this->map_topic_.c_str(),
      this->temporary_map_topic_.c_str());
    return CallbackReturn::FAILURE;
  }

  if (this->mapping_mode_) {
    if (
      this->mapping_pose_topic_.empty() || this->mapping_imu_topic_.empty() ||
      this->mapping_scan_topic_.empty())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Mapping mode requires non-empty pose, imu, and scan topics: pose='%s' imu='%s' scan='%s'",
        this->mapping_pose_topic_.c_str(),
        this->mapping_imu_topic_.c_str(),
        this->mapping_scan_topic_.c_str());
      return CallbackReturn::FAILURE;
    }
    if (
      this->mapping_resolution_ <= 0.0 || this->mapping_width_ <= 0 ||
      this->mapping_height_ <= 0)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Mapping mode requires positive resolution, width, and height: resolution=%.3f width=%d height=%d",
        this->mapping_resolution_,
        this->mapping_width_,
        this->mapping_height_);
      return CallbackReturn::FAILURE;
    }

    this->initialize_mapping_map();
  } else if (!this->load_static_map_from_files()) {
    return CallbackReturn::FAILURE;
  }

  this->official_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
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
  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (this->mapping_mode_) {
    this->odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->mapping_pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        this->handle_odometry(message);
      });
    this->imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      this->mapping_imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {
        this->handle_imu(message);
      });
    this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->mapping_scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        this->handle_scan(message);
      });
    this->mapping_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(50, this->mapping_publish_period_ms_)),
      [this]() { this->publish_maps(); });
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Configured map server: mode='%s', map='%s', temporary_map='%s', get='%s', freeze='%s', evaluate='%s', save='%s', yaml='%s'",
    this->mapping_mode_ ? "mapping" : "static",
    this->map_topic_.c_str(),
    this->temporary_map_topic_.c_str(),
    this->get_map_service_name_.c_str(),
    this->freeze_temporary_map_service_name_.c_str(),
    this->evaluate_temporary_map_service_name_.c_str(),
    this->save_temporary_map_service_name_.c_str(),
    this->yaml_path_.c_str());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->official_map_publisher_) {
    this->official_map_publisher_->on_activate();
  }
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_activate();
  }
  this->publish_maps();
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
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_deactivate();
  }
  if (this->mapping_publish_timer_) {
    this->mapping_publish_timer_->cancel();
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
    this->latest_corrected_pose_ = Pose2D{};
    this->occupancy_scores_.clear();
    this->has_latest_odometry_ = false;
    this->has_latest_corrected_pose_ = false;
    this->auto_save_consecutive_pass_count_ = 0;
    this->auto_save_completed_ = false;
    this->has_start_odom_yaw_ = false;
    this->has_latest_imu_ = false;
    this->has_start_imu_yaw_ = false;
  }
  this->mapping_publish_timer_.reset();
  this->odometry_subscription_.reset();
  this->imu_subscription_.reset();
  this->scan_subscription_.reset();
  this->transform_broadcaster_.reset();
  this->get_map_service_.reset();
  this->freeze_temporary_map_service_.reset();
  this->evaluate_temporary_map_service_.reset();
  this->save_temporary_map_service_.reset();
  this->official_map_publisher_.reset();
  this->temporary_map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  {
    std::scoped_lock lock(this->map_mutex_);
    this->official_map_ = nav_msgs::msg::OccupancyGrid();
    this->temporary_map_ = nav_msgs::msg::OccupancyGrid();
    this->official_map_loaded_ = false;
    this->temporary_map_loaded_ = false;
    this->latest_corrected_pose_ = Pose2D{};
    this->occupancy_scores_.clear();
    this->has_latest_odometry_ = false;
    this->has_latest_corrected_pose_ = false;
    this->auto_save_consecutive_pass_count_ = 0;
    this->auto_save_completed_ = false;
    this->has_start_odom_yaw_ = false;
    this->has_latest_imu_ = false;
    this->has_start_imu_yaw_ = false;
  }
  this->mapping_publish_timer_.reset();
  this->odometry_subscription_.reset();
  this->imu_subscription_.reset();
  this->scan_subscription_.reset();
  this->transform_broadcaster_.reset();
  this->get_map_service_.reset();
  this->freeze_temporary_map_service_.reset();
  this->evaluate_temporary_map_service_.reset();
  this->save_temporary_map_service_.reset();
  this->official_map_publisher_.reset();
  this->temporary_map_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

void MapServer::initialize_mapping_map()
{
  nav_msgs::msg::OccupancyGrid initialized_map;
  initialized_map.header.stamp = this->now();
  initialized_map.header.frame_id = this->frame_id_;
  initialized_map.info.map_load_time = initialized_map.header.stamp;
  initialized_map.info.resolution = static_cast<float>(this->mapping_resolution_);
  initialized_map.info.width = static_cast<uint32_t>(this->mapping_width_);
  initialized_map.info.height = static_cast<uint32_t>(this->mapping_height_);
  initialized_map.info.origin.position.x = this->mapping_origin_x_;
  initialized_map.info.origin.position.y = this->mapping_origin_y_;
  initialized_map.info.origin.position.z = 0.0;
  this->set_quaternion_from_yaw(initialized_map.info.origin.orientation, this->mapping_origin_yaw_);
  initialized_map.data.assign(
    static_cast<std::size_t>(this->mapping_width_ * this->mapping_height_),
    static_cast<int8_t>(-1));
  this->occupancy_scores_.assign(
    static_cast<std::size_t>(this->mapping_width_ * this->mapping_height_),
    0);

  {
    std::scoped_lock lock(this->map_mutex_);
    this->temporary_map_ = std::move(initialized_map);
    this->temporary_map_loaded_ = true;
    this->auto_save_consecutive_pass_count_ = 0;
    this->auto_save_completed_ = false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Initialized mapping map %d x %d at %.3f m/cell with origin (%.2f, %.2f, %.2f)",
    this->mapping_width_,
    this->mapping_height_,
    this->mapping_resolution_,
    this->mapping_origin_x_,
    this->mapping_origin_y_,
    this->mapping_origin_yaw_);
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
    this->official_map_ = std::move(loaded_map);
    this->official_map_loaded_ = true;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded map %ux%u at %.3f m/cell from '%s'",
    this->official_map_.info.width,
    this->official_map_.info.height,
    static_cast<double>(this->official_map_.info.resolution),
    resolved_yaml_path.c_str());
  return true;
}

void MapServer::publish_maps()
{
  if (this->mapping_mode_) {
    this->decay_occupied_scores();
    this->maybe_auto_save_temporary_map();
  }

  this->publish_official_map();
  this->publish_temporary_map();
  if (this->mapping_mode_ && this->mapping_publish_identity_tf_) {
    this->publish_map_to_odom_tf();
  }
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

void MapServer::publish_temporary_map()
{
  if (!this->temporary_map_publisher_ || !this->temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->temporary_map_loaded_) {
      return;
    }

    map_to_publish = this->temporary_map_;
    map_to_publish.header.stamp = this->now();
    map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  }

  this->temporary_map_publisher_->publish(map_to_publish);
}

void MapServer::handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  this->latest_odometry_ = *message;
  this->has_latest_odometry_ = true;
  if (!this->has_start_odom_yaw_) {
    this->start_odom_yaw_ = this->quaternion_to_yaw(message->pose.pose.orientation);
    this->has_start_odom_yaw_ = true;
  }
}

void MapServer::handle_imu(const sensor_msgs::msg::Imu::SharedPtr message)
{
  this->latest_imu_yaw_ = this->quaternion_to_yaw(message->orientation);
  this->has_latest_imu_ = true;
  if (!this->has_start_imu_yaw_) {
    this->start_imu_yaw_ = this->latest_imu_yaw_;
    this->has_start_imu_yaw_ = true;
  }
}

void MapServer::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  if (!this->mapping_mode_ || !this->has_latest_odometry_) {
    return;
  }

  this->update_map_from_scan(*message);
}

void MapServer::update_map_from_scan(const sensor_msgs::msg::LaserScan & scan)
{
  const Pose2D predicted_pose = this->build_predicted_pose();
  const Pose2D corrected_pose = this->refine_pose_with_scan_matching(scan, predicted_pose);
  {
    std::scoped_lock lock(this->map_mutex_);
    this->latest_corrected_pose_ = corrected_pose;
    this->has_latest_corrected_pose_ = true;
  }
  if (this->mapping_publish_identity_tf_) {
    this->publish_map_to_odom_tf();
  }
  const double robot_x = corrected_pose.x;
  const double robot_y = corrected_pose.y;
  const double robot_yaw = corrected_pose.yaw;

  int start_x = 0;
  int start_y = 0;
  if (!this->world_to_grid(robot_x, robot_y, start_x, start_y)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Robot pose is outside mapping grid: x=%.2f y=%.2f",
      robot_x,
      robot_y);
    return;
  }

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) || range < std::max(this->mapping_min_range_, static_cast<double>(scan.range_min))) {
      continue;
    }

    const double clipped_range = std::min(
      range,
      std::min(this->mapping_max_range_, static_cast<double>(scan.range_max)));
    const bool has_hit = range <= std::min(this->mapping_max_range_, static_cast<double>(scan.range_max));
    const double beam_angle = robot_yaw + static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const double end_x_world = robot_x + (clipped_range * std::cos(beam_angle));
    const double end_y_world = robot_y + (clipped_range * std::sin(beam_angle));

    int end_x = 0;
    int end_y = 0;
    if (!this->world_to_grid(end_x_world, end_y_world, end_x, end_y)) {
      continue;
    }

    this->raytrace_free_cells(start_x, start_y, end_x, end_y);
    if (has_hit) {
      this->mark_occupied_cell(end_x, end_y);
    }
  }
}

MapServer::Pose2D MapServer::build_predicted_pose() const
{
  const auto & pose = this->latest_odometry_.pose.pose;
  Pose2D predicted_pose{};
  predicted_pose.x = pose.position.x;
  predicted_pose.y = pose.position.y;
  predicted_pose.yaw = this->quaternion_to_yaw(pose.orientation);

  if (this->has_latest_imu_ && this->has_start_imu_yaw_ && this->has_start_odom_yaw_) {
    predicted_pose.yaw = this->normalize_angle(
      this->start_odom_yaw_ + (this->latest_imu_yaw_ - this->start_imu_yaw_));
  }

  return predicted_pose;
}

MapServer::Pose2D MapServer::refine_pose_with_scan_matching(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & predicted_pose) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

  if (!this->scan_matching_enabled_) {
    return predicted_pose;
  }

  nav_msgs::msg::OccupancyGrid map_snapshot;
  {
    std::scoped_lock lock(this->map_mutex_);
    if (!this->temporary_map_loaded_) {
      return predicted_pose;
    }
    map_snapshot = this->temporary_map_;
  }

  int occupied_cell_count = 0;
  for (const auto cell : map_snapshot.data) {
    if (cell >= 50) {
      ++occupied_cell_count;
    }
  }
  if (occupied_cell_count < this->scan_matching_minimum_occupied_cells_) {
    return predicted_pose;
  }

  const double linear_window = std::max(0.0, this->scan_matching_linear_window_);
  const double linear_step = std::max(0.01, this->scan_matching_linear_step_);
  const double angular_window_rad =
    std::max(0.0, this->scan_matching_angular_window_deg_) * kDegToRad;
  const double angular_step_rad =
    std::max(1.0, this->scan_matching_angular_step_deg_) * kDegToRad;

  Pose2D best_pose = predicted_pose;
  double best_score = this->score_scan_candidate(map_snapshot, scan, predicted_pose);

  for (double delta_x = -linear_window; delta_x <= linear_window + 1e-6; delta_x += linear_step) {
    for (double delta_y = -linear_window; delta_y <= linear_window + 1e-6; delta_y += linear_step) {
      for (
        double delta_yaw = -angular_window_rad;
        delta_yaw <= angular_window_rad + 1e-6;
        delta_yaw += angular_step_rad)
      {
        if (
          std::abs(delta_x) < 1e-9 && std::abs(delta_y) < 1e-9 &&
          std::abs(delta_yaw) < 1e-9)
        {
          continue;
        }

        Pose2D candidate_pose{};
        candidate_pose.x = predicted_pose.x + delta_x;
        candidate_pose.y = predicted_pose.y + delta_y;
        candidate_pose.yaw = this->normalize_angle(predicted_pose.yaw + delta_yaw);

        const double candidate_score =
          this->score_scan_candidate(map_snapshot, scan, candidate_pose);
        if (candidate_score > best_score) {
          best_score = candidate_score;
          best_pose = candidate_pose;
        }
      }
    }
  }

  return best_pose;
}

double MapServer::score_scan_candidate(
  const nav_msgs::msg::OccupancyGrid & map,
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & candidate_pose) const
{
  if (scan.ranges.empty()) {
    return -std::numeric_limits<double>::infinity();
  }

  const int beam_stride = std::max(
    1,
    static_cast<int>(scan.ranges.size()) / std::max(1, this->scan_matching_max_beams_));
  int valid_beam_count = 0;
  double score = 0.0;

  for (std::size_t index = 0; index < scan.ranges.size(); index += static_cast<std::size_t>(beam_stride)) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (
      !std::isfinite(range) ||
      range < std::max(this->mapping_min_range_, static_cast<double>(scan.range_min)) ||
      range > std::min(this->mapping_max_range_, static_cast<double>(scan.range_max)))
    {
      continue;
    }

    const double beam_angle = candidate_pose.yaw + static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const double end_x_world = candidate_pose.x + (range * std::cos(beam_angle));
    const double end_y_world = candidate_pose.y + (range * std::sin(beam_angle));

    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_grid(map, end_x_world, end_y_world, grid_x, grid_y)) {
      continue;
    }

    std::size_t cell_index = 0U;
    if (!this->grid_index(map, grid_x, grid_y, cell_index) || cell_index >= map.data.size()) {
      continue;
    }

    ++valid_beam_count;
    const auto cell_value = map.data[cell_index];
    if (
      cell_value >= 50 ||
      this->has_nearby_occupied_cell(
        map,
        grid_x,
        grid_y,
        this->scan_matching_occupied_search_radius_cells_))
    {
      score += this->scan_matching_occupied_match_score_;
    } else if (cell_value == 0) {
      score -= this->scan_matching_free_space_penalty_;
    }
  }

  if (valid_beam_count < this->scan_matching_min_valid_beams_) {
    return -std::numeric_limits<double>::infinity();
  }

  return score;
}

bool MapServer::has_nearby_occupied_cell(
  const nav_msgs::msg::OccupancyGrid & map,
  int grid_x,
  int grid_y,
  int radius_cells) const
{
  const int radius = std::max(0, radius_cells);
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      std::size_t cell_index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, cell_index)) {
        continue;
      }
      if (cell_index < map.data.size() && map.data[cell_index] >= 50) {
        return true;
      }
    }
  }

  return false;
}

bool MapServer::world_to_grid(double x, double y, int & grid_x, int & grid_y) const
{
  std::scoped_lock lock(this->map_mutex_);
  if (!this->temporary_map_loaded_) {
    return false;
  }

  grid_x = static_cast<int>(
    std::floor((x - this->temporary_map_.info.origin.position.x) / this->temporary_map_.info.resolution));
  grid_y = static_cast<int>(
    std::floor((y - this->temporary_map_.info.origin.position.y) / this->temporary_map_.info.resolution));

  return
    grid_x >= 0 && grid_x < static_cast<int>(this->temporary_map_.info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(this->temporary_map_.info.height);
}

bool MapServer::world_to_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  double x,
  double y,
  int & grid_x,
  int & grid_y) const
{
  grid_x = static_cast<int>(
    std::floor((x - map.info.origin.position.x) / map.info.resolution));
  grid_y = static_cast<int>(
    std::floor((y - map.info.origin.position.y) / map.info.resolution));

  return
    grid_x >= 0 && grid_x < static_cast<int>(map.info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(map.info.height);
}

bool MapServer::grid_index(int grid_x, int grid_y, std::size_t & index) const
{
  if (
    !this->temporary_map_loaded_ || grid_x < 0 || grid_y < 0 ||
    grid_x >= static_cast<int>(this->temporary_map_.info.width) ||
    grid_y >= static_cast<int>(this->temporary_map_.info.height))
  {
    return false;
  }

  index = static_cast<std::size_t>((grid_y * static_cast<int>(this->temporary_map_.info.width)) + grid_x);
  return true;
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

void MapServer::mark_free_cell(int grid_x, int grid_y)
{
  this->update_cell_score(grid_x, grid_y, -this->mapping_free_score_);
}

void MapServer::mark_occupied_cell(int grid_x, int grid_y)
{
  this->update_cell_score(grid_x, grid_y, this->mapping_hit_score_);
}

void MapServer::raytrace_free_cells(int start_x, int start_y, int end_x, int end_y)
{
  int current_x = start_x;
  int current_y = start_y;
  const int delta_x = std::abs(end_x - start_x);
  const int delta_y = std::abs(end_y - start_y);
  const int step_x = start_x < end_x ? 1 : -1;
  const int step_y = start_y < end_y ? 1 : -1;
  int error = delta_x - delta_y;

  while (current_x != end_x || current_y != end_y) {
    this->mark_free_cell(current_x, current_y);

    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      current_x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      current_y += step_y;
    }
  }

  this->mark_free_cell(end_x, end_y);
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

double MapServer::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  return std::atan2(
    2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y)),
    1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z))));
}

double MapServer::normalize_angle(double angle) const
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
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
          if (!this->grid_index(
              map_snapshot, grid_x + offset_x, grid_y + offset_y, inflated_index))
          {
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
  const std::string basename = this->save_basename_.empty() ? "temporary_map" : this->save_basename_;
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

void MapServer::update_cell_score(int grid_x, int grid_y, int delta)
{
  std::scoped_lock lock(this->map_mutex_);
  std::size_t index = 0U;
  if (!this->grid_index(grid_x, grid_y, index) || index >= this->occupancy_scores_.size()) {
    return;
  }

  const int updated_score = std::clamp(
    static_cast<int>(this->occupancy_scores_[index]) + delta,
    this->mapping_score_min_,
    this->mapping_score_max_);
  this->occupancy_scores_[index] = static_cast<int16_t>(updated_score);
  this->refresh_cell_from_score(index);
}

void MapServer::refresh_cell_from_score(std::size_t index)
{
  if (index >= this->temporary_map_.data.size() || index >= this->occupancy_scores_.size()) {
    return;
  }

  const int score = static_cast<int>(this->occupancy_scores_[index]);
  if (score >= this->mapping_occupied_score_threshold_) {
    this->temporary_map_.data[index] = 100;
  } else if (score <= this->mapping_free_score_threshold_) {
    this->temporary_map_.data[index] = 0;
  } else {
    this->temporary_map_.data[index] = -1;
  }
}

void MapServer::decay_occupied_scores()
{
  std::scoped_lock lock(this->map_mutex_);
  if (this->mapping_decay_score_ <= 0 || this->occupancy_scores_.empty()) {
    return;
  }

  for (std::size_t index = 0; index < this->occupancy_scores_.size(); ++index) {
    if (this->occupancy_scores_[index] > 0) {
      this->occupancy_scores_[index] = static_cast<int16_t>(std::max(
        0,
        static_cast<int>(this->occupancy_scores_[index]) - this->mapping_decay_score_));
      this->refresh_cell_from_score(index);
    }
  }
}

void MapServer::publish_map_to_odom_tf()
{
  if (!this->transform_broadcaster_) {
    return;
  }

  double translation_x = 0.0;
  double translation_y = 0.0;
  double rotation_yaw = 0.0;
  if (this->has_latest_odometry_) {
    const auto & raw_pose = this->latest_odometry_.pose.pose;
    const double raw_x = raw_pose.position.x;
    const double raw_y = raw_pose.position.y;
    const double raw_yaw = this->quaternion_to_yaw(raw_pose.orientation);

    Pose2D corrected_pose{};
    bool has_corrected_pose = false;
    {
      std::scoped_lock lock(this->map_mutex_);
      corrected_pose = this->latest_corrected_pose_;
      has_corrected_pose = this->has_latest_corrected_pose_;
    }

    if (has_corrected_pose) {
      rotation_yaw = this->normalize_angle(corrected_pose.yaw - raw_yaw);
      const double rotated_raw_x =
        (std::cos(rotation_yaw) * raw_x) - (std::sin(rotation_yaw) * raw_y);
      const double rotated_raw_y =
        (std::sin(rotation_yaw) * raw_x) + (std::cos(rotation_yaw) * raw_y);
      translation_x = corrected_pose.x - rotated_raw_x;
      translation_y = corrected_pose.y - rotated_raw_y;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = this->frame_id_;
  transform.child_frame_id = this->odom_frame_;
  transform.transform.translation.x = translation_x;
  transform.transform.translation.y = translation_y;
  transform.transform.translation.z = 0.0;
  this->set_quaternion_from_yaw(transform.transform.rotation, rotation_yaw);
  this->transform_broadcaster_->sendTransform(transform);
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

}  // namespace amr_map_server
