#include "amr_slam_mapper/slam_mapper.hpp"

namespace amr::slam::mapper
{

SlamMapper::SlamMapper(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("slam_mapper", options),
  frame_id_("map"),
  odom_frame_("odom"),
  base_frame_("base_link"),
  odom_topic_("/odom"),
  imu_topic_("/imu"),
  scan_topic_("/scan"),
  temporary_map_topic_("/amr/map/temp"),
  raw_temporary_map_topic_("/amr/map/temp/raw"),
  refined_temporary_map_topic_("/amr/map/temp/refined"),
  corrected_odometry_topic_("/amr/slam_mapper/odometry"),
  mapping_pose_topic_("/amr/slam_mapper/pose"),
  graph_debug_topic_("/amr/slam_mapper/graph_debug"),
  publish_period_ms_(250),
  mapping_resolution_(0.05),
  mapping_width_(400),
  mapping_height_(400),
  mapping_origin_x_(-10.0),
  mapping_origin_y_(-10.0),
  mapping_origin_yaw_(0.0),
  mapping_min_range_(0.05),
  mapping_max_range_(8.0),
  publish_map_to_odom_tf_(true),
  scan_matching_linear_window_(0.15),
  scan_matching_linear_step_(0.05),
  scan_matching_angular_window_deg_(12.0),
  scan_matching_angular_step_deg_(3.0),
  scan_matching_max_beams_(32),
  scan_matching_min_valid_beams_(8),
  scan_matching_occupied_search_radius_cells_(1),
  scan_matching_distance_match_radius_cells_(4),
  scan_matching_minimum_occupied_cells_(50),
  scan_matching_occupied_match_score_(3.0),
  scan_matching_distance_match_score_(2.0),
  scan_matching_distance_penalty_per_cell_(0.45),
  scan_matching_free_space_penalty_(1.0),
  scan_matching_min_score_improvement_(2.0),
  scan_matching_max_translation_correction_(0.08),
  scan_matching_max_yaw_correction_deg_(6.0),
  scan_matching_translation_regularization_weight_(5.0),
  scan_matching_yaw_regularization_weight_(0.75),
  mapping_hit_score_(20),
  mapping_free_score_(3),
  mapping_occupied_score_threshold_(20),
  mapping_free_score_threshold_(-5),
  mapping_score_min_(-20),
  mapping_score_max_(100),
  refinement_min_occupied_neighbor_count_(2),
  refinement_min_free_neighbor_count_(4),
  keyframe_distance_threshold_(0.30),
  keyframe_yaw_threshold_(0.30),
  submap_nodes_per_submap_(10),
  loop_closure_min_node_separation_(15),
  loop_closure_descriptor_threshold_(0.12),
  loop_closure_acceptance_score_(20.0),
  loop_closure_search_linear_window_(0.25),
  loop_closure_search_linear_step_(0.05),
  loop_closure_search_angular_window_deg_(18.0),
  loop_closure_search_angular_step_deg_(3.0),
  graph_optimization_iterations_(20),
  graph_optimization_step_size_(0.35),
  descriptor_beams_(32)
{
  this->declare_parameter("frames.map", this->frame_id_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("topics.odom", this->odom_topic_);
  this->declare_parameter("topics.imu", this->imu_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.temp_map", this->temporary_map_topic_);
  this->declare_parameter("topics.temp_map_raw", this->raw_temporary_map_topic_);
  this->declare_parameter("topics.temp_map_refined", this->refined_temporary_map_topic_);
  this->declare_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->declare_parameter("topics.mapping_pose", this->mapping_pose_topic_);
  this->declare_parameter("topics.graph_debug", this->graph_debug_topic_);
  this->declare_parameter("publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("mapping.resolution", this->mapping_resolution_);
  this->declare_parameter("mapping.width", this->mapping_width_);
  this->declare_parameter("mapping.height", this->mapping_height_);
  this->declare_parameter("mapping.origin.x", this->mapping_origin_x_);
  this->declare_parameter("mapping.origin.y", this->mapping_origin_y_);
  this->declare_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  this->declare_parameter("mapping.range.min", this->mapping_min_range_);
  this->declare_parameter("mapping.range.max", this->mapping_max_range_);
  this->declare_parameter("mapping.publish_map_to_odom_tf", this->publish_map_to_odom_tf_);
  this->declare_parameter("scan_matching.linear_window", this->scan_matching_linear_window_);
  this->declare_parameter("scan_matching.linear_step", this->scan_matching_linear_step_);
  this->declare_parameter(
    "scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  this->declare_parameter(
    "scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  this->declare_parameter("scan_matching.max_beams", this->scan_matching_max_beams_);
  this->declare_parameter(
    "scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  this->declare_parameter(
    "scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  this->declare_parameter(
    "scan_matching.distance_match_radius_cells",
    this->scan_matching_distance_match_radius_cells_);
  this->declare_parameter(
    "scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  this->declare_parameter(
    "scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  this->declare_parameter(
    "scan_matching.distance_match_score", this->scan_matching_distance_match_score_);
  this->declare_parameter(
    "scan_matching.distance_penalty_per_cell",
    this->scan_matching_distance_penalty_per_cell_);
  this->declare_parameter(
    "scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  this->declare_parameter(
    "scan_matching.min_score_improvement", this->scan_matching_min_score_improvement_);
  this->declare_parameter(
    "scan_matching.max_translation_correction", this->scan_matching_max_translation_correction_);
  this->declare_parameter(
    "scan_matching.max_yaw_correction_deg", this->scan_matching_max_yaw_correction_deg_);
  this->declare_parameter(
    "scan_matching.translation_regularization_weight",
    this->scan_matching_translation_regularization_weight_);
  this->declare_parameter(
    "scan_matching.yaw_regularization_weight",
    this->scan_matching_yaw_regularization_weight_);
  this->declare_parameter("occupancy.hit_score", this->mapping_hit_score_);
  this->declare_parameter("occupancy.free_score", this->mapping_free_score_);
  this->declare_parameter(
    "occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  this->declare_parameter(
    "occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  this->declare_parameter("occupancy.score_min", this->mapping_score_min_);
  this->declare_parameter("occupancy.score_max", this->mapping_score_max_);
  this->declare_parameter(
    "refinement.min_occupied_neighbor_count", this->refinement_min_occupied_neighbor_count_);
  this->declare_parameter(
    "refinement.min_free_neighbor_count", this->refinement_min_free_neighbor_count_);
  this->declare_parameter("pose_graph.keyframe_distance_threshold", this->keyframe_distance_threshold_);
  this->declare_parameter("pose_graph.keyframe_yaw_threshold", this->keyframe_yaw_threshold_);
  this->declare_parameter("pose_graph.submap_nodes_per_submap", this->submap_nodes_per_submap_);
  this->declare_parameter(
    "pose_graph.loop_closure_min_node_separation", this->loop_closure_min_node_separation_);
  this->declare_parameter(
    "pose_graph.loop_closure_descriptor_threshold", this->loop_closure_descriptor_threshold_);
  this->declare_parameter(
    "pose_graph.loop_closure_acceptance_score", this->loop_closure_acceptance_score_);
  this->declare_parameter(
    "pose_graph.loop_closure_search_linear_window", this->loop_closure_search_linear_window_);
  this->declare_parameter(
    "pose_graph.loop_closure_search_linear_step", this->loop_closure_search_linear_step_);
  this->declare_parameter(
    "pose_graph.loop_closure_search_angular_window_deg",
    this->loop_closure_search_angular_window_deg_);
  this->declare_parameter(
    "pose_graph.loop_closure_search_angular_step_deg",
    this->loop_closure_search_angular_step_deg_);
  this->declare_parameter(
    "pose_graph.graph_optimization_iterations", this->graph_optimization_iterations_);
  this->declare_parameter(
    "pose_graph.graph_optimization_step_size", this->graph_optimization_step_size_);
  this->declare_parameter("pose_graph.descriptor_beams", this->descriptor_beams_);
}

SlamMapper::CallbackReturn SlamMapper::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->get_parameter("frames.map", this->frame_id_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.imu", this->imu_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.temp_map", this->temporary_map_topic_);
  this->get_parameter("topics.temp_map_raw", this->raw_temporary_map_topic_);
  this->get_parameter("topics.temp_map_refined", this->refined_temporary_map_topic_);
  this->get_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->get_parameter("topics.mapping_pose", this->mapping_pose_topic_);
  this->get_parameter("topics.graph_debug", this->graph_debug_topic_);
  this->get_parameter("publish_period_ms", this->publish_period_ms_);
  this->get_parameter("mapping.resolution", this->mapping_resolution_);
  this->get_parameter("mapping.width", this->mapping_width_);
  this->get_parameter("mapping.height", this->mapping_height_);
  this->get_parameter("mapping.origin.x", this->mapping_origin_x_);
  this->get_parameter("mapping.origin.y", this->mapping_origin_y_);
  this->get_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  this->get_parameter("mapping.range.min", this->mapping_min_range_);
  this->get_parameter("mapping.range.max", this->mapping_max_range_);
  this->get_parameter("mapping.publish_map_to_odom_tf", this->publish_map_to_odom_tf_);
  this->get_parameter("scan_matching.linear_window", this->scan_matching_linear_window_);
  this->get_parameter("scan_matching.linear_step", this->scan_matching_linear_step_);
  this->get_parameter(
    "scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  this->get_parameter(
    "scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  this->get_parameter("scan_matching.max_beams", this->scan_matching_max_beams_);
  this->get_parameter(
    "scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  this->get_parameter(
    "scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  this->get_parameter(
    "scan_matching.distance_match_radius_cells",
    this->scan_matching_distance_match_radius_cells_);
  this->get_parameter(
    "scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  this->get_parameter(
    "scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  this->get_parameter(
    "scan_matching.distance_match_score", this->scan_matching_distance_match_score_);
  this->get_parameter(
    "scan_matching.distance_penalty_per_cell",
    this->scan_matching_distance_penalty_per_cell_);
  this->get_parameter(
    "scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  this->get_parameter(
    "scan_matching.min_score_improvement", this->scan_matching_min_score_improvement_);
  this->get_parameter(
    "scan_matching.max_translation_correction", this->scan_matching_max_translation_correction_);
  this->get_parameter(
    "scan_matching.max_yaw_correction_deg", this->scan_matching_max_yaw_correction_deg_);
  this->get_parameter(
    "scan_matching.translation_regularization_weight",
    this->scan_matching_translation_regularization_weight_);
  this->get_parameter(
    "scan_matching.yaw_regularization_weight",
    this->scan_matching_yaw_regularization_weight_);
  this->get_parameter("occupancy.hit_score", this->mapping_hit_score_);
  this->get_parameter("occupancy.free_score", this->mapping_free_score_);
  this->get_parameter(
    "occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  this->get_parameter(
    "occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  this->get_parameter("occupancy.score_min", this->mapping_score_min_);
  this->get_parameter("occupancy.score_max", this->mapping_score_max_);
  this->get_parameter(
    "refinement.min_occupied_neighbor_count", this->refinement_min_occupied_neighbor_count_);
  this->get_parameter(
    "refinement.min_free_neighbor_count", this->refinement_min_free_neighbor_count_);
  this->get_parameter("pose_graph.keyframe_distance_threshold", this->keyframe_distance_threshold_);
  this->get_parameter("pose_graph.keyframe_yaw_threshold", this->keyframe_yaw_threshold_);
  this->get_parameter("pose_graph.submap_nodes_per_submap", this->submap_nodes_per_submap_);
  this->get_parameter(
    "pose_graph.loop_closure_min_node_separation", this->loop_closure_min_node_separation_);
  this->get_parameter(
    "pose_graph.loop_closure_descriptor_threshold", this->loop_closure_descriptor_threshold_);
  this->get_parameter(
    "pose_graph.loop_closure_acceptance_score", this->loop_closure_acceptance_score_);
  this->get_parameter(
    "pose_graph.loop_closure_search_linear_window", this->loop_closure_search_linear_window_);
  this->get_parameter(
    "pose_graph.loop_closure_search_linear_step", this->loop_closure_search_linear_step_);
  this->get_parameter(
    "pose_graph.loop_closure_search_angular_window_deg",
    this->loop_closure_search_angular_window_deg_);
  this->get_parameter(
    "pose_graph.loop_closure_search_angular_step_deg",
    this->loop_closure_search_angular_step_deg_);
  this->get_parameter(
    "pose_graph.graph_optimization_iterations", this->graph_optimization_iterations_);
  this->get_parameter(
    "pose_graph.graph_optimization_step_size", this->graph_optimization_step_size_);
  this->get_parameter("pose_graph.descriptor_beams", this->descriptor_beams_);

  this->initialize_mapping_map();
  this->graph_nodes_.clear();
  this->graph_edges_.clear();
  this->submaps_.clear();
  this->next_graph_node_id_ = 1;
  this->next_submap_id_ = 1;
  this->map_to_odom_ = Pose2D{};
  this->current_corrected_pose_ = Pose2D{};
  this->has_latest_odometry_ = false;
  this->has_latest_scan_ = false;
  this->has_current_corrected_pose_ = false;
  this->has_start_odom_yaw_ = false;
  this->has_latest_imu_ = false;
  this->has_start_imu_yaw_ = false;

  this->temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->raw_temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->raw_temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->refined_temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->refined_temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->corrected_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->corrected_odometry_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  this->mapping_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->mapping_pose_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  this->graph_debug_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->graph_debug_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->odom_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      this->handle_odometry(message);
    });
  this->imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->imu_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr message) {
      this->handle_imu(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });

  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(50, this->publish_period_ms_)),
    [this]() { this->publish_outputs(); });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM mapper with odom='%s', imu='%s', scan='%s', temp_map='%s', temp_raw='%s', temp_refined='%s', corrected_odom='%s', mapping_pose='%s', graph_debug='%s'",
    this->odom_topic_.c_str(),
    this->imu_topic_.c_str(),
    this->scan_topic_.c_str(),
    this->temporary_map_topic_.c_str(),
    this->raw_temporary_map_topic_.c_str(),
    this->refined_temporary_map_topic_.c_str(),
    this->corrected_odometry_topic_.c_str(),
    this->mapping_pose_topic_.c_str(),
    this->graph_debug_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

SlamMapper::CallbackReturn SlamMapper::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_activate();
  }
  if (this->raw_temporary_map_publisher_) {
    this->raw_temporary_map_publisher_->on_activate();
  }
  if (this->refined_temporary_map_publisher_) {
    this->refined_temporary_map_publisher_->on_activate();
  }
  if (this->corrected_odometry_publisher_) {
    this->corrected_odometry_publisher_->on_activate();
  }
  if (this->mapping_pose_publisher_) {
    this->mapping_pose_publisher_->on_activate();
  }
  if (this->graph_debug_publisher_) {
    this->graph_debug_publisher_->on_activate();
  }
  this->publish_outputs();
  return CallbackReturn::SUCCESS;
}

SlamMapper::CallbackReturn SlamMapper::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_deactivate();
  }
  if (this->raw_temporary_map_publisher_) {
    this->raw_temporary_map_publisher_->on_deactivate();
  }
  if (this->refined_temporary_map_publisher_) {
    this->refined_temporary_map_publisher_->on_deactivate();
  }
  if (this->corrected_odometry_publisher_) {
    this->corrected_odometry_publisher_->on_deactivate();
  }
  if (this->mapping_pose_publisher_) {
    this->mapping_pose_publisher_->on_deactivate();
  }
  if (this->graph_debug_publisher_) {
    this->graph_debug_publisher_->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

SlamMapper::CallbackReturn SlamMapper::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  this->publish_timer_.reset();
  this->scan_subscription_.reset();
  this->imu_subscription_.reset();
  this->odometry_subscription_.reset();
  this->temporary_map_publisher_.reset();
  this->raw_temporary_map_publisher_.reset();
  this->refined_temporary_map_publisher_.reset();
  this->corrected_odometry_publisher_.reset();
  this->mapping_pose_publisher_.reset();
  this->graph_debug_publisher_.reset();
  this->transform_broadcaster_.reset();
  {
    std::scoped_lock lock(this->map_mutex_);
    this->temporary_map_ = nav_msgs::msg::OccupancyGrid();
    this->refined_temporary_map_ = nav_msgs::msg::OccupancyGrid();
    this->occupancy_scores_.clear();
  }
  this->graph_nodes_.clear();
  this->graph_edges_.clear();
  this->submaps_.clear();
  return CallbackReturn::SUCCESS;
}

SlamMapper::CallbackReturn SlamMapper::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return this->on_cleanup(state);
}

void SlamMapper::initialize_mapping_map()
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

  std::scoped_lock lock(this->map_mutex_);
  this->temporary_map_ = std::move(initialized_map);
  this->refined_temporary_map_ = this->temporary_map_;
  this->occupancy_scores_.assign(
    static_cast<std::size_t>(this->mapping_width_ * this->mapping_height_),
    0);
}

void SlamMapper::publish_outputs()
{
  this->refresh_refined_map();
  this->publish_raw_temporary_map();
  this->publish_refined_temporary_map();
  this->publish_temporary_map();
  this->publish_corrected_odometry();
  this->publish_mapping_pose();
  this->publish_graph_debug();
  if (this->publish_map_to_odom_tf_) {
    this->publish_map_to_odom_tf();
  }
}

void SlamMapper::publish_temporary_map()
{
  if (!this->temporary_map_publisher_ || !this->temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    map_to_publish = this->refined_temporary_map_;
  }
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->temporary_map_publisher_->publish(map_to_publish);
}

void SlamMapper::publish_raw_temporary_map()
{
  if (!this->raw_temporary_map_publisher_ || !this->raw_temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    map_to_publish = this->temporary_map_;
  }
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->raw_temporary_map_publisher_->publish(map_to_publish);
}

void SlamMapper::publish_refined_temporary_map()
{
  if (!this->refined_temporary_map_publisher_ || !this->refined_temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish;
  {
    std::scoped_lock lock(this->map_mutex_);
    map_to_publish = this->refined_temporary_map_;
  }
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->refined_temporary_map_publisher_->publish(map_to_publish);
}

void SlamMapper::publish_corrected_odometry()
{
  if (
    !this->corrected_odometry_publisher_ || !this->corrected_odometry_publisher_->is_activated() ||
    !this->has_current_corrected_pose_)
  {
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = this->now();
  odometry.header.frame_id = this->frame_id_;
  odometry.child_frame_id = this->base_frame_;
  odometry.pose.pose.position.x = this->current_corrected_pose_.x;
  odometry.pose.pose.position.y = this->current_corrected_pose_.y;
  odometry.pose.pose.position.z = 0.0;
  this->set_quaternion_from_yaw(odometry.pose.pose.orientation, this->current_corrected_pose_.yaw);
  this->corrected_odometry_publisher_->publish(odometry);
}

void SlamMapper::publish_mapping_pose()
{
  if (
    !this->mapping_pose_publisher_ || !this->mapping_pose_publisher_->is_activated() ||
    !this->has_current_corrected_pose_)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = this->frame_id_;
  pose.pose.position.x = this->current_corrected_pose_.x;
  pose.pose.position.y = this->current_corrected_pose_.y;
  pose.pose.position.z = 0.0;
  this->set_quaternion_from_yaw(pose.pose.orientation, this->current_corrected_pose_.yaw);
  this->mapping_pose_publisher_->publish(pose);
}

void SlamMapper::publish_graph_debug()
{
  if (!this->graph_debug_publisher_ || !this->graph_debug_publisher_->is_activated()) {
    return;
  }

  std_msgs::msg::String message;
  message.data = this->build_graph_debug_json();
  this->graph_debug_publisher_->publish(message);
}

void SlamMapper::refresh_refined_map()
{
  nav_msgs::msg::OccupancyGrid raw_snapshot;
  {
    std::scoped_lock lock(this->map_mutex_);
    raw_snapshot = this->temporary_map_;
  }

  nav_msgs::msg::OccupancyGrid refined_map = this->build_refined_map(raw_snapshot);
  refined_map.header.stamp = this->now();
  refined_map.info.map_load_time = refined_map.header.stamp;

  {
    std::scoped_lock lock(this->map_mutex_);
    this->refined_temporary_map_ = std::move(refined_map);
  }
}

nav_msgs::msg::OccupancyGrid SlamMapper::build_refined_map(
  const nav_msgs::msg::OccupancyGrid & source_map) const
{
  nav_msgs::msg::OccupancyGrid refined_map = source_map;
  if (source_map.data.empty()) {
    return refined_map;
  }

  const int width = static_cast<int>(source_map.info.width);
  const int height = static_cast<int>(source_map.info.height);
  for (int grid_y = 0; grid_y < height; ++grid_y) {
    for (int grid_x = 0; grid_x < width; ++grid_x) {
      std::size_t index = 0U;
      if (!this->grid_index(source_map, grid_x, grid_y, index) || index >= source_map.data.size()) {
        continue;
      }

      const int8_t cell_value = source_map.data[index];
      if (cell_value < 50) {
        continue;
      }

      const int occupied_neighbors =
        this->count_neighboring_cells(source_map, grid_x, grid_y, 50, 100);
      const int free_neighbors =
        this->count_neighboring_cells(source_map, grid_x, grid_y, 0, 49);

      if (
        occupied_neighbors < std::max(1, this->refinement_min_occupied_neighbor_count_) &&
        free_neighbors >= std::max(1, this->refinement_min_free_neighbor_count_))
      {
        refined_map.data[index] = -1;
      }
    }
  }

  return refined_map;
}

int SlamMapper::count_neighboring_cells(
  const nav_msgs::msg::OccupancyGrid & map,
  int grid_x,
  int grid_y,
  int minimum_value,
  int maximum_value) const
{
  int count = 0;
  for (int offset_y = -1; offset_y <= 1; ++offset_y) {
    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
      if (offset_x == 0 && offset_y == 0) {
        continue;
      }

      std::size_t index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, index) || index >= map.data.size()) {
        continue;
      }

      const int value = static_cast<int>(map.data[index]);
      if (value >= minimum_value && value <= maximum_value) {
        ++count;
      }
    }
  }

  return count;
}

void SlamMapper::publish_map_to_odom_tf()
{
  if (!this->transform_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = this->frame_id_;
  transform.child_frame_id = this->odom_frame_;
  transform.transform.translation.x = this->map_to_odom_.x;
  transform.transform.translation.y = this->map_to_odom_.y;
  transform.transform.translation.z = 0.0;
  this->set_quaternion_from_yaw(transform.transform.rotation, this->map_to_odom_.yaw);
  this->transform_broadcaster_->sendTransform(transform);
}

std::string SlamMapper::build_graph_debug_json() const
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);

  std::size_t loop_count = 0U;
  for (const auto & edge : this->graph_edges_) {
    if (edge.loop_closure) {
      ++loop_count;
    }
  }

  stream
    << "{"
    << "\"frame_id\":\"" << this->frame_id_ << "\","
    << "\"node_count\":" << this->graph_nodes_.size() << ","
    << "\"edge_count\":" << this->graph_edges_.size() << ","
    << "\"loop_count\":" << loop_count << ","
    << "\"nodes\":[";

  for (std::size_t index = 0; index < this->graph_nodes_.size(); ++index) {
    const auto & node = this->graph_nodes_[index];
    if (index > 0U) {
      stream << ",";
    }
    stream
      << "{"
      << "\"id\":" << node.id << ","
      << "\"x\":" << node.map_pose.x << ","
      << "\"y\":" << node.map_pose.y << ","
      << "\"yaw\":" << node.map_pose.yaw << ","
      << "\"submap_id\":" << node.submap_id
      << "}";
  }

  stream << "],\"edges\":[";
  bool first_edge = true;
  for (std::size_t index = 0; index < this->graph_edges_.size(); ++index) {
    const auto & edge = this->graph_edges_[index];
    if (edge.from < 0 || edge.to < 0) {
      continue;
    }
    const std::size_t from_index = static_cast<std::size_t>(edge.from);
    const std::size_t to_index = static_cast<std::size_t>(edge.to);
    if (from_index >= this->graph_nodes_.size() || to_index >= this->graph_nodes_.size()) {
      continue;
    }
    if (!first_edge) {
      stream << ",";
    }
    first_edge = false;
    const auto & from_node = this->graph_nodes_[from_index];
    const auto & to_node = this->graph_nodes_[to_index];
    stream
      << "{"
      << "\"from\":" << from_node.id << ","
      << "\"to\":" << to_node.id << ","
      << "\"from_x\":" << from_node.map_pose.x << ","
      << "\"from_y\":" << from_node.map_pose.y << ","
      << "\"to_x\":" << to_node.map_pose.x << ","
      << "\"to_y\":" << to_node.map_pose.y << ","
      << "\"weight\":" << edge.weight << ","
      << "\"loop_closure\":" << (edge.loop_closure ? "true" : "false")
      << "}";
  }

  stream << "],\"submaps\":[";
  for (std::size_t index = 0; index < this->submaps_.size(); ++index) {
    const auto & submap = this->submaps_[index];
    if (index > 0U) {
      stream << ",";
    }
    stream
      << "{"
      << "\"id\":" << submap.id << ","
      << "\"start_node_index\":" << submap.start_node_index << ","
      << "\"end_node_index\":" << submap.end_node_index << ","
      << "\"keyframe_count\":" << submap.keyframe_count << ","
      << "\"anchor_x\":" << submap.anchor_pose.x << ","
      << "\"anchor_y\":" << submap.anchor_pose.y << ","
      << "\"anchor_yaw\":" << submap.anchor_pose.yaw
      << "}";
  }

  stream << "]}";
  return stream.str();
}

void SlamMapper::handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  this->latest_odometry_ = *message;
  this->has_latest_odometry_ = true;
  if (!this->has_start_odom_yaw_) {
    this->start_odom_yaw_ = this->quaternion_to_yaw(message->pose.pose.orientation);
    this->has_start_odom_yaw_ = true;
  }
}

void SlamMapper::handle_imu(const sensor_msgs::msg::Imu::SharedPtr message)
{
  this->latest_imu_yaw_ = this->quaternion_to_yaw(message->orientation);
  this->has_latest_imu_ = true;
  if (!this->has_start_imu_yaw_) {
    this->start_imu_yaw_ = this->latest_imu_yaw_;
    this->has_start_imu_yaw_ = true;
  }
}

void SlamMapper::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  if (!this->has_latest_odometry_) {
    return;
  }

  this->has_latest_scan_ = true;
  const Pose2D raw_odom_pose = this->build_raw_odom_pose();
  const Pose2D predicted_pose = this->apply_map_to_odom_transform(raw_odom_pose);
  const Pose2D corrected_pose = this->refine_pose_with_scan_matching(*message, predicted_pose);

  this->current_corrected_pose_ = corrected_pose;
  this->has_current_corrected_pose_ = true;
  this->update_map_to_odom_transform(corrected_pose, raw_odom_pose);

  {
    std::scoped_lock lock(this->map_mutex_);
    this->integrate_scan_into_map(*message, corrected_pose, this->temporary_map_, this->occupancy_scores_);
  }

  this->maybe_add_pose_graph_node(*message, corrected_pose, raw_odom_pose);
}

SlamMapper::Pose2D SlamMapper::build_raw_odom_pose() const
{
  Pose2D pose{};
  pose.x = this->latest_odometry_.pose.pose.position.x;
  pose.y = this->latest_odometry_.pose.pose.position.y;
  pose.yaw = this->quaternion_to_yaw(this->latest_odometry_.pose.pose.orientation);

  if (this->has_latest_imu_ && this->has_start_imu_yaw_ && this->has_start_odom_yaw_) {
    pose.yaw = this->normalize_angle(
      this->start_odom_yaw_ + (this->latest_imu_yaw_ - this->start_imu_yaw_));
  }

  return pose;
}

SlamMapper::Pose2D SlamMapper::apply_map_to_odom_transform(const Pose2D & odom_pose) const
{
  return this->compose_pose(this->map_to_odom_, odom_pose);
}

void SlamMapper::update_map_to_odom_transform(
  const Pose2D & corrected_pose,
  const Pose2D & raw_odom_pose)
{
  this->map_to_odom_ = this->compose_pose(corrected_pose, this->inverse_pose(raw_odom_pose));
}

SlamMapper::Pose2D SlamMapper::refine_pose_with_scan_matching(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & predicted_pose) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  nav_msgs::msg::OccupancyGrid map_snapshot;
  {
    std::scoped_lock lock(this->map_mutex_);
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
  const double max_translation_correction =
    std::max(0.0, this->scan_matching_max_translation_correction_);
  const double max_yaw_correction_rad =
    std::max(0.0, this->scan_matching_max_yaw_correction_deg_) * kDegToRad;
  const double min_score_improvement =
    std::max(0.0, this->scan_matching_min_score_improvement_);
  const double translation_regularization_weight =
    std::max(0.0, this->scan_matching_translation_regularization_weight_);
  const double yaw_regularization_weight =
    std::max(0.0, this->scan_matching_yaw_regularization_weight_);

  Pose2D best_pose = predicted_pose;
  const double predicted_score = this->score_scan_candidate(map_snapshot, scan, predicted_pose);
  double best_score = predicted_score;

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
        if (!std::isfinite(candidate_score)) {
          continue;
        }

        const double correction_translation = std::hypot(delta_x, delta_y);
        const double correction_yaw_deg = std::abs(delta_yaw) / kDegToRad;
        const double regularized_score =
          candidate_score -
          (translation_regularization_weight * correction_translation) -
          (yaw_regularization_weight * correction_yaw_deg);
        if (regularized_score > best_score) {
          best_score = regularized_score;
          best_pose = candidate_pose;
        }
      }
    }
  }

  const double correction_x = best_pose.x - predicted_pose.x;
  const double correction_y = best_pose.y - predicted_pose.y;
  const double correction_translation = std::hypot(correction_x, correction_y);
  const double correction_yaw =
    std::abs(this->normalize_angle(best_pose.yaw - predicted_pose.yaw));

  if (
    !std::isfinite(best_score) ||
    (best_score - predicted_score) < min_score_improvement ||
    correction_translation > max_translation_correction ||
    correction_yaw > max_yaw_correction_rad)
  {
    return predicted_pose;
  }

  return best_pose;
}

double SlamMapper::score_scan_candidate(
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
    if (cell_value >= 50) {
      score += this->scan_matching_occupied_match_score_;
      continue;
    }

    const double nearest_distance_cells = this->nearest_occupied_distance_cells(
      map,
      grid_x,
      grid_y,
      this->scan_matching_distance_match_radius_cells_);
    if (std::isfinite(nearest_distance_cells)) {
      const double proximity_score =
        this->scan_matching_distance_match_score_ -
        (this->scan_matching_distance_penalty_per_cell_ * nearest_distance_cells);
      if (proximity_score > 0.0) {
        score += proximity_score;
        continue;
      }
    }

    if (
      this->has_nearby_occupied_cell(
        map,
        grid_x,
        grid_y,
        this->scan_matching_occupied_search_radius_cells_))
    {
      score += this->scan_matching_occupied_match_score_ * 0.5;
    } else if (cell_value == 0) {
      score -= this->scan_matching_free_space_penalty_;
    }
  }

  if (valid_beam_count < this->scan_matching_min_valid_beams_) {
    return -std::numeric_limits<double>::infinity();
  }
  return score;
}

double SlamMapper::nearest_occupied_distance_cells(
  const nav_msgs::msg::OccupancyGrid & map,
  int grid_x,
  int grid_y,
  int radius_cells) const
{
  const int radius = std::max(0, radius_cells);
  double best_distance = std::numeric_limits<double>::infinity();

  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      std::size_t cell_index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, cell_index)) {
        continue;
      }
      if (cell_index >= map.data.size() || map.data[cell_index] < 50) {
        continue;
      }

      const double distance = std::hypot(
        static_cast<double>(offset_x),
        static_cast<double>(offset_y));
      if (distance < best_distance) {
        best_distance = distance;
      }
    }
  }

  return best_distance;
}

bool SlamMapper::has_nearby_occupied_cell(
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

void SlamMapper::integrate_scan_into_map(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & corrected_pose,
  nav_msgs::msg::OccupancyGrid & map,
  std::vector<int16_t> & occupancy_scores) const
{
  int start_x = 0;
  int start_y = 0;
  if (!this->world_to_grid(map, corrected_pose.x, corrected_pose.y, start_x, start_y)) {
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
    const double beam_angle = corrected_pose.yaw + static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const double end_x_world = corrected_pose.x + (clipped_range * std::cos(beam_angle));
    const double end_y_world = corrected_pose.y + (clipped_range * std::sin(beam_angle));

    int end_x = 0;
    int end_y = 0;
    if (!this->world_to_grid(map, end_x_world, end_y_world, end_x, end_y)) {
      continue;
    }

    this->raytrace_free_cells(map, occupancy_scores, start_x, start_y, end_x, end_y);
    if (has_hit) {
      this->mark_occupied_cell(map, occupancy_scores, end_x, end_y);
    }
  }
}

void SlamMapper::rebuild_map_from_pose_graph()
{
  this->initialize_mapping_map();
  std::scoped_lock lock(this->map_mutex_);
  for (const auto & node : this->graph_nodes_) {
    this->integrate_scan_into_map(node.scan, node.map_pose, this->temporary_map_, this->occupancy_scores_);
  }
}

void SlamMapper::maybe_add_pose_graph_node(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & corrected_pose,
  const Pose2D & raw_odom_pose)
{
  const bool add_first = this->graph_nodes_.empty();
  bool should_add = add_first;
  if (!add_first) {
    const auto & last_node = this->graph_nodes_.back();
    const double dx = corrected_pose.x - last_node.map_pose.x;
    const double dy = corrected_pose.y - last_node.map_pose.y;
    const double distance = std::hypot(dx, dy);
    const double dyaw = std::abs(this->normalize_angle(corrected_pose.yaw - last_node.map_pose.yaw));
    should_add = distance >= this->keyframe_distance_threshold_ || dyaw >= this->keyframe_yaw_threshold_;
  }

  if (!should_add) {
    return;
  }

  GraphNode node;
  node.id = this->next_graph_node_id_++;
  node.map_pose = corrected_pose;
  node.raw_odom_pose = raw_odom_pose;
  node.scan = scan;
  node.descriptor = this->build_scan_descriptor(scan);
  node.submap_id = this->submaps_.empty() ? this->next_submap_id_ : this->submaps_.back().id;

  if (this->submaps_.empty() || this->submaps_.back().keyframe_count >= this->submap_nodes_per_submap_) {
    Submap submap;
    submap.id = this->next_submap_id_++;
    submap.anchor_pose = corrected_pose;
    submap.start_node_index = static_cast<int>(this->graph_nodes_.size());
    submap.end_node_index = submap.start_node_index;
    submap.keyframe_count = 0;
    this->submaps_.push_back(submap);
    node.submap_id = submap.id;
  }

  const int new_index = static_cast<int>(this->graph_nodes_.size());
  if (!this->graph_nodes_.empty()) {
    const auto & previous_node = this->graph_nodes_.back();
    GraphEdge odom_edge;
    odom_edge.from = static_cast<int>(this->graph_nodes_.size()) - 1;
    odom_edge.to = new_index;
    odom_edge.relative_pose = this->relative_pose(previous_node.raw_odom_pose, raw_odom_pose);
    odom_edge.weight = 1.0;
    odom_edge.loop_closure = false;
    this->graph_edges_.push_back(odom_edge);
  }

  this->graph_nodes_.push_back(node);
  this->update_submap_accumulation(node);

  const LoopClosureCandidate candidate =
    this->search_loop_closure_candidate(scan, node.descriptor);
  this->maybe_optimize_pose_graph(candidate, new_index);
}

std::vector<float> SlamMapper::build_scan_descriptor(const sensor_msgs::msg::LaserScan & scan) const
{
  const int beam_count = std::max(8, this->descriptor_beams_);
  std::vector<float> descriptor(static_cast<std::size_t>(beam_count), 1.0F);
  if (scan.ranges.empty()) {
    return descriptor;
  }

  const double max_range = std::min(this->mapping_max_range_, static_cast<double>(scan.range_max));
  for (int descriptor_index = 0; descriptor_index < beam_count; ++descriptor_index) {
    const std::size_t source_index = static_cast<std::size_t>(
      std::floor(
        static_cast<double>(descriptor_index) *
        static_cast<double>(scan.ranges.size()) /
        static_cast<double>(beam_count)));
    const auto clamped_index = std::min(source_index, scan.ranges.size() - 1U);
    double range = static_cast<double>(scan.ranges[clamped_index]);
    if (!std::isfinite(range)) {
      range = max_range;
    }
    range = std::clamp(range, this->mapping_min_range_, max_range);
    descriptor[static_cast<std::size_t>(descriptor_index)] =
      static_cast<float>(range / std::max(0.01, max_range));
  }
  return descriptor;
}

double SlamMapper::compute_descriptor_distance(
  const std::vector<float> & lhs,
  const std::vector<float> & rhs) const
{
  if (lhs.size() != rhs.size() || lhs.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double distance = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    distance += std::abs(static_cast<double>(lhs[index] - rhs[index]));
  }
  return distance / static_cast<double>(lhs.size());
}

SlamMapper::LoopClosureCandidate SlamMapper::search_loop_closure_candidate(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<float> & descriptor) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  LoopClosureCandidate best{};
  if (this->graph_nodes_.size() <= static_cast<std::size_t>(this->loop_closure_min_node_separation_)) {
    return best;
  }

  for (std::size_t index = 0; index + static_cast<std::size_t>(this->loop_closure_min_node_separation_) < this->graph_nodes_.size(); ++index) {
    const auto & node = this->graph_nodes_[index];
    const double descriptor_distance = this->compute_descriptor_distance(descriptor, node.descriptor);
    if (descriptor_distance > this->loop_closure_descriptor_threshold_) {
      continue;
    }

    const double linear_window = std::max(0.0, this->loop_closure_search_linear_window_);
    const double linear_step = std::max(0.01, this->loop_closure_search_linear_step_);
    const double angular_window_rad =
      std::max(0.0, this->loop_closure_search_angular_window_deg_) * kDegToRad;
    const double angular_step_rad =
      std::max(1.0, this->loop_closure_search_angular_step_deg_) * kDegToRad;

    Pose2D best_pose = node.map_pose;
    double best_score = this->score_scan_candidate(this->temporary_map_, scan, best_pose);
    for (double delta_x = -linear_window; delta_x <= linear_window + 1e-6; delta_x += linear_step) {
      for (double delta_y = -linear_window; delta_y <= linear_window + 1e-6; delta_y += linear_step) {
        for (
          double delta_yaw = -angular_window_rad;
          delta_yaw <= angular_window_rad + 1e-6;
          delta_yaw += angular_step_rad)
        {
          Pose2D candidate_pose{};
          candidate_pose.x = node.map_pose.x + delta_x;
          candidate_pose.y = node.map_pose.y + delta_y;
          candidate_pose.yaw = this->normalize_angle(node.map_pose.yaw + delta_yaw);
          const double candidate_score =
            this->score_scan_candidate(this->temporary_map_, scan, candidate_pose);
          if (candidate_score > best_score) {
            best_score = candidate_score;
            best_pose = candidate_pose;
          }
        }
      }
    }

    if (!best.found || best_score > best.match_score) {
      best.found = true;
      best.node_index = static_cast<int>(index);
      best.matched_pose = best_pose;
      best.descriptor_distance = descriptor_distance;
      best.match_score = best_score;
    }
  }

  return best;
}

void SlamMapper::maybe_optimize_pose_graph(const LoopClosureCandidate & candidate, int current_node_index)
{
  if (!candidate.found || candidate.match_score < this->loop_closure_acceptance_score_) {
    return;
  }

  GraphEdge loop_edge;
  loop_edge.from = candidate.node_index;
  loop_edge.to = current_node_index;
  loop_edge.relative_pose = this->relative_pose(
    this->graph_nodes_[candidate.node_index].map_pose,
    candidate.matched_pose);
  loop_edge.weight = 3.0;
  loop_edge.loop_closure = true;
  this->graph_edges_.push_back(loop_edge);

  this->graph_nodes_[current_node_index].map_pose = candidate.matched_pose;
  this->optimize_pose_graph();
  this->rebuild_map_from_pose_graph();
  this->current_corrected_pose_ = this->graph_nodes_.back().map_pose;
  this->update_map_to_odom_transform(this->current_corrected_pose_, this->graph_nodes_.back().raw_odom_pose);

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted loop closure: node=%d current=%d descriptor=%.3f score=%.3f",
    candidate.node_index,
    current_node_index,
    candidate.descriptor_distance,
    candidate.match_score);
}

void SlamMapper::optimize_pose_graph()
{
  if (this->graph_nodes_.size() < 2U || this->graph_edges_.empty()) {
    return;
  }

  for (int iteration = 0; iteration < std::max(1, this->graph_optimization_iterations_); ++iteration) {
    for (const auto & edge : this->graph_edges_) {
      auto & from_node = this->graph_nodes_[static_cast<std::size_t>(edge.from)];
      auto & to_node = this->graph_nodes_[static_cast<std::size_t>(edge.to)];

      const Pose2D predicted_to = this->compose_pose(from_node.map_pose, edge.relative_pose);
      const double error_x = predicted_to.x - to_node.map_pose.x;
      const double error_y = predicted_to.y - to_node.map_pose.y;
      const double error_yaw = this->normalize_angle(predicted_to.yaw - to_node.map_pose.yaw);
      const double gain = this->graph_optimization_step_size_ * edge.weight;

      if (edge.to != 0) {
        to_node.map_pose.x += gain * error_x;
        to_node.map_pose.y += gain * error_y;
        to_node.map_pose.yaw = this->normalize_angle(to_node.map_pose.yaw + gain * error_yaw);
      }
      if (edge.loop_closure && edge.from != 0) {
        from_node.map_pose.x -= 0.5 * gain * error_x;
        from_node.map_pose.y -= 0.5 * gain * error_y;
        from_node.map_pose.yaw =
          this->normalize_angle(from_node.map_pose.yaw - 0.5 * gain * error_yaw);
      }
    }
  }
}

void SlamMapper::update_submap_accumulation(const GraphNode & node)
{
  if (this->submaps_.empty()) {
    return;
  }

  auto & submap = this->submaps_.back();
  submap.end_node_index = static_cast<int>(this->graph_nodes_.size()) - 1;
  submap.keyframe_count += 1;
  submap.anchor_pose = node.map_pose;
}

SlamMapper::Pose2D SlamMapper::compose_pose(const Pose2D & lhs, const Pose2D & rhs) const
{
  Pose2D composed{};
  composed.x = lhs.x + (std::cos(lhs.yaw) * rhs.x) - (std::sin(lhs.yaw) * rhs.y);
  composed.y = lhs.y + (std::sin(lhs.yaw) * rhs.x) + (std::cos(lhs.yaw) * rhs.y);
  composed.yaw = this->normalize_angle(lhs.yaw + rhs.yaw);
  return composed;
}

SlamMapper::Pose2D SlamMapper::inverse_pose(const Pose2D & pose) const
{
  Pose2D inverse{};
  inverse.yaw = this->normalize_angle(-pose.yaw);
  inverse.x = -(std::cos(inverse.yaw) * pose.x - std::sin(inverse.yaw) * pose.y);
  inverse.y = -(std::sin(inverse.yaw) * pose.x + std::cos(inverse.yaw) * pose.y);
  return inverse;
}

SlamMapper::Pose2D SlamMapper::relative_pose(const Pose2D & from, const Pose2D & to) const
{
  return this->compose_pose(this->inverse_pose(from), to);
}

bool SlamMapper::world_to_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  double x,
  double y,
  int & grid_x,
  int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  grid_y = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return (
    grid_x >= 0 && grid_x < static_cast<int>(map.info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(map.info.height));
}

bool SlamMapper::grid_index(
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

void SlamMapper::update_cell_score(
  nav_msgs::msg::OccupancyGrid & map,
  std::vector<int16_t> & occupancy_scores,
  int grid_x,
  int grid_y,
  int delta) const
{
  std::size_t index = 0U;
  if (!this->grid_index(map, grid_x, grid_y, index) || index >= occupancy_scores.size()) {
    return;
  }

  const int updated_score = std::clamp(
    static_cast<int>(occupancy_scores[index]) + delta,
    this->mapping_score_min_,
    this->mapping_score_max_);
  occupancy_scores[index] = static_cast<int16_t>(updated_score);
  this->refresh_cell_from_score(map, occupancy_scores, index);
}

void SlamMapper::refresh_cell_from_score(
  nav_msgs::msg::OccupancyGrid & map,
  const std::vector<int16_t> & occupancy_scores,
  std::size_t index) const
{
  if (index >= map.data.size() || index >= occupancy_scores.size()) {
    return;
  }
  const int score = static_cast<int>(occupancy_scores[index]);
  if (score >= this->mapping_occupied_score_threshold_) {
    map.data[index] = 100;
  } else if (score <= this->mapping_free_score_threshold_) {
    map.data[index] = 0;
  } else {
    map.data[index] = -1;
  }
}

void SlamMapper::raytrace_free_cells(
  nav_msgs::msg::OccupancyGrid & map,
  std::vector<int16_t> & occupancy_scores,
  int start_x,
  int start_y,
  int end_x,
  int end_y) const
{
  int x = start_x;
  int y = start_y;
  const int delta_x = std::abs(end_x - start_x);
  const int delta_y = std::abs(end_y - start_y);
  const int step_x = (start_x < end_x) ? 1 : -1;
  const int step_y = (start_y < end_y) ? 1 : -1;
  int error = delta_x - delta_y;

  while (x != end_x || y != end_y) {
    this->mark_free_cell(map, occupancy_scores, x, y);
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      y += step_y;
    }
  }
}

void SlamMapper::mark_free_cell(
  nav_msgs::msg::OccupancyGrid & map,
  std::vector<int16_t> & occupancy_scores,
  int grid_x,
  int grid_y) const
{
  this->update_cell_score(map, occupancy_scores, grid_x, grid_y, -this->mapping_free_score_);
}

void SlamMapper::mark_occupied_cell(
  nav_msgs::msg::OccupancyGrid & map,
  std::vector<int16_t> & occupancy_scores,
  int grid_x,
  int grid_y) const
{
  this->update_cell_score(map, occupancy_scores, grid_x, grid_y, this->mapping_hit_score_);
}

void SlamMapper::set_quaternion_from_yaw(
  geometry_msgs::msg::Quaternion & orientation,
  double yaw) const
{
  const double half_yaw = yaw * 0.5;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(half_yaw);
  orientation.w = std::cos(half_yaw);
}

double SlamMapper::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const
{
  const double siny_cosp = 2.0 * (
    (orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp = 1.0 - 2.0 * (
    (orientation.y * orientation.y) + (orientation.z * orientation.z));
  return std::atan2(siny_cosp, cosy_cosp);
}

double SlamMapper::normalize_angle(double angle) const
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

}  // namespace amr::slam::mapper
